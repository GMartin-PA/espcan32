/*
 * SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
/*
 * ringbuf.c — thread-safe PSRAM-backed frame ring
 * See ringbuf.h for the interface contract.
 *
 * Design notes
 * ------------
 * Single-producer / single-consumer ring of pcan_frame_t, copied by value.
 * Concurrency uses two primitives:
 *   - rb->lock  : a FreeRTOS mutex that serialises every mutation of the
 *                 head/tail indices and the dropped counter. The critical
 *                 sections are tiny (a memcpy of one frame + an index bump),
 *                 so holding a mutex is cheap and keeps the code obviously
 *                 correct without hand-rolled lock-free ordering.
 *   - rb->items : committed frames available to the consumer.
 *   - rb->spaces: writable slots available to the producer. A consumer returns
 *                 one space only AFTER it has copied the reserved slot, so a
 *                 full ring cannot let the producer overwrite a frame whose
 *                 item token was taken just before the consumer was preempted.
 *
 * Ordering (SPSC-safe):
 *   push: take(spaces,0) -> take(lock) -> write slots[head], advance head ->
 *         give(lock) -> give(items).
 *   pop : take(items,ticks) -> take(lock) -> read slots[tail], advance tail ->
 *         give(lock) -> give(spaces).
 * The two counting semaphores make full and empty unambiguous even when
 * head==tail and preserve slot ownership across preemption.
 *
 * push() never blocks (mutex sections are bounded, semaphore give is O(1)) and
 * must be called from task context, never an ISR (uses the blocking mutex API).
 */
#include "ringbuf.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <string.h>
#include <inttypes.h>
#include <stdint.h>

static const char *TAG = "ringbuf";

/* Bound on the mutex acquire in pcan_rb_push(). push() is called from the
 * TinyUSB rx callback (tx_ring) which MUST NOT block, so we cap the wait and
 * drop-on-contention instead of blocking indefinitely. The critical section is
 * a single frame memcpy + index bump, so this bound is never hit in practice. */
#define RB_PUSH_LOCK_TIMEOUT_MS 5u

bool pcan_rb_init(pcan_ringbuf_t *rb, uint32_t capacity, bool use_psram)
{
    if (rb == NULL || capacity == 0) {
        return false;
    }

    /* Start from a known-clean slate so the failure path can blindly clean up
     * and re-zero without leaking or double-freeing. */
    memset(rb, 0, sizeof(*rb));

    /* Guard the size multiplication against overflow (capacity is host-supplied
     * config; sizeof(pcan_frame_t) is small, but be defensive). */
    const size_t elem = sizeof(pcan_frame_t);
    if (capacity > (SIZE_MAX / elem)) {
        ESP_LOGE(TAG, "capacity %" PRIu32 " too large", capacity);
        return false;
    }
    const size_t bytes = (size_t)capacity * elem;

    /* Backing storage: prefer PSRAM when requested, fall back to internal RAM
     * (byte-addressable) if PSRAM is absent or exhausted. */
    uint32_t caps = use_psram ? MALLOC_CAP_SPIRAM
                              : (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    rb->slots = (pcan_frame_t *)heap_caps_malloc(bytes, caps);
    if (rb->slots == NULL && use_psram) {
        ESP_LOGW(TAG, "PSRAM alloc of %u B failed; falling back to internal RAM",
                 (unsigned)bytes);
        rb->slots = (pcan_frame_t *)heap_caps_malloc(
            bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    if (rb->slots == NULL) {
        ESP_LOGE(TAG, "failed to allocate %u B for ring storage",
                 (unsigned)bytes);
        goto fail;
    }

    rb->lock = xSemaphoreCreateMutex();
    if (rb->lock == NULL) {
        ESP_LOGE(TAG, "failed to create ring mutex");
        goto fail;
    }

    /* Item tokens start empty; every slot starts writable. */
    rb->items = xSemaphoreCreateCounting(capacity, 0);
    rb->spaces = xSemaphoreCreateCounting(capacity, capacity);
    if (rb->items == NULL || rb->spaces == NULL) {
        ESP_LOGE(TAG, "failed to create ring counting semaphores");
        goto fail;
    }

    rb->capacity = capacity;
    rb->head = 0;
    rb->tail = 0;
    rb->dropped = 0;
    return true;

fail:
    if (rb->items != NULL) {
        vSemaphoreDelete(rb->items);
    }
    if (rb->spaces != NULL) {
        vSemaphoreDelete(rb->spaces);
    }
    if (rb->lock != NULL) {
        vSemaphoreDelete(rb->lock);
    }
    if (rb->slots != NULL) {
        heap_caps_free(rb->slots);
    }
    memset(rb, 0, sizeof(*rb));
    return false;
}

void pcan_rb_deinit(pcan_ringbuf_t *rb)
{
    if (rb == NULL) {
        return;
    }
    if (rb->items != NULL) {
        vSemaphoreDelete(rb->items);
    }
    if (rb->spaces != NULL) {
        vSemaphoreDelete(rb->spaces);
    }
    if (rb->lock != NULL) {
        vSemaphoreDelete(rb->lock);
    }
    if (rb->slots != NULL) {
        heap_caps_free(rb->slots);
    }
    memset(rb, 0, sizeof(*rb));
}

static bool pcan_rb_push_impl(pcan_ringbuf_t *rb, const pcan_frame_t *frame,
                              bool count_drop)
{
    if (rb == NULL || frame == NULL || rb->lock == NULL || rb->items == NULL ||
        rb->spaces == NULL) {
        return false;
    }

    /* Reserve a writable slot first. A consumer gives this token only after it
     * has copied its slot, so the reservation is safe even when head==tail. */
    if (xSemaphoreTake(rb->spaces, 0) != pdTRUE) {
        if (count_drop) {
            rb->dropped++;
        }
        return false;
    }

    /* Bounded acquire: retryable producers retain ownership; ordinary producers
     * return the reserved space and record a real drop on lock contention. */
    if (xSemaphoreTake(rb->lock, pdMS_TO_TICKS(RB_PUSH_LOCK_TIMEOUT_MS)) != pdTRUE) {
        (void)xSemaphoreGive(rb->spaces);
        if (count_drop) {
            rb->dropped++;   /* best-effort counter; benign race with take_dropped */
        }
        return false;
    }

    memcpy(&rb->slots[rb->head], frame, sizeof(*frame));
    rb->head++;
    if (rb->head >= rb->capacity) {
        rb->head = 0;
    }

    xSemaphoreGive(rb->lock);

    /* Publish only after the frame copy and head update are complete. */
    (void)xSemaphoreGive(rb->items);
    return true;
}

bool pcan_rb_push(pcan_ringbuf_t *rb, const pcan_frame_t *frame)
{
    return pcan_rb_push_impl(rb, frame, true);
}

bool pcan_rb_try_push(pcan_ringbuf_t *rb, const pcan_frame_t *frame)
{
    return pcan_rb_push_impl(rb, frame, false);
}

bool pcan_rb_pop(pcan_ringbuf_t *rb, pcan_frame_t *out, TickType_t ticks_to_wait)
{
    if (rb == NULL || out == NULL || rb->lock == NULL || rb->items == NULL ||
        rb->spaces == NULL) {
        return false;
    }

    /* Block (up to ticks_to_wait) until a frame is available. This decrements
     * the item count; the matching slot is guaranteed present. */
    if (xSemaphoreTake(rb->items, ticks_to_wait) != pdTRUE) {
        return false;  /* timeout / empty */
    }

    xSemaphoreTake(rb->lock, portMAX_DELAY);

    memcpy(out, &rb->slots[rb->tail], sizeof(*out));
    rb->tail++;
    if (rb->tail >= rb->capacity) {
        rb->tail = 0;
    }

    xSemaphoreGive(rb->lock);

    /* The producer may reuse this slot only after our copy is complete. */
    (void)xSemaphoreGive(rb->spaces);
    return true;
}

uint32_t pcan_rb_count(pcan_ringbuf_t *rb)
{
    if (rb == NULL || rb->items == NULL) {
        return 0;
    }
    /* The counting semaphore's value is the number of poppable frames — an
     * atomic snapshot that may change the instant after we read it. */
    return (uint32_t)uxSemaphoreGetCount(rb->items);
}

uint32_t pcan_rb_take_dropped(pcan_ringbuf_t *rb)
{
    if (rb == NULL || rb->lock == NULL) {
        return 0;
    }
    xSemaphoreTake(rb->lock, portMAX_DELAY);
    uint32_t old = rb->dropped;
    rb->dropped = 0;
    xSemaphoreGive(rb->lock);
    return old;
}
