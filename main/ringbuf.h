/*
 * SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
/*
 * ringbuf.h — thread-safe, PSRAM-backed frame queues
 * ============================================================================
 * A fixed-capacity single-producer/single-consumer-friendly ring of
 * pcan_frame_t entries, guarded by a FreeRTOS mutex for the metadata and sized
 * so the storage array lives in PSRAM (see config.h CFG_RB_USE_PSRAM).
 *
 * OWNERSHIP: frames are copied in and out by value; the ring owns its storage
 * for its whole lifetime. Callers never hold pointers into the ring.
 *
 * THREADING: safe for concurrent one-producer / one-consumer use, which is the
 * only pattern this firmware needs (e.g. TWAI-RX task pushes, USB-TX task
 * pops). The blocking variants use a counting semaphore so a consumer can
 * sleep until data arrives without polling. The *_isr variants are lock-free
 * enough to call from the TWAI alert path is NOT supported — push from a task,
 * not an ISR (the TWAI legacy driver already delivers frames to a task).
 * ============================================================================
 */
#ifndef PCAN_RINGBUF_H
#define PCAN_RINGBUF_H

#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "pcan_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    pcan_frame_t     *slots;    /* storage[capacity], PSRAM-backed            */
    uint32_t          capacity; /* number of slots                            */
    volatile uint32_t head;     /* producer index (next write)                */
    volatile uint32_t tail;     /* consumer index (next read)                 */
    SemaphoreHandle_t lock;     /* protects slot copy + head/tail movement    */
    SemaphoreHandle_t items;    /* counting sem: committed items to pop       */
    SemaphoreHandle_t spaces;   /* counting sem: writable slots               */
    uint32_t          dropped;  /* frames dropped on full (overrun counter)   */
} pcan_ringbuf_t;

/*
 * Initialize a ring with `capacity` slots. Allocates storage in PSRAM when
 * use_psram is true (falls back to internal RAM if PSRAM is absent), and
 * creates the mutex + counting semaphore.
 * Returns true on success. On failure the struct is left zeroed and no
 * resources leak. Not thread-safe against itself; call once at startup.
 */
bool pcan_rb_init(pcan_ringbuf_t *rb, uint32_t capacity, bool use_psram);

/* Free storage and delete the semaphores. Safe to call on a zeroed struct. */
void pcan_rb_deinit(pcan_ringbuf_t *rb);

/*
 * Copy `frame` into the ring. Non-blocking. Returns true if stored, false if
 * the ring was full (and increments rb->dropped). The frame is copied by
 * value; the caller retains ownership of its copy.
 */
bool pcan_rb_push(pcan_ringbuf_t *rb, const pcan_frame_t *frame);

/*
 * Retryable variant for producers that retain ownership and apply backpressure.
 * It has the same storage semantics as pcan_rb_push(), but a full/contended ring
 * returns false WITHOUT incrementing rb->dropped because no frame was lost — the
 * caller must retry the same frame later.
 */
bool pcan_rb_try_push(pcan_ringbuf_t *rb, const pcan_frame_t *frame);

/*
 * Pop one frame into `out`, blocking up to `ticks_to_wait` (portMAX_DELAY to
 * wait forever, 0 to poll). Returns true if a frame was written to `out`,
 * false on timeout/empty. `out` must point to caller storage.
 */
bool pcan_rb_pop(pcan_ringbuf_t *rb, pcan_frame_t *out, TickType_t ticks_to_wait);

/* Current number of queued frames (snapshot; may change immediately). */
uint32_t pcan_rb_count(pcan_ringbuf_t *rb);

/* Read and clear the dropped-frame (overrun) counter. */
uint32_t pcan_rb_take_dropped(pcan_ringbuf_t *rb);

#ifdef __cplusplus
}
#endif
#endif /* PCAN_RINGBUF_H */
