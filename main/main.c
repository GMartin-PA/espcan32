/*
 * SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
/*
 * main.c — application entry + task wiring
 * ============================================================================
 * app_main() boots the firmware, selects the operating mode (PCAN emulation
 * vs standalone ISO-TP/UDS bench), and starts the FreeRTOS task graph.
 *
 * See DESIGN.md §2 (task/threading model), §3 (data flow) and §4 (mode select).
 *
 * INTEGRATION NOTES:
 *   - Self-reception echo: twai_hal transmits SRR frames with message.self set,
 *     so the controller delivers its own copy back through the ordinary RX path
 *     (see twai_hal.c). That hardware path is authoritative, so can_tx_task does
 *     NOT also synthesise a software loopback echo — doing both would echo every
 *     self-request frame to the host twice.
 *   - reconfig_pending: pcan_cmd exposes no atomic take-and-clear, so ctrl_task
 *     reads the volatile flag, clears it BEFORE applying the reconfigure, then
 *     snapshots + applies. Clearing first means a CMD arriving during the
 *     reconfigure re-sets the flag and is re-applied next tick rather than lost.
 *   - TWAI teardown safety: twai_driver_uninstall() must not run while a task
 *     is blocked inside twai_receive/twai_transmit (the legacy driver deletes
 *     its queues). ctrl_task therefore parks twai_rx_task and can_tx_task at a
 *     safe point (outside any twai_hal call) via a quiesce handshake before
 *     every reconfigure, and releases them afterwards.
 * ============================================================================
 */
#include "config.h"
#include "pcan_proto.h"
#include "pcan_cmd.h"
#include "pcan_msg.h"
#include "pcan_time.h"
#include "ringbuf.h"
#include "twai_hal.h"
#include "usb_glue.h"
#if CFG_ENABLE_BENCH_MODE
#include "isotp.h"
#include "uds.h"
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_err.h"
#include <string.h>

static const char *TAG = "espcan32";

/* Shared singletons (definitions live here; other modules receive pointers). */
static pcan_dev_state_t s_dev_state;
static pcan_ringbuf_t   s_rx_ring;   /* CAN -> USB (dev->host)                 */
static pcan_ringbuf_t   s_tx_ring;   /* USB -> CAN (host->dev)                 */

/* ctrl_task -> usb_tx_task path for host-visible bus error records. Each item
 * is a PCAN_USB_ERROR_* bitmask; usb_tx_task folds them into REC_ERROR records
 * on the EP2-IN stream. */
static QueueHandle_t s_evt_queue;

/* ctrl_task -> can_tx_task TX completion events. can_tx_task keeps ONE frame
 * outstanding and only releases ownership when ctrl_task forwards one of these
 * (see the ownership rule above can_tx_task). */
#define CAN_TX_EVENT_OK     1
#define CAN_TX_EVENT_FAIL   2
#define CAN_TX_EVENT_ABORT  3   /* driver discarded its queue (stop / bus-off)   */
static QueueHandle_t s_tx_done;

/* Quiesce handshake so ctrl_task can reconfigure the TWAI driver while the RX
 * and TX tasks are parked outside every twai_hal call (see teardown note).
 * s_quiesce_seq is a single word carrying both the request flag and a cycle id:
 * ODD means "park requested", and every request gets a fresh id. A parking task
 * echoes the id it is parked FOR into its ack word and keeps re-echoing it while
 * parked, so a park that was decided on a previous cycle can never be read as
 * consent to tear the driver down in this one. One word rather than a flag plus
 * a counter, so the two can never be observed out of order across cores. */
static volatile uint32_t s_quiesce_seq;
static volatile uint32_t s_quiesce_rx_ack;
static volatile uint32_t s_quiesce_tx_ack;

/* Loop timing constants. */
#define USBTX_POLL_MS        5u   /* rx_ring pop timeout / calibration wakeups   */
#define USBTX_RETRY_MS       1u   /* yield while an EP2-IN transfer is in flight  */
#define TWAI_CALL_TIMEOUT_MS 20u  /* rx/tx twai_hal blocking-call timeout         */
#define TWAI_RETRY_MS        1u   /* driver TX queue full: retry the same frame   */
#define TWAI_BACKOFF_MS      2u   /* driver stopped/reconfiguring: back off       */
#define CTRL_PERIOD_MS       1u   /* ctrl_task cadence (paces TX completion)      */
#define BRIDGE_STATS_MS      1000u /* loss/backpressure telemetry cadence          */
#define USBTX_MAX_DRAIN      96u  /* max frames coalesced per usb_tx_task pass    */
#define CAN_TX_CONFIRM_SLICE_MS 100u /* quiesce-latency slice of the TX confirm wait */
#define CAN_TX_CONFIRM_SLICES 5u  /* silent slices before the stall is reported   */
#define QUIESCE_POLL_MS      1u   /* park loop / handshake polling cadence        */
#define QUIESCE_WAIT_MS      500u /* max wait for both TWAI tasks to park         */
#define BENCH_POLL_MS        1u   /* bench_task rx poll / ISO-TP timer cadence    */

/*
 * pdMS_TO_TICKS() rounds DOWN, so anything shorter than one tick period becomes
 * zero — and a zero-tick vTaskDelay is a bare yield, while a zero-tick blocking
 * receive is a busy poll. sdkconfig.defaults pins CONFIG_FREERTOS_HZ=1000, but
 * at the IDF default of 100 Hz that would turn twai_rx_task's boot back-off
 * (prio 12, core 1) into a spin that never lets ctrl_task (prio 8, same core)
 * install the driver, and the task watchdog would trip. Every delay and every
 * blocking timeout in this file therefore goes through this helper.
 */
static inline TickType_t ms_ticks(uint32_t ms)
{
    TickType_t t = pdMS_TO_TICKS(ms);
    return (t != 0) ? t : 1;
}

/* Single-writer diagnostic counters sampled by ctrl_task. Aligned 32-bit reads
 * and writes are atomic on ESP32-S3; each counter has exactly one incrementing
 * task, so no cross-core read-modify-write race exists. */
static volatile uint32_t s_usb_in_busy_retries;
static volatile uint32_t s_can_tx_timeout_retries;
static volatile uint32_t s_can_tx_state_retries;
static volatile uint32_t s_can_tx_resubmits;
static volatile uint32_t s_can_tx_confirm_timeouts;
static volatile uint32_t s_can_tx_listen_drops;

/* -------------------------------------------------------------------------
 * Status LED (mirrors CMD_LED). Configured once, driven from ctrl_task.
 * ------------------------------------------------------------------------- */
static void status_led_init(void)
{
    if ((int)CFG_STATUS_LED_GPIO < 0) {
        return;
    }
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << (int)CFG_STATUS_LED_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    (void)gpio_config(&io);
    gpio_set_level((gpio_num_t)CFG_STATUS_LED_GPIO, 0);
}

static void status_led_set(bool on)
{
    if ((int)CFG_STATUS_LED_GPIO < 0) {
        return;
    }
    gpio_set_level((gpio_num_t)CFG_STATUS_LED_GPIO, on ? 1 : 0);
}

/* -------------------------------------------------------------------------
 * Mode selection. Reads CFG_MODE_SELECT_GPIO (forces BENCH when asserted),
 * else the NVS key CFG_NVS_KEY_MODE, else CFG_DEFAULT_MODE.
 * ------------------------------------------------------------------------- */
static int app_mode_select(void)
{
    /* 1. Boot-strap GPIO has highest precedence. */
    if ((int)CFG_MODE_SELECT_GPIO >= 0) {
        gpio_config_t io = {
            .pin_bit_mask = (1ULL << (int)CFG_MODE_SELECT_GPIO),
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        if (gpio_config(&io) == ESP_OK) {
            int level = gpio_get_level((gpio_num_t)CFG_MODE_SELECT_GPIO);
#if CFG_MODE_SELECT_ACTIVE_LOW
            bool asserted = (level == 0);
#else
            bool asserted = (level != 0);
#endif
            if (asserted) {
                ESP_LOGI(TAG, "mode select GPIO asserted -> BENCH");
                return CFG_MODE_BENCH;
            }
        }
    }

    /* 2. NVS-pinned mode (survives reboots). */
    nvs_handle_t h;
    if (nvs_open(CFG_NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        uint8_t m = 0;
        esp_err_t e = nvs_get_u8(h, CFG_NVS_KEY_MODE, &m);
        nvs_close(h);
        if (e == ESP_OK) {
            if (m == CFG_MODE_BENCH) {
                ESP_LOGI(TAG, "NVS mode=BENCH");
                return CFG_MODE_BENCH;
            }
            if (m == CFG_MODE_PCAN) {
                ESP_LOGI(TAG, "NVS mode=PCAN");
                return CFG_MODE_PCAN;
            }
        }
    }

    /* 3. Compile-time default. */
    return CFG_DEFAULT_MODE;
}

/* =========================================================================
 * PCAN emulation tasks (see DESIGN.md §2)
 * ========================================================================= */

/* Park outside every twai_hal call until ctrl_task ends the cycle whose id we
 * saw, re-publishing that id on each pass so a cycle started while we were on
 * our way in still gets an explicit acknowledgement from us. Callers re-check
 * the request afterwards, so leaving on an even (released) id is always safe. */
static void quiesce_park(volatile uint32_t *ack, uint32_t cycle)
{
    do {
        *ack = cycle;
        vTaskDelay(ms_ticks(QUIESCE_POLL_MS));
        cycle = s_quiesce_seq;
    } while ((cycle & 1u) != 0u);
}

/* twai_rx_task (core 1, prio 12): drain the TWAI RX queue, stamp ts16 (done
 * inside pcan_twai_receive), push into rx_ring. Parks on the quiesce request. */
static void twai_rx_task(void *arg)
{
    (void)arg;
    for (;;) {
        uint32_t cycle = s_quiesce_seq;
        if ((cycle & 1u) != 0u) {
            quiesce_park(&s_quiesce_rx_ack, cycle);
            continue;
        }

        pcan_frame_t f;
        esp_err_t e = pcan_twai_receive(&f, ms_ticks(TWAI_CALL_TIMEOUT_MS));
        if (e == ESP_OK) {
            (void)pcan_rb_push(&s_rx_ring, &f);   /* drops on full (overrun)     */
        } else if (e == ESP_ERR_TIMEOUT) {
            /* idle bus: keep looping */
        } else {
            /* ESP_ERR_INVALID_STATE mid-reconfigure or driver stopped: back off. */
            vTaskDelay(ms_ticks(TWAI_BACKOFF_MS));
        }
    }
}

/*
 * Submit exactly one completed EP2-IN batch. A busy endpoint means the previous
 * transfer still owns the DMA buffer, not that this batch may be discarded.
 * Keep the caller-owned batch intact, yield to TinyUSB, and retry until accepted.
 *
 * THIS LOOP DOES NOT TERMINATE WHILE THE HOST IS ABSENT, AND THAT IS DELIBERATE.
 * usb_glue_send_msg_batch() takes ownership of a batch it accepted and retains it
 * across a USB reset/re-enumeration, releasing it only once that exact transfer
 * completed on the wire — it identifies the batch by its length, so handing it a
 * different one after abandoning this would let a stale completion release the
 * wrong batch. There is no way to withdraw a batch the glue already holds, so the
 * caller must keep presenting the identical buf/len. The cost of blocking here is
 * only that rx_ring stops draining while the host is gone, and its overflow is
 * counted (rx-ring-drop) — the same data would be lost either way, because there
 * is nowhere to send it.
 */
static void usb_tx_flush_pending(pcan_batch_t *b)
{
    bool logged_unmounted = false;

    while (pcan_msg_batch_count(b) > 0) {
        /* Also advance deferred host->CAN EP2-OUT work while we are awake. */
        usb_glue_service();

        if (usb_glue_send_msg_batch(b->buf, b->len) == b->len) {
            pcan_msg_batch_reset(b);
            return;
        }

        bool mounted = usb_glue_mounted();
        if (mounted) {
            s_usb_in_busy_retries++;
        } else if (!logged_unmounted) {
            ESP_LOGW(TAG, "USB unmounted with %u EP2-IN records pending; retaining",
                     (unsigned)pcan_msg_batch_count(b));
            logged_unmounted = true;
        }

        /* TinyUSB (prio 11) outranks this task and the deferred submit runs in
         * its context; still yield so lower-priority bookkeeping can run. With
         * no host there is nothing to race, so back off to the idle cadence
         * instead of spinning for the duration of the disconnect. */
        vTaskDelay(ms_ticks(mounted ? USBTX_RETRY_MS : USBTX_POLL_MS));
    }
}

/* usb_tx_task (core 0, prio 10): drain rx_ring into EP2-IN batches, emit a
 * periodic REC_TS calibration record, fold ctrl_task's REC_ERROR events in. */
static void usb_tx_task(void *arg)
{
    (void)arg;

    pcan_batch_t b;
    pcan_msg_batch_reset(&b);

    uint64_t last_calib_us = pcan_time_now_us();
    const uint64_t calib_period_us =
        (uint64_t)PCAN_USB_TS_CALIB_PERIOD_MS * 1000ULL;

    for (;;) {
        /* Retry deferred host->CAN frames before blocking for CAN RX traffic. */
        usb_glue_service();

        /* Block briefly for the next RX frame; the timeout also paces the
         * calibration/error cadence when the bus is idle. */
        pcan_frame_t f;
        if (pcan_rb_pop(&s_rx_ring, &f, ms_ticks(USBTX_POLL_MS))) {
            if (!pcan_msg_batch_add_frame(&b, &f)) {
                /* Batch full: retain until accepted, then add to the empty batch. */
                usb_tx_flush_pending(&b);
                if (!pcan_msg_batch_add_frame(&b, &f)) {
                    ESP_LOGE(TAG, "CAN frame cannot fit in an empty USB batch");
                }
            }
            /* Opportunistically coalesce more queued frames — bounded so a busy
             * bus cannot make this task monopolise core 0 and starve the
             * TinyUSB task that actually ships EP2-IN (DESIGN §2 concurrency). */
            unsigned drained = 0;
            while (drained < USBTX_MAX_DRAIN && pcan_rb_pop(&s_rx_ring, &f, 0)) {
                if (!pcan_msg_batch_add_frame(&b, &f)) {
                    usb_tx_flush_pending(&b);
                    if (!pcan_msg_batch_add_frame(&b, &f)) {
                        ESP_LOGE(TAG, "CAN frame cannot fit in an empty USB batch");
                    }
                }
                drained++;
            }
        }

        /* Periodic REC_TS calibration so the host anchors time before a wrap.
         * It SHIPS ALONE, in its own batch, and that placement is part of the
         * contract rather than a convenience: the host re-anchors its time
         * reference only for a REC_TS at record index 0 (peak_usb_set_ts_now);
         * at any later index it takes the update path, which re-adds the whole
         * elapsed tick count on top of the running total and makes host time
         * run away quadratically. The record also carries its tick with the
         * TIMESTAMP bit clear, so it leaves the host's previous single-byte
         * tick untouched — a 1-byte timestamp decoded after it in the same
         * batch would land 256 ticks out. Flush first, emit, flush again. */
        if (pcan_time_now_us() - last_calib_us >= calib_period_us) {
            usb_tx_flush_pending(&b);
            if (!pcan_msg_batch_add_calib_ts(&b, pcan_time_now16())) {
                ESP_LOGE(TAG, "calibration record cannot fit in empty USB batch");
            }
            usb_tx_flush_pending(&b);
            /* Re-read the clock: the flushes above block while the host is
             * absent, and pacing off a stale reading would then emit a burst. */
            last_calib_us = pcan_time_now_us();
        }

        /* Fold bus-error records posted by the control task. */
        uint8_t err_mask;
        while (xQueueReceive(s_evt_queue, &err_mask, 0) == pdTRUE) {
            if (!pcan_msg_batch_add_error(&b, err_mask, pcan_time_now16())) {
                usb_tx_flush_pending(&b);
                if (!pcan_msg_batch_add_error(&b, err_mask, pcan_time_now16())) {
                    ESP_LOGE(TAG, "error record cannot fit in empty USB batch");
                }
            }
        }

        /* Flush whatever accumulated this iteration. The helper resets only after
         * EP2-IN has accepted this exact batch. */
        usb_tx_flush_pending(&b);
    }
}

/*
 * can_tx_task (core 1, prio 9): drain tx_ring, transmit on the bus with
 * SINGLE-OUTSTANDING ownership.
 *
 * OWNERSHIP RULE — a frame accepted by pcan_twai_transmit() belongs to the TWAI
 * driver and CANNOT BE TAKEN BACK. ESP_OK means "queued", not "transmitted", and
 * the legacy driver offers no way to withdraw the copy it is already arbitrating
 * for (twai_clear_transmit_queue() empties the software queue but leaves the
 * frame in the hardware TX buffer). So the slot is released only by an event
 * that proves what became of the driver's copy:
 *   OK    — it reached the wire; take the next frame.
 *   FAIL  — the controller aborted it; the driver dropped it, so re-sending it
 *           cannot put a second copy on the bus.
 *   ABORT — ctrl_task reset the driver's TX queue (reconfigure, or the bus-off
 *           recovery that discards the queue without ever raising a TX alert),
 *           same guarantee.
 * Silence proves nothing: while the emulator is the only node on the bus no
 * alert arrives at all, and re-sending on a timeout would put the SAME frame in
 * the queue twice — one host write() becoming two frames on the wire the moment
 * a second node starts ACKing, and every later completion then attributed to the
 * wrong frame. A confirmation timeout therefore only reports the stall; the
 * frame stays owned by the driver until the driver says otherwise (bus-off is
 * reached within milliseconds on a dead bus, which produces the ABORT that
 * resolves it). Backpressure lands on tx_ring, where it is counted.
 *
 * The SRR self-reception echo is produced by the hardware path in twai_hal (see
 * the integration note at the top).
 */
static void can_tx_task(void *arg)
{
    (void)arg;
    pcan_frame_t pending;
    bool have_pending = false;
    bool inflight = false;
    unsigned confirm_misses = 0;

    for (;;) {
        uint32_t cycle = s_quiesce_seq;
        if ((cycle & 1u) != 0u) {
            quiesce_park(&s_quiesce_tx_ack, cycle);
            continue;
        }

        if (!have_pending) {
            if (!pcan_rb_pop(&s_tx_ring, &pending,
                             ms_ticks(TWAI_CALL_TIMEOUT_MS))) {
                continue;   /* nothing to send this window */
            }
            have_pending = true;
        }

        if (!inflight) {
            /* Discard completion events belonging to previous frames before
             * this submit; anything arriving afterwards is about THIS frame. */
            int stale;
            while (xQueueReceive(s_tx_done, &stale, 0) == pdTRUE) {
            }

            esp_err_t e = pcan_twai_transmit(&pending,
                                             ms_ticks(TWAI_CALL_TIMEOUT_MS));
            if (e == ESP_OK) {
                inflight = true;
                confirm_misses = 0;
            } else if (e == ESP_ERR_TIMEOUT) {
                /* Driver TX queue full: retain this exact frame and retry before
                 * popping another, preserving host ordering. */
                s_can_tx_timeout_retries++;
                vTaskDelay(ms_ticks(TWAI_RETRY_MS));
            } else if (e == ESP_ERR_INVALID_STATE) {
                /* Driver is stopped/reconfiguring. Keep ownership across the gap. */
                s_can_tx_state_retries++;
                vTaskDelay(ms_ticks(TWAI_BACKOFF_MS));
            } else if (e == ESP_ERR_NOT_SUPPORTED) {
                /* Listen-only is an explicit host-selected state; transmission is
                 * impossible by definition, so the discard is semantic rather
                 * than a fault. Counted rather than logged per frame: a host that
                 * keeps writing while silent would otherwise flood the console. */
                s_can_tx_listen_drops++;
                have_pending = false;
            } else {
                s_can_tx_state_retries++;
                ESP_LOGW(TAG, "CAN TX 0x%03X deferred: %s",
                         (unsigned)pending.id, esp_err_to_name(e));
                vTaskDelay(ms_ticks(TWAI_BACKOFF_MS));
            }
            continue;
        }

        /* Await the TX completion event for the outstanding frame. The slices
         * keep the quiesce park latency bounded and let a prolonged silence be
         * reported once, without ever releasing the driver's copy. */
        int ev;
        if (xQueueReceive(s_tx_done, &ev,
                          ms_ticks(CAN_TX_CONFIRM_SLICE_MS)) == pdTRUE) {
            inflight = false;
            confirm_misses = 0;
            if (ev == CAN_TX_EVENT_OK) {
                have_pending = false;
            } else {
                /* FAIL / ABORT: the driver discarded its copy, so putting the
                 * frame back cannot duplicate it on the wire. */
                s_can_tx_resubmits++;
            }
        } else if (++confirm_misses == CAN_TX_CONFIRM_SLICES) {
            s_can_tx_confirm_timeouts++;
            ESP_LOGW(TAG, "CAN TX 0x%03X unconfirmed for %ums; still owned by "
                     "the driver", (unsigned)pending.id,
                     (unsigned)(CAN_TX_CONFIRM_SLICE_MS * CAN_TX_CONFIRM_SLICES));
        }
    }
}

/* Park the two TWAI tasks outside any twai_hal call, apply an action while
 * they are safely stopped, then release them. */
/* Returns true only if BOTH TWAI tasks confirmed they reached their park point
 * before touching the driver is safe. On false the caller MUST NOT reconfigure
 * or uninstall — a task may still be blocked inside a twai_hal call on a queue
 * the driver would free (use-after-free). */
static bool ctrl_quiesce_begin(void)
{
    /* Odd id = request outstanding. Both tasks must echo THIS id; an ack left
     * over from an earlier cycle carries an earlier id and is ignored. */
    const uint32_t cycle = s_quiesce_seq + 1u;
    s_quiesce_seq = cycle;

    /* Bounded wait: the tasks only ever sit inside a twai_hal call for
     * TWAI_CALL_TIMEOUT_MS before re-checking, so both should park well within
     * this. If not (e.g. a wedged driver), we bail rather than free live queues.
     * Counted in ticks so the timeout is the same wall-clock at any tick rate. */
    const TickType_t poll = ms_ticks(QUIESCE_POLL_MS);
    TickType_t left = ms_ticks(QUIESCE_WAIT_MS);
    while (s_quiesce_rx_ack != cycle || s_quiesce_tx_ack != cycle) {
        if (left == 0) {
            return false;
        }
        vTaskDelay(poll);
        left = (left > poll) ? (left - poll) : 0;
    }
    return true;
}

static void ctrl_quiesce_end(void)
{
    /* Even id = released. Parked tasks see the parity flip and resume. */
    s_quiesce_seq++;
}

static void bridge_log_stats(void)
{
    static uint32_t last_usb_in_busy;
    static uint32_t last_can_tx_timeout;
    static uint32_t last_can_tx_state;
    static uint32_t last_can_tx_resubmits;
    static uint32_t last_can_tx_confirm_timeouts;
    static uint32_t last_can_tx_listen_drops;
    static pcan_twai_status_t last_twai;

    uint32_t rx_ring_drops = pcan_rb_take_dropped(&s_rx_ring);
    uint32_t tx_ring_drops = pcan_rb_take_dropped(&s_tx_ring);

    usb_glue_stats_t usb_stats;
    usb_glue_take_stats(&usb_stats);

    uint32_t usb_in_busy = s_usb_in_busy_retries;
    uint32_t can_tx_timeout = s_can_tx_timeout_retries;
    uint32_t can_tx_state = s_can_tx_state_retries;
    uint32_t can_tx_resubmits = s_can_tx_resubmits;
    uint32_t can_tx_conf_to = s_can_tx_confirm_timeouts;
    uint32_t can_tx_listen = s_can_tx_listen_drops;
    uint32_t usb_in_delta = usb_in_busy - last_usb_in_busy;
    uint32_t can_timeout_delta = can_tx_timeout - last_can_tx_timeout;
    uint32_t can_state_delta = can_tx_state - last_can_tx_state;
    uint32_t can_resub_delta = can_tx_resubmits - last_can_tx_resubmits;
    uint32_t can_conf_to_delta = can_tx_conf_to - last_can_tx_confirm_timeouts;
    uint32_t can_listen_delta = can_tx_listen - last_can_tx_listen_drops;

    pcan_twai_status_t twai;
    bool have_twai = (pcan_twai_get_status(&twai) == ESP_OK);
    bool twai_changed = have_twai &&
        (twai.rx_missed_count != last_twai.rx_missed_count ||
         twai.rx_overrun_count != last_twai.rx_overrun_count ||
         twai.tx_failed_count != last_twai.tx_failed_count);

    bool changed = rx_ring_drops != 0 || tx_ring_drops != 0 ||
        usb_stats.tx_backpressure_retries != 0 ||
        usb_stats.tx_pending_reset_drops != 0 ||
        usb_stats.cmd_out_rearm_failures != 0 ||
        usb_stats.msg_out_rearm_failures != 0 ||
        usb_stats.msg_in_failed_retries != 0 ||
        usb_stats.msg_in_reset_drops != 0 ||
        usb_in_delta != 0 || can_timeout_delta != 0 ||
        can_state_delta != 0 || can_resub_delta != 0 ||
        can_conf_to_delta != 0 || can_listen_delta != 0 || twai_changed;

    if (changed) {
        ESP_LOGW(TAG,
                 "bridge stats delta: usb-in-busy=%u host-tx-backpressure=%u "
                 "rx-ring-drop=%u tx-ring-drop=%u can-tx-timeout=%u "
                 "can-tx-state=%u can-tx-resubmit=%u can-tx-confirm-timeout=%u "
                 "can-tx-listen-drop=%u usb-reset-drop=%u cmd-rearm-fail=%u "
                 "out-rearm-fail=%u msg-in-fail=%u msg-in-reset-drop=%u "
                 "twai-rx-missed=%u twai-rx-overrun=%u twai-tx-failed=%u",
                 (unsigned)usb_in_delta,
                 (unsigned)usb_stats.tx_backpressure_retries,
                 (unsigned)rx_ring_drops, (unsigned)tx_ring_drops,
                 (unsigned)can_timeout_delta, (unsigned)can_state_delta,
                 (unsigned)can_resub_delta, (unsigned)can_conf_to_delta,
                 (unsigned)can_listen_delta,
                 (unsigned)usb_stats.tx_pending_reset_drops,
                 (unsigned)usb_stats.cmd_out_rearm_failures,
                 (unsigned)usb_stats.msg_out_rearm_failures,
                 (unsigned)usb_stats.msg_in_failed_retries,
                 (unsigned)usb_stats.msg_in_reset_drops,
                 have_twai ? (unsigned)twai.rx_missed_count : 0u,
                 have_twai ? (unsigned)twai.rx_overrun_count : 0u,
                 have_twai ? (unsigned)twai.tx_failed_count : 0u);
    }

    last_usb_in_busy = usb_in_busy;
    last_can_tx_timeout = can_tx_timeout;
    last_can_tx_state = can_tx_state;
    last_can_tx_resubmits = can_tx_resubmits;
    last_can_tx_confirm_timeouts = can_tx_conf_to;
    last_can_tx_listen_drops = can_tx_listen;
    if (have_twai) {
        last_twai = twai;
    }
}

/*
 * Drain the driver's pending alerts once and republish them: bus errors to
 * usb_tx_task as REC_ERROR masks, TX outcomes to can_tx_task.
 *
 * twai_read_alerts() CLEARS the bits it returns, so an alert this task chooses
 * not to forward is destroyed, not deferred. Every TX outcome is therefore
 * queued unconditionally — can_tx_task drains the queue immediately before each
 * submit, so an event that belongs to no frame is discarded there, where it is
 * safe, instead of here, where it would be indistinguishable from a lost one.
 */
static void ctrl_service_alerts(void)
{
    uint8_t pcan_err = 0;
    int     tx_event = PCAN_TX_EVENT_NONE;
    (void)pcan_twai_service_alerts(&pcan_err, &tx_event);

    if (pcan_err != 0) {
        (void)xQueueSend(s_evt_queue, &pcan_err, 0);   /* drop if full */
    }

    /* Forward TX completion independently of the host-visible state-change
     * return value: a pure TX_SUCCESS/TX_FAILED alert produces no REC_ERROR
     * mask and no state change, but can_tx_task still waits on it. */
    if (tx_event != PCAN_TX_EVENT_NONE) {
        int ev = (tx_event == PCAN_TX_EVENT_FAIL) ? CAN_TX_EVENT_FAIL
                                                  : CAN_TX_EVENT_OK;
        (void)xQueueSend(s_tx_done, &ev, 0);
    }

    /* Bus-off: pcan_twai_service_alerts() has just started recovery, which
     * resets the driver TX queue and drops the frame in the hardware buffer
     * WITHOUT ever raising a TX alert for it. That silent discard is the only
     * evidence can_tx_task will get that its frame is gone, so publish it —
     * queued after the completion above, which orders correctly for a frame
     * that made it out immediately before the bus went off. */
    if ((pcan_err & PCAN_USB_ERROR_BUS_OFF) != 0) {
        int ev = CAN_TX_EVENT_ABORT;
        (void)xQueueSend(s_tx_done, &ev, 0);
    }
}

/* ctrl_task (core 1, prio 8): apply pending reconfigurations, publish bridge
 * loss/backpressure telemetry, and translate TWAI alerts to REC_ERROR records. */
static void ctrl_task(void *arg)
{
    (void)arg;
    bool last_led = false;
    uint64_t last_stats_us = pcan_time_now_us();

    /* One-time CAN bring-up ON THIS CORE (core 1) so the TWAI ISR is allocated
     * on core 1, not core 0. Listen-only until the host opens the bus; the
     * subsequent CMD_SET_BUS reconfigure (also handled here) switches to NORMAL.
     * Done here rather than in start_pcan_mode() (core 0) purely for ISR core
     * affinity — esp_intr_alloc binds the interrupt to the installing core. */
    {
        esp_err_t ie = pcan_twai_start(PCAN_USB_DEFAULT_BITRATE,
                                       TWAI_HAL_MODE_LISTEN_ONLY);
        if (ie != ESP_OK) {
            ESP_LOGE(TAG, "initial pcan_twai_start (core %d) failed: %s",
                     CFG_TASK_CTRL_CORE, esp_err_to_name(ie));
        } else {
            ESP_LOGI(TAG, "TWAI brought up on core %d (ISR pinned here)",
                     CFG_TASK_CTRL_CORE);
        }
    }

    for (;;) {
        /* --- pending reconfigure (bitrate / mode / bus on-off) ------------- */
        if (s_dev_state.reconfig_pending) {
            /* Park both TWAI tasks BEFORE touching the driver. If they don't
             * reach their park points in time, leave reconfig_pending set and
             * retry next tick — never uninstall while a task is blocked in a
             * twai_hal call (that frees queues under it: use-after-free). */
            if (!ctrl_quiesce_begin()) {
                ctrl_quiesce_end();
                ESP_LOGW(TAG, "quiesce timeout; deferring reconfigure");
            } else {
                /* Committed. Clear first so a command racing in during teardown
                 * re-sets the flag and is re-applied next tick, not lost. */
                pcan_cmd_clear_reconfig(&s_dev_state);

                /* Deliver whatever the driver has already reported before the
                 * teardown erases it. A completion that arrived while the tasks
                 * were parking must reach can_tx_task as a completion — the
                 * abort issued below would otherwise make it resubmit a frame
                 * that did reach the wire. */
                ctrl_service_alerts();

                uint32_t bitrate = 0;
                bool     silent  = false;
                bool     bus_on  = false;
                pcan_cmd_snapshot(&s_dev_state, &bitrate, &silent, &bus_on);

                pcan_twai_mode_t mode = silent ? TWAI_HAL_MODE_LISTEN_ONLY
                                              : TWAI_HAL_MODE_NORMAL;

                esp_err_t e = bus_on ? pcan_twai_reconfigure(bitrate, mode)
                                     : pcan_twai_stop();

                /* The driver teardown just discarded any queued frame. Tell
                 * can_tx_task to resubmit the frame it still owns; if it owns
                 * none, it drains this as stale before its next submit. Posted
                 * while it is STILL PARKED: released first, it (prio 9) would
                 * preempt this task (prio 8), submit the next frame, and then
                 * take this abort as the verdict on THAT frame — resubmitting
                 * a copy the driver is already arbitrating for. */
                int abort_ev = CAN_TX_EVENT_ABORT;
                (void)xQueueSend(s_tx_done, &abort_ev, 0);

                ctrl_quiesce_end();

                if (e != ESP_OK) {
                    ESP_LOGW(TAG, "reconfigure failed: %s", esp_err_to_name(e));
                } else {
                    ESP_LOGI(TAG, "reconfigured: bus=%s mode=%s bitrate=%u",
                             bus_on ? "on" : "off",
                             silent ? "listen" : "normal", (unsigned)bitrate);
                }
            }
        }

        /* --- bus alert servicing -> TX completion + REC_ERROR -------------- */
        ctrl_service_alerts();

        /* --- LED mirror (cosmetic; unlocked read of a bool is fine) ------- */
        if (s_dev_state.led_on != last_led) {
            last_led = s_dev_state.led_on;
            status_led_set(last_led);
        }

        uint64_t now_us = pcan_time_now_us();
        if (now_us - last_stats_us >= (uint64_t)BRIDGE_STATS_MS * 1000ULL) {
            bridge_log_stats();
            last_stats_us = now_us;
        }

        vTaskDelay(ms_ticks(CTRL_PERIOD_MS));
    }
}

static void start_pcan_mode(void)
{
    ESP_LOGI(TAG, "starting PCAN-USB emulation mode");

    if (!pcan_cmd_init(&s_dev_state)) {
        ESP_LOGE(TAG, "pcan_cmd_init failed");
        return;
    }
    if (!pcan_rb_init(&s_rx_ring, CFG_RB_RX_DEPTH, CFG_RB_USE_PSRAM)) {
        ESP_LOGE(TAG, "rx_ring init failed");
        return;
    }
    if (!pcan_rb_init(&s_tx_ring, CFG_RB_TX_DEPTH, CFG_RB_USE_PSRAM)) {
        ESP_LOGE(TAG, "tx_ring init failed");
        return;
    }

    s_evt_queue = xQueueCreate(8, sizeof(uint8_t));
    /* Deep enough that a completion and the aborts a flapping bus can raise
     * around it never push each other out: a dropped TX event would strand the
     * outstanding frame until the next abort. */
    s_tx_done   = xQueueCreate(8, sizeof(int));
    if (s_evt_queue == NULL || s_tx_done == NULL) {
        ESP_LOGE(TAG, "event primitives alloc failed");
        return;
    }

    status_led_init();

    /* Register state + rings with the USB glue, then bring up TinyUSB. */
    usb_glue_bind(&s_dev_state, &s_rx_ring, &s_tx_ring);
    esp_err_t e = usb_glue_install();
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "usb_glue_install failed: %s", esp_err_to_name(e));
        return;
    }

    /* NOTE: the CAN controller is intentionally NOT started here. The initial
     * listen-only bring-up is performed by ctrl_task (pinned to core 1) so the
     * TWAI ISR is allocated on core 1 — see the CFG_TASK_CTRL_CORE note in
     * config.h. Until ctrl_task installs the driver, twai_rx_task sees
     * ESP_ERR_INVALID_STATE and can_tx_task sees !s_installed; both back off
     * cleanly (a sub-tick window at boot, before the host opens the bus). */

    xTaskCreatePinnedToCore(twai_rx_task, "twai_rx", CFG_TASK_TWAI_RX_STACK,
                            NULL, CFG_TASK_TWAI_RX_PRIO, NULL,
                            CFG_TASK_TWAI_RX_CORE);
    xTaskCreatePinnedToCore(usb_tx_task, "usb_tx", CFG_TASK_USBTX_STACK,
                            NULL, CFG_TASK_USBTX_PRIO, NULL,
                            CFG_TASK_USBTX_CORE);
    xTaskCreatePinnedToCore(can_tx_task, "can_tx", CFG_TASK_CANTX_STACK,
                            NULL, CFG_TASK_CANTX_PRIO, NULL,
                            CFG_TASK_CANTX_CORE);
    xTaskCreatePinnedToCore(ctrl_task, "ctrl", CFG_TASK_CTRL_STACK,
                            NULL, CFG_TASK_CTRL_PRIO, NULL,
                            CFG_TASK_CTRL_CORE);
}

/* =========================================================================
 * Bench mode: standalone ISO-TP/UDS server (owns TWAI exclusively)
 *
 * Compiled out entirely by CFG_ENABLE_BENCH_MODE=0. The link and server below
 * carry ISOTP_MAX_MSG-sized reassembly buffers that live in internal SRAM for
 * the lifetime of the image, and with nothing referencing them the linker's
 * --gc-sections also drops isotp.c/uds.c and their own request scratch — about
 * 16 KB that a PCAN-only build has no use for.
 * ========================================================================= */
#if CFG_ENABLE_BENCH_MODE

static isotp_link_t s_isotp;
static uds_server_t s_uds;
static uint8_t      s_uds_resp_buf[ISOTP_MAX_MSG];

/* ISO-TP platform hooks over twai_hal + pcan_time. */
static int bench_can_tx(uint32_t id, const uint8_t *data, uint8_t dlc, void *user)
{
    (void)user;
    pcan_frame_t f;
    memset(&f, 0, sizeof(f));
    f.id  = id;
    f.dlc = (dlc > PCAN_FRAME_MAX_DLC) ? PCAN_FRAME_MAX_DLC : dlc;
    f.flags = 0;   /* std id, data frame */
    if (data && f.dlc > 0) {
        memcpy(f.data, data, f.dlc);
    }
    esp_err_t e = pcan_twai_transmit(&f, ms_ticks(TWAI_CALL_TIMEOUT_MS));
    return (e == ESP_OK) ? 0 : -1;
}

static uint32_t bench_millis(void *user)
{
    (void)user;
    return (uint32_t)(pcan_time_now_us() / 1000ULL);
}

/* Built-in ECU service table. min_session_mask == 0 -> available in any
 * session, so the default bench ECU answers out of the box. */
static const uds_service_t s_bench_services[] = {
    { UDS_SID_DIAG_SESSION,  0, uds_h_diag_session          },
    { UDS_SID_ECU_RESET,     0, uds_h_ecu_reset             },
    { UDS_SID_READ_DID,      0, uds_h_read_did              },
    { UDS_SID_SECURITY,      0, uds_h_security              },
    { UDS_SID_ROUTINE,       0, uds_h_routine               },
    { UDS_SID_TESTER_PRES,   0, uds_h_tester_present        },
    { UDS_SID_REQ_DOWNLOAD,  0, uds_h_request_download      },
    { UDS_SID_TRANSFER_DATA, 0, uds_h_transfer_data         },
    { UDS_SID_REQ_XFER_EXIT, 0, uds_h_request_transfer_exit },
};
#define BENCH_N_SERVICES \
    (sizeof(s_bench_services) / sizeof(s_bench_services[0]))

static void bench_task(void *arg)
{
    (void)arg;

    esp_err_t e = pcan_twai_start(CFG_BENCH_BITRATE, TWAI_HAL_MODE_NORMAL);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "bench pcan_twai_start failed: %s", esp_err_to_name(e));
        vTaskDelete(NULL);
        return;
    }

    isotp_cfg_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.bs       = 0;      /* unlimited block size                             */
    cfg.stmin    = 0;      /* no separation-time constraint                    */
    cfg.pad_byte = 0xCCu;  /* pad classic frames to 8 bytes (common)           */
    /* timer fields left 0 -> isotp_init applies the 1000 ms defaults.         */

    isotp_hal_t hal = {
        .can_tx = bench_can_tx,
        .millis = bench_millis,
        .user   = NULL,
    };

    isotp_init(&s_isotp, CFG_BENCH_TX_ID, CFG_BENCH_RX_ID, CFG_BENCH_FUNC_ID,
               &cfg, &hal);
    uds_server_init(&s_uds, &s_isotp, s_bench_services, (uint8_t)BENCH_N_SERVICES,
                    s_uds_resp_buf, sizeof(s_uds_resp_buf), NULL);

    ESP_LOGI(TAG, "bench ready: rx=0x%03X tx=0x%03X func=0x%03X @ %u bit/s",
             (unsigned)CFG_BENCH_RX_ID, (unsigned)CFG_BENCH_TX_ID,
             (unsigned)CFG_BENCH_FUNC_ID, (unsigned)CFG_BENCH_BITRATE);

    uint32_t last_ms = bench_millis(NULL);

    for (;;) {
        pcan_frame_t f;
        esp_err_t re = pcan_twai_receive(&f, ms_ticks(BENCH_POLL_MS));
        if (re == ESP_OK && !(f.flags & PCAN_FRAME_FLAG_EXT)) {
            /* The link knows both of its addresses and drops anything else. */
            isotp_on_can_frame(&s_isotp, f.id, f.data, f.dlc);
        }

        isotp_poll(&s_isotp);

        uint32_t now_ms = bench_millis(NULL);
        uds_server_poll(&s_uds, now_ms - last_ms);
        last_ms = now_ms;
    }
}

static void start_bench_mode(void)
{
    ESP_LOGI(TAG, "starting ISO-TP/UDS bench mode");
    xTaskCreatePinnedToCore(bench_task, "bench", CFG_TASK_BENCH_STACK,
                            NULL, CFG_TASK_BENCH_PRIO, NULL,
                            CFG_TASK_BENCH_CORE);
}

#endif /* CFG_ENABLE_BENCH_MODE */

void app_main(void)
{
    /* NVS is needed both for mode persistence and (in PCAN mode) the TinyUSB /
     * PHY init paths. Initialise it first, recovering a full/old partition. */
    esp_err_t nv = nvs_flash_init();
    if (nv == ESP_ERR_NVS_NO_FREE_PAGES || nv == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nv = nvs_flash_init();
    }
    if (nv != ESP_OK) {
        ESP_LOGW(TAG, "nvs_flash_init: %s (continuing)", esp_err_to_name(nv));
    }

    pcan_time_init();

    if (app_mode_select() == CFG_MODE_BENCH) {
#if CFG_ENABLE_BENCH_MODE
        start_bench_mode();
        return;
#else
        /* The GPIO or the NVS key can still ask for a mode this image does not
         * carry; PCAN is the safe fallback because it is what the USB host on
         * the other end of the cable expects. */
        ESP_LOGW(TAG, "BENCH mode selected but not built in (CFG_ENABLE_BENCH_MODE=0)");
#endif
    }
    start_pcan_mode();
}
