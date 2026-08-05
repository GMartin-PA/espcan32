/*
 * SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
/*
 * twai_hal.h — TWAI (classic CAN 2.0) backend wrapper
 * ============================================================================
 * Thin wrapper over the ESP-IDF LEGACY TWAI driver (driver/twai.h). Presents
 * the CAN peripheral to the rest of the firmware in terms of the canonical
 * pcan_frame_t and the PCAN bitrate/mode model.
 *
 * The ESP32-S3 has ONE TWAI controller and is CLASSIC CAN ONLY (no FD; DLC<=8).
 * The legacy driver has no live re-timing call, so bitrate/mode changes do a
 * full stop -> uninstall -> reinstall -> start; twai_hal serializes that and
 * MUST only be driven from the single control task (see config.h CFG_TASK_CTRL).
 *
 * THREADING:
 *   - pcan_twai_receive() blocks a dedicated RX task on the driver RX queue.
 *   - pcan_twai_transmit() is called from the CAN-TX task; it queues into the
 *     driver TX queue (may block up to a timeout).
 *   - pcan_twai_reconfigure() / _start / _stop MUST run only on the control
 *     task, with the RX/TX tasks paused around the call (they will get
 *     ESP_ERR_INVALID_STATE while stopped, which they treat as "retry").
 * ============================================================================
 */
#ifndef PCAN_TWAI_HAL_H
#define PCAN_TWAI_HAL_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "pcan_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Mirror of twai_mode_t restricted to what PCAN needs. */
typedef enum {
    TWAI_HAL_MODE_NORMAL = 0,   /* normal TX/RX/ACK                            */
    TWAI_HAL_MODE_LISTEN_ONLY,  /* PCAN silent mode: RX only, no ACK/TX        */
} pcan_twai_mode_t;

/* Coarse bus state reported to the PCAN error-record encoder. */
typedef enum {
    TWAI_HAL_STATE_STOPPED = 0,
    TWAI_HAL_STATE_RUNNING,
    TWAI_HAL_STATE_BUS_OFF,
    TWAI_HAL_STATE_RECOVERING,
} pcan_twai_state_t;

/* Snapshot of controller status + error counters (REC_ERROR + bridge stats). */
typedef struct {
    pcan_twai_state_t state;
    uint32_t tx_error_counter;   /* TEC                                        */
    uint32_t rx_error_counter;   /* REC                                        */
    uint32_t tx_failed_count;
    uint32_t rx_missed_count;    /* driver RX queue overflow                   */
    uint32_t rx_overrun_count;   /* HW RX FIFO overrun                         */
    uint32_t arb_lost_count;
    uint32_t bus_error_count;
} pcan_twai_status_t;

/*
 * Map an incoming (BTR0,BTR1) pair to a nominal bitrate using
 * PCAN_BTR_TABLE_INIT. Returns the bitrate in bit/s, or 0 if unrecognized
 * (caller should keep the previous bitrate and still ACK the command — the
 * emulated device stores+ACKs BTR without needing to honor exotic values).
 */
uint32_t pcan_twai_btr_to_bitrate(uint8_t btr0, uint8_t btr1);

/*
 * Install and start the TWAI driver at `bitrate` (must be one of the table
 * bitrates that has a TWAI_TIMING_CONFIG_* macro; unsupported rates fall back
 * to the nearest supported one, logged) in the given mode. Idempotent-safe:
 * if already installed it is torn down first. Must run on the control task.
 */
esp_err_t pcan_twai_start(uint32_t bitrate, pcan_twai_mode_t mode);

/* Stop and uninstall the driver. Safe if not started. Control task only. */
esp_err_t pcan_twai_stop(void);

/*
 * Change bitrate and/or mode at runtime: stop -> uninstall -> reinstall ->
 * start. Control task only; the caller must have quiesced RX/TX tasks. A
 * bitrate of 0 keeps the current bitrate; mode is always applied.
 */
esp_err_t pcan_twai_reconfigure(uint32_t bitrate, pcan_twai_mode_t mode);

/*
 * Transmit one frame. Copies from `frame`; sets extd/rtr/ss/self from
 * frame->flags. Blocks up to ticks_to_wait for TX queue space. Returns
 * ESP_OK when queued (not when the frame hits the wire). Called on CAN-TX task.
 */
esp_err_t pcan_twai_transmit(const pcan_frame_t *frame, TickType_t ticks_to_wait);

/*
 * Block up to ticks_to_wait for a received frame; on success fills `out`
 * (including out->ts16 stamped via pcan_time at the moment of return) and
 * returns ESP_OK. Returns ESP_ERR_TIMEOUT when no frame arrived. Called on the
 * dedicated RX task. The legacy driver reports nothing that would distinguish a
 * self-reception echo from a frame another node sent, so `out->flags` only ever
 * carries PCAN_FRAME_FLAG_EXT / _RTR.
 */
esp_err_t pcan_twai_receive(pcan_frame_t *out, TickType_t ticks_to_wait);

/* Fill `st` with the current controller status. Any task. */
esp_err_t pcan_twai_get_status(pcan_twai_status_t *st);

/* CAN TX completion events reported by pcan_twai_service_alerts(). With the
 * single-outstanding-frame discipline in can_tx_task these map 1:1 onto the
 * currently in-flight frame. */
#define PCAN_TX_EVENT_NONE  0
#define PCAN_TX_EVENT_OK    1   /* TWAI_ALERT_TX_SUCCESS: frame hit the wire    */
#define PCAN_TX_EVENT_FAIL  2   /* TWAI_ALERT_TX_FAILED: never reached the bus  */

/*
 * Drain and translate pending TWAI alerts (bus-off, err-passive, recovery,
 * queue-full, ...) into a bitmask of PCAN_USB_ERROR_* flags suitable for a
 * REC_ERROR record, and drive bus-off recovery (twai_initiate_recovery +
 * re-start on TWAI_ALERT_BUS_RECOVERED). Non-blocking. Control task only.
 * Writes the resulting PCAN error bitmask to *pcan_err_mask (0 if none) and
 * any TX completion event to *tx_event (PCAN_TX_EVENT_*; FAIL wins over OK
 * when both accumulated). Returns true if a host-visible state change occurred.
 */
bool pcan_twai_service_alerts(uint8_t *pcan_err_mask, int *tx_event);

#ifdef __cplusplus
}
#endif
#endif /* PCAN_TWAI_HAL_H */
