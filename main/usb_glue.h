/*
 * SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
/*
 * usb_glue.h — TinyUSB init + custom PCAN class driver (one 4-EP interface)
 * ============================================================================
 * Owns TinyUSB installation (tinyusb_driver_install with our custom
 * descriptors) and a custom usbd_class_driver_t that owns the single vendor
 * interface carrying all four bulk endpoints. Routing is by ABSOLUTE ENDPOINT
 * ADDRESS (not interface index):
 *   EP 0x01 OUT -> pcan_cmd_handle; reply -> EP 0x81 IN.
 *   EP 0x02 OUT -> pcan_msg_decode_tx -> TX ring.
 *   EP 0x82 IN  <- RX-ring batches (usb_glue_send_msg_batch).
 *
 * DATA FLOW / OWNERSHIP:
 *   - Inbound: the class driver's xfer_cb() dispatches the completed OUT buffer
 *     by endpoint address, then re-arms that OUT endpoint. It does NOT block and
 *     does NOT keep pointers into the transfer buffers past the call.
 *   - Outbound: usb_glue_send_msg_batch() copies a completed batch into the DMA
 *     EP 0x82 buffer and submits it; usb_glue_send_cmd_reply() does the same for
 *     the 16-byte GET reply on EP 0x81. Both copy the caller's buffer (the
 *     caller may reuse it immediately) and are non-blocking.
 *
 * THREADING:
 *   - The class-driver callbacks (open/reset/xfer_cb) run in the TinyUSB task.
 *   - usb_glue_send_* may be called from another task (USB-TX task) or from
 *     xfer_cb; they use usbd_edpt_claim() to serialise against the IN-completion
 *     and return the bytes queued (0 if the endpoint is busy; caller retries).
 * ============================================================================
 */
#ifndef PCAN_USB_GLUE_H
#define PCAN_USB_GLUE_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "pcan_cmd.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Wire the glue to the shared device state and the frame rings before install.
 * `st` is the command/state mirror; `rx_ring`/`tx_ring` are opaque here (typed
 * void* to avoid a hard include cycle — they are pcan_ringbuf_t*). Only the
 * host->CAN direction is stored: `rx_ring` is drained by usb_tx_task, which
 * passes each batch to usb_glue_send_msg_batch(). Call once before
 * usb_glue_install().
 */
void usb_glue_bind(pcan_dev_state_t *st, void *rx_ring, void *tx_ring);

/*
 * Install TinyUSB with the PCAN descriptor set and start its task. Returns
 * ESP_OK on success. Must be called after usb_glue_bind(). Internally sets
 * tusb_cfg.descriptor.* to the pcan_desc_* providers (esp_tinyusb v2.x API;
 * see the v1.x flat-field note in usb_glue.c).
 */
esp_err_t usb_glue_install(void);

/* True once the host has enumerated and mounted the vendor interface. */
bool usb_glue_mounted(void);

/* Retry a deferred EP2-OUT batch and re-arm either OUT endpoint once every
 * decoded host frame has entered the CAN TX ring, or after a rejected submit.
 * Non-blocking; call from usb_tx_task. */
void usb_glue_service(void);

typedef struct {
    uint32_t tx_backpressure_retries;   /* retained host frames retried later    */
    uint32_t tx_pending_reset_drops;    /* pending host frames lost on USB reset */
    uint32_t cmd_out_rearm_failures;    /* EP1-OUT arm/re-arm calls that failed  */
    uint32_t msg_out_rearm_failures;    /* EP2-OUT re-arm calls that failed      */
    uint32_t msg_in_failed_retries;     /* failed EP2-IN submit/completions      */
    uint32_t msg_in_reset_drops;        /* in-flight EP2-IN batches lost on reset */
} usb_glue_stats_t;

/* Read and clear bridge-glue diagnostic counters. */
void usb_glue_take_stats(usb_glue_stats_t *out);

/*
 * Send a completed EP2-IN message batch (`buf`/`len`, len<=64) to the host.
 * Completion-based ownership: returns 0 while the previously retained batch is
 * queued or in flight, and returns `len` only after that batch's IN transfer
 * completed SUCCESSFULLY on the wire (the batch is then released and the caller
 * may reset/reuse it). The caller must retry with the identical buf/len; failed
 * transfers are resubmitted internally, USB resets retain the batch across
 * re-enumeration, and only an explicit driver deinit completes it as dropped.
 */
uint32_t usb_glue_send_msg_batch(const uint8_t *buf, uint16_t len);

/*
 * Send a 16-byte command GET reply to the host on EP 0x81. Returns bytes
 * queued. Normally called synchronously from the command dispatch path.
 */
uint32_t usb_glue_send_cmd_reply(const uint8_t *reply, uint16_t len);

#ifdef __cplusplus
}
#endif
#endif /* PCAN_USB_GLUE_H */
