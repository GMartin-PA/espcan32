/*
 * SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
/*
 * pcan_cmd.h — PCAN command channel (EP1 OUT parse / EP1 IN reply)
 * ============================================================================
 * Parses the fixed 16-byte host command (pcan_proto.h §2), mutates device
 * state (bitrate, bus on/off, silent, ext-vcc, err mask, LED, channel id), and
 * builds the 16-byte reply for GET commands. SET commands produce no reply.
 *
 * The command handler does NOT touch the TWAI hardware directly; it records the
 * requested configuration in a pcan_dev_state_t and signals the control task,
 * which serializes the actual pcan_twai_reconfigure(). This keeps the USB
 * callback fast and ISR-adjacent code free of blocking reinstall calls.
 *
 * THREADING: pcan_cmd_handle() runs in the USB glue context (TinyUSB task /
 * vendor rx callback). It only writes into pcan_dev_state_t under its mutex and
 * posts to the control task's queue; it must not block.
 * ============================================================================
 */
#ifndef PCAN_CMD_H
#define PCAN_CMD_H

#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "pcan_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Device configuration/state mirror. Owned by pcan_cmd; read by the control
 * task to drive twai_hal. All accesses go through the embedded mutex.
 */
typedef struct {
    SemaphoreHandle_t lock;

    uint32_t state;        /* PCAN_USB_STATE_* bits                            */
    uint32_t serial;       /* replied to CMD_SN GET (PCAN_USB_DEFAULT_SERIAL)  */
    uint8_t  channel_id;   /* CMD_DEVID get/set                                */

    uint8_t  btr0, btr1;   /* last BTR set by CMD_BITRATE                       */
    uint32_t bitrate;      /* nominal bitrate derived from BTR (0 if unknown)  */

    bool     bus_on;       /* CMD_SET_BUS/XCVER                                 */
    bool     silent;       /* CMD_SET_BUS/SILENT (listen-only)                 */
    bool     ext_vcc;      /* CMD_EXT_VCC                                       */
    uint8_t  err_mask;     /* CMD_ERR_FR (PCAN_USB_BERR_MASK)                   */
    bool     led_on;       /* CMD_LED                                          */

    /* Set true whenever a command changed something the control task must
     * apply to the TWAI hardware (bitrate/mode/bus). Cleared by the control
     * task after it reconfigures. */
    volatile bool reconfig_pending;
} pcan_dev_state_t;

/* Initialize device state to power-on defaults and create the mutex. */
bool pcan_cmd_init(pcan_dev_state_t *st);

/*
 * Handle one 16-byte command received on EP1 OUT. Updates *st. If the command
 * is a GET that requires a reply, writes a 16-byte reply into `reply` (which
 * must be >=PCAN_USB_CMD_LEN bytes) and sets *reply_len = 16; otherwise sets
 * *reply_len = 0. Returns true if the command was recognized (still ACKs
 * unknown SETs silently per device behavior). Non-blocking.
 *
 * `cmd`/`cmd_len` is the raw EP1 OUT transfer; cmd_len should be
 * PCAN_USB_CMD_LEN (shorter transfers are rejected -> returns false).
 */
bool pcan_cmd_handle(pcan_dev_state_t *st,
                     const uint8_t *cmd, uint16_t cmd_len,
                     uint8_t *reply, uint16_t *reply_len);

/* Snapshot the fields the control task needs, under the lock. */
void pcan_cmd_snapshot(pcan_dev_state_t *st,
                       uint32_t *bitrate, bool *silent, bool *bus_on);

/* Clear reconfig_pending after the control task has applied the config. */
void pcan_cmd_clear_reconfig(pcan_dev_state_t *st);

#ifdef __cplusplus
}
#endif
#endif /* PCAN_CMD_H */
