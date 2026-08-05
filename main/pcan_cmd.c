/*
 * SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
/*
 * pcan_cmd.c — PCAN command channel parse/build
 * ============================================================================
 * Implements the EP1 command handler and the device-state mirror described in
 * pcan_cmd.h and pcan_proto.h §2. Parses the fixed 16-byte host command,
 * mutates pcan_dev_state_t under its mutex, and builds the 16-byte GET reply.
 *
 * THREADING: pcan_cmd_handle() runs in the USB glue (TinyUSB) context and must
 * not block on I/O. The only lock it contends for is st->lock, which is held
 * for a handful of field writes by both this handler and the control task, so
 * a mutex take is effectively non-blocking. We still guard every take against
 * a NULL/uninitialised lock so a malformed early transfer cannot crash.
 * ============================================================================
 */
#include "pcan_cmd.h"
#include "twai_hal.h"
#include <string.h>

/* ---- internal locking helpers -------------------------------------------- */

static inline bool cmd_lock(pcan_dev_state_t *st)
{
    if (!st || !st->lock) {
        return false;
    }
    /* The mutex is only ever held for a few field assignments (here or on the
     * control task), so this does not meaningfully block the USB context. */
    return xSemaphoreTake(st->lock, portMAX_DELAY) == pdTRUE;
}

static inline void cmd_unlock(pcan_dev_state_t *st)
{
    (void)xSemaphoreGive(st->lock);
}

/* ---- init ---------------------------------------------------------------- */

bool pcan_cmd_init(pcan_dev_state_t *st)
{
    if (!st) {
        return false;
    }

    memset(st, 0, sizeof(*st));

    st->lock = xSemaphoreCreateMutex();
    if (!st->lock) {
        return false;
    }

    st->state            = PCAN_USB_STATE_CONNECTED;
    st->serial           = PCAN_USB_DEFAULT_SERIAL;
    st->channel_id       = PCAN_USB_DEFAULT_DEVID;

    /* Default bit timing: 500 kbit/s. Seed the BTR bytes from the table so a
     * snapshot taken before the host ever sends CMD_BITRATE is self-consistent. */
    st->bitrate          = PCAN_USB_DEFAULT_BITRATE;
    st->btr0             = 0x00u;   /* 500k BTR0 (see PCAN_BTR_TABLE_INIT)      */
    st->btr1             = 0x1Cu;   /* 500k BTR1                               */

    st->bus_on           = false;
    st->silent           = false;
    st->ext_vcc          = false;
    st->err_mask         = PCAN_USB_BERR_MASK;
    st->led_on           = false;
    st->reconfig_pending = false;

    return true;
}

/* ---- command handling ---------------------------------------------------- */

/* Build a 16-byte GET reply: echo func/num, zero the arg region, payload is
 * written by the caller after this returns. */
static void reply_begin(uint8_t *reply, uint8_t func, uint8_t num)
{
    memset(reply, 0, PCAN_USB_CMD_LEN);
    reply[PCAN_USB_CMD_FUNC] = func;
    reply[PCAN_USB_CMD_NUM]  = num;
}

bool pcan_cmd_handle(pcan_dev_state_t *st,
                     const uint8_t *cmd, uint16_t cmd_len,
                     uint8_t *reply, uint16_t *reply_len)
{
    if (reply_len) {
        *reply_len = 0;
    }

    /* Short/malformed transfers are rejected. A full command is always 16 B. */
    if (!st || !cmd || cmd_len < PCAN_USB_CMD_LEN) {
        return false;
    }

    const uint8_t  func = cmd[PCAN_USB_CMD_FUNC];
    const uint8_t  num  = cmd[PCAN_USB_CMD_NUM];
    const uint8_t *args = &cmd[PCAN_USB_CMD_ARGS];   /* 14 bytes, cmd[2..15]   */

    if (!cmd_lock(st)) {
        return false;
    }

    bool recognized = true;

    switch (func) {

    case PCAN_USB_CMD_SN:  /* 6 */
        if (num == PCAN_USB_GET) {
            /* Reply: LE u32 serial at reply args[0..3] (reply bytes [2..5]). */
            if (reply && reply_len) {
                reply_begin(reply, func, num);
                pcan_put_le32(&reply[PCAN_USB_CMD_ARGS], st->serial);
                *reply_len = PCAN_USB_CMD_LEN;
            }
        }
        /* CMD_SN SET is not part of the emulated surface; ACK silently. */
        break;

    case PCAN_USB_CMD_DEVID:  /* 4 */
        if (num == PCAN_USB_GET) {
            /* Reply: channel id at reply args[0] (reply byte [2]). */
            if (reply && reply_len) {
                reply_begin(reply, func, num);
                reply[PCAN_USB_CMD_ARGS] = st->channel_id;
                *reply_len = PCAN_USB_CMD_LEN;
            }
        } else if (num == PCAN_USB_SET) {
            st->channel_id = args[0];
        }
        break;

    case PCAN_USB_CMD_BITRATE:  /* 1 */
        /* args[0]=BTR1, args[1]=BTR0 (BTR1 FIRST — proto §2). */
        st->btr1 = args[0];
        st->btr0 = args[1];
        {
            uint32_t br = pcan_twai_btr_to_bitrate(st->btr0, st->btr1);
            if (br != 0u) {
                st->bitrate = br;   /* keep previous bitrate on exotic values  */
            }
        }
        st->reconfig_pending = true;
        break;

    case PCAN_USB_CMD_SET_BUS:  /* 3 */
        if (num == PCAN_USB_BUS_XCVER) {          /* transceiver on/off        */
            st->bus_on = (args[0] != 0u);
            if (st->bus_on) {
                st->state |= PCAN_USB_STATE_STARTED;
            } else {
                st->state &= ~PCAN_USB_STATE_STARTED;
            }
            st->reconfig_pending = true;
        } else if (num == PCAN_USB_BUS_SILENT_MODE) {  /* silent / listen-only */
            st->silent = (args[0] != 0u);
            st->reconfig_pending = true;
        } else {
            recognized = false;
        }
        break;

    case PCAN_USB_CMD_EXT_VCC:  /* 10 */
        if (num == PCAN_USB_SET) {
            st->ext_vcc = (args[0] != 0u);
        }
        break;

    case PCAN_USB_CMD_ERR_FR:  /* 11 */
        if (num == PCAN_USB_SET) {
            st->err_mask = (uint8_t)(args[0] & PCAN_USB_BERR_MASK);
        }
        break;

    case PCAN_USB_CMD_LED:  /* 12 */
        if (num == PCAN_USB_SET) {
            st->led_on = (args[0] != 0u);
        }
        break;

    case PCAN_USB_CMD_REGISTER:  /* 9 — SJA1000 register access on close/bus-off */
        /* The emulator has no SJA1000 register file; the host writes
         * SJA1000_MODE_INIT here to force init mode on close. Accept and ACK
         * without touching the TWAI peripheral (twai_hal owns bit timing). */
        break;

    default:
        recognized = false;
        break;
    }

    cmd_unlock(st);
    return recognized;
}

/* ---- accessors ----------------------------------------------------------- */

void pcan_cmd_snapshot(pcan_dev_state_t *st,
                       uint32_t *bitrate, bool *silent, bool *bus_on)
{
    if (!st) {
        if (bitrate) *bitrate = 0;
        if (silent)  *silent  = false;
        if (bus_on)  *bus_on  = false;
        return;
    }

    if (!cmd_lock(st)) {
        if (bitrate) *bitrate = 0;
        if (silent)  *silent  = false;
        if (bus_on)  *bus_on  = false;
        return;
    }

    if (bitrate) *bitrate = st->bitrate;
    if (silent)  *silent  = st->silent;
    if (bus_on)  *bus_on  = st->bus_on;

    cmd_unlock(st);
}

void pcan_cmd_clear_reconfig(pcan_dev_state_t *st)
{
    if (!cmd_lock(st)) {
        return;
    }
    st->reconfig_pending = false;
    cmd_unlock(st);
}
