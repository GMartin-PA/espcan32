/*
 * SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
/*
 * pcan_msg.c — CAN <-> USB record codec (EP2 IN / EP2 OUT)
 * ============================================================================
 * Serializes canonical pcan_frame_t / status events into EP2-IN wire batches
 * and parses host EP2-OUT TX batches back into pcan_frame_t. Every wire layout
 * used here is defined in pcan_proto.h §3-§5; this module is the ONLY place
 * that touches those bytes.
 *
 * Overflow policy: the encoders build into a fixed 64-byte buffer and refuse
 * (return false) any record that would push b->len past PCAN_USB_TX_BUFFER_SIZE.
 * The caller then flushes the batch over EP2 IN, resets, and retries.
 * ============================================================================
 */
#include "pcan_msg.h"
#include <string.h>

/* Number of timestamp bytes the NEXT pcan_ts_append() call will emit into this
 * batch: 2 for the first timestamped record, 1 for every subsequent one. Kept
 * in sync with pcan_ts_append()'s own first-vs-subsequent rule (pcan_time.c). */
static inline uint8_t pcan_msg_ts_width(const pcan_batch_t *b)
{
    return b->ts.have_first ? 1u : 2u;
}

void pcan_msg_batch_reset(pcan_batch_t *b)
{
    if (b == NULL) {
        return;
    }
    b->buf[0]  = PCAN_USB_MSG_TX_CAN;      /* [0] batch type                    */
    b->buf[1]  = 0;                        /* [1] rec_cnt                       */
    b->len     = PCAN_USB_MSG_HEADER_LEN;  /* header only, no records yet       */
    b->rec_cnt = 0;
    pcan_ts_batch_reset(&b->ts);
}

bool pcan_msg_batch_add_frame(pcan_batch_t *b, const pcan_frame_t *frame)
{
    if (b == NULL || frame == NULL) {
        return false;
    }

    const bool ext = (frame->flags & PCAN_FRAME_FLAG_EXT) != 0;
    const bool rtr = (frame->flags & PCAN_FRAME_FLAG_RTR) != 0;
    const bool srr = (frame->flags & PCAN_FRAME_FLAG_SRR) != 0;

    /* Emitted DLC is saturated at 8, mirroring the decoder's treatment of a raw
     * 9..15 nibble. pcan_frame_t has no room for the raw DLC of a host TX frame,
     * so an echoed self-reception frame reports 8 rather than the 9..15 the host
     * originally sent; the payload is identical either way. */
    uint8_t dlc = frame->dlc;
    if (dlc > PCAN_FRAME_MAX_DLC) {
        dlc = PCAN_FRAME_MAX_DLC;
    }

    const uint8_t id_bytes   = ext ? 4u : 2u;
    const uint8_t ts_bytes   = pcan_msg_ts_width(b);
    const uint8_t data_bytes = rtr ? 0u : dlc;
    /* The genuine driver's RX decoder consumes the SRR trailer ONLY in the
     * non-RTR branch, so a trailer on an RTR echo would desync the rest of the
     * batch. Emit it only for non-RTR self-reception frames. */
    const uint8_t srr_bytes  = (srr && !rtr) ? 1u : 0u;

    /* SL(1) + id + ts + data + optional SRR trailer. */
    const uint32_t size = 1u + id_bytes + ts_bytes + data_bytes + srr_bytes;

    if ((uint32_t)b->len + size > PCAN_USB_TX_BUFFER_SIZE) {
        return false;  /* strict overflow refusal: caller flushes + retries    */
    }

    /* Low-order TX flag bits packed into the shifted id word. */
    uint8_t txflags = 0;
    if (srr) {
        txflags |= PCAN_USB_TX_SRR;
    }
    if (frame->flags & PCAN_FRAME_FLAG_SS) {
        txflags |= PCAN_USB_TX_AT;
    }

    uint8_t *p = b->buf + b->len;

    /* Status/Len prefix byte. */
    uint8_t sl = (uint8_t)(dlc & PCAN_USB_STATUSLEN_DLC);
    sl |= PCAN_USB_STATUSLEN_TIMESTAMP;
    if (ext) {
        sl |= PCAN_USB_STATUSLEN_EXT_ID;
    }
    if (rtr) {
        sl |= PCAN_USB_STATUSLEN_RTR;
    }
    *p++ = sl;

    /* Shifted, little-endian CAN id word with flags in the low bits. */
    if (ext) {
        uint32_t id_word = ((frame->id & 0x1FFFFFFFu) << PCAN_USB_EXT_ID_SHIFT) |
                           txflags;
        pcan_put_le32(p, id_word);
        p += 4;
    } else {
        uint16_t id_word = (uint16_t)(((frame->id & 0x7FFu) << PCAN_USB_STD_ID_SHIFT) |
                                      txflags);
        pcan_put_le16(p, id_word);
        p += 2;
    }

    /* Timestamp (2 bytes on first record of the batch, 1 thereafter). */
    p = pcan_ts_append(&b->ts, p, frame->ts16);

    /* Data bytes (none for RTR). */
    if (data_bytes > 0) {
        memcpy(p, frame->data, data_bytes);
        p += data_bytes;
    }

    /* Self-reception trailer carries the echo-correlation (writer) id. Only for
     * non-RTR frames — the driver consumes it exclusively in its non-RTR path. */
    if (srr && !rtr) {
        *p++ = frame->writer_id;
    }

    b->rec_cnt++;
    b->buf[1] = b->rec_cnt;
    b->len    = (uint16_t)(p - b->buf);
    return true;
}

bool pcan_msg_batch_add_status(pcan_batch_t *b, uint8_t func, uint8_t num,
                               const uint8_t *payload, uint8_t payload_len,
                               bool with_ts, uint16_t ts16)
{
    if (b == NULL) {
        return false;
    }
    /* rec_len lives in the SL low nibble, so it must fit in 4 bits. */
    if (payload_len > PCAN_USB_STATUSLEN_DLC) {
        return false;
    }
    if (payload_len > 0 && payload == NULL) {
        return false;
    }

    const uint8_t ts_bytes = with_ts ? pcan_msg_ts_width(b) : 0u;

    /* SL(1) + func/num(2) + ts + payload. */
    const uint32_t size = 1u + 2u + ts_bytes + payload_len;

    if ((uint32_t)b->len + size > PCAN_USB_TX_BUFFER_SIZE) {
        return false;
    }

    uint8_t *p = b->buf + b->len;

    uint8_t sl = (uint8_t)(payload_len & PCAN_USB_STATUSLEN_DLC);
    sl |= PCAN_USB_STATUSLEN_INTERNAL;
    if (with_ts) {
        sl |= PCAN_USB_STATUSLEN_TIMESTAMP;
    }
    *p++ = sl;

    *p++ = func;
    *p++ = num;

    if (with_ts) {
        p = pcan_ts_append(&b->ts, p, ts16);
    }

    if (payload_len > 0) {
        memcpy(p, payload, payload_len);
        p += payload_len;
    }

    b->rec_cnt++;
    b->buf[1] = b->rec_cnt;
    b->len    = (uint16_t)(p - b->buf);
    return true;
}

bool pcan_msg_batch_add_calib_ts(pcan_batch_t *b, uint16_t ts16)
{
    /* REC_TS carries its tick in the record PAYLOAD with the TIMESTAMP bit
     * CLEAR: the host reads the LE16 at the payload position without advancing,
     * then closes the record with ptr += rec_len (proto §5). Setting TIMESTAMP
     * as well would make the host consume the timestamp bytes first and read
     * the tick past the record. Because the bit stays clear, this record also
     * does not take the batch's first-timestamped-record slot, matching the
     * host's rec_ts_idx accounting. */
    uint8_t payload[2];
    pcan_put_le16(payload, ts16);
    return pcan_msg_batch_add_status(b, PCAN_USB_REC_TS, 0, payload,
                                     sizeof(payload), false, 0);
}

bool pcan_msg_batch_add_error(pcan_batch_t *b, uint8_t err_mask, uint16_t ts16)
{
    return pcan_msg_batch_add_status(b, PCAN_USB_REC_ERROR, err_mask, NULL, 0,
                                     true, ts16);
}

int pcan_msg_decode_tx(const uint8_t *buf, uint16_t len,
                       pcan_frame_t *out, int max_out)
{
    if (buf == NULL || out == NULL || max_out <= 0) {
        return -1;
    }
    if (len <= PCAN_USB_MSG_HEADER_LEN) {
        return -1;  /* header-only / empty batch is malformed for a TX decode  */
    }

    const uint8_t rec_cnt = buf[1];

    /*
     * The host always submits a full 64-byte OUT URB whose last byte holds a
     * rolling 8-bit tx counter (writer id) used to correlate loopback echoes
     * (pcan_proto.h §3). It is only present when the transfer is the full
     * buffer size; short transfers carry no tail byte.
     */
    uint8_t writer_id = 0;
    if (len >= PCAN_USB_TX_BUFFER_SIZE) {
        writer_id = buf[PCAN_USB_TX_BUFFER_SIZE - 1u];
    }

    uint16_t pos = PCAN_USB_MSG_HEADER_LEN;
    int count = 0;

    for (uint8_t i = 0; i < rec_cnt; i++) {
        if (count >= max_out) {
            break;  /* out of caller-provided space; return what we decoded    */
        }
        if (pos >= len) {
            return -1;  /* truncated: SL byte missing                          */
        }

        const uint8_t sl = buf[pos++];

        if (sl & PCAN_USB_STATUSLEN_INTERNAL) {
            /*
             * Status/error record. Host TX batches do not normally contain
             * these, but skip cleanly if present: func(1) + num(1) + payload.
             * EP2-OUT records carry no timestamp (proto §3.1), so no ts skip.
             */
            const uint8_t rec_len = (uint8_t)(sl & PCAN_USB_STATUSLEN_DLC);
            const uint32_t skip = 2u + rec_len;
            if ((uint32_t)pos + skip > len) {
                return -1;
            }
            pos = (uint16_t)(pos + skip);
            continue;
        }

        const bool ext = (sl & PCAN_USB_STATUSLEN_EXT_ID) != 0;
        const bool rtr = (sl & PCAN_USB_STATUSLEN_RTR) != 0;
        const uint8_t id_bytes = ext ? 4u : 2u;

        if ((uint32_t)pos + id_bytes > len) {
            return -1;  /* truncated id word                                   */
        }

        uint32_t id;
        uint8_t  txflags;
        if (ext) {
            const uint32_t id_word = pcan_get_le32(&buf[pos]);
            id      = id_word >> PCAN_USB_EXT_ID_SHIFT;
            txflags = (uint8_t)(id_word & ((1u << PCAN_USB_EXT_ID_SHIFT) - 1u));
        } else {
            const uint16_t id_word = pcan_get_le16(&buf[pos]);
            id      = (uint32_t)(id_word >> PCAN_USB_STD_ID_SHIFT);
            txflags = (uint8_t)(id_word & ((1u << PCAN_USB_STD_ID_SHIFT) - 1u));
        }
        pos = (uint16_t)(pos + id_bytes);

        const bool srr = (txflags & PCAN_USB_TX_SRR) != 0;
        const bool ss  = (txflags & PCAN_USB_TX_AT)  != 0;

        /* A raw DLC of 9..15 is legal on classic CAN and means 8 data bytes;
         * the host emits it whenever CAN_CTRLMODE_CC_LEN8_DLC is enabled, since
         * it writes can_get_cc_dlc() straight into the SL nibble. Saturate to 8
         * rather than rejecting — a reject here would abort the whole batch. */
        uint8_t dlc = (uint8_t)(sl & PCAN_USB_STATUSLEN_DLC);
        if (dlc > PCAN_FRAME_MAX_DLC) {
            dlc = PCAN_FRAME_MAX_DLC;
        }

        const uint8_t data_bytes = rtr ? 0u : dlc;
        if ((uint32_t)pos + data_bytes > len) {
            return -1;  /* truncated data                                      */
        }

        pcan_frame_t *f = &out[count];
        memset(f, 0, sizeof(*f));
        f->id  = id;
        f->dlc = dlc;
        f->ts16 = 0;  /* host TX records carry no timestamp                    */
        f->flags = 0;
        if (ext) f->flags |= PCAN_FRAME_FLAG_EXT;
        if (rtr) f->flags |= PCAN_FRAME_FLAG_RTR;
        if (ss)  f->flags |= PCAN_FRAME_FLAG_SS;
        if (srr) f->flags |= PCAN_FRAME_FLAG_SRR;

        if (data_bytes > 0) {
            memcpy(f->data, &buf[pos], data_bytes);
            pos = (uint16_t)(pos + data_bytes);
        }

        /* SRR frames append a 1-byte trailer (echo tag); consume it. The
         * canonical writer id is taken from the batch tail byte per §3. */
        if (srr) {
            if ((uint32_t)pos + 1u > len) {
                return -1;
            }
            pos++;
        }

        f->writer_id = writer_id;
        count++;
    }

    return count;
}
