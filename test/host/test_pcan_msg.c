/*
 * SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
/*
 * test_pcan_msg.c — EP2 wire codec + timestamp engine
 * ============================================================================
 * The interesting assertions here are not "our encoder round-trips with our
 * decoder" — that would agree with itself no matter how wrong it was. They are
 * "the mainline Linux host accepts what we emit and recovers the exact values
 * we put in", so this file carries a model of the host decoder transcribed from
 * drivers/net/can/usb/peak_usb/pcan_usb.c (pcan_usb_decode_msg, _decode_status,
 * _decode_data, _decode_ts, _update_ts) and a model of its encoder
 * (pcan_usb_encode_msg) to generate EP2-OUT batches.
 *
 * The models are deliberately literal transcriptions, including the host's own
 * quirks (advancing by the RAW dlc nibble while copying only 8 bytes; REC_TS
 * reading its tick without advancing). Fixing a quirk here would hide a real
 * incompatibility.
 * ============================================================================
 */
#include "test.h"

#include "esp_stubs.h"
#include "pcan_msg.h"
#include "pcan_proto.h"
#include "pcan_time.h"

#include <stdlib.h>

#if defined(__unix__) || defined(__APPLE__)
#include <sys/mman.h>
#include <unistd.h>
#define HAVE_GUARD_PAGES 1
#ifndef MAP_ANONYMOUS
#define MAP_ANONYMOUS MAP_ANON
#endif
#endif

/* ==========================================================================
 * Model of the Linux host RX decoder (EP2 IN).
 * ========================================================================== */

#define HOST_MAX_REC 32

typedef struct {
    uint32_t id;
    uint8_t  ext, rtr, len;
    uint8_t  data[8];
    uint16_t ts16;   /* tick the host reconstructed for this frame            */
} host_frame_t;

typedef struct {
    /* observations */
    host_frame_t frame[HOST_MAX_REC];
    int          n_frames;
    uint16_t     calib[HOST_MAX_REC];  /* ticks recovered from REC_TS records */
    int          n_calib;
    uint8_t      err_mask[HOST_MAX_REC];
    int          n_err;
    size_t       consumed;             /* bytes the host walked over          */

    /* struct pcan_usb_msg_context */
    uint16_t       ts16;
    uint8_t        prev_ts8;
    const uint8_t *ptr, *end;
    uint8_t        rec_cnt, rec_idx, rec_ts_idx;
} host_mc_t;

static int host_decode_ts(host_mc_t *mc, int first_packet)
{
    if (first_packet) {
        if (mc->ptr + 2 > mc->end) {
            return -1;
        }
        mc->ts16     = pcan_get_le16(mc->ptr);
        mc->prev_ts8 = (uint8_t)(mc->ts16 & 0x00FFu);
        mc->ptr     += 2;
    } else {
        if (mc->ptr + 1 > mc->end) {
            return -1;
        }
        uint8_t ts8 = *mc->ptr++;
        if (ts8 < mc->prev_ts8) {
            mc->ts16 = (uint16_t)(mc->ts16 + 0x100u);
        }
        mc->ts16     = (uint16_t)((mc->ts16 & 0xFF00u) | ts8);
        mc->prev_ts8 = ts8;
    }
    return 0;
}

static int host_decode_status(host_mc_t *mc, uint8_t sl)
{
    uint8_t rec_len = (uint8_t)(sl & PCAN_USB_STATUSLEN_DLC);

    if (mc->ptr + 2 > mc->end) {
        return -1;
    }
    const uint8_t f = mc->ptr[0];
    const uint8_t n = mc->ptr[1];
    mc->ptr += 2;

    if (sl & PCAN_USB_STATUSLEN_TIMESTAMP) {
        if (host_decode_ts(mc, !mc->rec_ts_idx) != 0) {
            return -1;
        }
        mc->rec_ts_idx++;
    }

    switch (f) {
    case PCAN_USB_REC_ERROR:
        if (mc->n_err < HOST_MAX_REC) {
            mc->err_mask[mc->n_err++] = n;
        }
        break;
    case PCAN_USB_REC_ANALOG:
        rec_len = 2;
        break;
    case PCAN_USB_REC_BUSLOAD:
        rec_len = 1;
        break;
    case PCAN_USB_REC_TS:
        /* pcan_usb_update_ts(): reads the tick in place, does NOT advance. */
        if (mc->ptr + 2 > mc->end) {
            return -1;
        }
        mc->ts16 = pcan_get_le16(mc->ptr);
        if (mc->n_calib < HOST_MAX_REC) {
            mc->calib[mc->n_calib++] = mc->ts16;
        }
        break;
    default:
        break;
    }

    if (mc->ptr + rec_len > mc->end) {
        return -1;
    }
    mc->ptr += rec_len;
    return 0;
}

static int host_decode_data(host_mc_t *mc, uint8_t sl)
{
    const uint8_t rec_len = (uint8_t)(sl & PCAN_USB_STATUSLEN_DLC);
    host_frame_t  f;
    uint32_t      can_id_flags;

    memset(&f, 0, sizeof(f));

    if (sl & PCAN_USB_STATUSLEN_EXT_ID) {
        if (mc->ptr + 4 > mc->end) {
            return -1;
        }
        can_id_flags = pcan_get_le32(mc->ptr);
        f.id         = can_id_flags >> 3;
        f.ext        = 1;
        mc->ptr     += 4;
    } else {
        if (mc->ptr + 2 > mc->end) {
            return -1;
        }
        can_id_flags = pcan_get_le16(mc->ptr);
        f.id         = can_id_flags >> 5;
        mc->ptr     += 2;
    }

    /* can_frame_set_cc_len(): a raw nibble of 9..15 still means 8 bytes. */
    f.len = rec_len > 8u ? 8u : rec_len;

    if (host_decode_ts(mc, !mc->rec_ts_idx) != 0) {
        return -1;
    }
    mc->rec_ts_idx++;

    if (sl & PCAN_USB_STATUSLEN_RTR) {
        f.rtr = 1;
    } else {
        if (mc->ptr + rec_len > mc->end) {
            return -1;
        }
        memcpy(f.data, mc->ptr, f.len);
        mc->ptr += rec_len;
        if (can_id_flags & PCAN_USB_TX_SRR) {
            mc->ptr++;   /* client private id, ignored by the host             */
        }
    }

    f.ts16 = mc->ts16;
    if (mc->n_frames < HOST_MAX_REC) {
        mc->frame[mc->n_frames++] = f;
    }
    return 0;
}

/* pcan_usb_decode_msg(). Returns 0 if the host walked the whole batch without
 * an error, -1 otherwise. */
static int host_decode_batch(host_mc_t *mc, const uint8_t *buf, uint16_t len)
{
    memset(mc, 0, sizeof(*mc));
    if (len <= PCAN_USB_MSG_HEADER_LEN) {
        return -1;
    }
    mc->rec_cnt = buf[1];
    mc->ptr     = buf + PCAN_USB_MSG_HEADER_LEN;
    mc->end     = buf + len;

    int err = 0;
    for (; mc->rec_idx < mc->rec_cnt && !err; mc->rec_idx++) {
        if (mc->ptr >= mc->end) {
            return -1;   /* the real host reads sl unchecked; refuse instead   */
        }
        const uint8_t sl = *mc->ptr++;
        err = (sl & PCAN_USB_STATUSLEN_INTERNAL) ? host_decode_status(mc, sl)
                                                 : host_decode_data(mc, sl);
    }
    mc->consumed = (size_t)(mc->ptr - buf);
    return err;
}

/* ==========================================================================
 * Model of the Linux host TX encoder (EP2 OUT, pcan_usb_encode_msg).
 * ========================================================================== */

typedef struct {
    uint32_t id;
    uint8_t  ext, rtr, srr, at;
    uint8_t  raw_dlc;    /* nibble written into SL — can_get_cc_dlc(), 0..15   */
    uint8_t  data[8];
    uint8_t  data_len;   /* bytes the host actually writes — cf->len, 0..8     */
} txrec_t;

/* Builds a full-size 64-byte OUT batch the way the host driver does, including
 * the rolling tx counter in the last byte. */
static uint16_t host_encode_tx(uint8_t buf[PCAN_USB_TX_BUFFER_SIZE],
                               const txrec_t *rec, int n, uint8_t tx_counter)
{
    memset(buf, 0, PCAN_USB_TX_BUFFER_SIZE);
    buf[0] = PCAN_USB_MSG_TX_CAN;
    buf[1] = (uint8_t)n;

    uint8_t *pc = buf + PCAN_USB_MSG_HEADER_LEN;
    for (int i = 0; i < n; ++i) {
        const txrec_t *r = &rec[i];
        uint8_t *sl = pc++;
        uint32_t id_flags;

        *sl = (uint8_t)(r->raw_dlc & PCAN_USB_STATUSLEN_DLC);
        if (r->rtr) {
            *sl |= PCAN_USB_STATUSLEN_RTR;
        }
        if (r->ext) {
            *sl     |= PCAN_USB_STATUSLEN_EXT_ID;
            id_flags = r->id << PCAN_USB_EXT_ID_SHIFT;
        } else {
            id_flags = r->id << PCAN_USB_STD_ID_SHIFT;
        }
        if (r->srr) {
            id_flags |= PCAN_USB_TX_SRR;
        }
        if (r->at) {
            id_flags |= PCAN_USB_TX_AT;
        }
        if (r->ext) {
            pcan_put_le32(pc, id_flags);
            pc += 4;
        } else {
            pcan_put_le16(pc, (uint16_t)id_flags);
            pc += 2;
        }

        if (!r->rtr) {
            memcpy(pc, r->data, r->data_len);
            pc += r->data_len;
        }
        if (id_flags & PCAN_USB_TX_SRR) {
            *pc++ = 0x80;   /* the host's placeholder writer id                */
        }
    }

    buf[PCAN_USB_TX_BUFFER_SIZE - 1u] = tx_counter;
    return PCAN_USB_TX_BUFFER_SIZE;
}

/* ==========================================================================
 * Helpers
 * ========================================================================== */

static pcan_frame_t mk_frame(uint32_t id, uint8_t dlc, uint8_t flags,
                             uint16_t ts16)
{
    pcan_frame_t f;
    memset(&f, 0, sizeof(f));
    f.id    = id;
    f.dlc   = dlc;
    f.flags = flags;
    f.ts16  = ts16;
    for (uint8_t i = 0; i < dlc && i < PCAN_FRAME_MAX_DLC; ++i) {
        f.data[i] = (uint8_t)(0xA0u + i);
    }
    return f;
}

/* ==========================================================================
 * 1. REC_TS calibration record must be byte-exact
 * ========================================================================== */

static void test_calib_ts_bytes_exact(void)
{
    pcan_batch_t b;
    pcan_msg_batch_reset(&b);
    CHECK(pcan_msg_batch_add_calib_ts(&b, 0xBEEFu));

    const uint8_t want[] = {
        PCAN_USB_MSG_TX_CAN,                            /* batch type         */
        1,                                              /* rec_cnt            */
        PCAN_USB_STATUSLEN_INTERNAL | 2u,               /* SL: no TIMESTAMP   */
        PCAN_USB_REC_TS, 0,                             /* func, num          */
        0xEF, 0xBE,                                     /* tick, LE, payload  */
    };
    REQUIRE_EQ(pcan_msg_batch_len(&b), sizeof(want));
    CHECK_MEM(b.buf, want, sizeof(want));
    CHECK_EQ(pcan_msg_batch_count(&b), 1);
}

static void test_calib_ts_host_recovers_tick(void)
{
    pcan_batch_t b;
    host_mc_t    mc;

    pcan_msg_batch_reset(&b);
    CHECK(pcan_msg_batch_add_calib_ts(&b, 0x1234u));

    REQUIRE_EQ(host_decode_batch(&mc, b.buf, pcan_msg_batch_len(&b)), 0);
    REQUIRE_EQ(mc.n_calib, 1);
    CHECK_EQ(mc.calib[0], 0x1234u);
    /* The host must land exactly on the end of the batch: a record that leaves
     * its pointer short or long desynchronises everything after it. */
    CHECK_EQ(mc.consumed, pcan_msg_batch_len(&b));
    CHECK_EQ(mc.n_frames, 0);
}

/* A REC_TS must not consume the batch's first-timestamped-record slot: the
 * frame after it still has to carry a full 2-byte tick. */
static void test_calib_ts_does_not_take_first_ts_slot(void)
{
    pcan_batch_t b;
    host_mc_t    mc;

    pcan_msg_batch_reset(&b);
    CHECK(pcan_msg_batch_add_calib_ts(&b, 0x1000u));

    pcan_frame_t f1 = mk_frame(0x123u, 8, 0, 0x5678u);
    pcan_frame_t f2 = mk_frame(0x124u, 8, 0, 0x5679u);
    CHECK(pcan_msg_batch_add_frame(&b, &f1));
    CHECK(pcan_msg_batch_add_frame(&b, &f2));

    /* 2 hdr + 5 calib + (1+2+2+8) first frame + (1+2+1+8) second frame. */
    CHECK_EQ(pcan_msg_batch_len(&b), 2u + 5u + 13u + 12u);

    REQUIRE_EQ(host_decode_batch(&mc, b.buf, pcan_msg_batch_len(&b)), 0);
    CHECK_EQ(mc.consumed, pcan_msg_batch_len(&b));
    REQUIRE_EQ(mc.n_calib, 1);
    CHECK_EQ(mc.calib[0], 0x1000u);
    REQUIRE_EQ(mc.n_frames, 2);
    CHECK_EQ(mc.frame[0].ts16, 0x5678u);
    CHECK_EQ(mc.frame[1].ts16, 0x5679u);
    CHECK_EQ(mc.frame[0].id, 0x123u);
    CHECK_EQ(mc.frame[1].id, 0x124u);
}

/* usb_tx_task appends the periodic calibration record to whatever batch is
 * already accumulating, so REC_TS routinely lands after data records. The host
 * closes that record with ptr += rec_len and keeps its running prev_ts8, so the
 * frames on either side of it still decode exactly. */
static void test_calib_ts_mid_batch_does_not_disturb_neighbours(void)
{
    pcan_batch_t b;
    host_mc_t    mc;

    pcan_msg_batch_reset(&b);
    pcan_frame_t f1 = mk_frame(0x310u, 2, 0, 0x5610u);
    pcan_frame_t f2 = mk_frame(0x311u, 2, 0, 0x5630u);
    CHECK(pcan_msg_batch_add_frame(&b, &f1));
    CHECK(pcan_msg_batch_add_calib_ts(&b, 0x5620u));
    CHECK(pcan_msg_batch_add_frame(&b, &f2));

    REQUIRE_EQ(host_decode_batch(&mc, b.buf, pcan_msg_batch_len(&b)), 0);
    CHECK_EQ(mc.consumed, pcan_msg_batch_len(&b));
    REQUIRE_EQ(mc.n_frames, 2);
    REQUIRE_EQ(mc.n_calib, 1);
    CHECK_EQ(mc.calib[0], 0x5620u);
    CHECK_EQ(mc.frame[0].ts16, 0x5610u);
    CHECK_EQ(mc.frame[1].ts16, 0x5630u);
}

/* ==========================================================================
 * 2. Timestamp widths and the host's high-byte reconstruction
 * ========================================================================== */

static void test_ts_first_is_word_rest_are_bytes(void)
{
    pcan_batch_t b;
    pcan_msg_batch_reset(&b);

    pcan_frame_t f = mk_frame(0x100u, 0, 0, 0xAABBu);
    CHECK(pcan_msg_batch_add_frame(&b, &f));
    /* SL(1) + id(2) + ts(2) = 5 */
    CHECK_EQ(pcan_msg_batch_len(&b), PCAN_USB_MSG_HEADER_LEN + 5u);
    CHECK_EQ(b.buf[5], 0xBBu);
    CHECK_EQ(b.buf[6], 0xAAu);

    f.ts16 = 0xAACCu;
    CHECK(pcan_msg_batch_add_frame(&b, &f));
    /* SL(1) + id(2) + ts(1) = 4 */
    CHECK_EQ(pcan_msg_batch_len(&b), PCAN_USB_MSG_HEADER_LEN + 5u + 4u);
    CHECK_EQ(b.buf[10], 0xCCu);
}

static void test_ts_low_byte_wrap_reconstructs(void)
{
    pcan_batch_t b;
    host_mc_t    mc;
    pcan_msg_batch_reset(&b);

    /* 0x12FF -> 0x1305 crosses a low-byte wrap; the host must carry the high
     * byte from the 1-byte tick alone. */
    pcan_frame_t f1 = mk_frame(0x201u, 1, 0, 0x12FFu);
    pcan_frame_t f2 = mk_frame(0x202u, 1, 0, 0x1305u);
    pcan_frame_t f3 = mk_frame(0x203u, 1, 0, 0x1307u);
    CHECK(pcan_msg_batch_add_frame(&b, &f1));
    CHECK(pcan_msg_batch_add_frame(&b, &f2));
    CHECK(pcan_msg_batch_add_frame(&b, &f3));

    REQUIRE_EQ(host_decode_batch(&mc, b.buf, pcan_msg_batch_len(&b)), 0);
    REQUIRE_EQ(mc.n_frames, 3);
    CHECK_EQ(mc.frame[0].ts16, 0x12FFu);
    CHECK_EQ(mc.frame[1].ts16, 0x1305u);
    CHECK_EQ(mc.frame[2].ts16, 0x1307u);
    CHECK_EQ(mc.consumed, pcan_msg_batch_len(&b));
}

/* ==========================================================================
 * 3. Encode -> host decode round trip: id shifting, RTR, SRR, errors
 * ========================================================================== */

static void test_encode_std_ext_ids_round_trip(void)
{
    pcan_batch_t b;
    host_mc_t    mc;
    pcan_msg_batch_reset(&b);

    pcan_frame_t std = mk_frame(0x7FFu, 8, 0, 0x0001u);
    pcan_frame_t ext = mk_frame(0x1FFFFFFFu, 4, PCAN_FRAME_FLAG_EXT, 0x0002u);
    CHECK(pcan_msg_batch_add_frame(&b, &std));
    CHECK(pcan_msg_batch_add_frame(&b, &ext));

    REQUIRE_EQ(host_decode_batch(&mc, b.buf, pcan_msg_batch_len(&b)), 0);
    REQUIRE_EQ(mc.n_frames, 2);
    CHECK_EQ(mc.frame[0].ext, 0);
    CHECK_EQ(mc.frame[0].id, 0x7FFu);
    CHECK_EQ(mc.frame[0].len, 8);
    CHECK_MEM(mc.frame[0].data, std.data, 8);
    CHECK_EQ(mc.frame[1].ext, 1);
    CHECK_EQ(mc.frame[1].id, 0x1FFFFFFFu);
    CHECK_EQ(mc.frame[1].len, 4);
    CHECK_MEM(mc.frame[1].data, ext.data, 4);
    CHECK_EQ(mc.consumed, pcan_msg_batch_len(&b));
}

static void test_encode_rtr_carries_no_data_and_no_srr_trailer(void)
{
    pcan_batch_t b;
    host_mc_t    mc;
    pcan_msg_batch_reset(&b);

    /* RTR + SRR together: the host consumes the SRR trailer only in its
     * non-RTR branch, so emitting one here would desync the whole batch. */
    pcan_frame_t rtr = mk_frame(0x321u, 8,
                                PCAN_FRAME_FLAG_RTR | PCAN_FRAME_FLAG_SRR,
                                0x0010u);
    rtr.writer_id = 0x5Au;
    pcan_frame_t after = mk_frame(0x322u, 2, 0, 0x0011u);
    CHECK(pcan_msg_batch_add_frame(&b, &rtr));
    CHECK(pcan_msg_batch_add_frame(&b, &after));

    /* SL(1) + id(2) + ts(2), no data, no trailer. */
    CHECK_EQ(pcan_msg_batch_len(&b), PCAN_USB_MSG_HEADER_LEN + 5u + (1u + 2u + 1u + 2u));

    REQUIRE_EQ(host_decode_batch(&mc, b.buf, pcan_msg_batch_len(&b)), 0);
    REQUIRE_EQ(mc.n_frames, 2);
    CHECK_EQ(mc.frame[0].rtr, 1);
    CHECK_EQ(mc.frame[0].id, 0x321u);
    CHECK_EQ(mc.frame[1].rtr, 0);
    CHECK_EQ(mc.frame[1].id, 0x322u);
    CHECK_EQ(mc.frame[1].len, 2);
    CHECK_EQ(mc.consumed, pcan_msg_batch_len(&b));
}

static void test_encode_srr_trailer_is_writer_id(void)
{
    pcan_batch_t b;
    host_mc_t    mc;
    pcan_msg_batch_reset(&b);

    pcan_frame_t echo = mk_frame(0x123u, 3, PCAN_FRAME_FLAG_SRR, 0x0020u);
    echo.writer_id = 0x77u;
    pcan_frame_t next = mk_frame(0x124u, 1, 0, 0x0021u);
    CHECK(pcan_msg_batch_add_frame(&b, &echo));
    CHECK(pcan_msg_batch_add_frame(&b, &next));

    /* SL(1)+id(2)+ts(2)+data(3)+trailer(1) = 9 */
    CHECK_EQ(b.buf[2 + 8], 0x77u);
    /* The SRR bit lives in the low bits of the shifted id word. */
    CHECK_EQ(pcan_get_le16(&b.buf[3]) & PCAN_USB_TX_SRR, PCAN_USB_TX_SRR);

    REQUIRE_EQ(host_decode_batch(&mc, b.buf, pcan_msg_batch_len(&b)), 0);
    REQUIRE_EQ(mc.n_frames, 2);
    CHECK_EQ(mc.frame[0].id, 0x123u);
    CHECK_EQ(mc.frame[1].id, 0x124u);
    CHECK_EQ(mc.consumed, pcan_msg_batch_len(&b));
}

static void test_encode_error_record_round_trip(void)
{
    pcan_batch_t b;
    host_mc_t    mc;
    pcan_msg_batch_reset(&b);

    CHECK(pcan_msg_batch_add_error(&b, PCAN_USB_ERROR_BUS_OFF, 0x4321u));
    pcan_frame_t f = mk_frame(0x400u, 1, 0, 0x4322u);
    CHECK(pcan_msg_batch_add_frame(&b, &f));

    REQUIRE_EQ(host_decode_batch(&mc, b.buf, pcan_msg_batch_len(&b)), 0);
    REQUIRE_EQ(mc.n_err, 1);
    CHECK_EQ(mc.err_mask[0], PCAN_USB_ERROR_BUS_OFF);
    /* The error record IS timestamped, so it takes the first-ts slot and the
     * frame after it gets a 1-byte tick — which the host still resolves. */
    REQUIRE_EQ(mc.n_frames, 1);
    CHECK_EQ(mc.frame[0].ts16, 0x4322u);
    CHECK_EQ(mc.consumed, pcan_msg_batch_len(&b));
}

/* ==========================================================================
 * 4. Batch overflow refusal
 * ========================================================================== */

static void test_batch_overflow_refuses_and_never_writes_past(void)
{
    pcan_batch_t b;
    memset(&b, 0xA5, sizeof(b));   /* canary the whole 64-byte buffer          */
    pcan_msg_batch_reset(&b);

    pcan_frame_t f = mk_frame(0x555u, 8, 0, 0x0100u);

    int added = 0;
    while (pcan_msg_batch_add_frame(&b, &f)) {
        ++added;
        REQUIRE(added <= 16);       /* refuses to loop forever                  */
        f.ts16++;
    }

    /* hdr 2 + first record 13 + N*12 must be the largest run that fits in 64. */
    CHECK_EQ(added, 5);
    CHECK_EQ(pcan_msg_batch_len(&b), 63);
    CHECK_EQ(pcan_msg_batch_count(&b), 5);
    CHECK_EQ(b.buf[1], 5);

    /* Nothing was written into the tail the refused record would have used. */
    CHECK_EQ(b.buf[63], 0xA5u);

    /* Even the smallest possible records are refused once there is no room. */
    pcan_frame_t empty = mk_frame(0x001u, 0, 0, 0x0200u);
    CHECK(!pcan_msg_batch_add_frame(&b, &empty));
    CHECK(!pcan_msg_batch_add_status(&b, PCAN_USB_REC_BUSLOAD, 0, NULL, 0, false, 0));
    CHECK_EQ(pcan_msg_batch_len(&b), 63);
    CHECK_EQ(pcan_msg_batch_count(&b), 5);
    CHECK_EQ(b.buf[63], 0xA5u);
}

/* Packing density pins the timestamp-width accounting: reserving 2 bytes for
 * every record instead of 2-then-1 silently costs records per batch. */
static void test_batch_packs_small_records_densely(void)
{
    pcan_batch_t b;
    pcan_msg_batch_reset(&b);

    pcan_frame_t f = mk_frame(0x001u, 0, 0, 0x0300u);
    int added = 0;
    while (pcan_msg_batch_add_frame(&b, &f)) {
        ++added;
        REQUIRE(added <= 32);
        f.ts16++;
    }

    /* hdr 2 + first record 5 + 14 * 4 = 63. */
    CHECK_EQ(added, 15);
    CHECK_EQ(pcan_msg_batch_len(&b), 63);
    CHECK_EQ(pcan_msg_batch_count(&b), 15);
}

/* A batch may be filled to exactly 64 bytes: the refusal is at ">", not ">=",
 * and the size a record reserves has to be the size it writes. */
static void test_batch_may_fill_to_exactly_64_bytes(void)
{
    pcan_batch_t b;
    host_mc_t    mc;
    pcan_msg_batch_reset(&b);

    /* 2 + 13 + 3*12 = 51, + 9 = 60, + 4 = 64 exactly. */
    pcan_frame_t f8 = mk_frame(0x101u, 8, 0, 0x0400u);
    for (int i = 0; i < 4; ++i) {
        CHECK(pcan_msg_batch_add_frame(&b, &f8));
        f8.ts16++;
    }
    CHECK_EQ(pcan_msg_batch_len(&b), 51);

    pcan_frame_t f5 = mk_frame(0x102u, 5, 0, 0x0410u);
    CHECK(pcan_msg_batch_add_frame(&b, &f5));
    CHECK_EQ(pcan_msg_batch_len(&b), 60);

    pcan_frame_t f0 = mk_frame(0x103u, 0, 0, 0x0411u);
    CHECK(pcan_msg_batch_add_frame(&b, &f0));
    CHECK_EQ(pcan_msg_batch_len(&b), PCAN_USB_TX_BUFFER_SIZE);
    CHECK_EQ(pcan_msg_batch_count(&b), 6);

    CHECK(!pcan_msg_batch_add_frame(&b, &f0));

    REQUIRE_EQ(host_decode_batch(&mc, b.buf, pcan_msg_batch_len(&b)), 0);
    CHECK_EQ(mc.n_frames, 6);
    CHECK_EQ(mc.consumed, PCAN_USB_TX_BUFFER_SIZE);
}

static void test_status_payload_must_fit_the_length_nibble(void)
{
    pcan_batch_t b;
    pcan_msg_batch_reset(&b);

    uint8_t payload[16];
    memset(payload, 0x5A, sizeof(payload));

    /* rec_len lives in the SL low nibble, so 16 bytes cannot be described. */
    CHECK(!pcan_msg_batch_add_status(&b, PCAN_USB_REC_BUSEVT, 0, payload, 16, false, 0));
    CHECK(pcan_msg_batch_add_status(&b, PCAN_USB_REC_BUSEVT, 0, payload, 15, false, 0));
    CHECK_EQ(pcan_msg_batch_count(&b), 1);

    /* A non-NULL payload is required whenever payload_len > 0. */
    pcan_msg_batch_reset(&b);
    CHECK(!pcan_msg_batch_add_status(&b, PCAN_USB_REC_BUSEVT, 0, NULL, 3, false, 0));
    CHECK_EQ(pcan_msg_batch_count(&b), 0);
}

/* ==========================================================================
 * 5. decode_tx against the host's own encoder
 * ========================================================================== */

static void test_decode_tx_host_encoded_std_frame(void)
{
    uint8_t      buf[PCAN_USB_TX_BUFFER_SIZE];
    pcan_frame_t out[4];

    txrec_t r = { .id = 0x7FFu, .raw_dlc = 8, .data_len = 8 };
    for (int i = 0; i < 8; ++i) {
        r.data[i] = (uint8_t)(0x10u + i);
    }
    const uint16_t len = host_encode_tx(buf, &r, 1, 0x42u);

    REQUIRE_EQ(pcan_msg_decode_tx(buf, len, out, 4), 1);
    CHECK_EQ(out[0].id, 0x7FFu);
    CHECK_EQ(out[0].dlc, 8);
    CHECK_EQ(out[0].flags, 0);
    CHECK_EQ(out[0].ts16, 0);
    CHECK_EQ(out[0].writer_id, 0x42u);
    CHECK_MEM(out[0].data, r.data, 8);
}

static void test_decode_tx_host_encoded_ext_srr_at(void)
{
    uint8_t      buf[PCAN_USB_TX_BUFFER_SIZE];
    pcan_frame_t out[4];

    txrec_t r = { .id = 0x1ABCDEFu, .ext = 1, .srr = 1, .at = 1,
                  .raw_dlc = 2, .data_len = 2 };
    r.data[0] = 0xDE;
    r.data[1] = 0xAD;
    const uint16_t len = host_encode_tx(buf, &r, 1, 0x99u);

    REQUIRE_EQ(pcan_msg_decode_tx(buf, len, out, 4), 1);
    CHECK_EQ(out[0].id, 0x1ABCDEFu);
    CHECK_EQ(out[0].dlc, 2);
    CHECK_EQ(out[0].flags,
             PCAN_FRAME_FLAG_EXT | PCAN_FRAME_FLAG_SRR | PCAN_FRAME_FLAG_SS);
    CHECK_EQ(out[0].writer_id, 0x99u);
    CHECK_MEM(out[0].data, r.data, 2);
}

static void test_decode_tx_rtr_has_no_payload(void)
{
    uint8_t      buf[PCAN_USB_TX_BUFFER_SIZE];
    pcan_frame_t out[4];

    txrec_t r[2] = {
        { .id = 0x111u, .rtr = 1, .raw_dlc = 8, .data_len = 0 },
        { .id = 0x222u, .raw_dlc = 3, .data_len = 3 },
    };
    r[1].data[0] = 1; r[1].data[1] = 2; r[1].data[2] = 3;
    const uint16_t len = host_encode_tx(buf, r, 2, 0x01u);

    REQUIRE_EQ(pcan_msg_decode_tx(buf, len, out, 4), 2);
    CHECK_EQ(out[0].id, 0x111u);
    CHECK_EQ(out[0].flags, PCAN_FRAME_FLAG_RTR);
    CHECK_EQ(out[0].dlc, 8);
    CHECK_EQ(out[1].id, 0x222u);
    CHECK_EQ(out[1].dlc, 3);
    CHECK_MEM(out[1].data, r[1].data, 3);
}

/* A raw DLC of 9..15 is what the host emits with CAN_CTRLMODE_CC_LEN8_DLC on.
 * It must decode as an 8-byte frame, and the records around it must survive:
 * rejecting it would take the whole batch down with it. */
static void test_decode_tx_raw_dlc_9_to_15_is_eight_bytes(void)
{
    for (uint8_t raw = 9; raw <= 15; ++raw) {
        uint8_t      buf[PCAN_USB_TX_BUFFER_SIZE];
        pcan_frame_t out[4];

        txrec_t r[3] = {
            { .id = 0x0A0u, .raw_dlc = 1, .data_len = 1 },
            { .id = 0x0B0u, .raw_dlc = raw, .data_len = 8 },
            { .id = 0x0C0u, .raw_dlc = 2, .data_len = 2 },
        };
        r[0].data[0] = 0xE1;
        for (int i = 0; i < 8; ++i) {
            r[1].data[i] = (uint8_t)(0x70u + i);
        }
        r[2].data[0] = 0xE2;
        r[2].data[1] = 0xE3;

        const uint16_t len = host_encode_tx(buf, r, 3, 0x07u);

        CHECK_MSG(pcan_msg_decode_tx(buf, len, out, 4) == 3,
                  "raw dlc %u: batch must decode all 3 records", raw);
        CHECK_EQ(out[0].id, 0x0A0u);
        CHECK_EQ(out[1].id, 0x0B0u);
        CHECK_EQ(out[1].dlc, PCAN_FRAME_MAX_DLC);
        CHECK_MEM(out[1].data, r[1].data, 8);
        CHECK_EQ(out[2].id, 0x0C0u);
        CHECK_EQ(out[2].dlc, 2);
        CHECK_MEM(out[2].data, r[2].data, 2);
    }
}

static void test_decode_tx_skips_internal_records(void)
{
    uint8_t      buf[PCAN_USB_TX_BUFFER_SIZE];
    pcan_frame_t out[4];

    memset(buf, 0, sizeof(buf));
    buf[0] = PCAN_USB_MSG_TX_CAN;
    buf[1] = 2;
    /* Status record: SL(INTERNAL|3) + func + num + 3 payload bytes. */
    buf[2] = PCAN_USB_STATUSLEN_INTERNAL | 3u;
    buf[3] = PCAN_USB_REC_BUSLOAD;
    buf[4] = 0;
    buf[5] = 0xAA; buf[6] = 0xBB; buf[7] = 0xCC;
    /* Then a normal 1-byte std frame. */
    buf[8] = 1;
    pcan_put_le16(&buf[9], (uint16_t)(0x123u << PCAN_USB_STD_ID_SHIFT));
    buf[11] = 0x5A;
    buf[PCAN_USB_TX_BUFFER_SIZE - 1u] = 0x33;

    REQUIRE_EQ(pcan_msg_decode_tx(buf, PCAN_USB_TX_BUFFER_SIZE, out, 4), 1);
    CHECK_EQ(out[0].id, 0x123u);
    CHECK_EQ(out[0].dlc, 1);
    CHECK_EQ(out[0].data[0], 0x5Au);
}

static void test_decode_tx_rejects_malformed(void)
{
    pcan_frame_t out[4];
    uint8_t      buf[PCAN_USB_TX_BUFFER_SIZE];

    /* Header-only / empty. */
    memset(buf, 0, sizeof(buf));
    buf[0] = PCAN_USB_MSG_TX_CAN;
    CHECK_EQ(pcan_msg_decode_tx(buf, 0, out, 4), -1);
    CHECK_EQ(pcan_msg_decode_tx(buf, PCAN_USB_MSG_HEADER_LEN, out, 4), -1);

    /* NULL / zero-capacity guards. */
    CHECK_EQ(pcan_msg_decode_tx(NULL, 16, out, 4), -1);
    CHECK_EQ(pcan_msg_decode_tx(buf, 16, NULL, 4), -1);
    CHECK_EQ(pcan_msg_decode_tx(buf, 16, out, 0), -1);

    /* Truncated ext id word: claims one record, supplies 2 of 4 id bytes. */
    buf[1] = 1;
    buf[2] = PCAN_USB_STATUSLEN_EXT_ID | 0u;
    buf[3] = 0x00; buf[4] = 0x00;
    CHECK_EQ(pcan_msg_decode_tx(buf, 5, out, 4), -1);

    /* Truncated data: dlc 8 but only 3 payload bytes present. */
    buf[1] = 1;
    buf[2] = 8;
    buf[3] = 0x00; buf[4] = 0x00;
    CHECK_EQ(pcan_msg_decode_tx(buf, 8, out, 4), -1);

    /* Truncated SRR trailer: dlc 0 with SRR set and nothing after the id. */
    buf[1] = 1;
    buf[2] = 0;
    pcan_put_le16(&buf[3], PCAN_USB_TX_SRR);
    CHECK_EQ(pcan_msg_decode_tx(buf, 5, out, 4), -1);

    /* Truncated internal record: rec_len runs past the transfer. */
    buf[1] = 1;
    buf[2] = PCAN_USB_STATUSLEN_INTERNAL | 8u;
    buf[3] = PCAN_USB_REC_BUSEVT;
    buf[4] = 0;
    CHECK_EQ(pcan_msg_decode_tx(buf, 6, out, 4), -1);
}

static void test_decode_tx_never_exceeds_max_out(void)
{
    uint8_t      buf[PCAN_USB_TX_BUFFER_SIZE];
    pcan_frame_t out[8];

    txrec_t r[5];
    memset(r, 0, sizeof(r));
    for (int i = 0; i < 5; ++i) {
        r[i].id       = (uint32_t)(0x100u + i);
        r[i].raw_dlc  = 4;
        r[i].data_len = 4;
    }
    const uint16_t len = host_encode_tx(buf, r, 5, 0x11u);

    CHECK_EQ(pcan_msg_decode_tx(buf, len, out, 5), 5);
    for (int cap = 1; cap <= 4; ++cap) {
        memset(out, 0, sizeof(out));
        CHECK_EQ(pcan_msg_decode_tx(buf, len, out, cap), cap);
        CHECK_EQ(out[cap - 1].id, (uint32_t)(0x100u + cap - 1));
        /* Nothing was written past the caller's capacity. */
        CHECK_EQ(out[cap].id, 0u);
    }

    /* A wildly overstated rec_cnt still cannot produce more than max_out. */
    buf[1] = 0xFF;
    const int n = pcan_msg_decode_tx(buf, len, out, 3);
    CHECK(n <= 3);
}

/* ==========================================================================
 * 6. Fuzz decode_tx.
 *
 * Deterministic LCG, and every batch is handed to the decoder in a buffer
 * whose last byte sits flush against an unmapped guard page: a read even one
 * byte past the declared length segfaults immediately, sanitizer or not.
 * (Under ASan the malloc-style redzones catch it too, but the guard page is
 * what makes this test meaningful on a host whose ASan runtime is unusable.)
 * ========================================================================== */

#define FUZZ_ITERS 40000u

static uint32_t lcg_next(uint32_t *s)
{
    *s = (uint32_t)(*s * 1103515245u + 12345u);
    return *s;
}

typedef struct {
    void   *region;      /* mapping / allocation to release                    */
    size_t  region_len;
    uint8_t *buf;        /* n writable bytes, guarded immediately after        */
} guarded_t;

/* Returns 0 on success. n must be >= 1. */
static int guarded_alloc(guarded_t *g, size_t n)
{
#ifdef HAVE_GUARD_PAGES
    const size_t ps    = (size_t)sysconf(_SC_PAGESIZE);
    const size_t pages = (n + ps - 1u) / ps;
    const size_t total = (pages + 1u) * ps;

    uint8_t *p = (uint8_t *)mmap(NULL, total, PROT_READ | PROT_WRITE,
                                 MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (p == MAP_FAILED) {
        return -1;
    }
    if (mprotect(p + pages * ps, ps, PROT_NONE) != 0) {
        munmap(p, total);
        return -1;
    }
    g->region     = p;
    g->region_len = total;
    g->buf        = p + pages * ps - n;
    return 0;
#else
    /* No guard pages here; ASan redzones are the only backstop. */
    uint8_t *p = (uint8_t *)malloc(n);
    if (p == NULL) {
        return -1;
    }
    g->region     = p;
    g->region_len = n;
    g->buf        = p;
    return 0;
#endif
}

static void guarded_free(guarded_t *g)
{
#ifdef HAVE_GUARD_PAGES
    munmap(g->region, g->region_len);
#else
    free(g->region);
#endif
    g->region = NULL;
    g->buf    = NULL;
}

static void test_decode_tx_fuzz_random_bytes(void)
{
    uint32_t     seed = 0xC0FFEEu;
    pcan_frame_t out[PCAN_USB_TX_BUFFER_SIZE];
    unsigned     decoded_any = 0;

    for (unsigned it = 0; it < FUZZ_ITERS; ++it) {
        guarded_t      g;
        const uint16_t len = (uint16_t)(1u + lcg_next(&seed) %
                                        PCAN_USB_TX_BUFFER_SIZE);
        REQUIRE(guarded_alloc(&g, len) == 0);
        for (uint16_t i = 0; i < len; ++i) {
            g.buf[i] = (uint8_t)(lcg_next(&seed) >> 16);
        }

        const int max_out = (int)(lcg_next(&seed) % 8u) + 1;
        const int n = pcan_msg_decode_tx(g.buf, len, out, max_out);

        if (n < -1 || n > max_out) {
            CHECK_MSG(0, "iter %u: decode_tx returned %d for len %u max_out %d",
                      it, n, len, max_out);
            guarded_free(&g);
            return;
        }
        for (int i = 0; i < n; ++i) {
            if (out[i].dlc > PCAN_FRAME_MAX_DLC) {
                CHECK_MSG(0, "iter %u: frame %d has dlc %u", it, i, out[i].dlc);
                guarded_free(&g);
                return;
            }
            ++decoded_any;
        }
        guarded_free(&g);
    }
    ++t_checks;   /* the run itself is the assertion (ASan/UBSan are watching) */

    /* Guard against a fuzz loop that never actually reaches the decoder. */
    CHECK(decoded_any > 0);
}

/* Same idea, but every record header is structurally plausible, so the fuzzer
 * spends its time inside the record walk instead of bouncing off rec_cnt. */
static void test_decode_tx_fuzz_structured(void)
{
    uint32_t     seed = 0x1BADB002u;
    pcan_frame_t out[PCAN_USB_TX_BUFFER_SIZE];

    for (unsigned it = 0; it < FUZZ_ITERS; ++it) {
        guarded_t      g;
        const uint16_t len = (uint16_t)(3u + lcg_next(&seed) %
                                        (PCAN_USB_TX_BUFFER_SIZE - 2u));
        REQUIRE(guarded_alloc(&g, len) == 0);

        g.buf[0] = PCAN_USB_MSG_TX_CAN;
        g.buf[1] = (uint8_t)(1u + lcg_next(&seed) % 12u);
        for (uint16_t i = 2; i < len; ++i) {
            const uint32_t r = lcg_next(&seed);
            /* Bias towards plausible SL bytes: no TIMESTAMP (EP2 OUT has none),
             * INTERNAL rare, dlc spread over the whole 0..15 nibble. */
            g.buf[i] = (r % 3u == 0u) ? (uint8_t)((r >> 8) & 0x3Fu)
                                      : (uint8_t)(r >> 16);
        }

        const int max_out = (int)(lcg_next(&seed) % 8u) + 1;
        const int n = pcan_msg_decode_tx(g.buf, len, out, max_out);
        if (n < -1 || n > max_out) {
            CHECK_MSG(0, "iter %u: decode_tx returned %d for len %u max_out %d",
                      it, n, len, max_out);
            guarded_free(&g);
            return;
        }
        guarded_free(&g);
    }
    ++t_checks;
}

/* ==========================================================================
 * 7. Timestamp engine
 * ========================================================================== */

static void test_time_tick_derivation_and_wrap(void)
{
    stub_reset();
    stub_time_set_us(1000000);      /* a non-zero epoch must not leak in       */
    pcan_time_init();
    CHECK_EQ(pcan_time_now16(), 0);
    CHECK_EQ(pcan_time_now_us(), 0);

    /* 42667 us == 1000 ticks exactly (PCAN_USB_TS_TICK_NS is 42667 ns). */
    stub_time_advance_us(42667);
    CHECK_EQ(pcan_time_now16(), 1000);
    CHECK_EQ(pcan_time_now_us(), 42667);

    /* Tick 65541 aliases to 5 in 16 bits: (65541 * 42667 + 999) / 1000 us. */
    stub_time_set_us(1000000 + 2796438);
    CHECK_EQ(pcan_time_now16(), 5);

    /* A clock that appears to run backwards clamps rather than wrapping. */
    stub_time_set_us(0);
    CHECK_EQ(pcan_time_now16(), 0);
}

static void test_time_ticks_to_us_matches_host_formula(void)
{
    CHECK_EQ(pcan_time_ticks_to_us(0), 0u);
    CHECK_EQ(pcan_time_ticks_to_us(1), 42u);        /* 44739243 >> 20          */
    /* The host's tick is 42.66634 us, marginally shorter than the 42667 ns the
     * device timer targets — 1 us of drift per 1000 ticks, ~16 ppm. */
    CHECK_EQ(pcan_time_ticks_to_us(1000), 42666u);
    CHECK_EQ(pcan_time_ticks_to_us(0xFFFFu),
             (uint32_t)(((uint64_t)0xFFFFu * PCAN_USB_TS_US_PER_TICK) >>
                        PCAN_USB_TS_DIV_SHIFTER));
    /* ~2.796 s per full 16-bit wrap. */
    CHECK(pcan_time_ticks_to_us(0xFFFFu) > 2790000u);
    CHECK(pcan_time_ticks_to_us(0xFFFFu) < 2800000u);
}

/* ========================================================================== */

void suite_pcan_msg(void);

void suite_pcan_msg(void)
{
    printf("pcan_msg / pcan_time\n");
    RUN(test_calib_ts_bytes_exact);
    RUN(test_calib_ts_host_recovers_tick);
    RUN(test_calib_ts_does_not_take_first_ts_slot);
    RUN(test_calib_ts_mid_batch_does_not_disturb_neighbours);
    RUN(test_ts_first_is_word_rest_are_bytes);
    RUN(test_ts_low_byte_wrap_reconstructs);
    RUN(test_encode_std_ext_ids_round_trip);
    RUN(test_encode_rtr_carries_no_data_and_no_srr_trailer);
    RUN(test_encode_srr_trailer_is_writer_id);
    RUN(test_encode_error_record_round_trip);
    RUN(test_batch_overflow_refuses_and_never_writes_past);
    RUN(test_batch_packs_small_records_densely);
    RUN(test_batch_may_fill_to_exactly_64_bytes);
    RUN(test_status_payload_must_fit_the_length_nibble);
    RUN(test_decode_tx_host_encoded_std_frame);
    RUN(test_decode_tx_host_encoded_ext_srr_at);
    RUN(test_decode_tx_rtr_has_no_payload);
    RUN(test_decode_tx_raw_dlc_9_to_15_is_eight_bytes);
    RUN(test_decode_tx_skips_internal_records);
    RUN(test_decode_tx_rejects_malformed);
    RUN(test_decode_tx_never_exceeds_max_out);
    RUN(test_decode_tx_fuzz_random_bytes);
    RUN(test_decode_tx_fuzz_structured);
    RUN(test_time_tick_derivation_and_wrap);
    RUN(test_time_ticks_to_us_matches_host_formula);
}
