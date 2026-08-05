/*
 * SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
/*
 * pcan_msg.h — CAN <-> USB record codec (EP2 IN / EP2 OUT)
 * ============================================================================
 * Encodes canonical pcan_frame_t / status events into EP2-IN wire batches and
 * decodes host EP2-OUT TX batches back into pcan_frame_t. All wire layouts are
 * defined in pcan_proto.h §3-§5; this module is the ONLY place that serializes
 * or parses them.
 *
 * BUFFER CONTRACT: an encode batch is built into a caller-provided buffer of
 * exactly PCAN_USB_TX_BUFFER_SIZE (64) bytes. The encoder never writes past
 * 64 bytes and refuses a frame that would overflow (caller then flushes the
 * batch and starts a new one). Decoders take a const buffer + length and never
 * write into it.
 *
 * THREADING: stateless except for the per-batch pcan_batch_t the caller owns.
 * Multiple batches may be built concurrently on different tasks as long as each
 * has its own pcan_batch_t. No globals.
 * ============================================================================
 */
#ifndef PCAN_MSG_H
#define PCAN_MSG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "pcan_proto.h"
#include "pcan_time.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Batch encoder state. The caller zero/reset-inits this, then repeatedly calls
 * pcan_msg_batch_add_*() until one returns false (no room), flushes the buffer
 * over EP2 IN, and resets. The buffer[0..1] header is written by
 * pcan_msg_batch_reset(); rec_cnt is maintained incrementally.
 */
typedef struct {
    uint8_t         buf[PCAN_USB_TX_BUFFER_SIZE]; /* the 64-byte batch          */
    uint16_t        len;      /* bytes used so far (starts at HEADER_LEN)       */
    uint8_t         rec_cnt;  /* records added so far (mirrors buf[1])          */
    pcan_ts_batch_t ts;       /* first-vs-subsequent timestamp width tracker    */
} pcan_batch_t;

/* Start a new EP2-IN batch: writes [type=PCAN_USB_MSG_TX_CAN][rec_cnt=0]. */
void pcan_msg_batch_reset(pcan_batch_t *b);

/* Total bytes currently in the batch (>= HEADER_LEN). Ready to send if
 * pcan_msg_batch_count() > 0. */
static inline uint16_t pcan_msg_batch_len(const pcan_batch_t *b) { return b->len; }
static inline uint8_t  pcan_msg_batch_count(const pcan_batch_t *b) { return b->rec_cnt; }

/*
 * Append a CAN data frame record (with timestamp) to the batch. Returns true
 * on success; false if the record would exceed 64 bytes (caller flushes +
 * resets, then retries). Uses frame->ts16 for the timestamp and frame->flags
 * for EXT/RTR. If frame->flags has PCAN_FRAME_FLAG_SRR the SRR trailer byte is
 * appended (loopback echo correlation). DLC is saturated at 8 (classic CAN),
 * the same rule pcan_msg_decode_tx() applies to a raw 9..15 nibble.
 */
bool pcan_msg_batch_add_frame(pcan_batch_t *b, const pcan_frame_t *frame);

/*
 * Append a status/error record (INTERNAL bit set). `func` is one of
 * PCAN_USB_REC_*, `num` its sub-code/bitmask. `payload`/`payload_len` are the
 * record body (may be NULL/0). If with_ts, a timestamp is emitted using ts16.
 * Returns false if it would overflow the batch.
 */
bool pcan_msg_batch_add_status(pcan_batch_t *b, uint8_t func, uint8_t num,
                               const uint8_t *payload, uint8_t payload_len,
                               bool with_ts, uint16_t ts16);

/* Convenience: append a REC_TS(4) calibration record. The tick goes in the
 * 2-byte payload with the TIMESTAMP bit clear (pcan_proto.h §5). */
bool pcan_msg_batch_add_calib_ts(pcan_batch_t *b, uint16_t ts16);

/* Convenience: append a REC_ERROR(1) record carrying a PCAN_USB_ERROR_* mask. */
bool pcan_msg_batch_add_error(pcan_batch_t *b, uint8_t err_mask, uint16_t ts16);

/*
 * Decode ONE host TX batch received on EP2 OUT (buf/len, up to 64 bytes) into
 * up to `max_out` pcan_frame_t entries. Returns the number of frames decoded
 * (>=0), or -1 on a malformed batch (len <= HEADER_LEN, truncated record). A
 * raw DLC of 9..15 is a valid classic-CAN encoding of 8 data bytes and is
 * saturated to 8, not rejected. Sets each out frame's flags (EXT/RTR/SS/SRR)
 * and writer_id per the TX encode rules in pcan_proto.h §3. Does not transmit
 * anything itself.
 */
int pcan_msg_decode_tx(const uint8_t *buf, uint16_t len,
                       pcan_frame_t *out, int max_out);

#ifdef __cplusplus
}
#endif
#endif /* PCAN_MSG_H */
