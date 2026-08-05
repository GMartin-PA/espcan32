/*
 * SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
/*
 * pcan_time.h — device timestamp engine + host time calibration
 * ============================================================================
 * Produces the free-running 16-bit tick counter that PCAN RX records and
 * REC_TS calibration records carry (see pcan_proto.h §4). The device timestamp
 * advances at ~42.667 us/tick (PCAN_USB_TS_TICK_NS) so the host's fixed-point
 * us conversion (ticks * 44739243 >> 20) reproduces true wall time.
 *
 * IMPLEMENTATION NOTE: we do not run a real 23.4 kHz interrupt. Instead we
 * derive the 16-bit tick from esp_timer_get_time() (monotonic microseconds):
 *   tick16 = (uint16_t)((us_since_epoch * 1000 / PCAN_USB_TS_TICK_NS) & 0xFFFF)
 * computed on demand when a frame is stamped or a REC_TS is emitted. This keeps
 * the mapping exact-in-the-large while avoiding ISR jitter.
 *
 * THREADING: pcan_time_now16() is reentrant and lock-free (reads a monotonic
 * clock only). The per-batch helper state (first-vs-subsequent timestamp
 * width) is NOT global — it lives in the encoder's batch context (see
 * pcan_msg.h pcan_batch_t), so multiple encode passes never interfere.
 * ============================================================================
 */
#ifndef PCAN_TIME_H
#define PCAN_TIME_H

#include <stdint.h>
#include <stdbool.h>
#include "pcan_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize the timestamp engine (records the boot epoch). Call once. */
void pcan_time_init(void);

/*
 * Current 16-bit device tick, wrapping every ~2.796 s. Reentrant. This is the
 * value stored into pcan_frame_t.ts16 at RX time and emitted in REC_TS records.
 */
uint16_t pcan_time_now16(void);

/*
 * Full 64-bit microsecond clock since pcan_time_init(), for internal use
 * (cadence timers, ISO-TP millis hook). Monotonic.
 */
uint64_t pcan_time_now_us(void);

/*
 * Convert a raw 16-bit device tick to microseconds using the host's exact
 * formula (ticks * PCAN_USB_TS_US_PER_TICK >> PCAN_USB_TS_DIV_SHIFTER). Useful
 * for logging/verification; the host does this itself.
 */
uint32_t pcan_time_ticks_to_us(uint16_t ticks);

/* -------------------------------------------------------------------------
 * Batch timestamp-width tracker.
 * Within one EP2-IN batch the FIRST timestamped record carries a 2-byte tick
 * and every subsequent one carries a 1-byte (low) tick. pcan_msg drives an
 * instance of this while packing a batch.
 * ------------------------------------------------------------------------- */
typedef struct {
    bool     have_first;  /* set once the first ts has been written           */
    uint16_t last_ts16;   /* last full 16-bit value written                   */
} pcan_ts_batch_t;

/* Reset the tracker at the start of each batch. */
static inline void pcan_ts_batch_reset(pcan_ts_batch_t *b) {
    b->have_first = false;
    b->last_ts16 = 0;
}

/*
 * Append the correct timestamp encoding for `ts16` to `dst`, advancing and
 * returning the write pointer. Writes 2 bytes (LE) for the first timestamped
 * record of the batch, 1 byte (low) thereafter. Updates the tracker.
 * `dst` must have >=2 bytes of headroom.
 */
uint8_t *pcan_ts_append(pcan_ts_batch_t *b, uint8_t *dst, uint16_t ts16);

#ifdef __cplusplus
}
#endif
#endif /* PCAN_TIME_H */
