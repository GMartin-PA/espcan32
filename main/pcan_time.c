/*
 * SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
/*
 * pcan_time.c — device timestamp engine + calibration
 * ============================================================================
 * See pcan_time.h and pcan_proto.h §4.
 *
 * The device emulates a free-running 16-bit tick counter advancing at
 * ~PCAN_USB_TS_TICK_NS per tick (~42.667 us). Rather than run a real ~23.4 kHz
 * ISR, we derive the tick on demand from esp_timer_get_time() (monotonic
 * microseconds since boot):
 *
 *   ns_since_epoch = (now_us - epoch_us) * 1000
 *   ticks64        = ns_since_epoch / PCAN_USB_TS_TICK_NS
 *   tick16         = ticks64 & 0xFFFF
 *
 * All arithmetic is integer-only. The 16-bit wrap is handled by masking the
 * unbounded 64-bit tick count, which is naturally modular: consecutive samples
 * differ by the true elapsed tick delta modulo 2^16, so the host reconstructs
 * the high byte on low-byte wrap exactly as it would from real hardware.
 * ============================================================================
 */
#include "pcan_time.h"
#include "esp_timer.h"

/* esp_timer value (microseconds) captured at pcan_time_init(). All device
 * ticks are measured relative to this epoch so the counter starts near 0. */
static uint64_t s_epoch_us;

/* Microseconds elapsed since the epoch. esp_timer_get_time() is monotonic and
 * returns int64_t microseconds since boot; it never goes backwards and cannot
 * predate the epoch we captured, so the subtraction is non-negative. */
static inline uint64_t pcan_time_elapsed_us(void)
{
    uint64_t now = (uint64_t)esp_timer_get_time();
    /* Defensive: if the clock ever appeared to run backwards (it should not),
     * clamp to 0 rather than wrap to a huge unsigned value. */
    if (now < s_epoch_us) {
        return 0;
    }
    return now - s_epoch_us;
}

/*
 * Full 64-bit device tick count since the epoch (unbounded, monotonic).
 *
 * ns = us * 1000 does not overflow uint64_t for any realistic uptime:
 * 2^64 ns / 1000 ~= 1.8e16 us ~= 584 years. The division yields the exact
 * tick count; callers mask to 16 bits for the wire value.
 */
static inline uint64_t pcan_time_now_ticks64(void)
{
    uint64_t us = pcan_time_elapsed_us();
    uint64_t ns = us * 1000ULL;
    return ns / (uint64_t)PCAN_USB_TS_TICK_NS;
}

void pcan_time_init(void)
{
    s_epoch_us = (uint64_t)esp_timer_get_time();
}

uint16_t pcan_time_now16(void)
{
    return (uint16_t)(pcan_time_now_ticks64() & 0xFFFFu);
}

uint64_t pcan_time_now_us(void)
{
    return pcan_time_elapsed_us();
}

uint32_t pcan_time_ticks_to_us(uint16_t ticks)
{
    /* Host's exact fixed-point conversion (pcan_proto.h §4). */
    return (uint32_t)(((uint64_t)ticks * PCAN_USB_TS_US_PER_TICK) >>
                      PCAN_USB_TS_DIV_SHIFTER);
}

uint8_t *pcan_ts_append(pcan_ts_batch_t *b, uint8_t *dst, uint16_t ts16)
{
    if (!b->have_first) {
        /* First timestamped record of the batch: full 2-byte LE tick. */
        pcan_put_le16(dst, ts16);
        dst += 2;
        b->have_first = true;
    } else {
        /* Subsequent records: low byte only. The host reconstructs the high
         * byte from the running reference, detecting wrap on the low byte. */
        *dst++ = (uint8_t)(ts16 & 0xFFu);
    }
    b->last_ts16 = ts16;
    return dst;
}
