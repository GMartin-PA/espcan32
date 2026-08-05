/*
 * SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
/*
 * isotp.h — ISO-TP (ISO 15765-2) transport, classic CAN, Normal 11-bit
 * ============================================================================
 * Standalone bench-mode transport (independent of the PCAN-USB path). Runs a
 * per-direction state machine over the TWAI driver: SF/FF/CF/FC framing, block
 * size + STmin flow control, and the N_As/N_Ar/N_Bs/N_Cr timers.
 *
 * Scope: classic CAN, DLC<=8, Normal addressing (byte 0 = N_PCI, no address
 * extension byte). FF_DL capped at ISOTP_MAX_MSG (no 2016 32-bit escape).
 *
 * OWNERSHIP / BUFFERS: each isotp_link_t carries its own static tx_buf/rx_buf
 * sized to ISOTP_MAX_MSG — no malloc. isotp_send() copies the caller's payload
 * in; isotp_receive() copies the reassembled message out into caller storage.
 *
 * THREADING: single-task, poll-driven. All four entry points (init/send/
 * on_can_frame/poll/receive) run on the ONE bench task; none are ISR-safe. The
 * task pumps: twai_receive -> isotp_on_can_frame -> isotp_poll -> (uds).
 * ============================================================================
 */
#ifndef PCAN_ISOTP_H
#define PCAN_ISOTP_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ISOTP_CAN_DL          8       /* classic CAN frame data length          */
#define ISOTP_MAX_MSG         4095    /* FF_DL cap; sized to RAM budget          */

/* Sentinel for isotp_link_t.func_id: this link has no functional address. Not a
 * valid CAN identifier, so it can never match a received frame. */
#define ISOTP_NO_FUNC_ID      0xFFFFFFFFu

/* N_PCI frame types (high nibble of byte 0). */
#define ISOTP_PCI_SF          0x0u
#define ISOTP_PCI_FF          0x1u
#define ISOTP_PCI_CF          0x2u
#define ISOTP_PCI_FC          0x3u

/* Flow-control status (low nibble of FC byte 0). */
typedef enum {
    ISOTP_FS_CTS   = 0x0,  /* continue to send                                 */
    ISOTP_FS_WAIT  = 0x1,  /* hold, wait for next FC                           */
    ISOTP_FS_OVFLW = 0x2,  /* receiver overflow, abort                         */
} isotp_fs_t;

/* Return / protocol result codes (mirror driftregion/iso14229 isotp-c). */
typedef enum {
    ISOTP_OK         =  0,
    ISOTP_ERROR      = -1,
    ISOTP_INPROGRESS = -2,
    ISOTP_OVERFLOW   = -3,
    ISOTP_WRONG_SN   = -4,
    ISOTP_NO_DATA    = -5,
    ISOTP_TIMEOUT    = -6,
    ISOTP_LENGTH     = -7,
    ISOTP_NOSPACE    = -8,
} isotp_ret_t;

/* Timing / flow-control configuration (ms unless noted; 0 = use default). */
typedef struct {
    uint32_t n_as_ms;   /* sender frame-tx timeout   (default 1000)            */
    uint32_t n_ar_ms;   /* receiver frame-tx timeout (default 1000)            */
    uint32_t n_bs_ms;   /* sender: block sent -> FC received (default 1000)    */
    uint32_t n_cr_ms;   /* receiver: CF -> next CF (default 1000)              */
    uint8_t  bs;        /* BlockSize we advertise as receiver (0 = unlimited)  */
    uint8_t  stmin;     /* STmin we advertise (raw ISO byte: ms or 0xF1..0xF9) */
    uint8_t  wft_max;   /* max WAIT FCs we tolerate as sender                  */
    uint8_t  pad_byte;  /* CAN pad byte (0xCC common); 0xFF = no padding       */
} isotp_cfg_t;

/*
 * Platform hooks — implemented in isotp.c over twai_hal + pcan_time.
 *   can_tx: transmit one classic CAN frame (id, data, dlc). Return 0 on
 *           success, non-zero on failure. Must not block for long.
 *   millis: monotonic milliseconds (derive from pcan_time_now_us()/1000).
 */
typedef struct {
    int      (*can_tx)(uint32_t id, const uint8_t *data, uint8_t dlc, void *user);
    uint32_t (*millis)(void *user);
    void     *user;
} isotp_hal_t;

/* Internal send/receive sub-state machine phases (opaque to callers). */
typedef enum { ISOTP_IDLE = 0, ISOTP_SEND, ISOTP_WAIT_FC, ISOTP_RECV } isotp_phase_t;

typedef struct isotp_link {
    uint32_t     tx_id;     /* physical response id we transmit on             */
    uint32_t     rx_id;     /* physical request id we accept                   */
    uint32_t     func_id;   /* functional request id (SF only), or NO_FUNC_ID  */
    isotp_cfg_t  cfg;
    isotp_hal_t  hal;

    /* send side */
    uint8_t   tx_buf[ISOTP_MAX_MSG];
    uint16_t  tx_len, tx_off;
    uint8_t   tx_sn;
    uint8_t   tx_bs_left;
    uint8_t   tx_stmin;      /* STmin dictated by the peer's FC                */
    uint8_t   tx_wft;        /* consecutive WAIT FCs seen                      */
    isotp_phase_t tx_phase;
    uint32_t  tx_timer, tx_st_timer;

    /* receive side */
    uint8_t   rx_buf[ISOTP_MAX_MSG];
    uint16_t  rx_len, rx_off;
    uint8_t   rx_sn;
    uint8_t   rx_block_cf;   /* CFs taken in the block the last CTS granted    */
    isotp_phase_t rx_phase;
    uint32_t  rx_timer;
    int       rx_result;     /* isotp_ret_t on completion of a message         */
    bool      rx_complete;   /* a full message is ready for isotp_receive()    */
    bool      rx_functional; /* completed message arrived on func_id           */
} isotp_link_t;

/*
 * Initialize a link. `tx_id`/`rx_id` are the physical response/request CAN ids.
 * `func_id` is the functional (broadcast) request id, or ISOTP_NO_FUNC_ID for
 * none: functional addressing carries SingleFrames only (ISO 15765-2), so every
 * other N_PCI type seen on it is ignored and a broadcast can neither
 * flow-control nor abort a physically addressed transfer. `cfg` may be NULL for
 * all-defaults. `hal` is required. Zeroes buffers.
 */
void isotp_init(isotp_link_t *l, uint32_t tx_id, uint32_t rx_id, uint32_t func_id,
                const isotp_cfg_t *cfg, const isotp_hal_t *hal);

/*
 * Begin sending `size` bytes (<= ISOTP_MAX_MSG). Copies into the link's tx_buf
 * and starts the SF or FF/CF sequence. Returns ISOTP_INPROGRESS while multi-
 * frame transfer proceeds (driven by isotp_poll), ISOTP_OK for a single-frame
 * message already emitted, or a negative code on error (e.g. ISOTP_NOSPACE if
 * a send is already active).
 */
isotp_ret_t isotp_send(isotp_link_t *l, const uint8_t *data, uint16_t size);

/*
 * Feed one received CAN frame. Frames on rx_id advance the full receive state
 * machine (SF/FF -> reassembly, CF accumulation, FC handling for the send
 * side); frames on func_id are accepted as SingleFrames only; any other id is
 * ignored. Non-blocking, so the caller may hand it every frame off the bus.
 */
void isotp_on_can_frame(isotp_link_t *l, uint32_t id,
                        const uint8_t *data, uint8_t dlc);

/*
 * Drive time-based transitions: CF pacing/STmin on the send side, N_Bs/N_Cr
 * timeouts, sending FCs on the receive side. Call every task tick (~1 ms).
 */
void isotp_poll(isotp_link_t *l);

/*
 * If a full message has been reassembled, copy up to `out_cap` bytes into
 * `out`, set *out_len, report through *out_functional whether it arrived on the
 * functional address, clear the ready flag, and return ISOTP_OK. Returns
 * ISOTP_NO_DATA if nothing is ready, ISOTP_NOSPACE if out_cap is too small.
 * `out_len` and `out_functional` may be NULL.
 */
isotp_ret_t isotp_receive(isotp_link_t *l, uint8_t *out, uint16_t out_cap,
                          uint16_t *out_len, bool *out_functional);

#ifdef __cplusplus
}
#endif
#endif /* PCAN_ISOTP_H */
