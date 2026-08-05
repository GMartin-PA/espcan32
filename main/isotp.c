/*
 * SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
/*
 * isotp.c — ISO-TP (ISO 15765-2) transport, classic CAN, Normal 11-bit
 * ============================================================================
 * Bench-mode transport. Single link instance, poll-driven, single task.
 *
 * Scope (per isotp.h): classic CAN, DLC<=8, Normal addressing (byte0 = N_PCI,
 * no address-extension byte). FF_DL capped at ISOTP_MAX_MSG (no 32-bit escape).
 *
 * All of our transmissions (SF/FF/CF responses and the FC we send as a
 * receiver) go out on tx_id. Frames arrive on two addresses with very different
 * trust: rx_id is a point-to-point conversation and drives the whole state
 * machine, while func_id is a broadcast every node on the bus can write, so it
 * carries SingleFrames only (ISO 15765-2). That split is what keeps a stranger's
 * FC or CF from steering a physically addressed transfer.
 * ============================================================================
 */
#include "isotp.h"
#include <string.h>

/* ---- N_PCI helpers -------------------------------------------------------- */
#define PCI_TYPE(b0)   ((uint8_t)((b0) >> 4))
#define PCI_SF_LEN(b0) ((uint8_t)((b0) & 0x0Fu))
#define PCI_CF_SN(b0)  ((uint8_t)((b0) & 0x0Fu))
#define PCI_FC_FS(b0)  ((uint8_t)((b0) & 0x0Fu))

#define SF_MAX_DATA    7u   /* classic SF (DL<=8) payload cap                   */
#define FF_FIRST_DATA  6u   /* payload bytes carried in a First Frame           */
#define CF_MAX_DATA    7u   /* payload bytes carried in a Consecutive Frame     */

/* Default timing / flow-control values applied when cfg is NULL or a timer
 * field is left at 0 (per isotp.h: "0 = use default" for the ms fields). */
#define DEF_N_MS       1000u
#define DEF_WFT_MAX    8u

/* ---- small utilities ------------------------------------------------------ */

static inline uint32_t now_ms(isotp_link_t *l)
{
    return l->hal.millis ? l->hal.millis(l->hal.user) : 0u;
}

/* Elapsed-since with unsigned wrap safety (uint32 monotonic ms). */
static inline uint32_t elapsed(uint32_t now, uint32_t start)
{
    return now - start;   /* well-defined modular subtraction on uint32_t      */
}

/*
 * Decode a raw ISO STmin byte into whole milliseconds for our ms-granularity
 * pacing. 0x00..0x7F = that many ms. 0xF1..0xF9 = 100..900 us (sub-ms); we
 * clamp those to 1 ms since the bench poll tick is ~1 ms. All other (reserved)
 * values map to the safe maximum 0x7F ms (ISO 15765-2 recovery behaviour).
 */
static uint32_t stmin_to_ms(uint8_t raw)
{
    if (raw <= 0x7Fu)                 return raw;
    if (raw >= 0xF1u && raw <= 0xF9u) return 1u;
    return 0x7Fu;
}

/*
 * Transmit one frame on tx_id. `len` is the number of significant bytes already
 * placed in buf[0..len-1]; if padding is enabled (pad_byte != 0xFF) the frame
 * is grown to 8 bytes with the pad byte. Returns the hal.can_tx result (0 ok).
 */
static int tx_frame(isotp_link_t *l, uint8_t *buf, uint8_t len)
{
    uint8_t dlc = len;
    if (l->cfg.pad_byte != 0xFFu) {
        while (dlc < ISOTP_CAN_DL) {
            buf[dlc++] = l->cfg.pad_byte;
        }
    }
    if (!l->hal.can_tx) {
        return -1;
    }
    return l->hal.can_tx(l->tx_id, buf, dlc, l->hal.user);
}

/* Send a Flow Control frame from the receive side. */
static int send_fc(isotp_link_t *l, isotp_fs_t fs)
{
    uint8_t f[ISOTP_CAN_DL];
    f[0] = (uint8_t)((ISOTP_PCI_FC << 4) | ((uint8_t)fs & 0x0Fu));
    f[1] = l->cfg.bs;
    f[2] = l->cfg.stmin;
    return tx_frame(l, f, 3u);
}

static void tx_reset(isotp_link_t *l)
{
    l->tx_phase   = ISOTP_IDLE;
    l->tx_len     = 0;
    l->tx_off     = 0;
    l->tx_sn      = 0;
    l->tx_bs_left = 0;
    l->tx_stmin   = 0;
    l->tx_wft     = 0;
}

static void rx_reset(isotp_link_t *l)
{
    l->rx_phase    = ISOTP_IDLE;
    l->rx_len      = 0;
    l->rx_off      = 0;
    l->rx_sn       = 0;
    l->rx_block_cf = 0;
}

/* ========================================================================== */
/*  init                                                                       */
/* ========================================================================== */
void isotp_init(isotp_link_t *l, uint32_t tx_id, uint32_t rx_id, uint32_t func_id,
                const isotp_cfg_t *cfg, const isotp_hal_t *hal)
{
    if (!l) {
        return;
    }
    memset(l, 0, sizeof(*l));
    l->tx_id   = tx_id;
    l->rx_id   = rx_id;
    l->func_id = func_id;

    if (cfg) {
        l->cfg = *cfg;
    }
    /* Apply defaults to any unset timer fields (0 = use default).
     * NOTE: N_As/N_Ar (transmit-confirmation timeouts) are stored for
     * completeness but NOT enforced: hal.can_tx is treated as synchronous (it
     * returns the queue/transmit result inline), so there is no async TX-
     * confirmation event to time. Only N_Bs (await-FC) and N_Cr (await-CF) are
     * actively enforced in isotp_poll(). A stuck bus surfaces as can_tx failures
     * (which abort the transfer) or an N_Bs/N_Cr timeout on the peer. */
    if (l->cfg.n_as_ms == 0) l->cfg.n_as_ms = DEF_N_MS;
    if (l->cfg.n_ar_ms == 0) l->cfg.n_ar_ms = DEF_N_MS;
    if (l->cfg.n_bs_ms == 0) l->cfg.n_bs_ms = DEF_N_MS;
    if (l->cfg.n_cr_ms == 0) l->cfg.n_cr_ms = DEF_N_MS;
    if (l->cfg.wft_max == 0) l->cfg.wft_max = DEF_WFT_MAX;
    if (!cfg) {
        /* No config at all: default to the common 0xCC pad and unlimited BS. */
        l->cfg.pad_byte = 0xCCu;
        l->cfg.bs       = 0;
        l->cfg.stmin    = 0;
    }

    if (hal) {
        l->hal = *hal;
    }

    l->tx_phase      = ISOTP_IDLE;
    l->rx_phase      = ISOTP_IDLE;
    l->rx_complete   = false;
    l->rx_functional = false;
    l->rx_result     = ISOTP_OK;
}

/* ========================================================================== */
/*  send                                                                       */
/* ========================================================================== */
isotp_ret_t isotp_send(isotp_link_t *l, const uint8_t *data, uint16_t size)
{
    if (!l || (!data && size)) {
        return ISOTP_ERROR;
    }
    if (l->tx_phase != ISOTP_IDLE) {
        return ISOTP_NOSPACE;               /* a send is already active        */
    }
    if (size == 0 || size > ISOTP_MAX_MSG) {
        return ISOTP_LENGTH;
    }

    memcpy(l->tx_buf, data, size);
    l->tx_len = size;
    l->tx_off = 0;
    l->tx_wft = 0;

    if (size <= SF_MAX_DATA) {
        /* Single Frame: [0]=0x0L, then L data bytes. */
        uint8_t f[ISOTP_CAN_DL];
        f[0] = (uint8_t)((ISOTP_PCI_SF << 4) | (size & 0x0Fu));
        memcpy(&f[1], l->tx_buf, size);
        if (tx_frame(l, f, (uint8_t)(1u + size)) != 0) {
            tx_reset(l);
            return ISOTP_ERROR;
        }
        tx_reset(l);
        return ISOTP_OK;
    }

    /* First Frame: [0]=0x1 | (FF_DL>>8), [1]=FF_DL&0xFF, then 6 data bytes. */
    {
        uint8_t f[ISOTP_CAN_DL];
        f[0] = (uint8_t)((ISOTP_PCI_FF << 4) | ((size >> 8) & 0x0Fu));
        f[1] = (uint8_t)(size & 0xFFu);
        memcpy(&f[2], l->tx_buf, FF_FIRST_DATA);
        if (tx_frame(l, f, ISOTP_CAN_DL) != 0) {
            tx_reset(l);
            return ISOTP_ERROR;
        }
        l->tx_off   = FF_FIRST_DATA;
        l->tx_sn    = 1;                     /* first CF carries SN=1           */
        l->tx_phase = ISOTP_WAIT_FC;
        l->tx_timer = now_ms(l);             /* N_Bs: FF sent -> await FC       */
    }
    return ISOTP_INPROGRESS;
}

/* ========================================================================== */
/*  on_can_frame — SF/FF/CF (receive side) and FC (send side)                  */
/* ========================================================================== */

static void handle_sf(isotp_link_t *l, const uint8_t *data, uint8_t dlc,
                      bool functional)
{
    uint8_t len = PCI_SF_LEN(data[0]);
    /* len==0 would be the CAN-FD SF escape (length in byte1) — unsupported.
     * len>7 is invalid for a classic 8-byte SF. Reject either way. */
    if (len == 0 || len > SF_MAX_DATA) {
        return;
    }
    if ((uint16_t)len > (uint16_t)(dlc - 1)) {
        return;                              /* frame too short to hold payload */
    }
    /* A broadcast belongs to a different addressing context than the one-to-one
     * conversation on rx_id, so it must not tear down a reassembly in flight or
     * overwrite a message the application has not collected yet. */
    if (functional && (l->rx_phase == ISOTP_RECV || l->rx_complete)) {
        return;
    }
    /* A physically addressed SF supersedes any in-progress reassembly. */
    memcpy(l->rx_buf, &data[1], len);
    l->rx_len        = len;
    l->rx_off        = len;
    l->rx_phase      = ISOTP_IDLE;
    l->rx_result     = ISOTP_OK;
    l->rx_functional = functional;
    l->rx_complete   = true;
}

static void handle_ff(isotp_link_t *l, const uint8_t *data, uint8_t dlc)
{
    /* A conformant FF always fills the frame: 2 PCI bytes + 6 payload. Anything
     * shorter carries fewer than FF_FIRST_DATA bytes, which would offset every
     * CF that follows from the block boundaries we grant. */
    if (dlc != ISOTP_CAN_DL) {
        return;
    }
    uint16_t ff_dl = (uint16_t)(((uint16_t)(data[0] & 0x0Fu) << 8) | data[1]);

    /* FF must describe a multi-frame message (> 7). ff_dl==0 with byte1==0 is
     * the 32-bit length escape (unsupported / always exceeds our cap). */
    if (ff_dl <= SF_MAX_DATA) {
        return;                              /* malformed FF                    */
    }
    if (ff_dl > ISOTP_MAX_MSG) {
        /* Cannot buffer: tell the sender we overflowed and stay idle. */
        send_fc(l, ISOTP_FS_OVFLW);
        rx_reset(l);
        return;
    }

    memcpy(l->rx_buf, &data[2], FF_FIRST_DATA);
    l->rx_len        = ff_dl;
    l->rx_off        = FF_FIRST_DATA;
    l->rx_sn         = 1;                    /* next CF must carry SN=1         */
    l->rx_block_cf   = 0;
    l->rx_phase      = ISOTP_RECV;
    l->rx_result     = ISOTP_OK;
    l->rx_functional = false;                /* FF only ever arrives physically */
    l->rx_timer      = now_ms(l);            /* start N_Cr                      */

    /* Grant the first block. */
    send_fc(l, ISOTP_FS_CTS);
}

static void handle_cf(isotp_link_t *l, const uint8_t *data, uint8_t dlc)
{
    if (l->rx_phase != ISOTP_RECV) {
        return;                              /* unexpected CF: ignore           */
    }
    uint8_t sn = PCI_CF_SN(data[0]);
    if (sn != l->rx_sn) {
        /* Sequence error: abort this reception (ISO 15765-2). */
        l->rx_result = ISOTP_WRONG_SN;
        rx_reset(l);
        return;
    }

    uint16_t remaining = (uint16_t)(l->rx_len - l->rx_off);
    uint8_t  avail     = (dlc > 1) ? (uint8_t)(dlc - 1) : 0u;

    /* A non-final CF MUST carry a full 7-byte payload (ISO 15765-2). A short
     * (unpadded) mid-block CF both corrupts reassembly and desyncs the block-
     * size FC accounting below, so treat it as a protocol error and abort. Only
     * the final CF of a message may legitimately be short. */
    if (remaining > CF_MAX_DATA && avail < CF_MAX_DATA) {
        l->rx_result = ISOTP_LENGTH;
        rx_reset(l);
        return;
    }

    uint16_t take      = remaining < CF_MAX_DATA ? remaining : CF_MAX_DATA;
    if (take > avail) {
        take = avail;                        /* don't read past frame end       */
    }
    memcpy(&l->rx_buf[l->rx_off], &data[1], take);
    l->rx_off      = (uint16_t)(l->rx_off + take);
    l->rx_sn       = (uint8_t)((l->rx_sn + 1) & 0x0Fu);
    l->rx_block_cf = (uint8_t)(l->rx_block_cf + 1u);
    l->rx_timer    = now_ms(l);              /* restart N_Cr                    */

    if (l->rx_off >= l->rx_len) {
        l->rx_result   = ISOTP_OK;
        l->rx_complete = true;
        l->rx_phase    = ISOTP_IDLE;
        return;
    }

    /* Block-size flow control: the last CTS granted cfg.bs CFs, so once that
     * many have arrived we owe the sender the next grant. cfg.bs == 0 means
     * unlimited — the FF's CTS covers the whole message and no more FCs go out
     * (rx_block_cf then just counts on, unused). */
    if (l->cfg.bs != 0 && l->rx_block_cf >= l->cfg.bs) {
        l->rx_block_cf = 0;
        send_fc(l, ISOTP_FS_CTS);            /* open the next block             */
    }
}

static void handle_fc(isotp_link_t *l, const uint8_t *data, uint8_t dlc)
{
    if (l->tx_phase != ISOTP_WAIT_FC) {
        return;                              /* no send awaiting a FC           */
    }
    isotp_fs_t fs = (isotp_fs_t)PCI_FC_FS(data[0]);

    switch (fs) {
    case ISOTP_FS_CTS:
        l->tx_bs_left = (dlc > 1) ? data[1] : 0u;   /* 0 = whole message        */
        l->tx_stmin   = (dlc > 2) ? data[2] : 0u;
        l->tx_wft     = 0;
        l->tx_phase   = ISOTP_SEND;
        l->tx_st_timer = now_ms(l);           /* first CF paced from here       */
        break;

    case ISOTP_FS_WAIT:
        if (++l->tx_wft > l->cfg.wft_max) {
            tx_reset(l);                      /* too many WAITs: give up         */
        } else {
            l->tx_timer = now_ms(l);          /* restart N_Bs                    */
        }
        break;

    case ISOTP_FS_OVFLW:
    default:
        tx_reset(l);                          /* receiver overflow / reserved    */
        break;
    }
}

void isotp_on_can_frame(isotp_link_t *l, uint32_t id,
                        const uint8_t *data, uint8_t dlc)
{
    if (!l || !data || dlc < 1 || dlc > ISOTP_CAN_DL) {
        return;
    }

    bool functional;
    if (id == l->rx_id) {
        functional = false;
    } else if (l->func_id != ISOTP_NO_FUNC_ID && id == l->func_id) {
        functional = true;
    } else {
        return;                              /* not addressed to us            */
    }

    switch (PCI_TYPE(data[0])) {
    case ISOTP_PCI_SF: handle_sf(l, data, dlc, functional); break;
    /* FF/CF/FC only ever belong to a physical conversation: honouring them from
     * the broadcast address would let any node overflow, stall or hijack a
     * transfer it is not party to. */
    case ISOTP_PCI_FF: if (!functional) handle_ff(l, data, dlc); break;
    case ISOTP_PCI_CF: if (!functional) handle_cf(l, data, dlc); break;
    case ISOTP_PCI_FC: if (!functional) handle_fc(l, data, dlc); break;
    default:           /* unknown N_PCI: ignore */                 break;
    }
}

/* ========================================================================== */
/*  poll — CF pacing (send side) + N_Bs / N_Cr timeouts                        */
/* ========================================================================== */
void isotp_poll(isotp_link_t *l)
{
    if (!l) {
        return;
    }
    uint32_t now = now_ms(l);

    /* ---- send side ------------------------------------------------------- */
    if (l->tx_phase == ISOTP_WAIT_FC) {
        if (elapsed(now, l->tx_timer) >= l->cfg.n_bs_ms) {
            tx_reset(l);                      /* N_Bs timeout                    */
        }
    } else if (l->tx_phase == ISOTP_SEND) {
        uint32_t st = stmin_to_ms(l->tx_stmin);
        if (elapsed(now, l->tx_st_timer) >= st) {
            uint8_t  f[ISOTP_CAN_DL];
            uint16_t remaining = (uint16_t)(l->tx_len - l->tx_off);
            uint8_t  n = remaining < CF_MAX_DATA ? (uint8_t)remaining : CF_MAX_DATA;

            f[0] = (uint8_t)((ISOTP_PCI_CF << 4) | (l->tx_sn & 0x0Fu));
            memcpy(&f[1], &l->tx_buf[l->tx_off], n);

            if (tx_frame(l, f, (uint8_t)(1u + n)) != 0) {
                tx_reset(l);                  /* transmit failure: abort         */
                return;
            }

            l->tx_off      = (uint16_t)(l->tx_off + n);
            l->tx_sn       = (uint8_t)((l->tx_sn + 1) & 0x0Fu);
            l->tx_st_timer = now;

            if (l->tx_off >= l->tx_len) {
                tx_reset(l);                  /* whole message sent              */
                return;
            }
            /* Finite block: after BS CFs, wait for the next FC. */
            if (l->tx_bs_left != 0) {
                if (--l->tx_bs_left == 0) {
                    l->tx_phase = ISOTP_WAIT_FC;
                    l->tx_timer = now;        /* start N_Bs                       */
                }
            }
        }
    }

    /* ---- receive side ---------------------------------------------------- */
    if (l->rx_phase == ISOTP_RECV) {
        if (elapsed(now, l->rx_timer) >= l->cfg.n_cr_ms) {
            l->rx_result = ISOTP_TIMEOUT;     /* N_Cr timeout                    */
            rx_reset(l);
        }
    }
}

/* ========================================================================== */
/*  receive — deliver a completed PDU                                          */
/* ========================================================================== */
isotp_ret_t isotp_receive(isotp_link_t *l, uint8_t *out, uint16_t out_cap,
                          uint16_t *out_len, bool *out_functional)
{
    if (out_len) {
        *out_len = 0;
    }
    if (out_functional) {
        *out_functional = false;
    }
    if (!l || !out) {
        return ISOTP_ERROR;
    }
    if (!l->rx_complete) {
        return ISOTP_NO_DATA;
    }
    if (l->rx_len > out_cap) {
        return ISOTP_NOSPACE;                 /* leave rx_complete set: retry    */
    }
    memcpy(out, l->rx_buf, l->rx_len);
    if (out_len) {
        *out_len = l->rx_len;
    }
    if (out_functional) {
        *out_functional = l->rx_functional;
    }
    l->rx_complete = false;
    return ISOTP_OK;
}
