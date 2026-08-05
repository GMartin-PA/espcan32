/*
 * SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
/*
 * uds.c — minimal UDS (ISO 14229-1) server over ISO-TP (bench mode)
 * ============================================================================
 * Table-driven service dispatcher. See uds.h for the public contract.
 *
 * Design notes / assumptions:
 *  - uds_ctx_t carries no back-pointer to uds_server_t, so the built-in
 *    handlers keep their persistent state (SecurityAccess seed/level and the
 *    RequestDownload/TransferData session) in this translation unit's static
 *    state. The bench firmware runs a SINGLE uds_server_t, so a single set of
 *    file-static state is correct here; it is NOT multi-instance safe. The
 *    dispatcher mirrors the security state back into s->sec_level / s->sec_seed
 *    each poll so those fields stay observable.
 *  - Session changes are propagated cleanly through the ctx: a handler writes
 *    the new session into c->session on a POSITIVE result and the dispatcher
 *    commits it into s->session (and re-locks the server on change).
 *  - isotp_receive() reports whether the request arrived on the physical or the
 *    functional address; that drives uds_ctx_t.functional and the §A.2 rule that
 *    some NRCs are answered with silence on a broadcast.
 * ============================================================================
 */
#include "uds.h"

#include "esp_system.h"        /* esp_restart() for ECUReset               */
#include "esp_random.h"        /* esp_random() for SecurityAccess seed     */
#include "freertos/FreeRTOS.h" /* vTaskDelay before a deferred reset       */
#include "freertos/task.h"
#include <string.h>

/* ---- Tunables ------------------------------------------------------------ */

/* SecurityAccess key transform: expected_key = seed ^ UDS_SEC_KEY_XOR.
 * Configurable — swap for the real ECU algorithm on the bench. */
#define UDS_SEC_KEY_XOR        0x5A5A5A5Au

/* SecurityAccess anti-brute-force: after this many consecutive invalid keys the
 * server locks out for UDS_SEC_LOCK_MS, rejecting seed/key with NRC 0x37
 * (requiredTimeDelayNotExpired) meanwhile (ISO 14229-1 §9.4). */
#define UDS_SEC_MAX_ATTEMPTS   3u
#define UDS_SEC_LOCK_MS        10000u

/* SecurityAccess levels this bench ECU implements. A level is the odd
 * requestSeed sub-function; sendKey is the even value that follows it. One
 * level is enough for a bench target — add more here (and to
 * uds_sec_level_supported) rather than accepting whatever a tester invents,
 * since an unimplemented "level" would otherwise unlock the gated services. */
#define UDS_SEC_LEVEL_1        0x01u

/* Level that authorizes the download services (0x34/0x36/0x37). */
#define UDS_SEC_LEVEL_DOWNLOAD UDS_SEC_LEVEL_1

/* maxNumberOfBlockLength reported by RequestDownload (0x74). Per ISO 14229-1
 * this counts the COMPLETE TransferData message including SID + blockSequence-
 * Counter (2 bytes), not just the data. Kept well within ISOTP_MAX_MSG so a
 * single TransferData message always fits one ISO-TP msg. */
#define UDS_MAX_BLOCK_LEN      0x0402u

/* Wrong block-sequence-counter NRC (ISO 14229 0x73). Not enumerated in uds.h;
 * defined locally so TransferData can report it faithfully. */
#define UDS_NRC_WRONG_BSC      0x73u

/* Upper bound on 0x78 ResponsePending iterations before we give up, to keep a
 * misbehaving handler from wedging the bench task. Built-in handlers never
 * return UDS_RESPONSE_PENDING, so this only guards custom handlers. */
#define UDS_MAX_PENDING_ITERS  8u

/* Delay after emitting an ECUReset positive response before esp_restart(), to
 * let the ISO-TP single frame flush out of the TWAI TX queue onto the wire. */
#define UDS_RESET_DELAY_MS     50u

/* P2/P2* timing reported in the DiagnosticSessionControl positive response.
 * P2 is in ms; P2* is in 10 ms units (ISO 14229-1). Both big-endian. */
#define UDS_P2_ENC             ((uint16_t)UDS_P2_SERVER_MS)          /* 50 -> 0x0032 */
#define UDS_P2STAR_ENC         ((uint16_t)(UDS_P2_STAR_SERVER_MS / 10u)) /* 5000 -> 0x01F4 */

/* ---- File-static state for the built-in handlers ------------------------- */

static uint8_t  g_req_buf[ISOTP_MAX_MSG]; /* reassembled request scratch      */

static uint8_t  g_sec_level;    /* currently-unlocked security level (0=locked) */
static uint8_t  g_sec_pending;  /* level a seed was just issued for (0=none)     */
static uint32_t g_sec_seed;     /* last issued seed                              */
static uint8_t  g_sec_attempts; /* consecutive invalid-key count                 */
static uint32_t g_sec_lock_ms;  /* >0 => locked out; counts down each poll       */

static bool     g_xfer_active;   /* a RequestDownload transfer is open          */
static uint32_t g_xfer_size;     /* declared total transfer size (bytes)        */
static uint32_t g_xfer_received; /* bytes accepted via TransferData             */
static uint8_t  g_xfer_bsc;      /* next expected blockSequenceCounter          */

static bool     g_pending_reset; /* ECUReset asked us to restart after replying */

/* ---- DID table (ReadDataByIdentifier 0x22) ------------------------------- */

typedef struct {
    uint16_t       did;
    const uint8_t *data;
    uint8_t        len;
} uds_did_entry_t;

static const uint8_t k_did_vin[]  = "ZZZBENCH0P0000001"; /* 17 chars, 0xF190  */
static const uint8_t k_did_sppn[] = "PN-BENCH-0001";     /* spare part number  */
static const uint8_t k_did_swv[]  = "BENCH-UDS-1.0.0";   /* application sw ver  */
static const uint8_t k_did_esn[]  = "ESN-0000C0FFEE";    /* ecu serial number  */
static const uint8_t k_did_sysn[] = "ESPCAN32";      /* system name         */

static const uds_did_entry_t k_did_table[] = {
    { 0xF190u, k_did_vin,  sizeof(k_did_vin)  - 1u },
    { 0xF187u, k_did_sppn, sizeof(k_did_sppn) - 1u },
    { 0xF189u, k_did_swv,  sizeof(k_did_swv)  - 1u },
    { 0xF18Cu, k_did_esn,  sizeof(k_did_esn)  - 1u },
    { 0xF197u, k_did_sysn, sizeof(k_did_sysn) - 1u },
};
#define UDS_DID_TABLE_LEN (sizeof(k_did_table) / sizeof(k_did_table[0]))

/* ---- Small helpers ------------------------------------------------------- */

static void uds_put_be16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)((v >> 8) & 0xFFu);
    p[1] = (uint8_t)(v & 0xFFu);
}

/* Hand a message to the transport. False means the link would not take it —
 * ISOTP_NOSPACE while an earlier multi-frame response is still draining, or a
 * bus-level failure. Never blocks the bench task. */
static bool uds_send_raw(uds_server_t *s, const uint8_t *buf, uint16_t len)
{
    const isotp_ret_t r = isotp_send(s->link, buf, len);
    return r == ISOTP_OK || r == ISOTP_INPROGRESS;
}

/* NRCs a server must suppress (send nothing) when the request arrived on the
 * functional address (ISO 14229-1 §A.2). 0x78 is never suppressed. */
static bool uds_nrc_suppressed_on_functional(uint8_t nrc)
{
    switch (nrc) {
        case NRC_SNS:      /* 0x11 */
        case NRC_SFNS:     /* 0x12 */
        case NRC_ROOR:     /* 0x31 */
        case NRC_SFNSIAS:  /* 0x7E */
        case NRC_SNSIAS:   /* 0x7F */
            return true;
        default:
            return false;
    }
}

/* Emit a negative response, staying silent for the NRCs §A.2 requires silence
 * for on a broadcast. Returns true when the NRC went out or was legitimately
 * suppressed. A busy link is the end of the line here: answering an unsendable
 * NRC with another NRC would recurse, so it is counted and dropped. */
static bool uds_send_nrc(uds_server_t *s, uint8_t sid, uint8_t nrc,
                         bool functional)
{
    if (functional && uds_nrc_suppressed_on_functional(nrc)) {
        return true;
    }
    uint8_t b[3] = { UDS_NEG_RESP_SID, sid, nrc };
    if (!uds_send_raw(s, b, sizeof(b))) {
        ++s->resp_dropped;
        return false;
    }
    return true;
}

/* Emit a positive response. A response the transport refuses for a reason that
 * does not also block a 3-byte PDU — an over-long response, say — comes back as
 * busyRepeatRequest, so the tester retries rather than waiting out P2 on a
 * request the server actually accepted. A link still draining an earlier
 * multi-frame response refuses the NRC too, and must: slipping a SingleFrame
 * into a running CF sequence would corrupt the transfer in flight. That case is
 * counted in resp_dropped and left silent. Returns true only if the response
 * itself reached the transport. */
static bool uds_send_response(uds_server_t *s, uint8_t sid,
                              const uint8_t *buf, uint16_t len, bool functional)
{
    if (uds_send_raw(s, buf, len)) {
        return true;
    }
    uds_send_nrc(s, sid, NRC_BRR, functional);
    return false;
}

/* Return to the fully locked state: forget the SecurityAccess unlock and the
 * outstanding seed, and close any open download. The transfer goes with the
 * unlock because RequestTransferExit is itself security-gated — leaving it open
 * would strand the tester in a state it has no service to clear. */
static void uds_relock(uds_server_t *s)
{
    g_sec_level     = 0;
    g_sec_pending   = 0;
    g_xfer_active   = false;
    g_xfer_size     = 0;
    g_xfer_received = 0;
    g_xfer_bsc      = 0;
    s->sec_level    = 0;
}

/* Services carrying a sub-function byte whose bit 7 is the
 * suppressPosRspMsgIndication bit. SecurityAccess (0x27) is deliberately
 * excluded: its sub-function encodes the security level (odd/even), so bit 7
 * must not be treated as a suppress flag. */
static bool uds_sid_has_suppress(uint8_t sid)
{
    switch (sid) {
        case UDS_SID_DIAG_SESSION:
        case UDS_SID_ECU_RESET:
        case UDS_SID_ROUTINE:
        case UDS_SID_TESTER_PRES:
            return true;
        default:
            return false;
    }
}

/* SecurityAccess levels this server actually implements (see UDS_SEC_LEVEL_1). */
static bool uds_sec_level_supported(uint8_t level)
{
    return level == UDS_SEC_LEVEL_1;
}

static const uds_service_t *uds_find_service(const uds_server_t *s, uint8_t sid)
{
    for (uint8_t i = 0; i < s->n_services; ++i) {
        if (s->services[i].sid == sid) {
            return &s->services[i];
        }
    }
    return NULL;
}

/* ========================================================================== */
/*  Public API                                                                 */
/* ========================================================================== */

void uds_server_init(uds_server_t *s, isotp_link_t *link,
                     const uds_service_t *svcs, uint8_t n,
                     uint8_t *resp_buf, uint16_t resp_cap, void *user)
{
    if (s == NULL) {
        return;
    }
    memset(s, 0, sizeof(*s));
    s->link          = link;
    s->services      = svcs;
    s->n_services    = n;
    s->session       = UDS_SESSION_DEFAULT;
    s->s3_timer_ms   = 0;
    s->s3_timeout_ms = UDS_S3_SERVER_MS;
    s->sec_level     = 0;
    s->sec_seed      = 0;
    s->user          = user;
    s->resp_buf      = resp_buf;
    s->resp_cap      = resp_cap;

    /* Reset the built-in-handler state machine (single-instance, see header). */
    g_sec_level     = 0;
    g_sec_pending   = 0;
    g_sec_seed      = 0;
    g_sec_attempts  = 0;
    g_sec_lock_ms   = 0;
    g_xfer_active   = false;
    g_xfer_size     = 0;
    g_xfer_received = 0;
    g_xfer_bsc      = 0;
    g_pending_reset = false;
}

void uds_server_poll(uds_server_t *s, uint32_t elapsed_ms)
{
    if (s == NULL || s->link == NULL) {
        return;
    }

    /* Age the SecurityAccess lockout timer every tick (independent of traffic). */
    if (g_sec_lock_ms > 0) {
        g_sec_lock_ms = (g_sec_lock_ms > elapsed_ms) ? (g_sec_lock_ms - elapsed_ms) : 0u;
    }

    /* 1. Pull a completed request PDU from ISO-TP. */
    uint16_t    req_len    = 0;
    bool        functional = false;
    isotp_ret_t rr = isotp_receive(s->link, g_req_buf, sizeof(g_req_buf),
                                   &req_len, &functional);

    if (rr != ISOTP_OK || req_len == 0) {
        /* No request this tick: age the S3 timer and, on expiry, fall back to
         * the default session and re-lock. The aging runs in every session
         * because SecurityAccess is reachable from the default one too, so an
         * unlock can outlive a session that never changed. */
        s->s3_timer_ms += elapsed_ms;
        if (s->s3_timer_ms >= s->s3_timeout_ms) {
            s->s3_timer_ms = 0;
            s->session     = UDS_SESSION_DEFAULT;
            uds_relock(s);
        }
        return;
    }

    /* Any valid request resets the S3 (tester-present) timer. */
    s->s3_timer_ms = 0;

    const uint8_t sid = g_req_buf[0];

    /* 2. Service lookup + session gating. */
    const uds_service_t *svc = uds_find_service(s, sid);
    if (svc == NULL || svc->fn == NULL) {
        uds_send_nrc(s, sid, NRC_SNS, functional);
        return;
    }
    if (svc->min_session_mask != 0 &&
        !((1u << s->session) & svc->min_session_mask)) {
        uds_send_nrc(s, sid, NRC_SNSIAS, functional);
        return;
    }

    /* 3. Build the per-request context. */
    const bool suppress = uds_sid_has_suppress(sid) && req_len >= 2 &&
                          (g_req_buf[1] & UDS_SUPPRESS_POS_RSP);

    uds_ctx_t c;
    c.req        = g_req_buf;
    c.req_len    = req_len;
    c.resp       = s->resp_buf;
    c.resp_cap   = s->resp_cap;
    c.resp_len   = 0;
    c.nrc        = NRC_GENERAL_REJECT;
    c.functional = functional;
    c.session    = s->session;
    c.user       = s->user;

    /* 4. Dispatch, honouring the 0x78 ResponsePending re-poll pattern. */
    uds_disp_t disp = svc->fn(&c);

    uint32_t pending_iters = 0;
    while (disp == UDS_RESPONSE_PENDING && pending_iters < UDS_MAX_PENDING_ITERS) {
        uds_send_nrc(s, sid, NRC_RCRRP, functional); /* 0x7F <sid> 0x78 */
        c.resp_len = 0;
        c.nrc      = NRC_GENERAL_REJECT;
        disp       = svc->fn(&c);
        ++pending_iters;
    }
    if (disp == UDS_RESPONSE_PENDING) {
        /* Handler never settled: emit a final general reject and bail. */
        uds_send_nrc(s, sid, NRC_GENERAL_REJECT, functional);
        return;
    }

    /* 5. Emit the response. */
    switch (disp) {
        case UDS_POSITIVE: {
            /* Commit any session change requested by the handler. A change of
             * session re-locks, and so does an explicit re-selection of the
             * default session: returning to it resets security even when we
             * were already there (ISO 14229-1). */
            if (c.session != s->session ||
                (sid == UDS_SID_DIAG_SESSION && c.session == UDS_SESSION_DEFAULT)) {
                s->session = c.session;
                uds_relock(s);
            }
            if (!suppress && c.resp_len > 0 &&
                !uds_send_response(s, sid, c.resp, c.resp_len, functional)) {
                /* The tester was told to repeat the request, so the effects it
                 * has not been told about must not happen behind its back. */
                g_pending_reset = false;
            }
            break;
        }
        case UDS_NEGATIVE: {
            uds_send_nrc(s, sid, c.nrc, functional);
            break;
        }
        case UDS_NO_RESPONSE:
        default:
            break;
    }

    /* 6. Mirror the built-in security state into the observable server fields. */
    s->sec_level = g_sec_level;
    s->sec_seed  = g_sec_seed;

    /* 7. Deferred ECUReset, only if step 5 got the positive response out (or it
     *    was legitimately suppressed). */
    if (g_pending_reset) {
        g_pending_reset = false;
        vTaskDelay(pdMS_TO_TICKS(UDS_RESET_DELAY_MS));
        esp_restart();
        /* not reached */
    }
}

/* ========================================================================== */
/*  Built-in handlers                                                          */
/* ========================================================================== */

uds_disp_t uds_h_diag_session(uds_ctx_t *c)
{
    if (c->req_len < 2) {
        c->nrc = NRC_IMLOIF;
        return UDS_NEGATIVE;
    }
    const uint8_t sub = c->req[1] & (uint8_t)~UDS_SUPPRESS_POS_RSP;

    if (sub != UDS_SESSION_DEFAULT && sub != UDS_SESSION_PROG &&
        sub != UDS_SESSION_EXTENDED) {
        c->nrc = NRC_SFNS;
        return UDS_NEGATIVE;
    }
    if (c->resp_cap < 6) {
        c->nrc = NRC_GENERAL_REJECT;
        return UDS_NEGATIVE;
    }

    c->session = sub; /* dispatcher commits this into the server on POSITIVE */

    c->resp[0] = UDS_SID_DIAG_SESSION + UDS_POS_RESP_OFFSET; /* 0x50 */
    c->resp[1] = sub;
    uds_put_be16(&c->resp[2], UDS_P2_ENC);
    uds_put_be16(&c->resp[4], UDS_P2STAR_ENC);
    c->resp_len = 6;
    return UDS_POSITIVE;
}

uds_disp_t uds_h_ecu_reset(uds_ctx_t *c)
{
    if (c->req_len < 2) {
        c->nrc = NRC_IMLOIF;
        return UDS_NEGATIVE;
    }
    const uint8_t sub = c->req[1] & (uint8_t)~UDS_SUPPRESS_POS_RSP;

    /* 0x01 hardReset, 0x02 keyOffOnReset, 0x03 softReset,
     * 0x04 enableRapidPowerShutDown, 0x05 disableRapidPowerShutDown. */
    if (sub < 0x01u || sub > 0x05u) {
        c->nrc = NRC_SFNS;
        return UDS_NEGATIVE;
    }
    if (c->resp_cap < 2) {
        c->nrc = NRC_GENERAL_REJECT;
        return UDS_NEGATIVE;
    }

    /* Actually reboot on the three reset types that mean "restart now". */
    if (sub == 0x01u || sub == 0x02u || sub == 0x03u) {
        g_pending_reset = true;
        c->session      = UDS_SESSION_DEFAULT; /* post-reset state */
    }

    c->resp[0] = UDS_SID_ECU_RESET + UDS_POS_RESP_OFFSET; /* 0x51 */
    c->resp[1] = sub;
    c->resp_len = 2;
    return UDS_POSITIVE;
}

uds_disp_t uds_h_read_did(uds_ctx_t *c)
{
    /* Request is SID + one-or-more 2-byte DIDs. */
    if (c->req_len < 3 || ((c->req_len - 1u) & 1u) != 0u) {
        c->nrc = NRC_IMLOIF;
        return UDS_NEGATIVE;
    }
    if (c->resp_cap < 1) {
        c->nrc = NRC_GENERAL_REJECT;
        return UDS_NEGATIVE;
    }

    uint16_t oi = 0;
    c->resp[oi++] = UDS_SID_READ_DID + UDS_POS_RESP_OFFSET; /* 0x62 */

    uint16_t matched = 0;
    for (uint16_t i = 1; (uint16_t)(i + 1) < c->req_len; i += 2) {
        const uint16_t did = (uint16_t)(((uint16_t)c->req[i] << 8) | c->req[i + 1]);

        for (uint16_t t = 0; t < UDS_DID_TABLE_LEN; ++t) {
            if (k_did_table[t].did != did) {
                continue;
            }
            const uint8_t dlen = k_did_table[t].len;
            /* Need 2 bytes DID + dlen bytes data. Refuse if it would overflow
             * the response buffer / ISO-TP message. */
            if ((uint32_t)oi + 2u + dlen > c->resp_cap) {
                c->nrc = NRC_ROOR; /* effectively responseTooLong */
                return UDS_NEGATIVE;
            }
            c->resp[oi++] = (uint8_t)(did >> 8);
            c->resp[oi++] = (uint8_t)(did & 0xFFu);
            memcpy(&c->resp[oi], k_did_table[t].data, dlen);
            oi += dlen;
            ++matched;
            break;
        }
    }

    if (matched == 0) {
        c->nrc = NRC_ROOR;
        return UDS_NEGATIVE;
    }
    c->resp_len = oi;
    return UDS_POSITIVE;
}

uds_disp_t uds_h_security(uds_ctx_t *c)
{
    if (c->req_len < 2) {
        c->nrc = NRC_IMLOIF;
        return UDS_NEGATIVE;
    }

    /* Anti-brute-force lockout: while the delay timer is running, reject every
     * requestSeed/sendKey with requiredTimeDelayNotExpired (ISO 14229-1 §9.4). */
    if (g_sec_lock_ms > 0) {
        c->nrc = NRC_RTDNE; /* 0x37 */
        return UDS_NEGATIVE;
    }

    const uint8_t sub = c->req[1];

    /* The sub-function is the security level itself: odd = requestSeed, even =
     * sendKey for the odd level below it. Only levels this ECU implements are
     * valid — that rejects the reserved 0x00/0x7F, and the whole 0x80..0xFF
     * range which would otherwise collide with the suppressPosRsp bit and mint
     * a "level" that unlocks every gated service. */
    const uint8_t level = (sub & 0x01u) ? sub : (uint8_t)(sub - 1u);
    if (!uds_sec_level_supported(level)) {
        c->nrc = NRC_SFNS;
        return UDS_NEGATIVE;
    }

    if (sub & 0x01u) {
        /* requestSeed (odd sub-function). */
        if (c->resp_cap < 6) {
            c->nrc = NRC_GENERAL_REJECT;
            return UDS_NEGATIVE;
        }
        uint32_t seed;
        if (g_sec_level == sub) {
            /* Already unlocked at this level: reply with an all-zero seed. */
            seed = 0;
        } else {
            seed = esp_random();
            if (seed == 0) {
                seed = 0xA5A5A5A5u; /* never hand out a zero challenge */
            }
            g_sec_seed    = seed;
            g_sec_pending = sub;
        }
        c->resp[0] = UDS_SID_SECURITY + UDS_POS_RESP_OFFSET; /* 0x67 */
        c->resp[1] = sub;
        c->resp[2] = (uint8_t)(seed >> 24);
        c->resp[3] = (uint8_t)(seed >> 16);
        c->resp[4] = (uint8_t)(seed >> 8);
        c->resp[5] = (uint8_t)(seed & 0xFFu);
        c->resp_len = 6;
        return UDS_POSITIVE;
    }

    /* sendKey (even sub-function): the matching seed level is sub - 1. */
    const uint8_t seed_level = level;
    if (c->req_len < 6) {
        c->nrc = NRC_IMLOIF;
        return UDS_NEGATIVE;
    }
    if (g_sec_pending == 0u || g_sec_pending != seed_level) {
        c->nrc = NRC_RSE; /* requestSequenceError: key without a matching seed */
        return UDS_NEGATIVE;
    }
    if (c->resp_cap < 2) {
        c->nrc = NRC_GENERAL_REJECT;
        return UDS_NEGATIVE;
    }

    const uint32_t key = ((uint32_t)c->req[2] << 24) | ((uint32_t)c->req[3] << 16) |
                         ((uint32_t)c->req[4] << 8)  | (uint32_t)c->req[5];
    const uint32_t expected = g_sec_seed ^ (uint32_t)UDS_SEC_KEY_XOR;

    if (key != expected) {
        g_sec_pending = 0; /* invalidate the challenge; tester must re-request */
        /* Count the failure; after the limit, start the lockout delay and tell
         * the tester with exceededNumberOfAttempts (0x36). */
        if (++g_sec_attempts >= UDS_SEC_MAX_ATTEMPTS) {
            g_sec_attempts = 0;
            g_sec_lock_ms  = UDS_SEC_LOCK_MS;
            c->nrc = NRC_ENOA; /* 0x36 */
        } else {
            c->nrc = NRC_IK;   /* invalidKey (0x35) */
        }
        return UDS_NEGATIVE;
    }

    g_sec_level    = seed_level;
    g_sec_pending  = 0;
    g_sec_attempts = 0;  /* successful unlock clears the failure counter */
    c->resp[0] = UDS_SID_SECURITY + UDS_POS_RESP_OFFSET; /* 0x67 */
    c->resp[1] = sub;
    c->resp_len = 2;
    return UDS_POSITIVE;
}

uds_disp_t uds_h_routine(uds_ctx_t *c)
{
    /* SID + routineControlType + 2-byte RID (+ optional option record). */
    if (c->req_len < 4) {
        c->nrc = NRC_IMLOIF;
        return UDS_NEGATIVE;
    }
    const uint8_t sub = c->req[1] & (uint8_t)~UDS_SUPPRESS_POS_RSP;

    /* 0x01 startRoutine, 0x02 stopRoutine, 0x03 requestRoutineResults. */
    if (sub < 0x01u || sub > 0x03u) {
        c->nrc = NRC_SFNS;
        return UDS_NEGATIVE;
    }
    if (c->resp_cap < 5) {
        c->nrc = NRC_GENERAL_REJECT;
        return UDS_NEGATIVE;
    }

    c->resp[0] = UDS_SID_ROUTINE + UDS_POS_RESP_OFFSET; /* 0x71 */
    c->resp[1] = sub;
    c->resp[2] = c->req[2];  /* RID hi */
    c->resp[3] = c->req[3];  /* RID lo */
    c->resp[4] = 0x00;       /* routineStatusRecord: completed OK */
    c->resp_len = 5;
    return UDS_POSITIVE;
}

uds_disp_t uds_h_tester_present(uds_ctx_t *c)
{
    if (c->req_len < 2) {
        c->nrc = NRC_IMLOIF;
        return UDS_NEGATIVE;
    }
    const uint8_t sub = c->req[1] & (uint8_t)~UDS_SUPPRESS_POS_RSP;
    if (sub != 0x00u) {
        c->nrc = NRC_SFNS;
        return UDS_NEGATIVE;
    }
    if (c->resp_cap < 2) {
        c->nrc = NRC_GENERAL_REJECT;
        return UDS_NEGATIVE;
    }
    /* S3 timer was already reset by the dispatcher on receipt. suppressPosRsp
     * (bit 7) is honoured centrally in uds_server_poll(). */
    c->resp[0] = UDS_SID_TESTER_PRES + UDS_POS_RESP_OFFSET; /* 0x7E */
    c->resp[1] = 0x00;
    c->resp_len = 2;
    return UDS_POSITIVE;
}

uds_disp_t uds_h_request_download(uds_ctx_t *c)
{
    /* SID, dataFormatIdentifier, addressAndLengthFormatIdentifier, addr, size. */
    if (c->req_len < 4) {
        c->nrc = NRC_IMLOIF;
        return UDS_NEGATIVE;
    }
    /* Memory download requires the download security level (ISO 14229-1). */
    if (g_sec_level != UDS_SEC_LEVEL_DOWNLOAD) {
        c->nrc = NRC_SAD; /* securityAccessDenied */
        return UDS_NEGATIVE;
    }
    if (g_xfer_active) {
        c->nrc = NRC_CNC; /* a transfer is already open */
        return UDS_NEGATIVE;
    }

    const uint8_t alfid    = c->req[2];
    const uint8_t size_len = (uint8_t)(alfid & 0x0Fu);
    const uint8_t addr_len = (uint8_t)((alfid >> 4) & 0x0Fu);

    if (size_len == 0u || size_len > 4u || addr_len == 0u || addr_len > 4u) {
        c->nrc = NRC_ROOR;
        return UDS_NEGATIVE;
    }
    const uint16_t need = (uint16_t)(3u + addr_len + size_len);
    if (c->req_len < need) {
        c->nrc = NRC_IMLOIF;
        return UDS_NEGATIVE;
    }
    if (c->resp_cap < 4) {
        c->nrc = NRC_GENERAL_REJECT;
        return UDS_NEGATIVE;
    }

    /* Parse the (big-endian) memorySize; the memoryAddress is not used here. */
    uint32_t size = 0;
    for (uint8_t i = 0; i < size_len; ++i) {
        size = (size << 8) | c->req[3u + addr_len + i];
    }
    if (size == 0u) {
        c->nrc = NRC_ROOR;
        return UDS_NEGATIVE;
    }

    g_xfer_active   = true;
    g_xfer_size     = size;
    g_xfer_received = 0;
    g_xfer_bsc      = 1; /* first TransferData block carries BSC 0x01 */

    /* lengthFormatIdentifier 0x20 => 2-byte maxNumberOfBlockLength follows. */
    c->resp[0] = UDS_SID_REQ_DOWNLOAD + UDS_POS_RESP_OFFSET; /* 0x74 */
    c->resp[1] = 0x20;
    uds_put_be16(&c->resp[2], (uint16_t)UDS_MAX_BLOCK_LEN);
    c->resp_len = 4;
    return UDS_POSITIVE;
}

uds_disp_t uds_h_transfer_data(uds_ctx_t *c)
{
    /* SID + blockSequenceCounter + transferRequestParameterRecord. */
    if (c->req_len < 2) {
        c->nrc = NRC_IMLOIF;
        return UDS_NEGATIVE;
    }
    if (!g_xfer_active) {
        c->nrc = NRC_RSE; /* no active download */
        return UDS_NEGATIVE;
    }
    /* The transfer session itself is security-gated by RequestDownload; guard
     * again in case security was dropped (e.g. session change) mid-transfer. */
    if (g_sec_level != UDS_SEC_LEVEL_DOWNLOAD) {
        c->nrc = NRC_SAD;
        return UDS_NEGATIVE;
    }
    if (c->resp_cap < 2) {
        c->nrc = NRC_GENERAL_REJECT;
        return UDS_NEGATIVE;
    }

    const uint8_t  bsc  = c->req[1];
    const uint16_t dlen = (uint16_t)(c->req_len - 2u);

    if (bsc == g_xfer_bsc) {
        /* In-sequence new block. maxNumberOfBlockLength counts the WHOLE message
         * (SID + BSC + data), so compare req_len, not the payload length. */
        if ((uint32_t)c->req_len > (uint32_t)UDS_MAX_BLOCK_LEN) {
            c->nrc = NRC_ROOR; /* message larger than we advertised */
            return UDS_NEGATIVE;
        }
        if (g_xfer_received + dlen > g_xfer_size) {
            c->nrc = NRC_ROOR; /* would exceed the declared transfer size */
            return UDS_NEGATIVE;
        }
        /* Data is accepted and discarded (bench sink). */
        g_xfer_received += dlen;
        g_xfer_bsc = (uint8_t)(g_xfer_bsc + 1u); /* wraps 0xFF -> 0x00 */
    } else if (g_xfer_received > 0u && bsc == (uint8_t)(g_xfer_bsc - 1u)) {
        /* Retransmission of the last ACCEPTED block: ack idempotently. Requires
         * at least one block to have been accepted, so a spurious BSC 0x00
         * before the first (BSC 0x01) block is rejected, not phantom-acked. */
    } else {
        c->nrc = UDS_NRC_WRONG_BSC;
        return UDS_NEGATIVE;
    }

    c->resp[0] = UDS_SID_TRANSFER_DATA + UDS_POS_RESP_OFFSET; /* 0x76 */
    c->resp[1] = bsc;
    c->resp_len = 2;
    return UDS_POSITIVE;
}

uds_disp_t uds_h_request_transfer_exit(uds_ctx_t *c)
{
    if (!g_xfer_active) {
        c->nrc = NRC_RSE;
        return UDS_NEGATIVE;
    }
    /* Safe to gate: a transfer never outlives the unlock that opened it — both
     * are dropped together on a session change and on S3 expiry — so this can
     * never be the only service that would let the tester back to a clean
     * state. */
    if (g_sec_level != UDS_SEC_LEVEL_DOWNLOAD) {
        c->nrc = NRC_SAD;
        return UDS_NEGATIVE;
    }
    if (c->resp_cap < 1) {
        c->nrc = NRC_GENERAL_REJECT;
        return UDS_NEGATIVE;
    }

    /* Bench sink accepts whatever arrived (partial transfers allowed). */
    g_xfer_active   = false;
    g_xfer_size     = 0;
    g_xfer_received = 0;
    g_xfer_bsc      = 0;

    c->resp[0] = UDS_SID_REQ_XFER_EXIT + UDS_POS_RESP_OFFSET; /* 0x77 */
    c->resp_len = 1;
    return UDS_POSITIVE;
}
