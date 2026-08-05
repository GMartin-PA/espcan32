/*
 * SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
/*
 * test_isotp.c — ISO-TP state machine
 * ============================================================================
 * The link is driven exactly the way the bench task drives it: frames in
 * through isotp_on_can_frame(), time through a fake millis() the test steps by
 * hand, and everything the link transmits captured by a fake can_tx(). Nothing
 * pokes the state machine's internals to set up a scenario — only to assert on
 * one — so a test that passes describes traffic a real peer could produce.
 * ============================================================================
 */
#include "test.h"

#include "isotp.h"

#define TXID    0x7E8u
#define RXID    0x7E0u
#define FUNCID  0x7DFu

#define CAP_MAX 256

typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[ISOTP_CAN_DL];
} cap_t;

static cap_t        g_cap[CAP_MAX];
static int          g_cap_n;
static int          g_tx_fail;
static uint32_t     g_ms;
static isotp_link_t L;
static uint8_t      g_out[ISOTP_MAX_MSG];

static int fake_tx(uint32_t id, const uint8_t *data, uint8_t dlc, void *user)
{
    (void)user;
    if (g_tx_fail) {
        return -1;
    }
    if (g_cap_n < CAP_MAX) {
        g_cap[g_cap_n].id  = id;
        g_cap[g_cap_n].dlc = dlc;
        memset(g_cap[g_cap_n].data, 0, sizeof(g_cap[g_cap_n].data));
        memcpy(g_cap[g_cap_n].data, data,
               dlc > ISOTP_CAN_DL ? ISOTP_CAN_DL : dlc);
    }
    ++g_cap_n;   /* keeps counting past CAP_MAX so an overrun is visible       */
    return 0;
}

static uint32_t fake_ms(void *user)
{
    (void)user;
    return g_ms;
}

static void link_setup(uint8_t bs, uint8_t stmin, uint32_t func_id)
{
    isotp_cfg_t cfg;
    isotp_hal_t hal;

    memset(&cfg, 0, sizeof(cfg));
    cfg.bs       = bs;
    cfg.stmin    = stmin;
    cfg.pad_byte = 0xCCu;

    memset(&hal, 0, sizeof(hal));
    hal.can_tx = fake_tx;
    hal.millis = fake_ms;

    g_cap_n   = 0;
    g_tx_fail = 0;
    g_ms      = 1000u;   /* non-zero so a "timer never started" bug shows up   */
    isotp_init(&L, TXID, RXID, func_id, &cfg, &hal);
}

/* Deterministic payload so a mis-assembled message is obvious. */
static void fill_pattern(uint8_t *p, uint16_t n)
{
    for (uint16_t i = 0; i < n; ++i) {
        p[i] = (uint8_t)(i * 7u + 1u);
    }
}

static void feed(uint32_t id, const uint8_t *d, uint8_t dlc)
{
    isotp_on_can_frame(&L, id, d, dlc);
}

static void feed_sf(uint32_t id, const uint8_t *p, uint8_t n)
{
    uint8_t f[ISOTP_CAN_DL];
    memset(f, 0xAA, sizeof(f));
    f[0] = (uint8_t)n;
    memcpy(&f[1], p, n);
    feed(id, f, ISOTP_CAN_DL);
}

static void feed_ff(uint32_t id, const uint8_t *msg, uint16_t total)
{
    uint8_t f[ISOTP_CAN_DL];
    f[0] = (uint8_t)(0x10u | ((total >> 8) & 0x0Fu));
    f[1] = (uint8_t)(total & 0xFFu);
    memcpy(&f[2], msg, 6);
    feed(id, f, ISOTP_CAN_DL);
}

static void feed_cf(uint32_t id, uint8_t sn, const uint8_t *p, uint8_t n)
{
    uint8_t f[ISOTP_CAN_DL];
    memset(f, 0xCC, sizeof(f));
    f[0] = (uint8_t)(0x20u | (sn & 0x0Fu));
    memcpy(&f[1], p, n);
    feed(id, f, (uint8_t)(n == 7u ? 8u : n + 1u));
}

static void feed_fc(uint32_t id, uint8_t fs, uint8_t bs, uint8_t stmin)
{
    uint8_t f[3] = { (uint8_t)(0x30u | (fs & 0x0Fu)), bs, stmin };
    feed(id, f, 3);
}

/* Count the flow-control frames the link has transmitted. */
static int count_fc(void)
{
    int n = 0;
    for (int i = 0; i < g_cap_n && i < CAP_MAX; ++i) {
        if ((g_cap[i].data[0] >> 4) == ISOTP_PCI_FC) {
            ++n;
        }
    }
    return n;
}

/* ==========================================================================
 * Single frames
 * ========================================================================== */

static void test_send_single_frame_pads(void)
{
    link_setup(0, 0, FUNCID);
    const uint8_t msg[3] = { 0x11, 0x22, 0x33 };
    const uint8_t want[ISOTP_CAN_DL] =
        { 0x03, 0x11, 0x22, 0x33, 0xCC, 0xCC, 0xCC, 0xCC };

    CHECK_EQ(isotp_send(&L, msg, 3), ISOTP_OK);
    REQUIRE_EQ(g_cap_n, 1);
    CHECK_EQ(g_cap[0].id, TXID);
    CHECK_EQ(g_cap[0].dlc, ISOTP_CAN_DL);
    CHECK_MEM(g_cap[0].data, want, ISOTP_CAN_DL);
    CHECK_EQ(L.tx_phase, ISOTP_IDLE);
}

static void test_send_rejects_bad_sizes_and_reentry(void)
{
    link_setup(0, 0, FUNCID);
    uint8_t msg[64];
    fill_pattern(msg, sizeof(msg));

    CHECK_EQ(isotp_send(&L, msg, 0), ISOTP_LENGTH);
    CHECK_EQ(isotp_send(&L, g_out, ISOTP_MAX_MSG + 1), ISOTP_LENGTH);
    CHECK_EQ(isotp_send(&L, NULL, 4), ISOTP_ERROR);
    CHECK_EQ(g_cap_n, 0);

    /* A multi-frame send in flight refuses a second one. */
    uint8_t big[40];
    fill_pattern(big, sizeof(big));
    CHECK_EQ(isotp_send(&L, big, sizeof(big)), ISOTP_INPROGRESS);
    CHECK_EQ(isotp_send(&L, msg, 4), ISOTP_NOSPACE);
}

static void test_send_tx_failure_aborts(void)
{
    link_setup(0, 0, FUNCID);
    g_tx_fail = 1;

    const uint8_t msg[3] = { 1, 2, 3 };
    CHECK_EQ(isotp_send(&L, msg, 3), ISOTP_ERROR);
    CHECK_EQ(L.tx_phase, ISOTP_IDLE);

    uint8_t big[40];
    fill_pattern(big, sizeof(big));
    CHECK_EQ(isotp_send(&L, big, sizeof(big)), ISOTP_ERROR);
    CHECK_EQ(L.tx_phase, ISOTP_IDLE);
}

static void test_receive_single_frame(void)
{
    link_setup(0, 0, FUNCID);
    const uint8_t msg[5] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x01 };
    feed_sf(RXID, msg, 5);

    uint16_t len = 0;
    bool     fn  = true;
    REQUIRE_EQ(isotp_receive(&L, g_out, sizeof(g_out), &len, &fn), ISOTP_OK);
    CHECK_EQ(len, 5);
    CHECK_EQ(fn, false);
    CHECK_MEM(g_out, msg, 5);
    /* Consumed: a second collect finds nothing. */
    CHECK_EQ(isotp_receive(&L, g_out, sizeof(g_out), &len, &fn), ISOTP_NO_DATA);
}

static void test_receive_rejects_bad_single_frames(void)
{
    link_setup(0, 0, FUNCID);

    /* SF_DL 0 is the CAN-FD escape; unsupported on classic. */
    uint8_t f[ISOTP_CAN_DL] = { 0x00, 1, 2, 3, 4, 5, 6, 7 };
    feed(RXID, f, ISOTP_CAN_DL);
    CHECK_EQ(isotp_receive(&L, g_out, sizeof(g_out), NULL, NULL), ISOTP_NO_DATA);

    /* SF_DL larger than the frame can hold. */
    f[0] = 0x07;
    feed(RXID, f, 4);
    CHECK_EQ(isotp_receive(&L, g_out, sizeof(g_out), NULL, NULL), ISOTP_NO_DATA);

    /* An id that is neither the physical nor the functional address. */
    feed_sf(0x123u, f, 3);
    CHECK_EQ(isotp_receive(&L, g_out, sizeof(g_out), NULL, NULL), ISOTP_NO_DATA);
}

/* ==========================================================================
 * Multi-frame receive
 * ========================================================================== */

static void test_receive_ff_cf_happy_path(void)
{
    link_setup(0, 0x0A, FUNCID);
    uint8_t msg[20];
    fill_pattern(msg, sizeof(msg));

    feed_ff(RXID, msg, 20);
    REQUIRE_EQ(g_cap_n, 1);
    CHECK_EQ(g_cap[0].id, TXID);
    CHECK_EQ(g_cap[0].data[0], 0x30u);        /* FC, FS = CTS                  */
    CHECK_EQ(g_cap[0].data[1], 0x00u);        /* our advertised BS             */
    CHECK_EQ(g_cap[0].data[2], 0x0Au);        /* our advertised STmin          */
    CHECK_EQ(L.rx_phase, ISOTP_RECV);

    feed_cf(RXID, 1, &msg[6],  7);
    feed_cf(RXID, 2, &msg[13], 7);

    uint16_t len = 0;
    REQUIRE_EQ(isotp_receive(&L, g_out, sizeof(g_out), &len, NULL), ISOTP_OK);
    CHECK_EQ(len, 20);
    CHECK_MEM(g_out, msg, 20);
    CHECK_EQ(L.rx_phase, ISOTP_IDLE);
}

static void test_receive_sn_wraps_through_zero(void)
{
    link_setup(0, 0, FUNCID);
    /* 6 + 17*7 = 125 bytes: SNs run 1..15, 0, 1. */
    uint8_t msg[125];
    fill_pattern(msg, sizeof(msg));

    feed_ff(RXID, msg, sizeof(msg));
    uint16_t off = 6;
    for (int i = 0; i < 17; ++i) {
        const uint8_t sn = (uint8_t)((i + 1) & 0x0Fu);
        feed_cf(RXID, sn, &msg[off], 7);
        off = (uint16_t)(off + 7);
    }
    CHECK_EQ(off, sizeof(msg));

    uint16_t len = 0;
    REQUIRE_EQ(isotp_receive(&L, g_out, sizeof(g_out), &len, NULL), ISOTP_OK);
    CHECK_EQ(len, sizeof(msg));
    CHECK_MEM(g_out, msg, sizeof(msg));
}

/* A First Frame that does not fill the CAN frame carries fewer than 6 payload
 * bytes, which would offset every CF after it. It must be dropped outright. */
static void test_receive_short_first_frame_rejected(void)
{
    uint8_t msg[20];
    fill_pattern(msg, sizeof(msg));

    for (uint8_t dlc = 2; dlc < ISOTP_CAN_DL; ++dlc) {
        link_setup(0, 0, FUNCID);
        uint8_t f[ISOTP_CAN_DL];
        memset(f, 0, sizeof(f));
        f[0] = 0x10u;
        f[1] = 20u;
        memcpy(&f[2], msg, 6);
        feed(RXID, f, dlc);

        CHECK_MSG(g_cap_n == 0, "FF with dlc %u must not be flow-controlled", dlc);
        CHECK_MSG(L.rx_phase == ISOTP_IDLE, "FF with dlc %u must not open a transfer", dlc);
    }

    /* dlc 8 is the only acceptable width. */
    link_setup(0, 0, FUNCID);
    feed_ff(RXID, msg, 20);
    CHECK_EQ(g_cap_n, 1);
    CHECK_EQ(L.rx_phase, ISOTP_RECV);
}

static void test_receive_ff_dl_must_be_multiframe(void)
{
    link_setup(0, 0, FUNCID);
    uint8_t msg[8];
    fill_pattern(msg, sizeof(msg));

    /* FF_DL <= 7 belongs in a SingleFrame; FF_DL 0 is the 32-bit escape. */
    for (uint16_t dl = 0; dl <= 7; ++dl) {
        link_setup(0, 0, FUNCID);
        feed_ff(RXID, msg, dl);
        CHECK_MSG(g_cap_n == 0, "FF_DL %u must be rejected", dl);
        CHECK_MSG(L.rx_phase == ISOTP_IDLE, "FF_DL %u must not open a transfer", dl);
    }
}

static void test_receive_wrong_sn_aborts(void)
{
    link_setup(0, 0, FUNCID);
    uint8_t msg[20];
    fill_pattern(msg, sizeof(msg));

    feed_ff(RXID, msg, 20);
    feed_cf(RXID, 3, &msg[6], 7);        /* expected SN 1                      */

    CHECK_EQ(L.rx_phase, ISOTP_IDLE);
    CHECK_EQ(L.rx_result, ISOTP_WRONG_SN);
    CHECK_EQ(isotp_receive(&L, g_out, sizeof(g_out), NULL, NULL), ISOTP_NO_DATA);
}

static void test_receive_short_mid_block_cf_aborts(void)
{
    link_setup(0, 0, FUNCID);
    uint8_t msg[20];
    fill_pattern(msg, sizeof(msg));

    feed_ff(RXID, msg, 20);
    feed_cf(RXID, 1, &msg[6], 3);        /* 14 still to come, only 3 supplied  */

    CHECK_EQ(L.rx_phase, ISOTP_IDLE);
    CHECK_EQ(L.rx_result, ISOTP_LENGTH);
    CHECK_EQ(isotp_receive(&L, g_out, sizeof(g_out), NULL, NULL), ISOTP_NO_DATA);
}

static void test_receive_final_cf_may_be_short(void)
{
    link_setup(0, 0, FUNCID);
    uint8_t msg[10];
    fill_pattern(msg, sizeof(msg));

    feed_ff(RXID, msg, sizeof(msg));
    feed_cf(RXID, 1, &msg[6], 4);        /* last CF carries the remaining 4    */

    uint16_t len = 0;
    REQUIRE_EQ(isotp_receive(&L, g_out, sizeof(g_out), &len, NULL), ISOTP_OK);
    CHECK_EQ(len, sizeof(msg));
    CHECK_MEM(g_out, msg, sizeof(msg));
}

static void test_receive_stray_cf_when_idle_is_ignored(void)
{
    link_setup(0, 0, FUNCID);
    uint8_t junk[7] = { 1, 2, 3, 4, 5, 6, 7 };
    feed_cf(RXID, 1, junk, 7);

    CHECK_EQ(L.rx_phase, ISOTP_IDLE);
    CHECK_EQ(g_cap_n, 0);
    CHECK_EQ(isotp_receive(&L, g_out, sizeof(g_out), NULL, NULL), ISOTP_NO_DATA);
}

static void test_receive_out_buffer_too_small_keeps_message(void)
{
    link_setup(0, 0, FUNCID);
    uint8_t msg[20];
    fill_pattern(msg, sizeof(msg));

    feed_ff(RXID, msg, 20);
    feed_cf(RXID, 1, &msg[6],  7);
    feed_cf(RXID, 2, &msg[13], 7);

    uint16_t len = 0xFFFFu;
    CHECK_EQ(isotp_receive(&L, g_out, 10, &len, NULL), ISOTP_NOSPACE);
    CHECK_EQ(len, 0);
    /* Still available at the right capacity. */
    REQUIRE_EQ(isotp_receive(&L, g_out, sizeof(g_out), &len, NULL), ISOTP_OK);
    CHECK_EQ(len, 20);
}

/* ==========================================================================
 * Block size accounting (receive side)
 * ========================================================================== */

static void test_receive_block_size_grants(void)
{
    /* 27 bytes = FF(6) + 3 CFs. */
    uint8_t msg[27];
    fill_pattern(msg, sizeof(msg));

    struct { uint8_t bs; int want_fc; } cases[] = {
        { 0, 1 },   /* unlimited: the FF's CTS covers the whole message        */
        { 1, 3 },   /* CTS after the FF and after each of CF1, CF2             */
        { 2, 2 },   /* CTS after the FF and after CF2                          */
    };

    for (unsigned c = 0; c < sizeof(cases) / sizeof(cases[0]); ++c) {
        link_setup(cases[c].bs, 0, FUNCID);
        feed_ff(RXID, msg, sizeof(msg));
        feed_cf(RXID, 1, &msg[6],  7);
        feed_cf(RXID, 2, &msg[13], 7);
        feed_cf(RXID, 3, &msg[20], 7);

        CHECK_MSG(count_fc() == cases[c].want_fc,
                  "bs=%u: sent %d FCs, expected %d", cases[c].bs, count_fc(),
                  cases[c].want_fc);

        uint16_t len = 0;
        CHECK_EQ(isotp_receive(&L, g_out, sizeof(g_out), &len, NULL), ISOTP_OK);
        CHECK_EQ(len, sizeof(msg));
        CHECK_MEM(g_out, msg, sizeof(msg));
    }
}

/* ==========================================================================
 * Multi-frame send (flow control from the peer)
 * ========================================================================== */

static void test_send_multiframe_with_block_size(void)
{
    link_setup(0, 0, FUNCID);
    uint8_t msg[20];
    fill_pattern(msg, sizeof(msg));

    REQUIRE_EQ(isotp_send(&L, msg, sizeof(msg)), ISOTP_INPROGRESS);
    REQUIRE_EQ(g_cap_n, 1);
    CHECK_EQ(g_cap[0].data[0], 0x10u);
    CHECK_EQ(g_cap[0].data[1], 20u);
    CHECK_MEM(&g_cap[0].data[2], msg, 6);
    CHECK_EQ(L.tx_phase, ISOTP_WAIT_FC);

    /* BS=1: one CF, then back to waiting. */
    feed_fc(RXID, ISOTP_FS_CTS, 1, 0);
    CHECK_EQ(L.tx_phase, ISOTP_SEND);
    isotp_poll(&L);
    REQUIRE_EQ(g_cap_n, 2);
    CHECK_EQ(g_cap[1].data[0], 0x21u);
    CHECK_MEM(&g_cap[1].data[1], &msg[6], 7);
    CHECK_EQ(L.tx_phase, ISOTP_WAIT_FC);

    isotp_poll(&L);
    CHECK_EQ(g_cap_n, 2);   /* no CF without a fresh grant                     */

    feed_fc(RXID, ISOTP_FS_CTS, 1, 0);
    isotp_poll(&L);
    REQUIRE_EQ(g_cap_n, 3);
    CHECK_EQ(g_cap[2].data[0], 0x22u);
    CHECK_MEM(&g_cap[2].data[1], &msg[13], 7);
    CHECK_EQ(L.tx_phase, ISOTP_IDLE);   /* whole message out                   */
}

static void test_send_stmin_paces_consecutive_frames(void)
{
    link_setup(0, 0, FUNCID);
    uint8_t msg[20];
    fill_pattern(msg, sizeof(msg));

    REQUIRE_EQ(isotp_send(&L, msg, sizeof(msg)), ISOTP_INPROGRESS);
    feed_fc(RXID, ISOTP_FS_CTS, 0, 10);   /* STmin 10 ms, unlimited block      */

    isotp_poll(&L);
    CHECK_EQ(g_cap_n, 1);                 /* too early                         */
    g_ms += 9;
    isotp_poll(&L);
    CHECK_EQ(g_cap_n, 1);
    g_ms += 1;
    isotp_poll(&L);
    CHECK_EQ(g_cap_n, 2);
    CHECK_EQ(g_cap[1].data[0], 0x21u);

    /* Sub-millisecond STmin encodings clamp to 1 ms, not to "reserved". */
    link_setup(0, 0, FUNCID);
    REQUIRE_EQ(isotp_send(&L, msg, sizeof(msg)), ISOTP_INPROGRESS);
    feed_fc(RXID, ISOTP_FS_CTS, 0, 0xF1u);
    isotp_poll(&L);
    CHECK_EQ(g_cap_n, 1);
    g_ms += 1;
    isotp_poll(&L);
    CHECK_EQ(g_cap_n, 2);
}

static void test_send_wait_frames_bounded_by_wft_max(void)
{
    link_setup(0, 0, FUNCID);
    uint8_t msg[20];
    fill_pattern(msg, sizeof(msg));

    REQUIRE_EQ(isotp_send(&L, msg, sizeof(msg)), ISOTP_INPROGRESS);
    for (int i = 0; i < 8; ++i) {          /* the default wft_max              */
        g_ms += 100;                       /* each WAIT restarts N_Bs          */
        feed_fc(RXID, ISOTP_FS_WAIT, 0, 0);
        isotp_poll(&L);
        CHECK_MSG(L.tx_phase == ISOTP_WAIT_FC, "WAIT #%d must not abort", i + 1);
    }
    feed_fc(RXID, ISOTP_FS_WAIT, 0, 0);
    CHECK_EQ(L.tx_phase, ISOTP_IDLE);      /* one WAIT too many                */
}

static void test_send_overflow_fc_aborts(void)
{
    link_setup(0, 0, FUNCID);
    uint8_t msg[20];
    fill_pattern(msg, sizeof(msg));

    REQUIRE_EQ(isotp_send(&L, msg, sizeof(msg)), ISOTP_INPROGRESS);
    feed_fc(RXID, ISOTP_FS_OVFLW, 0, 0);
    CHECK_EQ(L.tx_phase, ISOTP_IDLE);
    isotp_poll(&L);
    CHECK_EQ(g_cap_n, 1);                  /* the FF, and nothing after it     */
}

/* ==========================================================================
 * Timeouts (fake clock)
 * ========================================================================== */

static void test_n_bs_timeout_releases_the_sender(void)
{
    link_setup(0, 0, FUNCID);
    uint8_t msg[20];
    fill_pattern(msg, sizeof(msg));

    REQUIRE_EQ(isotp_send(&L, msg, sizeof(msg)), ISOTP_INPROGRESS);
    CHECK_EQ(L.tx_phase, ISOTP_WAIT_FC);

    g_ms += 999;
    isotp_poll(&L);
    CHECK_EQ(L.tx_phase, ISOTP_WAIT_FC);   /* default N_Bs is 1000 ms          */

    g_ms += 1;
    isotp_poll(&L);
    CHECK_EQ(L.tx_phase, ISOTP_IDLE);
    CHECK_EQ(g_cap_n, 1);                  /* no CF ever went out              */

    /* The link is usable again. */
    CHECK_EQ(isotp_send(&L, msg, sizeof(msg)), ISOTP_INPROGRESS);
}

static void test_n_cr_timeout_drops_the_reassembly(void)
{
    link_setup(0, 0, FUNCID);
    uint8_t msg[20];
    fill_pattern(msg, sizeof(msg));

    feed_ff(RXID, msg, 20);
    feed_cf(RXID, 1, &msg[6], 7);
    CHECK_EQ(L.rx_phase, ISOTP_RECV);

    g_ms += 999;
    isotp_poll(&L);
    CHECK_EQ(L.rx_phase, ISOTP_RECV);      /* default N_Cr is 1000 ms          */

    g_ms += 1;
    isotp_poll(&L);
    CHECK_EQ(L.rx_phase, ISOTP_IDLE);
    CHECK_EQ(L.rx_result, ISOTP_TIMEOUT);
    CHECK_EQ(isotp_receive(&L, g_out, sizeof(g_out), NULL, NULL), ISOTP_NO_DATA);

    /* A fresh transfer still works after the timeout. */
    feed_ff(RXID, msg, 20);
    feed_cf(RXID, 1, &msg[6],  7);
    feed_cf(RXID, 2, &msg[13], 7);
    uint16_t len = 0;
    CHECK_EQ(isotp_receive(&L, g_out, sizeof(g_out), &len, NULL), ISOTP_OK);
    CHECK_EQ(len, 20);
}

/* N_Cr must be restarted by every CF, not just by the First Frame. */
static void test_n_cr_restarts_on_each_cf(void)
{
    link_setup(0, 0, FUNCID);
    uint8_t msg[27];
    fill_pattern(msg, sizeof(msg));

    feed_ff(RXID, msg, sizeof(msg));
    for (int i = 0; i < 3; ++i) {
        /* 900 ms of silence between every CF: under the 1000 ms N_Cr only if
         * the timer is restarted by each one. */
        g_ms += 900;
        isotp_poll(&L);
        CHECK_MSG(L.rx_phase == ISOTP_RECV, "N_Cr expired before CF %d", i + 1);
        feed_cf(RXID, (uint8_t)(i + 1), &msg[6 + i * 7], 7);
    }
    uint16_t len = 0;
    CHECK_EQ(isotp_receive(&L, g_out, sizeof(g_out), &len, NULL), ISOTP_OK);
    CHECK_EQ(len, sizeof(msg));
    CHECK_MEM(g_out, msg, sizeof(msg));
}

/* ==========================================================================
 * Functional addressing — the broadcast must not be able to steer a
 * physically addressed conversation.
 * ========================================================================== */

static void test_functional_single_frame_accepted_when_idle(void)
{
    link_setup(0, 0, FUNCID);
    const uint8_t msg[2] = { 0x3E, 0x00 };
    feed_sf(FUNCID, msg, 2);

    uint16_t len = 0;
    bool     fn  = false;
    REQUIRE_EQ(isotp_receive(&L, g_out, sizeof(g_out), &len, &fn), ISOTP_OK);
    CHECK_EQ(len, 2);
    CHECK_EQ(fn, true);
    CHECK_MEM(g_out, msg, 2);
}

static void test_functional_cf_cannot_abort_a_transfer(void)
{
    link_setup(0, 0, FUNCID);
    uint8_t msg[20];
    fill_pattern(msg, sizeof(msg));

    feed_ff(RXID, msg, 20);
    feed_cf(RXID, 1, &msg[6], 7);
    CHECK_EQ(L.rx_phase, ISOTP_RECV);
    CHECK_EQ(L.rx_sn, 2);

    /* A stranger's CF on the broadcast address, with a sequence number that
     * would be a fatal SN error if it were honoured. */
    const uint8_t junk[7] = { 9, 9, 9, 9, 9, 9, 9 };
    feed_cf(FUNCID, 7, junk, 7);

    CHECK_EQ(L.rx_phase, ISOTP_RECV);
    CHECK_EQ(L.rx_sn, 2);
    CHECK_EQ(L.rx_result, ISOTP_OK);
    CHECK_EQ(L.rx_off, 13);

    /* The real peer finishes undisturbed. */
    feed_cf(RXID, 2, &msg[13], 7);
    uint16_t len = 0;
    REQUIRE_EQ(isotp_receive(&L, g_out, sizeof(g_out), &len, NULL), ISOTP_OK);
    CHECK_EQ(len, 20);
    CHECK_MEM(g_out, msg, 20);
}

static void test_functional_fc_cannot_steer_a_send(void)
{
    link_setup(0, 0, FUNCID);
    uint8_t msg[20];
    fill_pattern(msg, sizeof(msg));

    REQUIRE_EQ(isotp_send(&L, msg, sizeof(msg)), ISOTP_INPROGRESS);
    CHECK_EQ(L.tx_phase, ISOTP_WAIT_FC);

    /* Neither an abort... */
    feed_fc(FUNCID, ISOTP_FS_OVFLW, 0, 0);
    CHECK_EQ(L.tx_phase, ISOTP_WAIT_FC);
    /* ...nor a grant. */
    feed_fc(FUNCID, ISOTP_FS_CTS, 0, 0);
    CHECK_EQ(L.tx_phase, ISOTP_WAIT_FC);
    isotp_poll(&L);
    CHECK_EQ(g_cap_n, 1);

    /* ...nor a stall that would burn the WAIT budget. */
    for (int i = 0; i < 12; ++i) {
        feed_fc(FUNCID, ISOTP_FS_WAIT, 0, 0);
    }
    CHECK_EQ(L.tx_phase, ISOTP_WAIT_FC);
    CHECK_EQ(L.tx_wft, 0);

    /* The real peer's grant still works. */
    feed_fc(RXID, ISOTP_FS_CTS, 0, 0);
    CHECK_EQ(L.tx_phase, ISOTP_SEND);
    isotp_poll(&L);
    CHECK_EQ(g_cap_n, 2);
}

static void test_functional_ff_is_ignored(void)
{
    link_setup(0, 0, FUNCID);
    uint8_t msg[20];
    fill_pattern(msg, sizeof(msg));

    feed_ff(FUNCID, msg, 20);
    CHECK_EQ(L.rx_phase, ISOTP_IDLE);
    CHECK_EQ(g_cap_n, 0);                 /* no FC granted to a broadcast      */
    CHECK_EQ(isotp_receive(&L, g_out, sizeof(g_out), NULL, NULL), ISOTP_NO_DATA);
}

static void test_functional_sf_cannot_clobber_state(void)
{
    uint8_t msg[20];
    uint8_t intruder[4] = { 0xEE, 0xEE, 0xEE, 0xEE };
    fill_pattern(msg, sizeof(msg));

    /* (a) mid-reassembly */
    link_setup(0, 0, FUNCID);
    feed_ff(RXID, msg, 20);
    feed_cf(RXID, 1, &msg[6], 7);
    feed_sf(FUNCID, intruder, 4);
    CHECK_EQ(L.rx_phase, ISOTP_RECV);
    CHECK_EQ(L.rx_off, 13);
    feed_cf(RXID, 2, &msg[13], 7);
    uint16_t len = 0;
    bool     fn  = true;
    REQUIRE_EQ(isotp_receive(&L, g_out, sizeof(g_out), &len, &fn), ISOTP_OK);
    CHECK_EQ(len, 20);
    CHECK_EQ(fn, false);
    CHECK_MEM(g_out, msg, 20);

    /* (b) message completed but not yet collected */
    link_setup(0, 0, FUNCID);
    feed_sf(RXID, msg, 5);
    feed_sf(FUNCID, intruder, 4);
    len = 0;
    fn  = true;
    REQUIRE_EQ(isotp_receive(&L, g_out, sizeof(g_out), &len, &fn), ISOTP_OK);
    CHECK_EQ(len, 5);
    CHECK_EQ(fn, false);
    CHECK_MEM(g_out, msg, 5);
}

/* With no functional address configured, the sentinel must never match. */
static void test_no_func_id_ignores_everything_else(void)
{
    link_setup(0, 0, ISOTP_NO_FUNC_ID);
    const uint8_t msg[3] = { 1, 2, 3 };

    feed_sf(FUNCID, msg, 3);
    CHECK_EQ(isotp_receive(&L, g_out, sizeof(g_out), NULL, NULL), ISOTP_NO_DATA);
    feed_sf(ISOTP_NO_FUNC_ID, msg, 3);
    CHECK_EQ(isotp_receive(&L, g_out, sizeof(g_out), NULL, NULL), ISOTP_NO_DATA);

    feed_sf(RXID, msg, 3);
    uint16_t len = 0;
    CHECK_EQ(isotp_receive(&L, g_out, sizeof(g_out), &len, NULL), ISOTP_OK);
    CHECK_EQ(len, 3);
}

/* ==========================================================================
 * Guards
 * ========================================================================== */

static void test_null_and_zero_length_frames_are_safe(void)
{
    link_setup(0, 0, FUNCID);
    const uint8_t d[ISOTP_CAN_DL] = { 0 };

    isotp_on_can_frame(NULL, RXID, d, 8);
    isotp_on_can_frame(&L, RXID, NULL, 8);
    isotp_on_can_frame(&L, RXID, d, 0);
    isotp_on_can_frame(&L, RXID, d, ISOTP_CAN_DL + 1u);
    isotp_poll(NULL);
    CHECK_EQ(isotp_receive(NULL, g_out, sizeof(g_out), NULL, NULL), ISOTP_ERROR);
    CHECK_EQ(isotp_receive(&L, NULL, 0, NULL, NULL), ISOTP_ERROR);

    CHECK_EQ(g_cap_n, 0);
    CHECK_EQ(L.rx_phase, ISOTP_IDLE);
}

/* ========================================================================== */

void suite_isotp(void);

void suite_isotp(void)
{
    printf("isotp\n");
    RUN(test_send_single_frame_pads);
    RUN(test_send_rejects_bad_sizes_and_reentry);
    RUN(test_send_tx_failure_aborts);
    RUN(test_receive_single_frame);
    RUN(test_receive_rejects_bad_single_frames);
    RUN(test_receive_ff_cf_happy_path);
    RUN(test_receive_sn_wraps_through_zero);
    RUN(test_receive_short_first_frame_rejected);
    RUN(test_receive_ff_dl_must_be_multiframe);
    RUN(test_receive_wrong_sn_aborts);
    RUN(test_receive_short_mid_block_cf_aborts);
    RUN(test_receive_final_cf_may_be_short);
    RUN(test_receive_stray_cf_when_idle_is_ignored);
    RUN(test_receive_out_buffer_too_small_keeps_message);
    RUN(test_receive_block_size_grants);
    RUN(test_send_multiframe_with_block_size);
    RUN(test_send_stmin_paces_consecutive_frames);
    RUN(test_send_wait_frames_bounded_by_wft_max);
    RUN(test_send_overflow_fc_aborts);
    RUN(test_n_bs_timeout_releases_the_sender);
    RUN(test_n_cr_timeout_drops_the_reassembly);
    RUN(test_n_cr_restarts_on_each_cf);
    RUN(test_functional_single_frame_accepted_when_idle);
    RUN(test_functional_cf_cannot_abort_a_transfer);
    RUN(test_functional_fc_cannot_steer_a_send);
    RUN(test_functional_ff_is_ignored);
    RUN(test_functional_sf_cannot_clobber_state);
    RUN(test_no_func_id_ignores_everything_else);
    RUN(test_null_and_zero_length_frames_are_safe);
}
