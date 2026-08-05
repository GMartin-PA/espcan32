/*
 * SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
/*
 * test_uds.c — UDS dispatcher and built-in handlers
 * ============================================================================
 * Driven end to end through a real isotp_link_t: a request goes in as CAN
 * frames on the physical or functional id, uds_server_poll() runs, and the
 * response is reassembled back out of the frames the link transmitted. That
 * keeps the tests honest about the things the dispatcher only gets right in
 * combination with the transport — functional addressing, a busy link, and a
 * multi-frame positive response.
 *
 * Time is a parameter (uds_server_poll takes elapsed_ms), so the S3 and
 * security-lockout tests step it directly rather than sleeping.
 * ============================================================================
 */
#include "test.h"

#include "esp_stubs.h"
#include "freertos/FreeRTOS.h"   /* pdMS_TO_TICKS, to check the reset delay   */
#include "isotp.h"
#include "uds.h"

#define TXID    0x7E8u
#define RXID    0x7E0u
#define FUNCID  0x7DFu

#define CAP_MAX 512

/* Mirrors UDS_SEC_KEY_XOR in uds.c — the bench key transform. */
#define SEC_KEY_XOR 0x5A5A5A5Au

typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[ISOTP_CAN_DL];
} cap_t;

static cap_t    g_cap[CAP_MAX];
static int      g_cap_n;
static int      g_tx_fail;
static uint32_t g_ms;

static isotp_link_t L;
static uds_server_t S;
static uint8_t      g_uds_resp_buf[ISOTP_MAX_MSG];

static uint8_t  g_resp[ISOTP_MAX_MSG];
static uint16_t g_resp_len;
static bool     g_resp_ok;

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
    ++g_cap_n;
    return 0;
}

static uint32_t fake_ms(void *user)
{
    (void)user;
    return g_ms;
}

/* ---- the bench ECU's service table (mirrors main.c) ---------------------- */

static const uds_service_t k_services[] = {
    { UDS_SID_DIAG_SESSION,  0, uds_h_diag_session          },
    { UDS_SID_ECU_RESET,     0, uds_h_ecu_reset             },
    { UDS_SID_READ_DID,      0, uds_h_read_did              },
    { UDS_SID_SECURITY,      0, uds_h_security              },
    { UDS_SID_ROUTINE,       0, uds_h_routine               },
    { UDS_SID_TESTER_PRES,   0, uds_h_tester_present        },
    { UDS_SID_REQ_DOWNLOAD,  0, uds_h_request_download      },
    { UDS_SID_TRANSFER_DATA, 0, uds_h_transfer_data         },
    { UDS_SID_REQ_XFER_EXIT, 0, uds_h_request_transfer_exit },
};
#define N_SERVICES ((uint8_t)(sizeof(k_services) / sizeof(k_services[0])))

static void ecu_setup_table(const uds_service_t *svcs, uint8_t n)
{
    isotp_cfg_t cfg;
    isotp_hal_t hal;

    memset(&cfg, 0, sizeof(cfg));
    cfg.pad_byte = 0xCCu;      /* bs 0 / stmin 0: no pacing to work around     */

    memset(&hal, 0, sizeof(hal));
    hal.can_tx = fake_tx;
    hal.millis = fake_ms;

    g_cap_n    = 0;
    g_tx_fail  = 0;
    g_ms       = 1000u;
    g_resp_len = 0;
    g_resp_ok  = false;

    isotp_init(&L, TXID, RXID, FUNCID, &cfg, &hal);
    uds_server_init(&S, &L, svcs, n, g_uds_resp_buf, sizeof(g_uds_resp_buf),
                    NULL);
}

static void ecu_setup(void)
{
    ecu_setup_table(k_services, N_SERVICES);
}

/* ---- request injection --------------------------------------------------- */

static void feed(uint32_t id, const uint8_t *d, uint8_t dlc)
{
    isotp_on_can_frame(&L, id, d, dlc);
}

static void feed_request_frames(uint32_t id, const uint8_t *req, uint16_t n)
{
    uint8_t f[ISOTP_CAN_DL];

    if (n <= 7u) {
        memset(f, 0xCC, sizeof(f));
        f[0] = (uint8_t)n;
        memcpy(&f[1], req, n);
        feed(id, f, ISOTP_CAN_DL);
        return;
    }

    f[0] = (uint8_t)(0x10u | ((n >> 8) & 0x0Fu));
    f[1] = (uint8_t)(n & 0xFFu);
    memcpy(&f[2], req, 6);
    feed(id, f, ISOTP_CAN_DL);

    uint16_t off = 6;
    uint8_t  sn  = 1;
    while (off < n) {
        uint16_t take = (uint16_t)(n - off);
        if (take > 7u) {
            take = 7u;
        }
        memset(f, 0xCC, sizeof(f));
        f[0] = (uint8_t)(0x20u | (sn & 0x0Fu));
        memcpy(&f[1], &req[off], take);
        feed(id, f, (uint8_t)(take == 7u ? 8u : take + 1u));
        off = (uint16_t)(off + take);
        sn  = (uint8_t)((sn + 1u) & 0x0Fu);
    }
}

/* Grant flow control and pump until the link has nothing left to send. */
static void drain_tx(void)
{
    for (int guard = 0; guard < 4096 && L.tx_phase != ISOTP_IDLE; ++guard) {
        if (L.tx_phase == ISOTP_WAIT_FC) {
            const uint8_t fc[3] = { 0x30u, 0x00u, 0x00u };
            feed(RXID, fc, 3);
        }
        isotp_poll(&L);
    }
}

/* Reassemble whatever the ECU transmitted. Flow-control frames belong to the
 * receive side of the link, not to the response, so they are skipped. */
static void collect_response(void)
{
    uint16_t expect = 0, off = 0;

    g_resp_len = 0;
    g_resp_ok  = false;

    for (int i = 0; i < g_cap_n && i < CAP_MAX; ++i) {
        if (g_cap[i].id != TXID) {
            continue;
        }
        const uint8_t *d = g_cap[i].data;
        switch (d[0] >> 4) {
        case ISOTP_PCI_SF: {
            const uint8_t n = (uint8_t)(d[0] & 0x0Fu);
            memcpy(g_resp, &d[1], n);
            g_resp_len = n;
            g_resp_ok  = true;
            expect     = 0;
            break;
        }
        case ISOTP_PCI_FF:
            expect = (uint16_t)(((uint16_t)(d[0] & 0x0Fu) << 8) | d[1]);
            memcpy(g_resp, &d[2], 6);
            off = 6;
            break;
        case ISOTP_PCI_CF: {
            if (expect == 0) {
                break;
            }
            uint16_t take = (uint16_t)(expect - off);
            if (take > 7u) {
                take = 7u;
            }
            memcpy(&g_resp[off], &d[1], take);
            off = (uint16_t)(off + take);
            if (off >= expect) {
                g_resp_len = expect;
                g_resp_ok  = true;
                expect     = 0;
            }
            break;
        }
        default:
            break;
        }
    }
}

/* One request/response exchange. */
static void request_on(uint32_t id, const uint8_t *req, uint16_t n)
{
    g_cap_n = 0;
    feed_request_frames(id, req, n);
    uds_server_poll(&S, 0);
    drain_tx();
    collect_response();
}

static void request(const uint8_t *req, uint16_t n)
{
    request_on(RXID, req, n);
}

static void request_func(const uint8_t *req, uint16_t n)
{
    request_on(FUNCID, req, n);
}

/* Number of frames the ECU transmitted that are part of a response (i.e. not
 * the flow control our own receive side emitted). */
static int response_frames(void)
{
    int n = 0;
    for (int i = 0; i < g_cap_n && i < CAP_MAX; ++i) {
        if (g_cap[i].id == TXID && (g_cap[i].data[0] >> 4) != ISOTP_PCI_FC) {
            ++n;
        }
    }
    return n;
}

/* Count single-frame negative responses matching sid/nrc. */
static int count_nrc(uint8_t sid, uint8_t nrc)
{
    int n = 0;
    for (int i = 0; i < g_cap_n && i < CAP_MAX; ++i) {
        const uint8_t *d = g_cap[i].data;
        if (g_cap[i].id == TXID && d[0] == 0x03u && d[1] == UDS_NEG_RESP_SID &&
            d[2] == sid && d[3] == nrc) {
            ++n;
        }
    }
    return n;
}

static void check_resp(const uint8_t *want, uint16_t n, const char *what)
{
    CHECK_MSG(g_resp_ok, "%s: no response", what);
    if (!g_resp_ok) {
        return;
    }
    CHECK_MSG(g_resp_len == n, "%s: response is %u bytes, want %u", what,
              g_resp_len, n);
    if (g_resp_len == n) {
        CHECK_MEM(g_resp, want, n);
    }
}

static void check_nrc(uint8_t sid, uint8_t nrc, const char *what)
{
    const uint8_t want[3] = { UDS_NEG_RESP_SID, sid, nrc };
    check_resp(want, 3, what);
}

static void check_silence(const char *what)
{
    CHECK_MSG(response_frames() == 0, "%s: expected silence, got %d frame(s)",
              what, response_frames());
}

static uint32_t be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

/* Unlock SecurityAccess level 1. Returns the seed the ECU issued. */
static uint32_t do_unlock(void)
{
    const uint8_t seed_req[2] = { UDS_SID_SECURITY, 0x01u };
    request(seed_req, sizeof(seed_req));
    if (!g_resp_ok || g_resp_len != 6) {
        return 0;
    }
    const uint32_t seed = be32(&g_resp[2]);
    const uint32_t key  = seed ^ SEC_KEY_XOR;

    const uint8_t key_req[6] = {
        UDS_SID_SECURITY, 0x02u,
        (uint8_t)(key >> 24), (uint8_t)(key >> 16),
        (uint8_t)(key >> 8),  (uint8_t)key,
    };
    request(key_req, sizeof(key_req));
    return seed;
}

/* ==========================================================================
 * Dispatcher basics
 * ========================================================================== */

static void test_tester_present(void)
{
    ecu_setup();
    const uint8_t req[2]  = { UDS_SID_TESTER_PRES, 0x00u };
    const uint8_t want[2] = { 0x7Eu, 0x00u };
    request(req, sizeof(req));
    check_resp(want, sizeof(want), "TesterPresent");
}

static void test_suppress_positive_response(void)
{
    ecu_setup();
    const uint8_t req[2] = { UDS_SID_TESTER_PRES, 0x80u };
    request(req, sizeof(req));
    check_silence("TesterPresent with suppressPosRsp");
}

static void test_diagnostic_session_control(void)
{
    ecu_setup();
    /* P2 = 50 ms, P2* = 5000 ms reported in 10 ms units. */
    const uint8_t req[2]  = { UDS_SID_DIAG_SESSION, UDS_SESSION_EXTENDED };
    const uint8_t want[6] = { 0x50u, UDS_SESSION_EXTENDED,
                              0x00u, 0x32u, 0x01u, 0xF4u };
    request(req, sizeof(req));
    check_resp(want, sizeof(want), "DiagnosticSessionControl(extended)");
    CHECK_EQ(S.session, UDS_SESSION_EXTENDED);

    const uint8_t bad[2] = { UDS_SID_DIAG_SESSION, 0x42u };
    request(bad, sizeof(bad));
    check_nrc(UDS_SID_DIAG_SESSION, NRC_SFNS, "unknown session");
}

static void test_unknown_service(void)
{
    ecu_setup();
    const uint8_t req[2] = { 0x28u, 0x00u };   /* CommunicationControl          */

    request(req, sizeof(req));
    check_nrc(0x28u, NRC_SNS, "unknown SID, physical");

    request_func(req, sizeof(req));
    check_silence("unknown SID, functional");
}

static void test_session_gating(void)
{
    static const uds_service_t gated[] = {
        { UDS_SID_DIAG_SESSION, 0, uds_h_diag_session },
        { UDS_SID_ROUTINE, (uint8_t)(1u << UDS_SESSION_EXTENDED), uds_h_routine },
    };
    ecu_setup_table(gated, 2);

    const uint8_t rc[4] = { UDS_SID_ROUTINE, 0x01u, 0x02u, 0x03u };
    request(rc, sizeof(rc));
    check_nrc(UDS_SID_ROUTINE, NRC_SNSIAS, "RoutineControl in default session");

    request_func(rc, sizeof(rc));
    check_silence("session-gated NRC on functional");

    const uint8_t ext[2] = { UDS_SID_DIAG_SESSION, UDS_SESSION_EXTENDED };
    request(ext, sizeof(ext));

    const uint8_t want[5] = { 0x71u, 0x01u, 0x02u, 0x03u, 0x00u };
    request(rc, sizeof(rc));
    check_resp(want, sizeof(want), "RoutineControl in extended session");
}

static void test_read_data_by_identifier(void)
{
    ecu_setup();
    /* 0x62 + DID + 17-byte VIN = a 20-byte, multi-frame positive response. */
    const uint8_t req[3] = { UDS_SID_READ_DID, 0xF1u, 0x90u };
    uint8_t want[20];
    want[0] = 0x62u;
    want[1] = 0xF1u;
    want[2] = 0x90u;
    memcpy(&want[3], "ZZZBENCH0P0000001", 17);

    request(req, sizeof(req));
    check_resp(want, sizeof(want), "ReadDataByIdentifier(VIN)");

    const uint8_t unknown[3] = { UDS_SID_READ_DID, 0x00u, 0x01u };
    request(unknown, sizeof(unknown));
    check_nrc(UDS_SID_READ_DID, NRC_ROOR, "unknown DID, physical");

    request_func(unknown, sizeof(unknown));
    check_silence("unknown DID, functional");

    const uint8_t odd[4] = { UDS_SID_READ_DID, 0xF1u, 0x90u, 0x00u };
    request(odd, sizeof(odd));
    check_nrc(UDS_SID_READ_DID, NRC_IMLOIF, "odd-length DID list");
}

/* §A.2 lists the NRCs a broadcast must be answered with silence; everything
 * else still gets a negative response. */
static void test_functional_nrc_only_suppressed_when_required(void)
{
    ecu_setup();
    const uint8_t truncated[1] = { UDS_SID_TESTER_PRES };

    request(truncated, sizeof(truncated));
    check_nrc(UDS_SID_TESTER_PRES, NRC_IMLOIF, "short TesterPresent, physical");

    request_func(truncated, sizeof(truncated));
    check_nrc(UDS_SID_TESTER_PRES, NRC_IMLOIF, "short TesterPresent, functional");
}

/* ==========================================================================
 * SecurityAccess
 * ========================================================================== */

static void test_security_seed_key_unlock(void)
{
    stub_reset();
    ecu_setup();

    const uint32_t expect_seed = stub_random_peek();
    const uint8_t  seed_req[2] = { UDS_SID_SECURITY, 0x01u };
    request(seed_req, sizeof(seed_req));

    REQUIRE(g_resp_ok);
    REQUIRE_EQ(g_resp_len, 6);
    CHECK_EQ(g_resp[0], 0x67u);
    CHECK_EQ(g_resp[1], 0x01u);
    const uint32_t seed = be32(&g_resp[2]);
    CHECK_EQ(seed, expect_seed);
    CHECK(seed != 0);
    CHECK_EQ(S.sec_seed, seed);
    CHECK_EQ(S.sec_level, 0);

    const uint32_t key = seed ^ SEC_KEY_XOR;
    const uint8_t  key_req[6] = {
        UDS_SID_SECURITY, 0x02u,
        (uint8_t)(key >> 24), (uint8_t)(key >> 16),
        (uint8_t)(key >> 8),  (uint8_t)key,
    };
    const uint8_t want[2] = { 0x67u, 0x02u };
    request(key_req, sizeof(key_req));
    check_resp(want, sizeof(want), "sendKey");
    CHECK_EQ(S.sec_level, 0x01u);

    /* Re-requesting a seed at an already-unlocked level yields an all-zero
     * challenge rather than a fresh one. */
    request(seed_req, sizeof(seed_req));
    REQUIRE_EQ(g_resp_len, 6);
    CHECK_EQ(be32(&g_resp[2]), 0u);
}

static void test_security_rejects_unimplemented_levels(void)
{
    const uint8_t subs[] = { 0x00u, 0x03u, 0x04u, 0x05u, 0x7Fu, 0x80u, 0x81u, 0xFFu };

    for (unsigned i = 0; i < sizeof(subs) / sizeof(subs[0]); ++i) {
        ecu_setup();
        const uint8_t req[6] = { UDS_SID_SECURITY, subs[i], 0, 0, 0, 0 };
        request(req, sizeof(req));
        CHECK_MSG(g_resp_ok && g_resp_len == 3 && g_resp[2] == NRC_SFNS,
                  "sub-function 0x%02X must be rejected as unsupported", subs[i]);
        CHECK_EQ(S.sec_level, 0);
    }
}

static void test_security_key_without_seed(void)
{
    ecu_setup();
    const uint8_t key_req[6] = { UDS_SID_SECURITY, 0x02u, 0, 0, 0, 0 };
    request(key_req, sizeof(key_req));
    check_nrc(UDS_SID_SECURITY, NRC_RSE, "sendKey with no outstanding seed");

    /* A truncated key is a length error, not an authentication attempt. */
    const uint8_t seed_req[2] = { UDS_SID_SECURITY, 0x01u };
    request(seed_req, sizeof(seed_req));
    const uint8_t short_key[3] = { UDS_SID_SECURITY, 0x02u, 0x00u };
    request(short_key, sizeof(short_key));
    check_nrc(UDS_SID_SECURITY, NRC_IMLOIF, "short sendKey");
}

static void test_security_lockout_after_repeated_bad_keys(void)
{
    stub_reset();
    ecu_setup();

    const uint8_t seed_req[2] = { UDS_SID_SECURITY, 0x01u };
    const uint8_t bad_key[6]  = { UDS_SID_SECURITY, 0x02u, 0xDE, 0xAD, 0xBE, 0xEF };

    for (int attempt = 1; attempt <= 3; ++attempt) {
        request(seed_req, sizeof(seed_req));
        REQUIRE_EQ(g_resp_len, 6);
        request(bad_key, sizeof(bad_key));
        REQUIRE_EQ(g_resp_len, 3);
        /* The third failure trips the delay and says so with 0x36. */
        CHECK_MSG(g_resp[2] == (attempt < 3 ? NRC_IK : NRC_ENOA),
                  "attempt %d: got NRC 0x%02X", attempt, g_resp[2]);
    }

    request(seed_req, sizeof(seed_req));
    check_nrc(UDS_SID_SECURITY, NRC_RTDNE, "requestSeed during lockout");

    /* The delay ages on the poll tick, with or without traffic. */
    uds_server_poll(&S, 9999);
    request(seed_req, sizeof(seed_req));
    check_nrc(UDS_SID_SECURITY, NRC_RTDNE, "requestSeed just before the delay ends");

    uds_server_poll(&S, 1);
    request(seed_req, sizeof(seed_req));
    REQUIRE_EQ(g_resp_len, 6);
    CHECK_EQ(g_resp[0], 0x67u);
    CHECK(be32(&g_resp[2]) != 0);
}

/* ==========================================================================
 * Session / S3 interaction with the unlock
 * ========================================================================== */

static void test_s3_expiry_relocks_in_default_session(void)
{
    stub_reset();
    ecu_setup();
    CHECK_EQ(S.session, UDS_SESSION_DEFAULT);
    do_unlock();
    REQUIRE_EQ(S.sec_level, 0x01u);

    /* Just short of S3: the unlock survives. */
    uds_server_poll(&S, UDS_S3_SERVER_MS - 1u);
    CHECK_EQ(S.sec_level, 0x01u);

    uds_server_poll(&S, 1);
    CHECK_EQ(S.sec_level, 0);
    CHECK_EQ(S.session, UDS_SESSION_DEFAULT);

    /* And the gated service is gated again. */
    const uint8_t dl[11] = { UDS_SID_REQ_DOWNLOAD, 0x00u, 0x44u,
                             0, 0, 0, 0, 0, 0, 0x10u, 0x00u };
    request(dl, sizeof(dl));
    check_nrc(UDS_SID_REQ_DOWNLOAD, NRC_SAD, "RequestDownload after S3 relock");
}

static void test_explicit_default_session_relocks(void)
{
    stub_reset();
    ecu_setup();
    do_unlock();
    REQUIRE_EQ(S.sec_level, 0x01u);

    /* Selecting the session we are already in still resets security. */
    const uint8_t def[2] = { UDS_SID_DIAG_SESSION, UDS_SESSION_DEFAULT };
    request(def, sizeof(def));
    REQUIRE(g_resp_ok);
    CHECK_EQ(g_resp[0], 0x50u);
    CHECK_EQ(S.sec_level, 0);

    /* A change of session relocks too. */
    do_unlock();
    REQUIRE_EQ(S.sec_level, 0x01u);
    const uint8_t ext[2] = { UDS_SID_DIAG_SESSION, UDS_SESSION_EXTENDED };
    request(ext, sizeof(ext));
    CHECK_EQ(S.session, UDS_SESSION_EXTENDED);
    CHECK_EQ(S.sec_level, 0);
}

/* ==========================================================================
 * RequestDownload / TransferData
 * ========================================================================== */

/* alfid 0x44: 4-byte address, 4-byte size. */
static void open_download(uint32_t size)
{
    const uint8_t dl[11] = {
        UDS_SID_REQ_DOWNLOAD, 0x00u, 0x44u,
        0x00u, 0x01u, 0x00u, 0x00u,
        (uint8_t)(size >> 24), (uint8_t)(size >> 16),
        (uint8_t)(size >> 8),  (uint8_t)size,
    };
    request(dl, sizeof(dl));
}

static void test_download_requires_security(void)
{
    stub_reset();
    ecu_setup();
    open_download(0x1000u);
    check_nrc(UDS_SID_REQ_DOWNLOAD, NRC_SAD, "RequestDownload while locked");

    const uint8_t td[3] = { UDS_SID_TRANSFER_DATA, 0x01u, 0xAAu };
    request(td, sizeof(td));
    check_nrc(UDS_SID_TRANSFER_DATA, NRC_RSE, "TransferData with no download");

    const uint8_t xe[1] = { UDS_SID_REQ_XFER_EXIT };
    request(xe, sizeof(xe));
    check_nrc(UDS_SID_REQ_XFER_EXIT, NRC_RSE, "TransferExit with no download");
}

static void test_download_happy_path(void)
{
    stub_reset();
    ecu_setup();
    do_unlock();
    REQUIRE_EQ(S.sec_level, 0x01u);

    const uint8_t want_dl[4] = { 0x74u, 0x20u, 0x04u, 0x02u };
    open_download(0x1000u);
    check_resp(want_dl, sizeof(want_dl), "RequestDownload");

    /* A second RequestDownload while one is open is a conditions-not-correct. */
    open_download(0x1000u);
    check_nrc(UDS_SID_REQ_DOWNLOAD, NRC_CNC, "second RequestDownload");

    const uint8_t b1[4] = { UDS_SID_TRANSFER_DATA, 0x01u, 0xAAu, 0xBBu };
    const uint8_t ack1[2] = { 0x76u, 0x01u };
    request(b1, sizeof(b1));
    check_resp(ack1, sizeof(ack1), "TransferData block 1");

    /* Retransmission of the last accepted block is acked idempotently. */
    request(b1, sizeof(b1));
    check_resp(ack1, sizeof(ack1), "TransferData block 1 retransmit");

    /* Out of sequence. */
    const uint8_t b5[3] = { UDS_SID_TRANSFER_DATA, 0x05u, 0xCCu };
    request(b5, sizeof(b5));
    check_nrc(UDS_SID_TRANSFER_DATA, 0x73u, "out-of-sequence block");

    const uint8_t b2[3]   = { UDS_SID_TRANSFER_DATA, 0x02u, 0xCCu };
    const uint8_t ack2[2] = { 0x76u, 0x02u };
    request(b2, sizeof(b2));
    check_resp(ack2, sizeof(ack2), "TransferData block 2");

    const uint8_t xe[1]  = { UDS_SID_REQ_XFER_EXIT };
    const uint8_t ack[1] = { 0x77u };
    request(xe, sizeof(xe));
    check_resp(ack, sizeof(ack), "RequestTransferExit");

    /* The transfer is closed. */
    request(b2, sizeof(b2));
    check_nrc(UDS_SID_TRANSFER_DATA, NRC_RSE, "TransferData after exit");
}

/* Was this exchange acked with 0x76 <bsc>? */
static bool acked(uint8_t bsc)
{
    return g_resp_ok && g_resp_len == 2 && g_resp[0] == 0x76u &&
           g_resp[1] == bsc;
}

/* The block sequence counter is a byte and wraps 0xFF -> 0x00; the
 * retransmission check has to wrap with it. */
static void test_transfer_data_bsc_wraps(void)
{
    stub_reset();
    ecu_setup();
    do_unlock();
    open_download(0x1000u);
    REQUIRE(g_resp_ok && g_resp_len == 4);

    /* 256 blocks of one byte each: BSC runs 1..255 and then wraps to 0x00. */
    uint8_t req[3] = { UDS_SID_TRANSFER_DATA, 0x00u, 0xA5u };
    for (int i = 0; i < 256; ++i) {
        const uint8_t bsc = (uint8_t)(i + 1);
        req[1] = bsc;
        request(req, sizeof(req));
        if (!acked(bsc)) {
            CHECK_MSG(0, "block %d (BSC 0x%02X) was not acked", i + 1, bsc);
            return;
        }
    }
    ++t_checks;

    /* The last accepted block is 0x00, so 0x00 is the retransmission and 0x01
     * is the next new block. Getting the wrap wrong here rejects both. */
    req[1] = 0x00u;
    request(req, sizeof(req));
    CHECK_MSG(acked(0x00u), "retransmission of the wrapped block was rejected");

    req[1] = 0x02u;
    request(req, sizeof(req));
    check_nrc(UDS_SID_TRANSFER_DATA, 0x73u, "block after the wrap, out of order");

    req[1] = 0x01u;
    request(req, sizeof(req));
    CHECK_MSG(acked(0x01u), "first block after the wrap was rejected");
}

/* BSC 0x00 before any block has been accepted looks like the retransmission of
 * a block that never existed; it must be refused rather than phantom-acked. */
static void test_transfer_data_rejects_spurious_first_block(void)
{
    stub_reset();
    ecu_setup();
    do_unlock();
    open_download(0x1000u);
    REQUIRE(g_resp_ok && g_resp_len == 4);

    const uint8_t b0[3] = { UDS_SID_TRANSFER_DATA, 0x00u, 0xAAu };
    request(b0, sizeof(b0));
    check_nrc(UDS_SID_TRANSFER_DATA, 0x73u, "BSC 0x00 before the first block");

    const uint8_t b2[3] = { UDS_SID_TRANSFER_DATA, 0x02u, 0xAAu };
    request(b2, sizeof(b2));
    check_nrc(UDS_SID_TRANSFER_DATA, 0x73u, "BSC 0x02 before the first block");

    /* The transfer is still usable — a bad counter is not fatal. */
    const uint8_t b1[3] = { UDS_SID_TRANSFER_DATA, 0x01u, 0xAAu };
    request(b1, sizeof(b1));
    CHECK(acked(0x01u));
}

/* The declared transfer size is the ceiling, whatever the block counter says. */
static void test_transfer_data_honours_declared_size(void)
{
    stub_reset();
    ecu_setup();
    do_unlock();
    open_download(4u);
    REQUIRE(g_resp_ok && g_resp_len == 4);

    const uint8_t too_big[7] = { UDS_SID_TRANSFER_DATA, 0x01u, 1, 2, 3, 4, 5 };
    request(too_big, sizeof(too_big));
    check_nrc(UDS_SID_TRANSFER_DATA, NRC_ROOR, "block larger than the transfer");

    const uint8_t fits[6] = { UDS_SID_TRANSFER_DATA, 0x01u, 1, 2, 3, 4 };
    request(fits, sizeof(fits));
    CHECK(acked(0x01u));

    const uint8_t overrun[3] = { UDS_SID_TRANSFER_DATA, 0x02u, 0xFFu };
    request(overrun, sizeof(overrun));
    check_nrc(UDS_SID_TRANSFER_DATA, NRC_ROOR, "block past the declared size");
}

/* ==========================================================================
 * ECUReset
 * ========================================================================== */

static void test_ecu_reset_restarts_after_replying(void)
{
    stub_reset();
    ecu_setup();

    const uint8_t req[2]  = { UDS_SID_ECU_RESET, 0x01u };
    const uint8_t want[2] = { 0x51u, 0x01u };
    request(req, sizeof(req));
    check_resp(want, sizeof(want), "ECUReset(hardReset)");
    CHECK_EQ(stub_restart_count(), 1u);
    /* The response is given time to reach the wire first. */
    CHECK_EQ(stub_delay_ticks_total(), pdMS_TO_TICKS(50));
}

static void test_ecu_reset_rapid_shutdown_does_not_restart(void)
{
    stub_reset();
    ecu_setup();

    const uint8_t req[2]  = { UDS_SID_ECU_RESET, 0x04u };
    const uint8_t want[2] = { 0x51u, 0x04u };
    request(req, sizeof(req));
    check_resp(want, sizeof(want), "ECUReset(enableRapidPowerShutDown)");
    CHECK_EQ(stub_restart_count(), 0u);

    const uint8_t bad[2] = { UDS_SID_ECU_RESET, 0x06u };
    request(bad, sizeof(bad));
    check_nrc(UDS_SID_ECU_RESET, NRC_SFNS, "ECUReset with unknown sub-function");
    CHECK_EQ(stub_restart_count(), 0u);
}

static void test_ecu_reset_suppressed_response_still_restarts(void)
{
    stub_reset();
    ecu_setup();

    const uint8_t req[2] = { UDS_SID_ECU_RESET, 0x81u };  /* suppressPosRsp    */
    request(req, sizeof(req));
    check_silence("ECUReset with suppressPosRsp");
    CHECK_EQ(stub_restart_count(), 1u);
}

/* A tester that was told to repeat its request must not find the ECU rebooted
 * behind its back. */
static void test_ecu_reset_not_taken_when_response_undeliverable(void)
{
    stub_reset();
    ecu_setup();

    const uint8_t req[2] = { UDS_SID_ECU_RESET, 0x01u };
    g_cap_n = 0;
    feed_request_frames(RXID, req, sizeof(req));
    g_tx_fail = 1;                       /* the bus refuses everything now      */
    uds_server_poll(&S, 0);

    CHECK_EQ(stub_restart_count(), 0u);
    CHECK(S.resp_dropped >= 1u);

    /* The server is still serving once the bus recovers. */
    g_tx_fail = 0;
    const uint8_t tp[2]   = { UDS_SID_TESTER_PRES, 0x00u };
    const uint8_t want[2] = { 0x7Eu, 0x00u };
    request(tp, sizeof(tp));
    check_resp(want, sizeof(want), "TesterPresent after a dropped response");
    CHECK_EQ(stub_restart_count(), 0u);
}

/* ==========================================================================
 * ResponsePending
 * ========================================================================== */

static unsigned g_pending_left;

static uds_disp_t h_pending(uds_ctx_t *c)
{
    if (g_pending_left > 0) {
        --g_pending_left;
        return UDS_RESPONSE_PENDING;
    }
    c->resp[0] = 0x6Eu;
    c->resp[1] = 0x12u;
    c->resp_len = 2;
    return UDS_POSITIVE;
}

static void test_response_pending_pattern(void)
{
    static const uds_service_t svcs[] = { { 0x2Eu, 0, h_pending } };
    const uint8_t req[2]  = { 0x2Eu, 0x12u };
    const uint8_t want[2] = { 0x6Eu, 0x12u };

    ecu_setup_table(svcs, 1);
    g_pending_left = 3;
    request(req, sizeof(req));
    CHECK_EQ(count_nrc(0x2Eu, NRC_RCRRP), 3);
    check_resp(want, sizeof(want), "settled ResponsePending");

    /* A handler that never settles is cut off, not looped forever. */
    ecu_setup_table(svcs, 1);
    g_pending_left = 1000;
    request(req, sizeof(req));
    CHECK_EQ(count_nrc(0x2Eu, NRC_RCRRP), 8);
    check_nrc(0x2Eu, NRC_GENERAL_REJECT, "runaway ResponsePending");
}

/* ========================================================================== */

void suite_uds(void);

void suite_uds(void)
{
    printf("uds\n");
    RUN(test_tester_present);
    RUN(test_suppress_positive_response);
    RUN(test_diagnostic_session_control);
    RUN(test_unknown_service);
    RUN(test_session_gating);
    RUN(test_read_data_by_identifier);
    RUN(test_functional_nrc_only_suppressed_when_required);
    RUN(test_security_seed_key_unlock);
    RUN(test_security_rejects_unimplemented_levels);
    RUN(test_security_key_without_seed);
    RUN(test_security_lockout_after_repeated_bad_keys);
    RUN(test_s3_expiry_relocks_in_default_session);
    RUN(test_explicit_default_session_relocks);
    RUN(test_download_requires_security);
    RUN(test_download_happy_path);
    RUN(test_transfer_data_bsc_wraps);
    RUN(test_transfer_data_rejects_spurious_first_block);
    RUN(test_transfer_data_honours_declared_size);
    RUN(test_ecu_reset_restarts_after_replying);
    RUN(test_ecu_reset_rapid_shutdown_does_not_restart);
    RUN(test_ecu_reset_suppressed_response_still_restarts);
    RUN(test_ecu_reset_not_taken_when_response_undeliverable);
    RUN(test_response_pending_pattern);
}
