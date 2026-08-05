/*
 * SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
/*
 * uds.h — minimal UDS (ISO 14229-1) server over ISO-TP (bench mode)
 * ============================================================================
 * Thin service dispatcher on top of isotp_link_t. Pulls a reassembled request,
 * routes by SID to a registered handler, and sends the positive/negative
 * response back through isotp_send. Never touches CAN directly — transport is
 * isolated in isotp.[ch].
 *
 * THREADING: poll-driven on the single bench task, after isotp_poll(). Not
 * ISR-safe. Static response buffers; no malloc.
 * ============================================================================
 */
#ifndef PCAN_UDS_H
#define PCAN_UDS_H

#include <stdbool.h>
#include <stdint.h>
#include "isotp.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Response SID offset and negative-response marker. */
#define UDS_POS_RESP_OFFSET   0x40u
#define UDS_NEG_RESP_SID      0x7Fu

/* Service IDs (subset implemented for bench). */
#define UDS_SID_DIAG_SESSION  0x10u
#define UDS_SID_ECU_RESET     0x11u
#define UDS_SID_READ_DID      0x22u
#define UDS_SID_SECURITY      0x27u
#define UDS_SID_ROUTINE       0x31u
#define UDS_SID_REQ_DOWNLOAD  0x34u
#define UDS_SID_TRANSFER_DATA 0x36u
#define UDS_SID_REQ_XFER_EXIT 0x37u
#define UDS_SID_TESTER_PRES   0x3Eu

/* Diagnostic sessions. */
#define UDS_SESSION_DEFAULT   0x01u
#define UDS_SESSION_PROG      0x02u
#define UDS_SESSION_EXTENDED  0x03u

/* Suppress-positive-response bit in a sub-function byte. */
#define UDS_SUPPRESS_POS_RSP  0x80u

/* Negative Response Codes (ISO 14229-1). */
enum {
    NRC_GENERAL_REJECT = 0x10, NRC_SNS   = 0x11, NRC_SFNS  = 0x12,
    NRC_IMLOIF         = 0x13, NRC_BRR   = 0x21, NRC_CNC   = 0x22,
    NRC_RSE            = 0x24, NRC_ROOR  = 0x31, NRC_SAD   = 0x33,
    NRC_IK             = 0x35, NRC_ENOA  = 0x36, NRC_RTDNE = 0x37,
    NRC_RCRRP          = 0x78, NRC_SFNSIAS = 0x7E, NRC_SNSIAS = 0x7F,
};

/* Default timing (ms). */
#define UDS_P2_SERVER_MS      50u
#define UDS_P2_STAR_SERVER_MS 5000u
#define UDS_S3_SERVER_MS      5000u

/* Handler dispatch outcome. */
typedef enum {
    UDS_POSITIVE = 0,          /* handler wrote a positive response into resp   */
    UDS_NEGATIVE,              /* handler set c->nrc; dispatcher sends 0x7F..    */
    UDS_RESPONSE_PENDING,      /* dispatcher emits 7F <sid> 78, re-polls handler */
    UDS_NO_RESPONSE,           /* suppress / functional: send nothing            */
} uds_disp_t;

/* Per-request context passed to a handler. */
typedef struct {
    const uint8_t *req;      /* SID + payload                                  */
    uint16_t       req_len;
    uint8_t       *resp;     /* handler fills, INCLUDING the SID+0x40 byte      */
    uint16_t       resp_cap;
    uint16_t       resp_len; /* handler sets total bytes written                */
    uint8_t        nrc;      /* handler sets when returning UDS_NEGATIVE         */
    bool           functional; /* request arrived on the functional address     */
    uint8_t        session;  /* current diagnostic session                      */
    void          *user;     /* server->user passthrough                        */
} uds_ctx_t;

typedef uds_disp_t (*uds_handler_fn)(uds_ctx_t *c);

/* Service registration entry. `min_session_mask` gates by active session
 * (bitmask of (1<<session)); 0 = any session. */
typedef struct {
    uint8_t        sid;
    uint8_t        min_session_mask;
    uds_handler_fn fn;
} uds_service_t;

/* Server instance. */
typedef struct {
    isotp_link_t        *link;
    const uds_service_t *services;
    uint8_t              n_services;
    uint8_t              session;          /* current session                  */
    uint32_t             s3_timer_ms;      /* counts up; reset by TesterPresent */
    uint32_t             s3_timeout_ms;    /* UDS_S3_SERVER_MS                  */
    uint8_t              sec_level;        /* SecurityAccess unlocked level     */
    uint32_t             sec_seed;         /* last issued seed                  */
    uint32_t             resp_dropped;     /* responses the busy link refused   */
    void                *user;

    /* scratch response buffer (ISOTP_MAX_MSG) lives in uds.c to keep the
     * struct small; a pointer is stored here at init. */
    uint8_t             *resp_buf;
    uint16_t             resp_cap;
} uds_server_t;

/*
 * Initialize the server: bind the isotp link, service table, and user pointer.
 * `resp_buf`/`resp_cap` is caller-provided scratch for building responses
 * (size to ISOTP_MAX_MSG). Sets session=DEFAULT and starts the S3 timer.
 */
void uds_server_init(uds_server_t *s, isotp_link_t *link,
                     const uds_service_t *svcs, uint8_t n,
                     uint8_t *resp_buf, uint16_t resp_cap, void *user);

/*
 * Pump one iteration: pull a completed request from the isotp link, dispatch
 * to the matching handler, build and isotp_send the response (handling
 * suppress-pos-rsp, negative responses, and 0x78 ResponsePending), and age the
 * S3 session timer (falling back to the default session and re-locking security
 * on expiry). A response the link refuses becomes NRC 0x21 busyRepeatRequest
 * where the link can still take one; while an earlier multi-frame response is
 * draining it cannot, and the request is counted in resp_dropped and answered
 * with silence — the tester waits out P2 and retries.
 * `elapsed_ms` is the time since the last call (for S3 aging). Call each tick.
 */
void uds_server_poll(uds_server_t *s, uint32_t elapsed_ms);

/* -------------------------------------------------------------------------
 * Built-in handlers (registered by the bench app in a uds_service_t[]). Each
 * follows uds_handler_fn. Provided so the bench has a working default ECU;
 * override by supplying your own table.
 * ------------------------------------------------------------------------- */
uds_disp_t uds_h_diag_session(uds_ctx_t *c);
uds_disp_t uds_h_ecu_reset(uds_ctx_t *c);
uds_disp_t uds_h_read_did(uds_ctx_t *c);
uds_disp_t uds_h_security(uds_ctx_t *c);
uds_disp_t uds_h_routine(uds_ctx_t *c);
uds_disp_t uds_h_tester_present(uds_ctx_t *c);
uds_disp_t uds_h_request_download(uds_ctx_t *c);
uds_disp_t uds_h_transfer_data(uds_ctx_t *c);
uds_disp_t uds_h_request_transfer_exit(uds_ctx_t *c);

#ifdef __cplusplus
}
#endif
#endif /* PCAN_UDS_H */
