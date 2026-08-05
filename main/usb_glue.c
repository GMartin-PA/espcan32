/*
 * SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
/*
 * usb_glue.c — TinyUSB init + custom PCAN class driver (esp_tinyusb v2.2.x)
 * ============================================================================
 * Installs TinyUSB with the PCAN descriptor set and owns the ONE vendor
 * interface (bInterfaceNumber 0) that carries all four bulk endpoints. Routing
 * is by ABSOLUTE ENDPOINT ADDRESS, not interface index:
 *   EP 0x01 OUT -> pcan_cmd_handle; 16-byte reply -> EP 0x81 IN.
 *   EP 0x02 OUT -> pcan_msg_decode_tx -> tx_ring.
 *   EP 0x82 IN  <- RX-ring batches (usb_glue_send_msg_batch).
 *
 * WHY A CUSTOM CLASS DRIVER --------------------------------------------------
 * The genuine PCAN-USB puts all four bulk endpoints under a single vendor
 * interface. TinyUSB's built-in vendor class (TUD_VENDOR_DESCRIPTOR) hardcodes
 * two endpoints per interface, so it cannot own this layout. We instead
 * register a usbd_class_driver_t via usbd_app_driver_get_cb() (a weak hook in
 * TinyUSB core, usbd.c: the default returns 0 drivers; our strong definition
 * overrides it) and drive the four endpoints directly with usbd_edpt_*().
 *
 * TINYUSB API VERSION -------------------------------------------------------
 * esp_tinyusb 2.2.x pins espressif/tinyusb >= 0.17.0. In 0.17/0.18 the class
 * driver hooks and endpoint API are:
 *   usbd_class_driver_t { name, init, deinit, reset, open, control_xfer_cb,
 *                         xfer_cb, sof }  (no xfer_isr member)
 *   usbd_edpt_xfer(rhport, ep_addr, buffer, total_bytes, is_isr)  -- FIVE args
 *                         in TinyUSB 0.21 (what esp_tinyusb 2.2.1 pins). All of
 *                         our calls run in the TinyUSB task (open/xfer_cb) or
 *                         another FreeRTOS task (send_*), never a raw ISR, so
 *                         is_isr = false everywhere.
 * Verified against the resolved managed_components tinyusb 0.21 usbd_pvt.h.
 *
 * DESCRIPTOR WIRING (esp_tinyusb v2.x nested API):
 *   esp_tinyusb OWNS tud_descriptor_{device,configuration,string}_cb as STRONG
 *   symbols. They serve whatever we place in tinyusb_config_t.descriptor.*:
 *     descriptor.device            = pcan_desc_device()          (18B dev desc)
 *     descriptor.full_speed_config = pcan_desc_configuration(0)  (cfg+itf+4 ep)
 *     descriptor.string            = pcan_desc_string_table()    (UTF-8 table)
 *   The string field must be a `const char **` array of UTF-8 strings (index 0
 *   is the 2-byte LANGID); esp_tinyusb's tud_descriptor_string_cb does the
 *   UTF-8 -> UTF-16LE conversion itself.
 *
 * DMA BUFFERS / THREADING ---------------------------------------------------
 *   Every buffer handed to usbd_edpt_xfer() must live in DMA-capable RAM and
 *   persist until its completion callback. We use static CFG_TUSB_MEM_SECTION /
 *   CFG_TUSB_MEM_ALIGN buffers (ESP32-S3 internal RAM is DMA-capable). EP1-OUT
 *   is re-armed inside xfer_cb; EP2-OUT stays unarmed until every decoded host
 *   frame has been queued (usb_glue_msg_out_deferred, TinyUSB task context);
 *   the IN buffers are single-owner via the MSG_IN_* state machine. A submit
 *   that fails leaves the OUT endpoint unarmed for the rest of the session, so
 *   both OUT paths record a "needs re-arm" flag that usb_glue_service() retries.
 *   NOTHING in a USB callback blocks.
 * ============================================================================
 */
#include "usb_glue.h"
#include "config.h"
#include "usb_descriptors.h"
#include "pcan_cmd.h"
#include "pcan_msg.h"
#include "ringbuf.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tusb.h"
#include "device/usbd_pvt.h"   /* usbd_class_driver_t, usbd_edpt_* */
#include "esp_log.h"
#include <string.h>

static const char *TAG = "usb_glue";

static pcan_dev_state_t *s_state;
static pcan_ringbuf_t   *s_tx_ring;  /* USB -> CAN (host TX frames pushed here)   */

/* rhport TinyUSB opened our interface on; set in open(), used by the send
 * paths. TinyUSB device is single-port on the S3, so this is always 0, but we
 * store it rather than hardcode. */
static uint8_t          s_rhport;
/* True between open() and reset(): the host has configured our interface and
 * the four endpoints are live. */
static volatile bool    s_itf_open;

/*
 * Max canonical frames decodable from one 64-byte EP2-OUT batch. The smallest
 * TX record is 3 bytes (1 SL byte + 2-byte std id, DLC 0, no data/ts), and the
 * 2-byte batch header is fixed, so (64 - 2) / 3 = 20 is the tight upper bound;
 * 32 gives comfortable headroom without a large stack footprint in the USB task.
 */
#define PCAN_TX_MAX_FRAMES_PER_BATCH  32

/* --------------------------------------------------------------------------
 * DMA-capable endpoint transfer buffers. Each buffer is owned by exactly one
 * endpoint and must outlive any in-flight transfer, so they are static:
 *   s_cmd_out : EP 0x01 OUT receive buffer (host commands)
 *   s_msg_out : EP 0x02 OUT receive buffer (host CAN TX batches)
 *   s_cmd_in  : EP 0x81 IN  transmit buffer (16-byte GET reply)
 *   s_msg_in  : EP 0x82 IN  transmit buffer (RX/status batch, up to 64 bytes)
 * OUT buffers are sized to the full 64-byte MPS so a max-length packet fits.
 * -------------------------------------------------------------------------- */
CFG_TUSB_MEM_SECTION CFG_TUSB_MEM_ALIGN static uint8_t s_cmd_out[PCAN_USB_EP_MPS];
CFG_TUSB_MEM_SECTION CFG_TUSB_MEM_ALIGN static uint8_t s_msg_out[PCAN_USB_EP_MPS];
CFG_TUSB_MEM_SECTION CFG_TUSB_MEM_ALIGN static uint8_t s_cmd_in[PCAN_USB_CMD_LEN];
CFG_TUSB_MEM_SECTION CFG_TUSB_MEM_ALIGN static uint8_t s_msg_in[PCAN_USB_EP_MPS];

/* EP2-OUT backpressure state. The endpoint stays unarmed while any decoded host
 * frame is pending, so the static OUT buffer cannot be overwritten. usb_tx_task
 * retries these frames via usb_glue_service(); no callback ever waits for room. */
static pcan_frame_t s_msg_pending[PCAN_TX_MAX_FRAMES_PER_BATCH];
static uint8_t      s_msg_pending_count;
static uint8_t      s_msg_pending_index;
static bool         s_msg_out_needs_rearm;
static uint32_t     s_usb_generation;

/* EP1-OUT recovery. A command transfer that fails to submit would leave the
 * command channel dead for the whole session (every host command then hits the
 * 1000 ms driver timeout), so the arm is retried from usb_glue_service(). */
static bool         s_cmd_out_needs_rearm;

typedef enum {
    MSG_IN_IDLE = 0,
    MSG_IN_QUEUED,
    MSG_IN_SUBMITTED,
    MSG_IN_COMPLETE,
    MSG_IN_RETRY,
    MSG_IN_DROPPED,
} msg_in_state_t;

static msg_in_state_t s_msg_in_state;
static uint16_t       s_msg_in_len;
static uint32_t       s_msg_in_generation;

static portMUX_TYPE s_msg_pending_lock = portMUX_INITIALIZER_UNLOCKED;
static usb_glue_stats_t s_stats;

void usb_glue_bind(pcan_dev_state_t *st, void *rx_ring, void *tx_ring)
{
    /* The CAN -> USB ring is drained by usb_tx_task, which owns it directly. */
    (void)rx_ring;
    s_state   = st;
    s_tx_ring = (pcan_ringbuf_t *)tx_ring;
}

esp_err_t usb_glue_install(void)
{
    const uint8_t *dev_desc = pcan_desc_device();
    const uint8_t *cfg_desc = pcan_desc_configuration(0);

    if (dev_desc == NULL || cfg_desc == NULL) {
        ESP_LOGE(TAG, "descriptor providers not ready (dev=%p cfg=%p)",
                 (const void *)dev_desc, (const void *)cfg_desc);
        return ESP_ERR_INVALID_STATE;
    }

    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
    tusb_cfg.task.xCoreID = CFG_TASK_TINYUSB_CORE;
    tusb_cfg.task.priority = CFG_TASK_TINYUSB_PRIO;
    tusb_cfg.descriptor.device            = (const tusb_desc_device_t *)dev_desc;
    tusb_cfg.descriptor.full_speed_config = cfg_desc;
    tusb_cfg.descriptor.string            = pcan_desc_string_table();
    tusb_cfg.descriptor.string_count      = PCAN_STRING_DESC_COUNT;
#if (TUD_OPT_HIGH_SPEED)
    /* The classic PCAN-USB is a full-speed device (bcdUSB 0x0100); reuse the FS
     * config if the S3 port is ever built high-speed so enumeration still works. */
    tusb_cfg.descriptor.high_speed_config = cfg_desc;
    tusb_cfg.descriptor.qualifier         = NULL;
#endif

    esp_err_t err = tinyusb_driver_install(&tusb_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "tinyusb_driver_install failed: %s", esp_err_to_name(err));
    }
    return err;
}

bool usb_glue_mounted(void)
{
    return tud_mounted() && s_itf_open;
}

/* ---- Command / message routing (TinyUSB task context, MUST NOT block) ----- */

/* Route a completed EP1-OUT command transfer. `buf`/`len` is one USB transfer;
 * the host issues fixed 16-byte commands, so we walk it in 16-byte units (a
 * defensive loop in case the host ever coalesces commands into one transfer). */
static void usb_glue_handle_cmd(const uint8_t *buf, uint16_t len)
{
    if (s_state == NULL) {
        return;
    }
    for (uint16_t off = 0; (uint16_t)(len - off) >= PCAN_USB_CMD_LEN; off += PCAN_USB_CMD_LEN) {
        uint8_t  reply[PCAN_USB_CMD_LEN];
        uint16_t reply_len = 0;
        (void)pcan_cmd_handle(s_state, buf + off, PCAN_USB_CMD_LEN, reply, &reply_len);
        if (reply_len > 0) {
            (void)usb_glue_send_cmd_reply(reply, reply_len);
        }
    }
}

/* Route a completed EP2-OUT TX batch into retained static storage. The OUT
 * endpoint is deliberately NOT re-armed here; usb_glue_service() first moves
 * every frame into tx_ring, applying USB backpressure instead of dropping. */
static void usb_glue_handle_msg(const uint8_t *buf, uint16_t len)
{
    pcan_frame_t frames[PCAN_TX_MAX_FRAMES_PER_BATCH];
    int n = pcan_msg_decode_tx(buf, len, frames, PCAN_TX_MAX_FRAMES_PER_BATCH);
    if (n < 0) {
        ESP_LOGW(TAG, "malformed EP2-OUT batch (len=%u)", (unsigned)len);
        n = 0;
    }

    portENTER_CRITICAL(&s_msg_pending_lock);
    if (s_msg_pending_index < s_msg_pending_count) {
        /* This should be unreachable because EP2-OUT stays unarmed while pending.
         * Preserve telemetry if a controller reset/driver race violates it. */
        s_stats.tx_pending_reset_drops +=
            (uint32_t)(s_msg_pending_count - s_msg_pending_index);
    }
    if (n > 0) {
        memcpy(s_msg_pending, frames, (size_t)n * sizeof(frames[0]));
    }
    s_msg_pending_index = 0;
    s_msg_pending_count = (uint8_t)n;
    s_msg_out_needs_rearm = true;
    portEXIT_CRITICAL(&s_msg_pending_lock);
}

static void usb_glue_msg_out_deferred(void *param)
{
    const uint32_t generation = (uint32_t)(uintptr_t)param;

    /* This function runs only in the TinyUSB task. reset/open callbacks therefore
     * cannot interleave with frame commit or endpoint re-arm. */
    for (;;) {
        pcan_frame_t frame;
        bool have_frame = false;

        portENTER_CRITICAL(&s_msg_pending_lock);
        if (!s_itf_open || generation != s_usb_generation ||
            !s_msg_out_needs_rearm) {
            portEXIT_CRITICAL(&s_msg_pending_lock);
            return;
        }
        if (s_msg_pending_index < s_msg_pending_count) {
            frame = s_msg_pending[s_msg_pending_index];
            have_frame = true;
        }
        portEXIT_CRITICAL(&s_msg_pending_lock);

        if (have_frame) {
            if (!pcan_rb_try_push(s_tx_ring, &frame)) {
                portENTER_CRITICAL(&s_msg_pending_lock);
                if (generation == s_usb_generation) {
                    s_stats.tx_backpressure_retries++;
                }
                portEXIT_CRITICAL(&s_msg_pending_lock);
                return;   /* flag still set: the next service pass resumes here */
            }

            portENTER_CRITICAL(&s_msg_pending_lock);
            if (generation == s_usb_generation &&
                s_msg_pending_index < s_msg_pending_count) {
                s_msg_pending_index++;
                if (s_msg_pending_index >= s_msg_pending_count) {
                    s_msg_pending_index = 0;
                    s_msg_pending_count = 0;
                }
            }
            portEXIT_CRITICAL(&s_msg_pending_lock);
            continue;
        }

        /* Every decoded frame is in tx_ring, so the static OUT buffer is free.
         * The flag is cleared only by a submit that was actually accepted —
         * that is what makes a retry idempotent and a lost retry harmless. */
        const bool armed = usbd_edpt_xfer(s_rhport, PCAN_USB_EP_MSGOUT,
                                          s_msg_out, sizeof(s_msg_out), false);
        portENTER_CRITICAL(&s_msg_pending_lock);
        if (s_itf_open && generation == s_usb_generation) {
            if (armed) {
                s_msg_out_needs_rearm = false;
            } else {
                s_stats.msg_out_rearm_failures++;
            }
        }
        portEXIT_CRITICAL(&s_msg_pending_lock);
        return;
    }
}

/* Re-arm EP1-OUT from the TinyUSB task after a failed submit. The flag survives
 * until a submit is accepted, so a failure here simply leaves the next service
 * tick to try again. */
static void usb_glue_cmd_out_deferred(void *param)
{
    const uint32_t generation = (uint32_t)(uintptr_t)param;

    portENTER_CRITICAL(&s_msg_pending_lock);
    const bool live = (s_itf_open && generation == s_usb_generation &&
                       s_cmd_out_needs_rearm);
    portEXIT_CRITICAL(&s_msg_pending_lock);
    if (!live) {
        return;
    }

    const bool armed = usbd_edpt_xfer(s_rhport, PCAN_USB_EP_CMDOUT, s_cmd_out,
                                      sizeof(s_cmd_out), false);
    portENTER_CRITICAL(&s_msg_pending_lock);
    if (s_itf_open && generation == s_usb_generation) {
        if (armed) {
            s_cmd_out_needs_rearm = false;
        } else {
            s_stats.cmd_out_rearm_failures++;
        }
    }
    portEXIT_CRITICAL(&s_msg_pending_lock);
}

/*
 * Neither flag is consumed here: usbd_defer_func() returns void and drops the
 * request outright when TinyUSB's event queue is full, so a flag cleared on the
 * way in would leave an OUT endpoint unarmed with nobody left to notice — every
 * host command then hits the driver's 1000 ms timeout for the rest of the
 * session. Only a submit the stack accepted clears the flag; scheduling is
 * therefore idempotent and a dropped defer costs one service tick.
 */
void usb_glue_service(void)
{
    bool     schedule_msg;
    bool     schedule_cmd;
    uint32_t generation;

    portENTER_CRITICAL(&s_msg_pending_lock);
    generation   = s_usb_generation;
    schedule_cmd = (s_itf_open && s_cmd_out_needs_rearm);
    schedule_msg = (s_tx_ring != NULL && s_itf_open && s_msg_out_needs_rearm);
    portEXIT_CRITICAL(&s_msg_pending_lock);

    if (schedule_cmd) {
        usbd_defer_func(usb_glue_cmd_out_deferred,
                        (void *)(uintptr_t)generation, false);
    }
    if (schedule_msg) {
        usbd_defer_func(usb_glue_msg_out_deferred,
                        (void *)(uintptr_t)generation, false);
    }
}

void usb_glue_take_stats(usb_glue_stats_t *out)
{
    if (out == NULL) {
        return;
    }
    portENTER_CRITICAL(&s_msg_pending_lock);
    *out = s_stats;
    memset(&s_stats, 0, sizeof(s_stats));
    portEXIT_CRITICAL(&s_msg_pending_lock);
}

/* ---- IN transmit helpers -------------------------------------------------- */
static void usb_glue_msg_in_deferred(void *param)
{
    const uint32_t generation = (uint32_t)(uintptr_t)param;
    uint16_t len = 0;

    portENTER_CRITICAL(&s_msg_pending_lock);
    if (s_itf_open && generation == s_usb_generation &&
        s_msg_in_state == MSG_IN_QUEUED &&
        s_msg_in_generation == generation) {
        len = s_msg_in_len;
    }
    portEXIT_CRITICAL(&s_msg_pending_lock);
    if (len == 0) {
        return;
    }

    bool claimed = usbd_edpt_claim(s_rhport, PCAN_USB_EP_MSGIN);
    bool submitted = claimed &&
        usbd_edpt_xfer(s_rhport, PCAN_USB_EP_MSGIN, s_msg_in, len, false);
    if (claimed && !submitted) {
        usbd_edpt_release(s_rhport, PCAN_USB_EP_MSGIN);
    }

    portENTER_CRITICAL(&s_msg_pending_lock);
    if (generation == s_usb_generation &&
        s_msg_in_state == MSG_IN_QUEUED &&
        s_msg_in_generation == generation) {
        if (submitted) {
            s_msg_in_state = MSG_IN_SUBMITTED;
        } else {
            s_msg_in_state = MSG_IN_RETRY;
            s_stats.msg_in_failed_retries++;
        }
    }
    portEXIT_CRITICAL(&s_msg_pending_lock);
}

/*
 * Retain one EP2-IN batch through TinyUSB completion. The first call copies and
 * defers submission to the TinyUSB task; subsequent calls return 0 while queued
 * or in flight. Only a successful completion callback (or an explicitly counted
 * USB-reset discard) returns len and lets usb_tx_task reset the caller batch.
 */
uint32_t usb_glue_send_msg_batch(const uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0 || len > PCAN_USB_TX_BUFFER_SIZE) {
        return 0;
    }

    bool schedule = false;
    bool finished = false;
    uint32_t generation = 0;

    portENTER_CRITICAL(&s_msg_pending_lock);
    switch (s_msg_in_state) {
    case MSG_IN_COMPLETE:
    case MSG_IN_DROPPED:
        if (len == s_msg_in_len) {
            s_msg_in_state = MSG_IN_IDLE;
            s_msg_in_len = 0;
            finished = true;
        }
        break;

    case MSG_IN_IDLE:
        if (s_itf_open) {
            memcpy(s_msg_in, buf, len);
            s_msg_in_len = len;
            s_msg_in_generation = s_usb_generation;
            s_msg_in_state = MSG_IN_QUEUED;
            generation = s_usb_generation;
            schedule = true;
        }
        break;

    case MSG_IN_RETRY:
        if (s_itf_open && s_msg_in_generation == s_usb_generation) {
            s_msg_in_state = MSG_IN_QUEUED;
            generation = s_usb_generation;
            schedule = true;
        }
        break;

    case MSG_IN_QUEUED:
    case MSG_IN_SUBMITTED:
    default:
        break;
    }
    portEXIT_CRITICAL(&s_msg_pending_lock);

    if (schedule) {
        usbd_defer_func(usb_glue_msg_in_deferred,
                        (void *)(uintptr_t)generation, false);
    }
    return finished ? len : 0;
}

/*
 * usb_glue_send_cmd_reply: submit the 16-byte GET reply on EP 0x81. Called from
 * xfer_cb context (TinyUSB task), so it must be non-blocking. Same claim/submit
 * pattern as the message path.
 */
uint32_t usb_glue_send_cmd_reply(const uint8_t *reply, uint16_t len)
{
    if (reply == NULL || len == 0 || len > PCAN_USB_MAX_CMD_LEN || len > sizeof(s_cmd_in)) {
        return 0;
    }
    if (!s_itf_open) {
        return 0;
    }
    if (!usbd_edpt_claim(s_rhport, PCAN_USB_EP_CMDIN)) {
        return 0;
    }
    memcpy(s_cmd_in, reply, len);
    if (!usbd_edpt_xfer(s_rhport, PCAN_USB_EP_CMDIN, s_cmd_in, len, false)) {
        usbd_edpt_release(s_rhport, PCAN_USB_EP_CMDIN);
        return 0;
    }
    return len;
}

/* ==========================================================================
 * Custom TinyUSB application class driver.
 * ========================================================================== */

/* init: reset static transfer/backpressure state. */
static void pcan_drv_init(void)
{
    s_itf_open = false;
    portENTER_CRITICAL(&s_msg_pending_lock);
    s_usb_generation++;
    s_msg_pending_count = 0;
    s_msg_pending_index = 0;
    s_msg_out_needs_rearm = false;
    s_cmd_out_needs_rearm = false;
    s_msg_in_state = MSG_IN_IDLE;
    s_msg_in_len = 0;
    memset(&s_stats, 0, sizeof(s_stats));
    portEXIT_CRITICAL(&s_msg_pending_lock);
}

/* deinit: the driver is going away for good; complete any retained EP2-IN batch
 * as dropped so usb_tx_task stops retrying a transfer that can never run. */
static bool pcan_drv_deinit(void)
{
    s_itf_open = false;
    portENTER_CRITICAL(&s_msg_pending_lock);
    s_usb_generation++;
    s_stats.tx_pending_reset_drops +=
        (uint32_t)(s_msg_pending_count - s_msg_pending_index);
    s_msg_pending_count = 0;
    s_msg_pending_index = 0;
    s_msg_out_needs_rearm = false;
    s_cmd_out_needs_rearm = false;
    if (s_msg_in_state == MSG_IN_QUEUED || s_msg_in_state == MSG_IN_SUBMITTED ||
        s_msg_in_state == MSG_IN_RETRY) {
        s_msg_in_state = MSG_IN_DROPPED;
        s_stats.msg_in_reset_drops++;
    }
    portEXIT_CRITICAL(&s_msg_pending_lock);
    return true;
}

/* reset: bus reset / de-configure. Bump the connection generation so stale
 * deferred work from the old session cannot commit. A retained EP2-IN batch is
 * NOT dropped: it moves to RETRY under the new generation and is resubmitted
 * after re-enumeration (the host flushed its own state across the reset too).
 * Pending host->CAN frames are bounded by the USB session and counted as lost. */
static void pcan_drv_reset(uint8_t rhport)
{
    (void)rhport;
    s_itf_open = false;
    portENTER_CRITICAL(&s_msg_pending_lock);
    s_usb_generation++;
    s_stats.tx_pending_reset_drops +=
        (uint32_t)(s_msg_pending_count - s_msg_pending_index);
    s_msg_pending_count = 0;
    s_msg_pending_index = 0;
    s_msg_out_needs_rearm = false;
    s_cmd_out_needs_rearm = false;
    if (s_msg_in_state == MSG_IN_QUEUED || s_msg_in_state == MSG_IN_SUBMITTED ||
        s_msg_in_state == MSG_IN_RETRY) {
        s_msg_in_state = MSG_IN_RETRY;
        s_msg_in_generation = s_usb_generation;
    }
    portEXIT_CRITICAL(&s_msg_pending_lock);
}

/*
 * open: the host selected the configuration. `itf_desc` points at our vendor
 * interface descriptor; walk the four endpoint descriptors, open each, arm the
 * two OUT endpoints for receive, and return the number of descriptor bytes we
 * consumed (interface 9 + 4*7 = 37) so the stack advances correctly.
 */
static uint16_t pcan_drv_open(uint8_t rhport,
                              tusb_desc_interface_t const *itf_desc,
                              uint16_t max_len)
{
    /* Only claim our own vendor interface with exactly four endpoints. */
    TU_VERIFY(itf_desc->bInterfaceClass == TUSB_CLASS_VENDOR_SPECIFIC, 0);
    TU_VERIFY(itf_desc->bNumEndpoints == 4, 0);

    uint16_t const drv_len =
        (uint16_t)(sizeof(tusb_desc_interface_t) +
                   itf_desc->bNumEndpoints * sizeof(tusb_desc_endpoint_t));
    TU_VERIFY(max_len >= drv_len, 0);

    /* Walk and open every endpoint descriptor that follows the interface. */
    uint8_t const *p_desc = tu_desc_next(itf_desc);
    for (uint8_t i = 0; i < itf_desc->bNumEndpoints; i++) {
        tusb_desc_endpoint_t const *ep = (tusb_desc_endpoint_t const *)p_desc;
        TU_VERIFY(TUSB_DESC_ENDPOINT == tu_desc_type(ep), 0);
        TU_VERIFY(usbd_edpt_open(rhport, ep), 0);
        p_desc = tu_desc_next(p_desc);
    }

    s_rhport   = rhport;
    s_itf_open = true;
    portENTER_CRITICAL(&s_msg_pending_lock);
    s_usb_generation++;
    s_msg_pending_count = 0;
    s_msg_pending_index = 0;
    s_msg_out_needs_rearm = false;
    s_cmd_out_needs_rearm = false;
    if (s_msg_in_state == MSG_IN_RETRY) {
        /* Rebind a batch retained across reset to this connection so the sender
         * resubmits it against the freshly opened endpoints. */
        s_msg_in_generation = s_usb_generation;
    }
    portEXIT_CRITICAL(&s_msg_pending_lock);

    /* Arm both OUT endpoints so the host can send commands (0x01) and CAN TX
     * batches (0x02) immediately. Enumeration itself survives a failure here,
     * but an unarmed OUT endpoint never completes and so never re-arms itself:
     * flag it for usb_glue_service() instead, or that direction stays dead
     * until re-enumeration. */
    if (!usbd_edpt_xfer(rhport, PCAN_USB_EP_CMDOUT, s_cmd_out, sizeof(s_cmd_out), false)) {
        ESP_LOGW(TAG, "arm EP 0x%02x OUT failed", PCAN_USB_EP_CMDOUT);
        portENTER_CRITICAL(&s_msg_pending_lock);
        s_cmd_out_needs_rearm = true;
        s_stats.cmd_out_rearm_failures++;
        portEXIT_CRITICAL(&s_msg_pending_lock);
    }
    if (!usbd_edpt_xfer(rhport, PCAN_USB_EP_MSGOUT, s_msg_out, sizeof(s_msg_out), false)) {
        ESP_LOGW(TAG, "arm EP 0x%02x OUT failed", PCAN_USB_EP_MSGOUT);
        portENTER_CRITICAL(&s_msg_pending_lock);
        s_msg_out_needs_rearm = true;
        s_stats.msg_out_rearm_failures++;
        portEXIT_CRITICAL(&s_msg_pending_lock);
    }

    /* drv_len == 37: interface(9) + 4*endpoint(7). p_desc has advanced the same
     * amount; return the computed length (they match — no class descriptors). */
    return drv_len;
}

/*
 * control_xfer_cb: the classic PCAN protocol carries NO class-specific EP0
 * control requests — every command travels over bulk EP 0x01. Standard control
 * requests (SET_ADDRESS/SET_CONFIGURATION/GET_DESCRIPTOR, etc.) are handled by
 * the TinyUSB core before a driver is consulted, so anything that reaches here
 * is unsupported: return false to stall it. This does NOT stall enumeration.
 */
static bool pcan_drv_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                     tusb_control_request_t const *request)
{
    (void)rhport; (void)stage; (void)request;
    return false;
}

/*
 * xfer_cb: a bulk transfer on one of our endpoints completed. Route by absolute
 * endpoint address. Command OUT is re-armed immediately; message OUT remains
 * unarmed until usb_glue_service() has queued every decoded CAN frame. IN
 * endpoints need no action after completion. NEVER blocks.
 */
static bool pcan_drv_xfer_cb(uint8_t rhport, uint8_t ep_addr,
                             xfer_result_t result, uint32_t xferred_bytes)
{
    switch (ep_addr) {
    case PCAN_USB_EP_CMDOUT:
        if (result == XFER_RESULT_SUCCESS && xferred_bytes > 0) {
            usb_glue_handle_cmd(s_cmd_out, (uint16_t)xferred_bytes);
        }
        /* Re-arm regardless so the OUT endpoint keeps receiving; a rejected
         * submit is retried from usb_glue_service(). */
        if (!usbd_edpt_xfer(rhport, PCAN_USB_EP_CMDOUT, s_cmd_out, sizeof(s_cmd_out), false)) {
            portENTER_CRITICAL(&s_msg_pending_lock);
            s_cmd_out_needs_rearm = true;
            s_stats.cmd_out_rearm_failures++;
            portEXIT_CRITICAL(&s_msg_pending_lock);
        }
        return true;

    case PCAN_USB_EP_MSGOUT:
        if (result == XFER_RESULT_SUCCESS && xferred_bytes > 0) {
            usb_glue_handle_msg(s_msg_out, (uint16_t)xferred_bytes);
        } else {
            /* No payload to retain, but still defer the re-arm through the same
             * ownership path so only usb_glue_service() submits EP2-OUT. */
            portENTER_CRITICAL(&s_msg_pending_lock);
            s_msg_out_needs_rearm = true;
            portEXIT_CRITICAL(&s_msg_pending_lock);
        }
        return true;

    case PCAN_USB_EP_CMDIN:
        /* Command reply done; the stack released the endpoint claim. */
        return true;

    case PCAN_USB_EP_MSGIN:
        /* Complete the retained-batch state machine. Only a SUCCESS releases
         * the caller's batch; a failed completion moves to RETRY so the exact
         * same bytes are resubmitted. Runs in the TinyUSB task, serialised
         * against reset/open, so state SUBMITTED always belongs to the current
         * generation (reset moves it to RETRY first). */
        portENTER_CRITICAL(&s_msg_pending_lock);
        if (s_msg_in_state == MSG_IN_SUBMITTED) {
            if (result == XFER_RESULT_SUCCESS) {
                s_msg_in_state = MSG_IN_COMPLETE;
            } else {
                s_msg_in_state = MSG_IN_RETRY;
                s_stats.msg_in_failed_retries++;
            }
        }
        portEXIT_CRITICAL(&s_msg_pending_lock);
        return true;

    default:
        return false;  /* not one of ours */
    }
}

/* The class-driver table. Program-lifetime static so it is always accessible to
 * the stack (required by usbd_app_driver_get_cb's contract). */
static const usbd_class_driver_t s_pcan_driver[] = {
    {
        .name            = "PCAN",
        .init            = pcan_drv_init,
        .deinit          = pcan_drv_deinit,
        .reset           = pcan_drv_reset,
        .open            = pcan_drv_open,
        .control_xfer_cb = pcan_drv_control_xfer_cb,
        .xfer_cb         = pcan_drv_xfer_cb,
        .sof             = NULL,
    },
};

/*
 * usbd_app_driver_get_cb: TinyUSB core calls this during tud_init() to collect
 * application-provided class drivers. It is declared TU_ATTR_WEAK in TinyUSB
 * (usbd.c ships a stub returning 0 drivers); this strong definition overrides
 * it. esp_tinyusb does NOT define this symbol, so there is no collision.
 */
usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count)
{
    *driver_count = 1;
    return s_pcan_driver;
}
