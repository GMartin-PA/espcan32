/*
 * SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
/*
 * twai_hal.c — legacy TWAI (classic CAN 2.0) backend wrapper
 * ============================================================================
 * See twai_hal.h. Implemented against the ESP-IDF v5.x LEGACY TWAI driver
 * (driver/twai.h + driver/twai_types_legacy.h): twai_driver_install/start/stop/
 * uninstall, twai_transmit/receive, twai_read_alerts, twai_get_status_info,
 * twai_initiate_recovery.
 *
 * The ESP32-S3 has ONE TWAI controller and is classic-CAN only (DLC<=8). The
 * legacy driver cannot re-time a live controller, so every bitrate/mode change
 * is a full stop -> uninstall -> install -> start (control task only).
 * ============================================================================
 */
#include "twai_hal.h"
#include "config.h"
#include "pcan_time.h"
#include "driver/twai.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "twai_hal";

static const pcan_btr_entry_t s_btr_table[] = PCAN_BTR_TABLE_INIT;

/* Bitrates that have a TWAI_TIMING_CONFIG_* macro we support directly. Any
 * other requested rate is snapped to the nearest of these (logged). */
static const uint32_t s_supported_bitrates[] = {
    1000000u, 500000u, 250000u, 125000u, 100000u, 50000u,
};
#define S_SUPPORTED_LEN (sizeof(s_supported_bitrates) / sizeof(s_supported_bitrates[0]))

/*
 * Alert set enabled at install. Covers RX activity, queue/FIFO overruns, the
 * error-warning/passive/active transitions (bus light/heavy + clear), and the
 * bus-off / recovery pair that drives pcan_twai_service_alerts().
 */
#define TWAI_HAL_ALERTS ( \
        TWAI_ALERT_RX_DATA          | \
        TWAI_ALERT_TX_SUCCESS       | \
        TWAI_ALERT_TX_FAILED        | \
        TWAI_ALERT_RX_QUEUE_FULL    | \
        TWAI_ALERT_RX_FIFO_OVERRUN  | \
        TWAI_ALERT_ABOVE_ERR_WARN   | \
        TWAI_ALERT_BELOW_ERR_WARN   | \
        TWAI_ALERT_ERR_PASS         | \
        TWAI_ALERT_ERR_ACTIVE       | \
        TWAI_ALERT_BUS_OFF          | \
        TWAI_ALERT_BUS_RECOVERED)

/* Driver lifecycle mirror (control task owns all mutations). */
static bool            s_installed = false;
static uint32_t        s_bitrate   = PCAN_USB_DEFAULT_BITRATE;
static pcan_twai_mode_t s_mode      = TWAI_HAL_MODE_NORMAL;

/* -------------------------------------------------------------------------- */
/* Helpers                                                                    */
/* -------------------------------------------------------------------------- */

static twai_mode_t hal_to_twai_mode(pcan_twai_mode_t m)
{
    /* The HAL only exposes NORMAL / LISTEN_ONLY. Self-reception (SRR) frames
     * are transmitted with message.self set (see pcan_twai_transmit) and the
     * echo is delivered by the driver in NORMAL mode on a live bus; no
     * dedicated NO_ACK/self-test controller mode is required here. */
    return (m == TWAI_HAL_MODE_LISTEN_ONLY) ? TWAI_MODE_LISTEN_ONLY
                                            : TWAI_MODE_NORMAL;
}

/* Snap an arbitrary bitrate to the nearest rate we have a timing macro for. */
static uint32_t nearest_supported_bitrate(uint32_t bitrate)
{
    uint32_t best      = s_supported_bitrates[0];
    uint32_t best_diff = (bitrate > best) ? (bitrate - best) : (best - bitrate);

    for (size_t i = 1; i < S_SUPPORTED_LEN; i++) {
        uint32_t cand = s_supported_bitrates[i];
        uint32_t diff = (bitrate > cand) ? (bitrate - cand) : (cand - bitrate);
        if (diff < best_diff) {
            best_diff = diff;
            best      = cand;
        }
    }
    return best;
}

/* Fill *out with the timing config for a supported bitrate. */
static esp_err_t timing_for_bitrate(uint32_t bitrate, twai_timing_config_t *out)
{
    switch (bitrate) {
    case 1000000u: { twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();   *out = t; return ESP_OK; }
    case  500000u: { twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS(); *out = t; return ESP_OK; }
    case  250000u: { twai_timing_config_t t = TWAI_TIMING_CONFIG_250KBITS(); *out = t; return ESP_OK; }
    case  125000u: { twai_timing_config_t t = TWAI_TIMING_CONFIG_125KBITS(); *out = t; return ESP_OK; }
    case  100000u: { twai_timing_config_t t = TWAI_TIMING_CONFIG_100KBITS(); *out = t; return ESP_OK; }
    case   50000u: { twai_timing_config_t t = TWAI_TIMING_CONFIG_50KBITS();  *out = t; return ESP_OK; }
    default:
        return ESP_ERR_NOT_SUPPORTED;
    }
}

/*
 * Drive the SN65HVD230 Rs (standby) pin. standby=false -> normal/high-speed.
 * No-op if the pin is disabled (CFG_TWAI_STANDBY_GPIO < 0). Configured lazily
 * on first use.
 */
static void twai_set_standby(bool standby)
{
#if CFG_TWAI_STANDBY_GPIO >= 0
    static bool configured = false;
    if (!configured) {
        gpio_config_t io = {
            .pin_bit_mask = (1ULL << (int)CFG_TWAI_STANDBY_GPIO),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        esp_err_t e = gpio_config(&io);
        if (e != ESP_OK) {
            ESP_LOGW(TAG, "standby gpio %d config failed: %s",
                     (int)CFG_TWAI_STANDBY_GPIO, esp_err_to_name(e));
            return;
        }
        configured = true;
    }

#if CFG_TWAI_STANDBY_ACTIVE_LOW
    /* "drive low = normal (not standby)" */
    int level = standby ? 1 : 0;
#else
    int level = standby ? 0 : 1;
#endif
    gpio_set_level((gpio_num_t)CFG_TWAI_STANDBY_GPIO, level);
#else
    (void)standby;  /* standby pin disabled; nothing to drive */
#endif
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

uint32_t pcan_twai_btr_to_bitrate(uint8_t btr0, uint8_t btr1)
{
    for (size_t i = 0; i < PCAN_BTR_TABLE_LEN; i++) {
        if (s_btr_table[i].btr0 == btr0 && s_btr_table[i].btr1 == btr1) {
            return s_btr_table[i].bitrate;
        }
    }
    return 0u; /* unrecognized: caller keeps previous bitrate, still ACKs */
}

esp_err_t pcan_twai_start(uint32_t bitrate, pcan_twai_mode_t mode)
{
    /* Idempotent-safe: tear down an existing instance first. */
    if (s_installed) {
        esp_err_t se = pcan_twai_stop();
        if (se != ESP_OK) {
            ESP_LOGE(TAG, "start: prior stop failed: %s", esp_err_to_name(se));
            return se;
        }
    }

    if (bitrate == 0u) {
        bitrate = (s_bitrate != 0u) ? s_bitrate : PCAN_USB_DEFAULT_BITRATE;
    }

    uint32_t snapped = nearest_supported_bitrate(bitrate);
    if (snapped != bitrate) {
        ESP_LOGW(TAG, "bitrate %u unsupported; falling back to %u",
                 (unsigned)bitrate, (unsigned)snapped);
    }

    twai_timing_config_t t_config;
    esp_err_t e = timing_for_bitrate(snapped, &t_config);
    if (e != ESP_OK) {
        /* nearest_supported_bitrate only ever returns supported values, so
         * this is unreachable — guard defensively regardless. */
        ESP_LOGE(TAG, "no timing config for %u", (unsigned)snapped);
        return e;
    }

    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CFG_TWAI_TX_GPIO,
                                    (gpio_num_t)CFG_TWAI_RX_GPIO,
                                    hal_to_twai_mode(mode));
    g_config.tx_queue_len   = CFG_TWAI_TX_QUEUE_LEN;
    g_config.rx_queue_len   = CFG_TWAI_RX_QUEUE_LEN;
    g_config.alerts_enabled = TWAI_HAL_ALERTS;
    g_config.intr_flags     = CFG_TWAI_INTR_FLAGS;

    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    e = twai_driver_install(&g_config, &t_config, &f_config);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "twai_driver_install failed: %s", esp_err_to_name(e));
        return e;
    }

    e = twai_start();
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "twai_start failed: %s", esp_err_to_name(e));
        (void)twai_driver_uninstall();
        return e;
    }

    s_installed = true;
    s_bitrate   = snapped;
    s_mode      = mode;

    twai_set_standby(false); /* enable transceiver */

    ESP_LOGI(TAG, "started @ %u bit/s, mode=%s", (unsigned)snapped,
             (mode == TWAI_HAL_MODE_LISTEN_ONLY) ? "listen-only" : "normal");
    return ESP_OK;
}

esp_err_t pcan_twai_stop(void)
{
    if (!s_installed) {
        return ESP_OK;
    }

    /* twai_stop() returns ESP_ERR_INVALID_STATE if the controller is not in
     * the RUNNING state (e.g. already bus-off/recovering) — that is fine, we
     * only need it stopped before uninstall. */
    esp_err_t e = twai_stop();
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "twai_stop: %s", esp_err_to_name(e));
    }

    e = twai_driver_uninstall();
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "twai_driver_uninstall failed: %s", esp_err_to_name(e));
        return e; /* leave s_installed set so we don't leak/desync */
    }

    s_installed = false;
    twai_set_standby(true); /* transceiver to low-power standby */
    return ESP_OK;
}

esp_err_t pcan_twai_reconfigure(uint32_t bitrate, pcan_twai_mode_t mode)
{
    /* bitrate 0 keeps current; mode always applied. pcan_twai_start() performs
     * the stop -> uninstall -> install -> start sequence internally. */
    uint32_t br = (bitrate != 0u) ? bitrate : s_bitrate;
    return pcan_twai_start(br, mode);
}

esp_err_t pcan_twai_transmit(const pcan_frame_t *frame, TickType_t ticks_to_wait)
{
    if (frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_installed) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_mode == TWAI_HAL_MODE_LISTEN_ONLY) {
        /* Silent mode: never source traffic (driver would reject anyway). */
        return ESP_ERR_NOT_SUPPORTED;
    }

    twai_message_t m;
    memset(&m, 0, sizeof(m));
    m.identifier = frame->id;
    m.extd = (frame->flags & PCAN_FRAME_FLAG_EXT) ? 1 : 0;
    m.rtr  = (frame->flags & PCAN_FRAME_FLAG_RTR) ? 1 : 0;
    m.ss   = (frame->flags & PCAN_FRAME_FLAG_SS)  ? 1 : 0; /* single-shot / no ART */
    m.self = (frame->flags & PCAN_FRAME_FLAG_SRR) ? 1 : 0; /* self-reception req */

    uint8_t dlc = frame->dlc;
    if (dlc > PCAN_FRAME_MAX_DLC) {
        dlc = PCAN_FRAME_MAX_DLC;
    }
    m.data_length_code = dlc;
    if (!m.rtr) {
        memcpy(m.data, frame->data, dlc);
    }

    return twai_transmit(&m, ticks_to_wait);
}

esp_err_t pcan_twai_receive(pcan_frame_t *out, TickType_t ticks_to_wait)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    twai_message_t m;
    esp_err_t e = twai_receive(&m, ticks_to_wait);
    if (e != ESP_OK) {
        /* ESP_ERR_TIMEOUT when idle, ESP_ERR_INVALID_STATE mid-reconfigure. */
        return e;
    }

    /* Stamp the device timestamp at the instant of return (proto §4). */
    out->ts16 = pcan_time_now16();

    out->id    = m.identifier;
    out->flags = 0;
    if (m.extd) out->flags |= PCAN_FRAME_FLAG_EXT;
    if (m.rtr)  out->flags |= PCAN_FRAME_FLAG_RTR;
    /* NOTE: the ESP-IDF legacy TWAI driver does NOT set twai_message_t.self on
     * received frames ("Unused for received"), so there is no per-frame echo
     * flag to read here. A self-reception (SRR) frame is delivered as an
     * ordinary RX record — exactly how a genuine PCAN-USB presents it on the
     * wire — so nothing downstream can or needs to tell the two apart. */

    uint8_t dlc = m.data_length_code;
    if (dlc > PCAN_FRAME_MAX_DLC) {
        dlc = PCAN_FRAME_MAX_DLC; /* clamp any non-compliant DLC */
    }
    out->dlc = dlc;

    memset(out->data, 0, sizeof(out->data));
    if (!m.rtr) {
        memcpy(out->data, m.data, dlc);
    }
    out->writer_id = 0;

    return ESP_OK;
}

esp_err_t pcan_twai_get_status(pcan_twai_status_t *st)
{
    if (st == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    twai_status_info_t info;
    esp_err_t e = twai_get_status_info(&info);
    if (e != ESP_OK) {
        memset(st, 0, sizeof(*st));
        st->state = TWAI_HAL_STATE_STOPPED;
        return e;
    }

    switch (info.state) {
    case TWAI_STATE_RUNNING:    st->state = TWAI_HAL_STATE_RUNNING;    break;
    case TWAI_STATE_BUS_OFF:    st->state = TWAI_HAL_STATE_BUS_OFF;    break;
    case TWAI_STATE_RECOVERING: st->state = TWAI_HAL_STATE_RECOVERING; break;
    case TWAI_STATE_STOPPED:    /* fall through */
    default:                    st->state = TWAI_HAL_STATE_STOPPED;    break;
    }

    st->tx_error_counter = info.tx_error_counter;
    st->rx_error_counter = info.rx_error_counter;
    st->tx_failed_count  = info.tx_failed_count;
    st->rx_missed_count  = info.rx_missed_count;
    st->rx_overrun_count = info.rx_overrun_count;
    st->arb_lost_count   = info.arb_lost_count;
    st->bus_error_count  = info.bus_error_count;

    return ESP_OK;
}

bool pcan_twai_service_alerts(uint8_t *pcan_err_mask, int *tx_event)
{
    if (pcan_err_mask) {
        *pcan_err_mask = 0;
    }
    if (tx_event) {
        *tx_event = PCAN_TX_EVENT_NONE;
    }
    if (!s_installed) {
        return false;
    }

    uint32_t alerts = 0;
    esp_err_t e = twai_read_alerts(&alerts, 0); /* non-blocking */
    if (e != ESP_OK || alerts == 0u) {
        return false; /* ESP_ERR_TIMEOUT when no alerts pending */
    }

    uint8_t mask         = 0;
    bool    state_change = false;

    /* TX completion: FAIL wins over OK if both accumulated in one read (the
     * caller resubmits a frame it still owns, which is safe; the reverse would
     * risk releasing a frame that never reached the bus). */
    if (tx_event) {
        if (alerts & TWAI_ALERT_TX_FAILED) {
            *tx_event = PCAN_TX_EVENT_FAIL;
        } else if (alerts & TWAI_ALERT_TX_SUCCESS) {
            *tx_event = PCAN_TX_EVENT_OK;
        }
    }

    /* Queue / FIFO overruns -> rx queue overrun bit (host-visible loss). */
    if (alerts & (TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_RX_FIFO_OVERRUN)) {
        mask |= PCAN_USB_ERROR_RXQOVR;
    }

    /* Error-warning limit exceeded -> bus light. */
    if (alerts & TWAI_ALERT_ABOVE_ERR_WARN) {
        mask |= PCAN_USB_ERROR_BUS_LIGHT;
        state_change = true;
    }

    /* Error passive -> bus heavy. */
    if (alerts & TWAI_ALERT_ERR_PASS) {
        mask |= PCAN_USB_ERROR_BUS_HEAVY;
        state_change = true;
    }

    /* Counters back below warning / controller error-active again: the bus
     * light/heavy condition cleared. Report a state change with no error bit
     * so the host clears its bus-error indicator. */
    if (alerts & (TWAI_ALERT_BELOW_ERR_WARN | TWAI_ALERT_ERR_ACTIVE)) {
        state_change = true;
    }

    /* Bus-off: report and kick off recovery. */
    if (alerts & TWAI_ALERT_BUS_OFF) {
        mask |= PCAN_USB_ERROR_BUS_OFF;
        state_change = true;
        esp_err_t re = twai_initiate_recovery();
        if (re != ESP_OK) {
            ESP_LOGE(TAG, "twai_initiate_recovery failed: %s",
                     esp_err_to_name(re));
        } else {
            ESP_LOGW(TAG, "bus-off detected; recovery initiated");
        }
    }

    /* Recovery complete: driver is back in STOPPED — restart it. */
    if (alerts & TWAI_ALERT_BUS_RECOVERED) {
        esp_err_t se = twai_start();
        if (se == ESP_OK) {
            ESP_LOGI(TAG, "bus recovered; controller restarted");
        } else {
            ESP_LOGE(TAG, "restart after recovery failed: %s",
                     esp_err_to_name(se));
        }
        state_change = true;
    }

    if (pcan_err_mask) {
        *pcan_err_mask = mask;
    }
    return state_change || (mask != 0);
}
