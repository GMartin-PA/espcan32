/*
 * SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
/*
 * config.h — build-time configuration & board pin map
 * ============================================================================
 * Central place for GPIO assignments, task priorities/stack sizes, ring
 * buffer depths, and compile-time feature switches. Values here are the
 * project defaults; override via sdkconfig / menuconfig where a Kconfig
 * equivalent exists, or edit here for a one-off board.
 * ============================================================================
 */
#ifndef PCAN_CONFIG_H
#define PCAN_CONFIG_H

#include "driver/gpio.h"

/* --------------------------------------------------------------------------
 * TWAI (CAN) pin map — SN65HVD230 3.3 V transceiver.
 *   ESP32-S3 TWAI_TX -> transceiver D (TXD)
 *   ESP32-S3 TWAI_RX <- transceiver R (RXD)
 * The SN65HVD230 Rs pin selects slope/standby; tie to GND (or a GPIO) for
 * high-speed. GPIO choices avoid strapping pins (0,3,45,46), USB (19,20), the
 * S3-PICO-1 embedded flash/PSRAM (26-37), and GPIO5/6 (reserved for UART on
 * this board).
 * -------------------------------------------------------------------------- */
#define CFG_TWAI_TX_GPIO            GPIO_NUM_7
#define CFG_TWAI_RX_GPIO            GPIO_NUM_8
#define CFG_TWAI_STANDBY_GPIO       GPIO_NUM_10  /* SN65HVD230 Rs; -1 if Rs tied to GND */
#define CFG_TWAI_STANDBY_ACTIVE_LOW 1            /* drive low = normal (not standby)*/

/* Optional activity/status LED (mirrors CMD_LED). -1 to disable. */
#define CFG_STATUS_LED_GPIO        GPIO_NUM_2

/* --------------------------------------------------------------------------
 * TWAI queue and driver tuning (see twai_hal.c).
 * -------------------------------------------------------------------------- */
#define CFG_TWAI_RX_QUEUE_LEN      64u
#define CFG_TWAI_TX_QUEUE_LEN      32u
#define CFG_TWAI_INTR_FLAGS        ESP_INTR_FLAG_LEVEL1

/* --------------------------------------------------------------------------
 * Ring buffers (ringbuf.c). PSRAM-backed to keep 512 KB SRAM free for stacks
 * and the USB/TWAI drivers. Depth in whole pcan_frame_t entries.
 * -------------------------------------------------------------------------- */
#define CFG_RB_RX_DEPTH            1024u  /* CAN -> USB (dev->host)             */
#define CFG_RB_TX_DEPTH            512u   /* USB -> CAN (host->dev)             */
#define CFG_RB_USE_PSRAM           1

/* --------------------------------------------------------------------------
 * FreeRTOS task layout (see DESIGN.md §"Task model"). Higher number = higher
 * priority. tskIDLE_PRIORITY == 0.
 * -------------------------------------------------------------------------- */
#define CFG_TASK_TWAI_RX_PRIO      12   /* drains TWAI, timestamps, -> rx ring */
#define CFG_TASK_TWAI_RX_STACK     4096
#define CFG_TASK_TWAI_RX_CORE      1

#define CFG_TASK_TINYUSB_PRIO      11   /* USB ISR/task + endpoint callbacks    */
#define CFG_TASK_TINYUSB_CORE      0

#define CFG_TASK_USBTX_PRIO        10   /* rx ring -> EP2 IN batches           */
#define CFG_TASK_USBTX_STACK       4096
#define CFG_TASK_USBTX_CORE        0

#define CFG_TASK_CANTX_PRIO        9    /* tx ring -> TWAI transmit            */
#define CFG_TASK_CANTX_STACK       4096
#define CFG_TASK_CANTX_CORE        1

#define CFG_TASK_CTRL_PRIO         8    /* command/state machine, reconfigure  */
#define CFG_TASK_CTRL_STACK        4096
/* Core 1 (was 0): ctrl_task performs every twai_driver_install/reconfigure, and
 * the ESP-IDF TWAI ISR is allocated on the core that installs it. Pinning
 * ctrl_task to core 1 keeps the CAN ISR on core 1 alongside twai_rx_task and
 * can_tx_task — CAN is EXCLUSIVE to core 1, with TinyUSB and usb_tx_task on
 * core 0. This removes USB/TWAI ISR contention; queue ownership/backpressure is
 * handled separately and must remain lossless regardless of core placement. */
#define CFG_TASK_CTRL_CORE         1

/* Standalone ISO-TP/UDS bench task (only started in BENCH mode). */
#define CFG_TASK_BENCH_PRIO        9
#define CFG_TASK_BENCH_STACK       6144
#define CFG_TASK_BENCH_CORE        1

/* --------------------------------------------------------------------------
 * Operating mode selection.
 *   PCAN_MODE_PCAN  : USB PCAN-USB emulation (default).
 *   PCAN_MODE_BENCH : standalone on-device ISO-TP/UDS server, no USB CAN path.
 * The two modes are MUTUALLY EXCLUSIVE — the S3 has a single TWAI controller,
 * so bench mode owns it exclusively. Selection is resolved at boot by
 * app_mode_select() (see main.c): compile-time default below, overridable by
 * a boot-strap GPIO or an NVS key.
 * -------------------------------------------------------------------------- */
#define CFG_MODE_PCAN              0
#define CFG_MODE_BENCH             1
#define CFG_DEFAULT_MODE           CFG_MODE_PCAN

/* Build the bench-mode ISO-TP/UDS server into the image at all. Set to 0 for a
 * PCAN-only build: the link/server reassembly buffers are statically allocated
 * in internal SRAM whether or not bench mode is ever selected, so dropping them
 * (and, via --gc-sections, isotp.c/uds.c with them) frees ~16 KB. A 0 build
 * falls back to PCAN mode if the GPIO or the NVS key still asks for BENCH. */
#define CFG_ENABLE_BENCH_MODE      1

/* Boot GPIO that forces BENCH mode when asserted at reset (-1 to disable).
 * Read once in app_mode_select(); uses internal pull-up, active low. */
#define CFG_MODE_SELECT_GPIO       GPIO_NUM_9   /* moved off 7 (now TWAI_TX) */
#define CFG_MODE_SELECT_ACTIVE_LOW 1

/* NVS namespace/key that can pin the mode across reboots (checked if the GPIO
 * is not asserted). */
#define CFG_NVS_NAMESPACE          "pcan"
#define CFG_NVS_KEY_MODE           "mode"

/* --------------------------------------------------------------------------
 * Bench-mode ISO-TP/UDS addressing defaults (see isotp.h / uds.h).
 * -------------------------------------------------------------------------- */
#define CFG_BENCH_RX_ID            0x7E0u  /* physical request  (tester->ecu)   */
#define CFG_BENCH_TX_ID            0x7E8u  /* physical response (ecu->tester)   */
#define CFG_BENCH_FUNC_ID          0x7DFu  /* functional broadcast (SF only)    */
#define CFG_BENCH_BITRATE          500000u

#endif /* PCAN_CONFIG_H */
