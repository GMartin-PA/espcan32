/*
 * SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
/*
 * usb_descriptors.h — PCAN-USB device / config / string descriptor set
 * ============================================================================
 * Builds the exact descriptor bytes a real classic PCAN-USB presents (see
 * pcan_proto.h §0-§1 and the research USB-descriptor findings): VID 0x0C72 /
 * PID 0x000C, ONE vendor interface (bInterfaceNumber 0, bNumEndpoints 4)
 * carrying the two bulk EP pairs, EP addresses 0x01/0x81 (command) and
 * 0x02/0x82 (message).
 *
 * SINGLE-INTERFACE, 4-ENDPOINT LAYOUT --------------------------------------
 * This matches the genuine device exactly (verified against the Linux
 * peak_usb driver + an lsusb -v dump of 0c72:000c): interface 0 owns all four
 * bulk endpoints. TinyUSB's TUD_VENDOR_DESCRIPTOR() macro hardcodes
 * bNumEndpoints=2 and cannot express this, so the configuration descriptor is
 * hand-rolled (usb_descriptors.c) and the four endpoints are owned by a custom
 * TinyUSB application class driver (usb_glue.c) that routes by ENDPOINT ADDRESS
 * rather than by interface index. A two-interface split would make the mainline
 * Linux driver bind twice (it probes per interface) and risk leaving the data
 * endpoints unconfigured on Windows.
 *
 * OWNERSHIP: all descriptor arrays have program lifetime. usb_glue.c hands the
 * pointers below to esp_tinyusb (tinyusb_config_t.descriptor.*), which owns the
 * tud_descriptor_*_cb symbols; callers must not free.
 * ============================================================================
 */
#ifndef PCAN_USB_DESCRIPTORS_H
#define PCAN_USB_DESCRIPTORS_H

#include <stdint.h>
#include "tusb.h"
#include "pcan_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Interface numbering. The real PCAN-USB exposes a SINGLE vendor interface that
 * owns all four bulk endpoints; there is no per-EP-pair interface split. The
 * custom class driver routes by absolute endpoint address (0x01/0x81/0x02/0x82),
 * not by interface index.
 */
enum {
    PCAN_ITF_NUM_VENDOR = 0,  /* the one vendor interface, bNumEndpoints 4      */
    PCAN_ITF_NUM_TOTAL  = 1
};

/* Number of entries in the string descriptor table (index 0..N-1); the indices
 * themselves are the PCAN_USB_STR_* constants of pcan_proto.h §0. */
#define PCAN_STRING_DESC_COUNT   6

/*
 * Pointer to the 18-byte device descriptor (VID/PID/bcdDevice from
 * pcan_proto.h). Returned from TinyUSB's tud_descriptor_device_cb().
 */
const uint8_t *pcan_desc_device(void);

/*
 * Pointer to the full-speed configuration descriptor (config + one vendor
 * interface + four bulk endpoints). Returned from
 * tud_descriptor_configuration_cb(). `index` is ignored (single config).
 */
const uint8_t *pcan_desc_configuration(uint8_t index);

/*
 * The PCAN_STRING_DESC_COUNT-entry string table, in the shape esp_tinyusb wants
 * for tinyusb_config_t.descriptor.string: UTF-8 strings with index 0 holding the
 * raw two-byte LANGID. esp_tinyusb's tud_descriptor_string_cb() converts to
 * UTF-16LE on demand.
 */
const char **pcan_desc_string_table(void);

#ifdef __cplusplus
}
#endif
#endif /* PCAN_USB_DESCRIPTORS_H */
