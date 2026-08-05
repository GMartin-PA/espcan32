/*
 * SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
/*
 * usb_descriptors.c — PCAN-USB descriptor set
 * ============================================================================
 * Emits the exact descriptor bytes a real classic PCAN-USB presents (see
 * pcan_proto.h §0-§1). VID 0x0C72 / PID 0x000C, bcdDevice 0x54FF, bcdUSB
 * 0x0100 (full speed set by the D+ pull-up), four bulk endpoints on a SINGLE
 * vendor interface (bInterfaceNumber 0, bNumEndpoints 4):
 *   command  pair: EP 0x01 OUT / 0x81 IN
 *   message  pair: EP 0x02 OUT / 0x82 IN
 *
 * SINGLE-INTERFACE, 4-ENDPOINT LAYOUT --------------------------------------
 * The genuine device groups all four bulk endpoints under one vendor
 * interface. TinyUSB's TUD_VENDOR_DESCRIPTOR() macro hardcodes bNumEndpoints=2
 * (one OUT/IN pair per interface) and cannot express this, so the
 * configuration descriptor below is HAND-ROLLED: a config header + one 9-byte
 * interface descriptor + four 7-byte bulk endpoint descriptors. The four
 * endpoints are owned by a custom TinyUSB application class driver
 * (usb_glue.c) that routes by absolute endpoint address, not interface index.
 * A two-interface split would make the mainline Linux peak_usb driver bind
 * twice (it probes per interface) and risk leaving the data endpoints
 * unconfigured on Windows.
 *
 * bDeviceClass STAYS 0xFF (vendor). It must NOT be 0: a class-0 device with a
 * multi-descriptor config makes Windows load the composite parent driver
 * (usbccgp), which the PEAK INF does not match. 0xFF keeps it a single
 * vendor-specific function the PEAK/WinUSB INF binds directly.
 *
 * EP1 MPS DEVIATION: the on-wire classic device reports EP1 (command) with a
 * 16-byte wMaxPacketSize; the ESP32-S3 DWC2 core requires 64-byte bulk EPs at
 * full speed, so all four are declared at 64 (PCAN_USB_EP_MPS). The command
 * protocol never exceeds 16 bytes/transfer, so this is behaviourally identical
 * to the host (see pcan_proto.h §1).
 *
 * OWNERSHIP: every descriptor here has program lifetime. usb_glue installs them
 * via tinyusb_config_t.descriptor.*; esp_tinyusb owns the tud_descriptor_*_cb
 * symbols and serves them from those pointers, so we intentionally do NOT define
 * the tud_descriptor_*_cb symbols here (doing so would collide with
 * esp_tinyusb's strong definitions) and the string table is kept in the UTF-8
 * shape esp_tinyusb expects — it does the UTF-16LE conversion itself.
 * ============================================================================
 */
#include "usb_descriptors.h"

/* --------------------------------------------------------------------------
 * Device descriptor (18 bytes). Fields from pcan_proto.h §0.
 * Vendor device class (0xFF/0/0); binding is on VID/PID regardless.
 * -------------------------------------------------------------------------- */
static const tusb_desc_device_t s_device_desc = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = PCAN_USB_BCD_USB,        /* 0x0100 (FS via pull-up)  */
    .bDeviceClass       = TUSB_CLASS_VENDOR_SPECIFIC, /* 0xFF                  */
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = PCAN_USB_EP0_MPS,        /* 64                       */
    .idVendor           = PCAN_USB_VENDOR_ID,      /* 0x0C72                   */
    .idProduct          = PCAN_USB_PRODUCT_ID,     /* 0x000C                   */
    .bcdDevice          = PCAN_USB_BCD_DEVICE,     /* 0x54FF (device_rev 84)   */
    .iManufacturer      = PCAN_USB_STR_MANUFACTURER, /* string index 1         */
    .iProduct           = PCAN_USB_STR_PRODUCT,      /* string index 2         */
    .iSerialNumber      = PCAN_USB_STR_SERIAL,       /* 0: serial is a fw cmd  */
    .bNumConfigurations = 1,
};
_Static_assert(sizeof(s_device_desc) == 18, "device descriptor must be 18B");

/* --------------------------------------------------------------------------
 * Full-speed configuration descriptor (hand-rolled, single interface / 4 EP):
 *   config header (9) + interface (9) + 4 x bulk endpoint (7) = 46 bytes.
 * bMaxPower = PCAN_USB_MAX_POWER_MA (200 mA -> encoded 0x64). Bus powered,
 * no remote wakeup (attribute 0 -> reserved bit7 set by the macro => 0x80).
 *
 * TinyUSB has no generic single-endpoint descriptor macro, so each endpoint is
 * emitted as raw bytes (bLength 7, type TUSB_DESC_ENDPOINT, address, bmAttr
 * bulk 0x02, wMaxPacketSize LE, bInterval 0). Endpoint order matches the real
 * device: 0x01 OUT, 0x81 IN, 0x02 OUT, 0x82 IN.
 * -------------------------------------------------------------------------- */
#define PCAN_CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + 9u + 4u * 7u)  /* = 46 */

/* One 7-byte bulk endpoint descriptor. */
#define PCAN_EP_DESC(addr) \
    7, TUSB_DESC_ENDPOINT, (addr), TUSB_XFER_BULK, \
    U16_TO_U8S_LE(PCAN_USB_EP_MPS), 0

static const uint8_t s_fs_config_desc[] = {
    /* config: 1, itf count, str idx 0, total len, attr 0 (bus-pwr), power  */
    TUD_CONFIG_DESCRIPTOR(1, PCAN_ITF_NUM_TOTAL, 0, PCAN_CONFIG_TOTAL_LEN,
                          0x00, PCAN_USB_MAX_POWER_MA),

    /* interface 0 — vendor-specific, 4 bulk endpoints, no interface string.
     * The real device exposes no iInterface, so we emit 0.                   */
    9,                              /* bLength                                 */
    TUSB_DESC_INTERFACE,            /* bDescriptorType (0x04)                  */
    PCAN_ITF_NUM_VENDOR,            /* bInterfaceNumber = 0                    */
    0x00,                           /* bAlternateSetting                       */
    4,                              /* bNumEndpoints                           */
    TUSB_CLASS_VENDOR_SPECIFIC,     /* bInterfaceClass = 0xFF                  */
    0x00,                           /* bInterfaceSubClass                      */
    0x00,                           /* bInterfaceProtocol                      */
    0x00,                           /* iInterface (none)                       */

    /* four bulk endpoints, real-device address order                         */
    PCAN_EP_DESC(PCAN_USB_EP_CMDOUT),  /* 0x01 OUT — host -> dev commands      */
    PCAN_EP_DESC(PCAN_USB_EP_CMDIN),   /* 0x81 IN  — dev -> host cmd reply     */
    PCAN_EP_DESC(PCAN_USB_EP_MSGOUT),  /* 0x02 OUT — host -> dev CAN TX        */
    PCAN_EP_DESC(PCAN_USB_EP_MSGIN),   /* 0x82 IN  — dev -> host CAN RX+status */
};
_Static_assert(sizeof(s_fs_config_desc) == PCAN_CONFIG_TOTAL_LEN,
               "config descriptor length mismatch");
_Static_assert(PCAN_CONFIG_TOTAL_LEN == 46, "config descriptor must be 46B");

/* --------------------------------------------------------------------------
 * String table, indexed by the PCAN_USB_STR_* constants of pcan_proto.h §0.
 * Index 0 is the LANGID (0x0409 US-English) as two raw bytes; indices 1.. are
 * UTF-8 source strings. Content is cosmetic — the PEAK drivers bind on VID/PID.
 * Not `const char *const *`: esp_tinyusb takes a `const char **`.
 * -------------------------------------------------------------------------- */
static const char *s_str_desc[PCAN_STRING_DESC_COUNT] = {
    /* [0] */ (const char[]){ 0x09, 0x04 }, /* 0x0409 US-English LANGID       */
    /* [1] */ "PEAK-System Technik GmbH",   /* PCAN_USB_STR_MANUFACTURER      */
    /* [2] */ "PCAN-USB",                    /* PCAN_USB_STR_PRODUCT           */
    /* [3] */ "",                            /* reserved / unused              */
    /* [4] */ "PCAN-USB Command",            /* PCAN_USB_STR_IF_CMD (iItf = 0) */
    /* [5] */ "PCAN-USB Message",            /* PCAN_USB_STR_IF_MSG (iItf = 0) */
};

const uint8_t *pcan_desc_device(void)
{
    return (const uint8_t *)&s_device_desc;
}

const uint8_t *pcan_desc_configuration(uint8_t index)
{
    /* Single configuration; the host only ever requests index 0. */
    (void)index;
    return s_fs_config_desc;
}

const char **pcan_desc_string_table(void)
{
    return s_str_desc;
}
