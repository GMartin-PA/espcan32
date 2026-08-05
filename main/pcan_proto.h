/*
 * SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
/*
 * pcan_proto.h — THE CONTRACT
 * ============================================================================
 * Single source of truth for the PEAK PCAN-USB (classic, non-FD) wire
 * protocol as emulated by this ESP32-S3 firmware.
 *
 * Every USB identity constant, endpoint address, PCAN command function/number
 * code, message-record bitfield layout, timestamp/calibration format, and the
 * BTR0BTR1 -> bitrate table live here as #defines / enums / packed structs.
 * Every implementer codes against THIS file. Do not redefine any of these
 * symbols elsewhere; include this header instead.
 *
 * Provenance: reverse-engineered from the mainline Linux GPLv2 kernel driver
 *   drivers/net/can/usb/peak_usb/pcan_usb.c
 *   drivers/net/can/usb/peak_usb/pcan_usb_core.c
 *   drivers/net/can/usb/peak_usb/pcan_usb_core.h
 * plus lsusb -v dumps of real PCAN-USB units (bcdDevice 54.ff).
 *
 * ENDIANNESS: All multi-byte wire fields are LITTLE-ENDIAN unless a comment
 * says otherwise. The ESP32-S3 (Xtensa LX7) is little-endian, so packed
 * structs map directly, but code MUST still use the get/put_le helpers below
 * for any field that is not naturally aligned, because several PCAN wire
 * fields are deliberately unaligned (e.g. the 4-byte extended CAN id that
 * follows a 1-byte status/len prefix).
 * ============================================================================
 */
#ifndef PCAN_PROTO_H
#define PCAN_PROTO_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Compiler helpers ---------------------------------------------------------- */
#define PCAN_PACKED __attribute__((packed))

/* ==========================================================================
 * 0. USB DEVICE IDENTITY  (usb_descriptors.c)
 * ==========================================================================
 * The Linux/Windows PEAK drivers bind purely on VID/PID. The device must
 * still expose the exact endpoint map (section 1) to *function*.
 */
#define PCAN_USB_VENDOR_ID          0x0C72u  /* PEAK-System Technik GmbH        */
#define PCAN_USB_PRODUCT_ID         0x000Cu  /* PCAN-USB classic (NON-FD)       */

/* USB descriptor fixed field values (see usb_descriptors.c). ---------------- */
#define PCAN_USB_BCD_USB            0x0100u  /* matches real PCAN-USB (USB 1.00);
                                              * FS is set by the D+ pull-up, not
                                              * this field. */
#define PCAN_USB_BCD_DEVICE         0x54FFu  /* fw/hw rev; high byte = device_rev
                                              * low byte 0xFF. 0x54 -> rev 84,
                                              * which is >= 41 (unlocks ONE_SHOT
                                              * + LOOPBACK) and > 3 (unlocks
                                              * silent mode). Any 0x??FF where
                                              * high byte >= 0x29 is authentic
                                              * AND advertises >= rev 41.        */
#define PCAN_USB_DEVICE_REV         0x54u    /* == high byte of bcdDevice        */

#define PCAN_USB_EP0_MPS            64u      /* bMaxPacketSize0 (modern fw)      */
#define PCAN_USB_MAX_POWER_MA       200u     /* bMaxPower encodes as 0x64        */

/* device_rev capability thresholds (compared against PCAN_USB_DEVICE_REV) ---- */
#define PCAN_USB_REV_SILENT_MIN     3u       /* device_rev > 3  -> silent mode   */
#define PCAN_USB_REV_ONESHOT_MIN    41u      /* device_rev >=41 -> ONE_SHOT+LB   */

/* String descriptor indices (content does not affect binding). -------------- */
#define PCAN_USB_STR_LANGID         0u       /* 0x0409 US-English                */
#define PCAN_USB_STR_MANUFACTURER   1u       /* "PEAK-System Technik GmbH"       */
#define PCAN_USB_STR_PRODUCT        2u       /* "PCAN-USB"                       */
#define PCAN_USB_STR_SERIAL         0u       /* NONE: serial is a firmware cmd   */
#define PCAN_USB_STR_IF_CMD         4u       /* command interface string idx     */
#define PCAN_USB_STR_IF_MSG         5u       /* message interface string idx     */

/* ==========================================================================
 * 1. USB ENDPOINTS  (usb_descriptors.c / usb_glue.c)
 * ==========================================================================
 * Four BULK endpoints on a SINGLE vendor interface (bInterfaceNumber 0,
 * bNumEndpoints 4), matching the genuine PCAN-USB exactly. USB_DIR_IN = 0x80.
 *
 * The real device groups all four endpoints under one interface; a two-
 * interface split makes the mainline Linux peak_usb driver bind twice (it
 * probes per interface) and risks leaving the data EPs unconfigured on Windows.
 * We therefore emit a hand-rolled single-interface config descriptor and own
 * the four endpoints with a custom TinyUSB application class driver (TinyUSB's
 * TUD_VENDOR_DESCRIPTOR hardcodes bNumEndpoints=2 and cannot express this).
 *
 * On the wire the classic PCAN-USB reports EP1 (command) with 16-byte
 * wMaxPacketSize and EP2 (data) with 64-byte. We declare all four at 64: the
 * command protocol never exceeds 16 bytes/transfer, so a 64-byte EP1 is
 * behaviourally transparent (the host issues 16-byte bulk transfers regardless).
 */
#define USB_DIR_IN                  0x80u

#define PCAN_USB_EP_CMDOUT          0x01u    /* bulk OUT: host -> dev commands   */
#define PCAN_USB_EP_CMDIN           0x81u    /* bulk IN : dev -> host cmd reply  */
#define PCAN_USB_EP_MSGOUT          0x02u    /* bulk OUT: host -> dev CAN TX     */
#define PCAN_USB_EP_MSGIN           0x82u    /* bulk IN : dev -> host CAN RX+sts */

#define PCAN_USB_EP_CMD_WMAXPKT     16u      /* on-wire command EP packet size   */
#define PCAN_USB_EP_MSG_WMAXPKT     64u      /* on-wire data EP packet size      */
#define PCAN_USB_EP_MPS             64u      /* TinyUSB declared bulk MPS (S3)   */

/* Host-side buffer sizing (informational; constrains what we may emit). ------ */
#define PCAN_USB_RX_BUFFER_SIZE     64u      /* EP2 IN URB size the host submits */
#define PCAN_USB_TX_BUFFER_SIZE     64u      /* EP2 OUT URB size the host uses   */
#define PCAN_USB_MAX_CMD_LEN        32u      /* host cmd_buf alloc (16 used)     */
#define PCAN_USB_MAX_RX_URBS        4u
#define PCAN_USB_MAX_TX_URBS        10u
#define PCAN_USB_COMMAND_TIMEOUT_MS 1000u
#define PCAN_USB_STARTUP_TIMEOUT_MS 10u

/* ==========================================================================
 * 2. COMMAND MESSAGE FORMAT  (pcan_cmd.c) — EP1 OUT, reply on EP1 IN
 * ==========================================================================
 * Every command is a FIXED 16-byte bulk transfer: func, num, 14 arg bytes.
 * A GET reply is also 16 bytes; the host reads args from reply bytes [2..15].
 * A SET is fire-and-forget (host does not read EP1 IN afterward).
 */
#define PCAN_USB_CMD_FUNC           0u   /* byte offset of function code        */
#define PCAN_USB_CMD_NUM            1u   /* byte offset of number/sub-code      */
#define PCAN_USB_CMD_ARGS           2u   /* byte offset of arg region           */
#define PCAN_USB_CMD_ARGS_LEN       14u
#define PCAN_USB_CMD_LEN            16u  /* == ARGS + ARGS_LEN                  */

typedef struct PCAN_PACKED {
    uint8_t func;        /* [0]  PCAN_USB_CMD_*                                 */
    uint8_t num;         /* [1]  sub-code (GET/SET/bus-mode/...)               */
    uint8_t args[14];    /* [2..15]  little-endian argument payload           */
} pcan_cmd_t;
_Static_assert(sizeof(pcan_cmd_t) == PCAN_USB_CMD_LEN, "pcan_cmd_t must be 16B");

/* Function codes (byte [0]). ------------------------------------------------ */
#define PCAN_USB_CMD_BITRATE        1u   /* set BTR0/BTR1                       */
#define PCAN_USB_CMD_SET_BUS        3u   /* bus transceiver on/off, silent mode */
#define PCAN_USB_CMD_DEVID          4u   /* get/set CAN channel (device) id     */
#define PCAN_USB_CMD_SN             6u   /* get serial number                   */
#define PCAN_USB_CMD_REGISTER       9u   /* register access (SJA1000 init mode) */
#define PCAN_USB_CMD_EXT_VCC        10u  /* external VCC on/off                 */
#define PCAN_USB_CMD_ERR_FR         11u  /* error-frame / BERR reporting mask   */
#define PCAN_USB_CMD_LED            12u  /* LED on/off                          */

/* Sub-code values (byte [1]). ---------------------------------------------- */
#define PCAN_USB_GET                1u   /* generic GET                         */
#define PCAN_USB_SET                2u   /* generic SET                         */
#define PCAN_USB_BUS_XCVER          2u   /* num for CMD_SET_BUS: transceiver     */
#define PCAN_USB_BUS_SILENT_MODE    3u   /* num for CMD_SET_BUS: silent/listen   */

/* SJA1000 register-access value written via CMD_REGISTER on bus-off/close.    */
#define SJA1000_MODE_INIT           0x01u

/*
 * Per-command argument encoding (args[] is relative to pcan_cmd_t.args):
 *
 *   CMD_BITRATE (1),  num=SET(2):   args[0]=BTR1, args[1]=BTR0  (BTR1 FIRST!)
 *   CMD_SET_BUS (3),  num=XCVER(2): args[0] = !!bus_on
 *   CMD_SET_BUS (3),  num=SILENT(3):args[0] = !!listen_only  (rev>3 only)
 *   CMD_DEVID   (4),  num=GET(1):   reply args[0] = 8-bit channel id
 *   CMD_DEVID   (4),  num=SET(2):   args[0] = channel id (0..255)
 *   CMD_SN      (6),  num=GET(1):   reply args[0..3] = LE u32 serial
 *   CMD_EXT_VCC (10), num=SET(2):   args[0] = !!ext_vcc
 *   CMD_ERR_FR  (11), num=SET(2):   args[0] = err mask (PCAN_USB_BERR_MASK)
 *   CMD_LED     (12), num=SET(2):   args[0] = !!led_on
 *
 * Async restart written raw by the host: func=3, num=2, args[0]=1 (bus-on).
 */

/* Serial number reply is a little-endian u32 at args[0..3]. ----------------- */
#define PCAN_USB_DEFAULT_SERIAL     0x00C0FFEEu  /* emulator's advertised serial */
#define PCAN_USB_DEFAULT_DEVID      0x00u        /* emulator's default channel id*/

/* Error-frame reporting mask (host->device via CMD_ERR_FR at open). --------- */
#define PCAN_USB_ERR_RXERR          0x02u   /* enable rx-error reporting        */
#define PCAN_USB_ERR_TXERR          0x04u   /* enable tx-error reporting        */
#define PCAN_USB_BERR_MASK          (PCAN_USB_ERR_RXERR | PCAN_USB_ERR_TXERR) /*0x06*/

/* dev->state bits (device-side lifecycle mirror). --------------------------- */
#define PCAN_USB_STATE_CONNECTED    0x00000001u
#define PCAN_USB_STATE_STARTED      0x00000002u

/* ==========================================================================
 * 3. DATA (MESSAGE) RECORD FORMAT  (pcan_msg.c) — EP2 IN and EP2 OUT
 * ==========================================================================
 * Both directions begin with a 2-byte batch header, then 1..N records packed
 * up to 64 bytes total.
 */
#define PCAN_USB_MSG_HEADER_LEN     2u
#define PCAN_USB_MSG_TX_CAN         2u  /* header[0] type for a CAN TX/RX batch */

typedef struct PCAN_PACKED {
    uint8_t type;      /* [0] batch type; == PCAN_USB_MSG_TX_CAN for CAN data.
                        *     Host ignores this on RX (reads only rec_cnt).     */
    uint8_t rec_cnt;   /* [1] number of records packed after this header.      */
    /* records follow: uint8_t payload[] */
} pcan_msg_hdr_t;
_Static_assert(sizeof(pcan_msg_hdr_t) == PCAN_USB_MSG_HEADER_LEN, "hdr 2B");

/* --- 3.1 Per-record Status/Len (SL) prefix byte --------------------------- */
#define PCAN_USB_STATUSLEN_TIMESTAMP (1u << 7)  /* 0x80 record carries a ts     */
#define PCAN_USB_STATUSLEN_INTERNAL  (1u << 6)  /* 0x40 status/error record     */
#define PCAN_USB_STATUSLEN_EXT_ID    (1u << 5)  /* 0x20 29-bit extended id      */
#define PCAN_USB_STATUSLEN_RTR       (1u << 4)  /* 0x10 remote frame            */
#define PCAN_USB_STATUSLEN_DLC       (0x0Fu)    /* low nibble: DLC / rec_len    */

/* --- 3.2 CAN id shift/flag encoding (INTERNAL bit clear) ------------------ */
/*
 * Standard (11-bit): 2-byte LE word `id_flags`. can_id = id_flags >> 5.
 *   The 11-bit id occupies bits [15:5]; low 5 bits are TX flags.
 * Extended (29-bit): 4-byte LE word `id_flags`. can_id = id_flags >> 3.
 *   The 29-bit id occupies bits [31:3]; low 3 bits are TX flags.
 */
#define PCAN_USB_STD_ID_SHIFT       5u
#define PCAN_USB_EXT_ID_SHIFT       3u

/* TX low-order flag bits packed into the shifted id word (bits 0..2). ------- */
#define PCAN_USB_TX_SRR             0x01u  /* self-reception request: +1 trailer*/
#define PCAN_USB_TX_AT              0x02u  /* single-shot / no auto-retransmit  */

/*
 * CAN DATA record byte order after the SL byte (pcan_msg_decode_data):
 *   1. id word:  2 bytes (std) or 4 bytes (ext), LE, shifted as above.
 *   2. DLC:      taken from SL & 0x0F (NOT a separate wire byte). A raw value
 *                of 9..15 is legal classic CAN and means 8 data bytes; the host
 *                emits it when CAN_CTRLMODE_CC_LEN8_DLC is enabled.
 *   3. timestamp: present only on EP2 IN when SL & TIMESTAMP; 2 bytes on the
 *                 first timestamped record of the batch, else 1 byte (see §4).
 *                 EP2 OUT (host TX) records carry NO timestamp.
 *   4. data:     DLC bytes, unless SL & RTR (then none).
 *   5. SRR trailer: 1 extra byte if (id_flags & PCAN_USB_TX_SRR).
 *
 * EP2 OUT TX batch tail: obuf[TX_BUFFER_SIZE-1] holds a rolling 8-bit tx
 * counter used to correlate loopback echoes (writer id).
 */

/* ==========================================================================
 * 4. TIMESTAMP ENGINE  (pcan_time.c)
 * ==========================================================================
 * Free-running 16-bit counter. First timestamped record in a batch carries
 * the full LE 16-bit value; subsequent timestamped records carry only the low
 * 8 bits. The host reconstructs the high byte on low-byte wrap.
 *
 * Host microsecond conversion:  us = ticks * 44739243 >> 20  ~= 42.667 us/tick.
 * 16-bit counter wraps every ~2.796 s. The emulator should tick at ~42.667 us
 * (~23437.5 ticks/s) so host time is faithful, and emit REC_TS calibration
 * records periodically so the host anchors time_ref before a wrap aliases.
 */
#define PCAN_USB_TS_USED_BITS       16u
#define PCAN_USB_TS_US_PER_TICK     44739243u  /* us_per_ts_scale               */
#define PCAN_USB_TS_DIV_SHIFTER     20u        /* us_per_ts_shift               */
/* Nanoseconds per tick, exact: 44739243000 / 2^20 = 42667.4... ns.
 * Practical tick period target for the device timer: 42667 ns (~23437 Hz).  */
#define PCAN_USB_TS_TICK_NS         42667u
/* Recommended cadence for standalone REC_TS calibration records. The 16-bit
 * counter wraps at ~2.796 s; emit well inside that so the host never misses a
 * wrap. UNCERTAIN vs real hardware (see open_risks) — tune on the bench.     */
#define PCAN_USB_TS_CALIB_PERIOD_MS 1000u

/* ==========================================================================
 * 5. STATUS / ERROR RECORDS  (pcan_msg.c) — INTERNAL bit set
 * ==========================================================================
 * After the SL byte: func, num, then optional timestamp (same rule as data),
 * then rec_len payload bytes. rec_len = SL & 0x0F, except ANALOG->2, BUSLOAD->1.
 *
 * REC_TS is the exception to the timestamp rule: its tick is the PAYLOAD, not
 * the record timestamp. The host reads an LE16 at the payload position without
 * advancing its pointer and then closes the record with ptr += rec_len, so the
 * record must be exactly:
 *     SL = INTERNAL | 2       (TIMESTAMP bit CLEAR, rec_len = 2)
 *     func = REC_TS, num = 0
 *     payload = ts16 little-endian          -> 5 bytes total
 * Setting TIMESTAMP would make the host consume the timestamp bytes first and
 * then read the tick past the end of the record. With the bit clear the record
 * also does not count towards the batch's first-timestamped-record slot, which
 * keeps the 2-vs-1 byte timestamp width in step with the host's rec_ts_idx.
 */
typedef struct PCAN_PACKED {
    uint8_t func;  /* [0] PCAN_USB_REC_*                                       */
    uint8_t num;   /* [1] sub-code / error bitmask / counter direction         */
    /* optional timestamp + rec_len payload follow */
} pcan_status_rec_t;

/* Status record function codes (func). ------------------------------------- */
#define PCAN_USB_REC_ERROR          1u  /* error/bus-state; num = error bitmask */
#define PCAN_USB_REC_ANALOG         2u  /* analog/VCC; rec_len forced 2; ignored*/
#define PCAN_USB_REC_BUSLOAD        3u  /* bus load; rec_len forced 1; ignored  */
#define PCAN_USB_REC_TS             4u  /* pure timestamp calibration (§4)      */
#define PCAN_USB_REC_BUSEVT         5u  /* bus event: rx/tx error counters      */

/* Error record bitmask (num for REC_ERROR). -------------------------------- */
#define PCAN_USB_ERROR_TXFULL       0x01u
#define PCAN_USB_ERROR_RXQOVR       0x02u  /* rx queue overrun                  */
#define PCAN_USB_ERROR_BUS_LIGHT    0x04u  /* bus warning (light)               */
#define PCAN_USB_ERROR_BUS_HEAVY    0x08u  /* bus error passive/warning (heavy) */
#define PCAN_USB_ERROR_BUS_OFF      0x10u  /* bus-off                           */
#define PCAN_USB_ERROR_RXQEMPTY     0x20u
#define PCAN_USB_ERROR_QOVR         0x40u  /* general overrun (dropped pre-ts)  */
#define PCAN_USB_ERROR_TXQFULL      0x80u  /* device tx queue full              */
#define PCAN_USB_ERROR_BUS          (PCAN_USB_ERROR_BUS_LIGHT | \
                                     PCAN_USB_ERROR_BUS_HEAVY | \
                                     PCAN_USB_ERROR_BUS_OFF)   /* 0x1C          */

/* Bus-event record counter direction (num for REC_BUSEVT). ------------------ */
#define PCAN_USB_ERR_CNT_DEC        0x00u  /* counters decreasing               */
#define PCAN_USB_ERR_CNT_INC        0x80u  /* counters increasing               */
/* For BUSEVT, host reads rxerr at payload[+1], txerr at payload[+2] relative
 * to the func/num pointer (see open_risks: confirm against a real capture).  */

/* ==========================================================================
 * 6. BTR0BTR1 -> BITRATE TABLE  (pcan_cmd.c / twai_hal.c)
 * ==========================================================================
 * SJA1000 @ 8 MHz (PCAN_USB_CRYSTAL_HZ / 2). The emulated device only STORES
 * and ACKs BTR0/BTR1; the ESP32-S3 TWAI peripheral owns real bit timing. We
 * map the incoming (BTR0,BTR1) to a nominal bitrate so twai_hal can pick the
 * matching TWAI_TIMING_CONFIG_* macro. The 16-bit column is BTR0<<8 | BTR1.
 */
#define PCAN_USB_CRYSTAL_HZ         16000000u
#define PCAN_USB_CAN_CLOCK_HZ       (PCAN_USB_CRYSTAL_HZ / 2u)  /* 8 MHz         */

typedef struct {
    uint32_t bitrate;   /* nominal CAN bitrate in bit/s                        */
    uint8_t  btr0;      /* SJA1000 BTR0                                        */
    uint8_t  btr1;      /* SJA1000 BTR1                                        */
} pcan_btr_entry_t;

/* Canonical PEAK/PCANBasic SJA1000@8MHz values (safe reference table). ------ */
#define PCAN_BTR_TABLE_INIT { \
    { 1000000u, 0x00u, 0x14u }, \
    {  800000u, 0x00u, 0x16u }, \
    {  500000u, 0x00u, 0x1Cu }, \
    {  250000u, 0x01u, 0x1Cu }, \
    {  125000u, 0x03u, 0x1Cu }, \
    {  100000u, 0x43u, 0x2Fu }, \
    {   50000u, 0x47u, 0x2Fu }, \
    {   20000u, 0x53u, 0x2Fu }, \
    {   10000u, 0x67u, 0x2Fu }, \
    {    5000u, 0x7Fu, 0x7Fu }, \
}
#define PCAN_BTR_TABLE_LEN          10u
#define PCAN_USB_DEFAULT_BITRATE    500000u

/* ==========================================================================
 * 7. INTERNAL CANONICAL FRAME  (shared: ringbuf / twai_hal / pcan_msg)
 * ==========================================================================
 * The in-firmware representation of one classic CAN 2.0 frame. This is NOT a
 * wire format; it is the currency passed between the TWAI HAL, the ring
 * buffers, and the message codec. pcan_msg encodes it to / decodes it from the
 * EP2 wire records above.
 */
#define PCAN_FRAME_MAX_DLC          8u

/* pcan_frame_t.flags bits (device-internal, NOT the wire SL byte). ---------- */
#define PCAN_FRAME_FLAG_EXT         0x01u  /* extended 29-bit id               */
#define PCAN_FRAME_FLAG_RTR         0x02u  /* remote frame                     */
#define PCAN_FRAME_FLAG_SS          0x04u  /* single-shot (one-shot) TX        */
#define PCAN_FRAME_FLAG_SRR         0x08u  /* self-reception requested (TX)    */

typedef struct {
    uint32_t id;        /* 11- or 29-bit CAN id (right-justified, no flags)    */
    uint16_t ts16;      /* device 16-bit timestamp (filled by pcan_time on RX) */
    uint8_t  dlc;       /* 0..8                                                */
    uint8_t  flags;     /* PCAN_FRAME_FLAG_*                                   */
    uint8_t  data[PCAN_FRAME_MAX_DLC];
    uint8_t  writer_id; /* TX echo correlation byte (EP2 OUT tail / SRR)       */
} pcan_frame_t;

/* ==========================================================================
 * 8. LITTLE-ENDIAN ACCESS HELPERS (use for all unaligned wire fields)
 * ==========================================================================
 */
static inline uint16_t pcan_get_le16(const uint8_t *p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static inline uint32_t pcan_get_le32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline void pcan_put_le16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}
static inline void pcan_put_le32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

#ifdef __cplusplus
}
#endif
#endif /* PCAN_PROTO_H */
