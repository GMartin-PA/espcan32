# espcan32

ESP32-S3 firmware that emulates a **PEAK PCAN-USB classic (non-FD)** USB→CAN
adapter. It presents VID `0x0C72` / PID `0x000C` with the genuine device's
descriptor layout, so PEAK's own host drivers — PCAN-Basic on Windows, the
mainline Linux `peak_usb` — bind to it directly and see a PCAN-USB.

It also ships an optional standalone **ISO-TP / UDS** bench server that runs the
diagnostic stack on-device, with no host involved.

## What it does

- **PCAN mode (default).** Four bulk endpoints on one vendor interface (EP
  0x01/0x81 command, EP 0x02/0x82 message), bridged to the ESP32-S3 TWAI
  peripheral. Classic CAN 2.0, 11- and 29-bit IDs, RTR, single-shot,
  self-reception, listen-only, bus-off recovery, hardware timestamps.
- **Bench mode (optional).** Ignores USB and runs an ISO 15765-2 transport plus
  an ISO 14229-1 UDS server directly on the CAN bus. Selected by a boot GPIO or
  an NVS key.

`DESIGN.md` covers the architecture, task model and per-module invariants.

## Hardware

### Chip

**ESP32-S3.** The firmware needs a USB-OTG peripheral (the USB-Serial-JTAG block
on the C-series is fixed-function and cannot present custom descriptors), a TWAI
controller, and two cores — the TWAI tasks are pinned to core 1 and the USB stack
to core 0 so CAN interrupts and USB servicing never contend. That rules out the
original ESP32 and the C-series (no USB device controller) and the S2 (single
core).

### What the board needs

- **An external CAN transceiver.** TWAI is a controller only; there is no PHY on
  the chip. SN65HVD230 or equivalent, 3.3 V.
- **At least 4 MB of flash.** The partition table tops out at 0x310000 (≈3.06
  MiB).
- **USB D+/D− routed to a connector** — GPIO19 and GPIO20, hardwired to the PHY.
- **Five spare GPIOs**, all remappable in `main/config.h`. Avoid the strapping
  pins (0, 3, 45, 46), the USB pins, and GPIO26–37 on modules with embedded
  octal flash or PSRAM.

PSRAM is optional. The ring buffers are about 30 KB and fall back to internal
RAM; set `CFG_RB_USE_PSRAM` to 0 on a module without it.

### Wiring

| SN65HVD230 | ESP32-S3 |
|---|---|
| D (TXD) | GPIO7 (`CFG_TWAI_TX_GPIO`) |
| R (RXD) | GPIO8 (`CFG_TWAI_RX_GPIO`) |
| Rs (slope) | GPIO10 (`CFG_TWAI_STANDBY_GPIO`), or tie to GND |
| Vcc / GND | 3V3 / GND |
| CANH / CANL | CAN bus, 120 Ω termination at each end |

**Mode select:** GPIO9 (`CFG_MODE_SELECT_GPIO`) held low at reset boots bench
mode. **Status LED:** GPIO2 (`CFG_STATUS_LED_GPIO`), mirrors the PCAN LED
command.

**Bitrates:** 1 M, 500 k, 250 k, 125 k, 100 k and 50 k. Other rates the host
requests are snapped to the nearest of these.

## Prebuilt binaries

Every push to `main` publishes a [release](../../releases) tagged `YYYYMMDDNN`
with all six board variants. Flash the merged image at offset 0:

```bash
esptool --chip esp32s3 write-flash 0x0 espcan32-<tag>-<board>-merged.bin
```

The matching `.zip` holds the separate app / bootloader / partition-table images
and the board config used, if you would rather flash them individually.

## Build

Requires **ESP-IDF 6.0**.

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p <port> flash monitor
```

`<port>` is the USB-Serial-JTAG device on a single-port board (macOS
`/dev/tty.usbmodem*`, Linux `/dev/ttyACM*`), or the UART bridge on a two-port
board (`/dev/tty.usbserial-*`, `/dev/ttyUSB*`).

On a single-port board the app takes the USB PHY once it starts, so the console
goes quiet and the port re-enumerates as `0c72:000c`. To re-flash, hold **BOOT**
(GPIO0), tap **RESET**, release BOOT — the ROM comes up as `303a:1001`. Booting
into bench mode also leaves the console alone.

### Building for a specific module

`sdkconfig.defaults` targets 8 MB flash with 8 MB octal PSRAM. `boards/` holds
one overlay per common ESP32-S3 module; pass it as a second defaults file and it
wins over the base:

```bash
idf.py -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;boards/esp32s3-n4.defaults" build
```

| Overlay | Flash | PSRAM |
|---|---|---|
| `esp32s3-n4` | 4 MB | none |
| `esp32s3-n8` | 8 MB | none |
| `esp32s3-n16` | 16 MB | none |
| `esp32s3-n8r2` | 8 MB | 2 MB quad |
| `esp32s3-n8r8` | 8 MB | 8 MB octal |
| `esp32s3-n16r8` | 16 MB | 8 MB octal |

The overlays change only the flash size and the PSRAM mode; every variant runs
the same firmware from the same partition table. If your module is not listed,
copy the closest overlay and adjust — those are the only two settings that have
to match your hardware.

## Bench mode

Hold GPIO9 low at reset. The device stops being a USB adapter and becomes a UDS
server on the CAN bus at 500 kbit/s: physical request `0x7E0`, response `0x7E8`,
functional `0x7DF`.

Implemented services: DiagnosticSessionControl, ECUReset, ReadDataByIdentifier,
SecurityAccess, RoutineControl, TesterPresent, RequestDownload, TransferData and
RequestTransferExit — enough to exercise a tester end to end, including
multi-frame transfers with block-size and STmin flow control.

Addressing, bitrate and the DID table are in `main/config.h` and `main/uds.c`.

## Tests

`test/host/` compiles the pure-logic modules — the EP2 wire codec, the ISO-TP
transport and the UDS server — straight out of `main/` against minimal ESP-IDF
stubs, so they cannot drift from the firmware.

```bash
./test/host/run_tests.sh
```

Needs a C99 compiler and `make`; no ESP-IDF and no test framework. Runs under
ASan and UBSan in well under a second. The wire-codec assertions are written
against a transcription of the Linux driver's decoder, so a pass means the real
host recovers what we encoded rather than that our encoder agrees with our
decoder. `idf.py build` never sees these.

## Layout

```
espcan32/
├── .github/workflows/         CI: build all board variants, publish a release
├── boards/                    per-module sdkconfig overlays (flash size, PSRAM)
├── partitions.csv             nvs + phy_init + 3 MB factory app
├── sdkconfig.defaults         TinyUSB + PSRAM + TWAI options
├── DESIGN.md                  architecture, tasks, per-module invariants
├── test/host/                 host unit tests
└── main/
    ├── pcan_proto.h           the wire-format contract
    ├── config.h               pins, tasks, mode select
    ├── usb_descriptors.[ch]   descriptor set
    ├── usb_glue.[ch]          TinyUSB + endpoint routing
    ├── pcan_cmd.[ch]          command channel
    ├── pcan_msg.[ch]          CAN <-> USB record codec
    ├── pcan_time.[ch]         timestamp engine
    ├── twai_hal.[ch]          TWAI backend
    ├── ringbuf.[ch]           frame queues
    ├── isotp.[ch]             ISO-TP transport (bench)
    ├── uds.[ch]               UDS server (bench)
    └── main.c                 entry + task wiring
```

## License

AGPL-3.0-or-later — see [LICENSE](LICENSE). Every source file carries an SPDX
header.

The PCAN-USB wire protocol was reverse-engineered from the mainline Linux
`peak_usb` driver. Only the wire constants and record layouts were derived from
it; no kernel source was copied.

PCAN and PEAK-System are trademarks of PEAK-System Technik GmbH. This project is
not affiliated with or endorsed by them, and deliberately presents their VID/PID
so their host drivers bind to it.
