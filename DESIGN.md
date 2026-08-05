# espcan32 — Design

ESP32-S3 firmware that (a) emulates a **PEAK PCAN-USB classic (non-FD)** USB
adapter over TinyUSB + the legacy TWAI CAN peripheral, and (b) optionally runs a
**standalone on-device ISO-TP / UDS server** for bench diagnostics. The two
modes are mutually exclusive because the S3 has a single TWAI controller.

`main/pcan_proto.h` is the **locked contract**: every wire constant, endpoint
address, command code, record bitfield, timestamp format, and BTR table lives
there. No other file redefines those symbols. This document describes how the
modules around that contract fit together.

---

## 1. Module map

| Module | Role | Key contract types |
|--------|------|--------------------|
| `pcan_proto.h` | THE CONTRACT — all wire formats & the internal `pcan_frame_t` | `pcan_cmd_t`, `pcan_msg_hdr_t`, `pcan_status_rec_t`, `pcan_frame_t`, `pcan_btr_entry_t` |
| `config.h` | GPIO map, task layout, ring depths, mode selection | `CFG_*` |
| `usb_descriptors.[ch]` | Device/config/string descriptors (VID 0x0C72/PID 0x000C) | `pcan_desc_device/configuration/string` |
| `usb_glue.[ch]` | TinyUSB install + custom class driver routing the 2 bulk pairs | `usb_glue_*`, `pcan_drv_xfer_cb` |
| `pcan_cmd.[ch]` | Parse 16-byte EP1 commands, build GET replies, hold device state | `pcan_dev_state_t`, `pcan_cmd_handle` |
| `pcan_msg.[ch]` | Encode CAN→EP2-IN batches, decode EP2-OUT→CAN | `pcan_batch_t`, `pcan_msg_*` |
| `pcan_time.[ch]` | 16-bit device tick + calibration record helper | `pcan_ts_batch_t`, `pcan_time_now16` |
| `twai_hal.[ch]` | Legacy TWAI init/start/stop/tx/rx/state + BTR→timing | `pcan_twai_*`, `pcan_twai_status_t` |
| `ringbuf.[ch]` | PSRAM-backed SPSC `pcan_frame_t` queues | `pcan_ringbuf_t` |
| `isotp.[ch]` | ISO 15765-2 transport (bench mode) | `isotp_link_t` |
| `uds.[ch]` | ISO 14229-1 service dispatcher (bench mode) | `uds_server_t`, `uds_ctx_t` |
| `main.c` | Boot, mode select, task graph | `app_main` |

The canonical CAN frame `pcan_frame_t` (proto §7) is the single currency shared
by `twai_hal`, `ringbuf`, and `pcan_msg`, so no module invents its own frame
struct.

---

## 2. Task / threading model (PCAN mode)

Four FreeRTOS tasks plus the TinyUSB internal task. Priorities and core pinning
are in `config.h`.

```
                 EP1 OUT (cmd)                       EP1 IN (reply)
   host ──────────────────────► pcan_drv_xfer_cb ──────────────► host
                                    (TinyUSB task)
                                        │ pcan_cmd_handle
                                        ▼
                                  pcan_dev_state_t ◄──── ctrl_task (reconfigure)
                                                              │
   EP2 OUT (TX frames)                                        │ pcan_twai_reconfigure
   host ──► pcan_drv_xfer_cb ──► pcan_msg_decode_tx ──► tx_ring│
                                    (TinyUSB task)          │  ▼
                                                     can_tx_task ──► pcan_twai_transmit ──► CAN bus
                                                        (core 1)
   CAN bus ──► pcan_twai_receive ──► rx_ring ──► usb_tx_task ──► pcan_msg batch ──► EP2 IN ──► host
                twai_rx_task                     (core 0)         usb_glue_send_msg_batch
                 (core 1)                                          + periodic REC_TS calib
```

- **`twai_rx_task`** (core 1, prio 12): blocks on `pcan_twai_receive()`, stamps
  `ts16` at the instant of return, pushes into `rx_ring`. Highest priority so
  RX is never starved (RX loss shows up as `rx_ring` drops / TWAI overruns).
- **`usb_tx_task`** (core 0, prio 10): pops `rx_ring`, packs frames into a
  `pcan_batch_t` (≤64 B), flushes via `usb_glue_send_msg_batch()`. Emits a
  `REC_TS` calibration record every `PCAN_USB_TS_CALIB_PERIOD_MS` **alone in its
  own batch** (§5, `pcan_time.c` — the host only re-anchors on a `REC_TS` at
  record index 0), and folds any `REC_ERROR` posted by the control task into the
  same EP2-IN stream. A batch is retained and resubmitted until TinyUSB reports
  the transfer completed, so a busy or reset endpoint never silently eats frames.
- **`can_tx_task`** (core 1, prio 9): pops `tx_ring`, calls
  `pcan_twai_transmit()`, and keeps exactly ONE frame outstanding. A frame the
  driver accepted cannot be withdrawn, so ownership is released only by an event
  that says what became of the driver's copy: TX-success takes the next frame,
  TX-failure or a queue-discarding abort resubmits this one. A confirmation
  **timeout resubmits nothing** — silence is not evidence the frame is gone, and
  re-queuing it would put one host `write()` on the wire twice the moment a
  second node starts ACKing; the stall is counted and logged instead. It also
  does **not** synthesise a software loopback echo for self-reception (SRR)
  frames: `twai_hal` transmits them with the driver's `self` bit set, so the
  controller delivers its own copy back through the ordinary RX path and the
  host already sees the echo. Doing both would echo every self-request twice
  (see the integration notes at the top of `main.c`).
- **`ctrl_task`** (core 1, prio 8): watches `pcan_dev_state_t.reconfig_pending`;
  when set, quiesces the TWAI tasks, runs `pcan_twai_reconfigure(bitrate,mode)`
  (stop→uninstall→reinstall→start — the legacy driver has no live re-timing),
  clears the flag, resumes. Also polls `pcan_twai_service_alerts()` and turns
  bus-state changes into `REC_ERROR` records handed to `usb_tx_task` and TX
  completions into the events `can_tx_task` waits on. It runs on core 1
  because the ESP-IDF TWAI ISR is allocated on whichever core installs the
  driver, and every install happens here — this keeps all CAN work on core 1 and
  all USB work on core 0.

### ISR-safe boundaries
- Nothing in this firmware pushes into a ring from an ISR. The legacy TWAI
  driver delivers frames to a **task** via `twai_receive`, and alerts via
  `twai_read_alerts`; both are task-context. `ringbuf` therefore only needs
  task-level mutual exclusion (mutex + counting semaphore), not ISR-safe
  primitives.
- The class driver's `pcan_drv_xfer_cb()` runs in the TinyUSB task, not an ISR. It
  must not block; command handling copies the 16-byte transfer out and returns
  quickly, and TX-frame decode pushes into `tx_ring` before the endpoint is
  re-armed.

### Concurrency rules
- `pcan_twai_start/stop/reconfigure` are **control-task-only**. The RX/TX tasks
  treat `ESP_ERR_INVALID_STATE` (driver stopped mid-reconfigure) as "back off
  and retry", never as a fatal error. Because `twai_driver_uninstall()` deletes
  the driver's queues, `ctrl_task` first parks both tasks outside every
  `pcan_twai_*` call through a quiesce handshake: one `s_quiesce_seq` word whose
  parity is the request flag and whose value is the cycle id each parked task
  echoes back, so an ack left over from an earlier cycle can never be read as
  consent to tear the driver down in this one.
- Everything `ctrl_task` owes `can_tx_task` about a reconfigure — including the
  `CAN_TX_EVENT_ABORT` that says the driver discarded its queued copy — is
  posted **while `can_tx_task` is still parked**. Released first, it (prio 9)
  would preempt `ctrl_task` (prio 8) on core 1, submit the next frame, and then
  consume the abort as the verdict on *that* frame — resubmitting a copy the
  driver is already arbitrating for.
- `pcan_dev_state_t` is guarded by its own mutex; the USB callback and the
  control task both go through `pcan_cmd_*` accessors.

---

## 3. Data flow, both directions

### Device → host (RX / status), EP2 IN
1. Frame arrives on the wire → `pcan_twai_receive` → `pcan_frame_t{ts16}`.
2. `rx_ring` (PSRAM, `CFG_RB_RX_DEPTH`).
3. `usb_tx_task` builds a `pcan_batch_t`: `[type=2][rec_cnt][SL|id|ts|data]…`.
   - First timestamped record in the batch → 2-byte tick; subsequent → 1-byte
     low tick (`pcan_ts_append`, proto §4).
   - On-demand `REC_ERROR(1)` from the control task's alert masks, plus the
     periodic `REC_TS(4)` calibration, which ships as the only record of its
     batch. `REC_BUSEVT(5)` would carry the controller's rxerr/txerr counters,
     and this firmware never emits one — the byte offsets inside that record are
     still unresolved (§6), so those counters stay device-side telemetry
     (`bridge stats delta:` log lines).
4. `usb_glue_send_msg_batch` → EP 0x82 IN, ≤64 B.

### Host → device (TX), EP2 OUT
1. `pcan_drv_xfer_cb(ep=0x02)` → `pcan_msg_decode_tx` → `pcan_frame_t[]`.
   - Std id: `id = le16 >> 5`; Ext id: `id = le32 >> 3`; low flag bits →
     SRR/AT → `PCAN_FRAME_FLAG_SRR/SS`.
2. `tx_ring` → `can_tx_task` → `pcan_twai_transmit` (sets `extd/rtr/ss/self`).

### Command channel, EP1
- `pcan_drv_xfer_cb(ep=0x01)` → `pcan_cmd_handle`:
  - GET (SN=6/num1, DEVID=4/num1) → 16-byte reply on EP 0x81 (payload in
    bytes [2..15]).
  - SET (BITRATE/SET_BUS/EXT_VCC/ERR_FR/LED/REGISTER, and the raw restart
    SET_BUS-on) → mutate state, mark `reconfig_pending`, **no reply**.

---

## 4. Emulation fidelity plan

What we reproduce exactly (host-visible, load-bearing):
- **VID/PID** 0x0C72/0x000C and the four bulk EP **addresses** 0x01/0x81/0x02/
  0x82 — the drivers address endpoints absolutely.
- **`bcdDevice` high byte ≥ 41** (we ship 0x54FF → rev 84) so the host unlocks
  ONE_SHOT + LOOPBACK, and `> 3` for silent mode.
- **16-byte command framing** and GET-reply payload placement (bytes [2..15]).
- **BTR order** `args[0]=BTR1, args[1]=BTR0` on CMD_BITRATE.
- **EP2 record grammar**: SL byte flags, shifted/LE id words, first/subsequent
  timestamp widths, SRR trailer, `[type=2][rec_cnt]` header.
- **Timestamp cadence** ~42.667 µs/tick + periodic `REC_TS` so host µs time and
  16-bit wrap tracking stay accurate.
- **Open/close sequence tolerance**: SN(get) at probe, then any order of
  {ERR_FR, [SILENT], EXT_VCC, BITRATE, BUS-on} at open; CMD_BITRATE may arrive
  before the CMD_SET_BUS-on. We store+ACK BTR without decoding it (the host
  never reads it back); real bit timing is the S3 TWAI peripheral's job.

Interface layout — **single interface, four endpoints (matches real hardware).**
A fidelity review against the real `peak_usb` driver source and an `lsusb -v`
dump confirmed the genuine device is `bNumInterfaces=1` with all four bulk EPs
(0x01/0x81/0x02/0x82) under interface 0. An earlier two-`TUD_VENDOR_DESCRIPTOR`
layout was rejected: the mainline Linux driver probes **per interface** and would
bind twice (two colliding candev instances), and Windows risks leaving the data
EPs unconfigured. We therefore emit a **hand-rolled single-interface, 4-endpoint
config descriptor** and own the endpoints with a **custom TinyUSB application
class driver** (`usbd_app_driver_get_cb()`); `TUD_VENDOR_DESCRIPTOR` cannot
express a 4-EP interface. `bDeviceClass` stays `0xFF` (vendor) — setting it to 0
would make Windows treat the device as composite (`usbccgp`) and the PEAK INF
would not match.

One remaining **deliberate deviation** (host-transparent):
- **EP1 declared as 64-byte MPS, not 16.** The real device reports EP1 at 16-byte
  MPS; we declare all four EPs at 64. The command protocol never exceeds 16
  bytes/transfer, so a 64-byte EP1 is behaviorally identical to the host (it
  issues 16-byte bulk transfers regardless).

### ISO-TP/UDS bench mode selection
`app_mode_select()` (main.c) resolves the mode at boot:
1. `CFG_MODE_SELECT_GPIO` asserted at reset → **BENCH** (highest precedence).
2. else NVS key `pcan/mode` if present.
3. else `CFG_DEFAULT_MODE` (PCAN).

In BENCH mode the USB CAN path is **not** started (TinyUSB is never installed, so
the console keeps the USB PHY); a single `bench_task` owns the TWAI controller and
pumps `pcan_twai_receive → isotp_on_can_frame → isotp_poll →
uds_server_poll` on a 1 ms tick. Addressing defaults `CFG_BENCH_RX_ID=0x7E0`,
`CFG_BENCH_TX_ID=0x7E8`, functional `0x7DF` (SF-only). This keeps the diagnostic
stack fully decoupled from the emulation (UDS never touches CAN; only ISO-TP
does).

---

## 5. Per-module notes

§1 gives each file's role. This section records the non-obvious invariants —
the things that are not visible from a function signature and that a change is
likely to break silently.

### `usb_descriptors.c`
The full-speed configuration descriptor is **hand-rolled**: config header + one
9-byte vendor interface with `bNumEndpoints = 4` + four 7-byte bulk endpoint
descriptors, byte-matching the genuine device (§4). `bDeviceClass` must stay
`0xFF`; a class-0 device with this config makes Windows load `usbccgp` and the
PEAK INF stops matching. All four endpoints are declared at 64-byte MPS (the
real EP1 is 16 — host-transparent, §4) because the S3 DWC2 core requires 64-byte
bulk endpoints at full speed. `pcan_desc_string_table()` returns a pointer to a
program-lifetime static table of UTF-8 strings; esp_tinyusb keeps that pointer
in `tinyusb_config_t.descriptor.string` and dereferences it on every
GET_DESCRIPTOR(STRING), so the table must never be a caller-owned or scratch
buffer.

### `usb_glue.c`
Registers a custom `usbd_class_driver_t` via `usbd_app_driver_get_cb()` and
routes by **absolute endpoint address**, never by interface index.
- EP2-OUT is left unarmed until every frame decoded from the current transfer
  has been queued into `tx_ring`; the re-arm is deferred into TinyUSB task
  context with `usbd_defer_func` and gated on a USB connection generation
  counter, so a re-arm scheduled before a bus reset cannot fire against the
  re-enumerated device.
- `usbd_defer_func()` returns `void` and silently drops the request when
  TinyUSB's event queue is full, so neither OUT path clears its "needs re-arm"
  flag when it schedules: only a submit the stack accepted clears it. Scheduling
  is idempotent and a dropped defer costs one `usb_glue_service()` tick instead
  of leaving the endpoint unarmed for the rest of the session.
- An EP2-IN batch is retained until the completion callback reports success, and
  resubmitted after a USB reset. Losing a batch here is invisible to the host,
  so the buffer is never released optimistically.
- `usbd_edpt_xfer()` gained a fifth `is_isr` argument in TinyUSB 0.21; every
  call site is task context and passes `false`.
- `control_xfer_cb` stalls any unexpected EP0 class/vendor request rather than
  inventing a reply.

### `pcan_cmd.c`
Parses the fixed 16-byte EP1 command, mirrors it into `pcan_dev_state_t`, and
builds the 16-byte GET reply (payload at bytes [2..15]). Runs in TinyUSB task
context, so it must not block: the state mutex is only ever held for a handful
of field writes. GET replies; SET never does.

### `pcan_msg.c`
The only module that touches EP2 record bytes. The encoders build into a fixed
64-byte buffer and **refuse** (return `false`) any record that would overflow
instead of truncating — the caller flushes the batch, resets it and retries. The
batch also tracks whether the next timestamp will be 2 bytes or 1, so record
size is known before the record is committed.

### `pcan_time.c`
There is no timer ISR. The 16-bit device tick is derived on demand from
`esp_timer_get_time()` and the `PCAN_USB_TS_TICK_NS` (~42.667 µs) divisor, then
masked to 16 bits; masking an unbounded 64-bit tick count is naturally modular,
so the host reconstructs the high byte on low-byte wrap exactly as it would from
real hardware. Note the host-side rule this has to satisfy: the mainline driver
re-anchors its host↔device time reference (`peak_usb_set_ts_now`) **only when
the calibration record is the first record in its batch** — at any later index
it takes `peak_usb_update_ts_now`, which adds the whole elapsed tick count to a
running total that was never re-based, so host time then runs away by the sum of
every calibration interval so far. The record also updates the host's `ts16`
without touching its `prev_ts8`, so a single-byte timestamp decoded after it in
the same batch lands 256 ticks out. `usb_tx_task` therefore flushes before and
after the calibration record, shipping it as record 0 of a batch of one:
**`REC_TS` placement is as load-bearing as its cadence, and the flush pair is
what enforces it.**

### `twai_hal.c`
Wraps the legacy `driver/twai.h` API. Public symbols are prefixed `pcan_twai_*`
because ESP-IDF 6.0's `esp_hal_twai` exports its own `twai_hal_start/stop/…` and
collided at link time. One controller, classic CAN only (DLC ≤ 8). The legacy
driver cannot re-time a live controller, so every bitrate/mode change is a full
stop→uninstall→install→start and is control-task-only. A requested bitrate with
no `TWAI_TIMING_CONFIG_*` macro is snapped to the nearest supported rate and
logged. SRR frames are transmitted with `message.self` set, so the controller's
own copy comes back through the ordinary RX path — that is the echo the host
sees (§2). The legacy driver leaves `twai_message_t.self` unset on receive
("unused for received"), so there is deliberately **no** per-frame echo flag:
an SRR echo is indistinguishable from any other RX frame, which is exactly how a
genuine PCAN-USB presents it.

### `ringbuf.c`
SPSC ring of `pcan_frame_t` copied by value, PSRAM-backed with an internal-RAM
fallback. Two counting semaphores (`items`, `spaces`) plus a short mutex; the
consumer returns a space only **after** it has copied the slot out, which is
what prevents a producer from overwriting a frame whose item token the consumer
took just before being preempted. Task context only: `push()` uses the blocking
mutex API and must never be called from an ISR.

### `isotp.c`
Classic CAN, Normal 11-bit addressing, one link instance, poll-driven, no 32-bit
FF_DL escape. The receive side counts CFs taken since the last CTS in
`rx_block_cf` and sends the next Flow Control when that reaches `cfg.bs`, rather
than deriving block position from `rx_off` — the two only agree while every CF
is full, and a short mid-block CF (which `handle_cf` rejects as a protocol
error) would otherwise desync the FC accounting silently.

### `uds.c`
Table-driven dispatcher. SecurityAccess and TransferData state is file-static —
correct for the single bench server, **not** multi-instance safe. Session changes
are committed by the dispatcher (and drop any security unlock) only on a positive
handler result. `isotp_receive()` reports through its `out_functional` argument
whether the request arrived on the physical or the functional address; that feeds
`uds_ctx_t.functional`, which is what drives the ISO 14229-1 §A.2 rule that some
NRCs are answered with silence on the broadcast address. A response the transport
refuses while an earlier multi-frame reply is still draining cannot be answered
with an NRC either (it would interleave into the running CF sequence), so it is
counted in `resp_dropped` and left silent.

### `main.c`
Mode select, task graph, and the handshakes between them. Its header comment is
the authoritative record of three integration decisions: hardware echo instead of
software echo, clearing `reconfig_pending` *before* applying the reconfigure so a
command racing the reconfigure is re-applied rather than lost, and the quiesce
handshake that keeps `twai_driver_uninstall()` from running while a task is
blocked inside the driver.
