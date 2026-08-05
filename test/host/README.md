# Host tests

Unit tests for the three modules that are pure logic with no hardware
dependency: the EP2 wire codec (`main/pcan_msg.c` + `main/pcan_time.c`), the
ISO-TP transport (`main/isotp.c`) and the UDS server (`main/uds.c`).

They compile the **real** sources out of `../../main` — nothing is copied or
forked — against minimal ESP-IDF stub headers in `stubs/`, so the tests cannot
drift from the firmware.

## Run them

```sh
./run_tests.sh
```

That is the whole thing: a C99 compiler and `make`. No ESP-IDF, no toolchain,
no test framework. Exit status is non-zero if any assertion fails. The suite
takes well under a second.

```sh
make            # same as run_tests.sh
make SAN=0      # skip the sanitizers
make clean
CC=gcc-14 make  # a different compiler
```

The tests live outside the firmware build: ESP-IDF only scans `main/` and
`components/` for components, and `main/CMakeLists.txt` lists its `SRCS`
explicitly, so `idf.py build` never sees this directory.

## What is covered

**`pcan_msg` / `pcan_time`** — the assertions are written against a model of
the *mainline Linux host driver* (`drivers/net/can/usb/peak_usb/pcan_usb.c`)
transcribed into `test_pcan_msg.c`: `pcan_usb_decode_msg`/`_status`/`_data`/
`_ts`/`_update_ts` for what we send, and `pcan_usb_encode_msg` for what we
receive. A test therefore says "the real host accepts this and recovers the
exact value we put in", not merely "our encoder agrees with our decoder".

- The `REC_TS` calibration record is byte-exact, the host recovers the tick,
  and the record does not consume the batch's first-timestamped-record slot.
- Timestamp widths: 2 bytes on the first timestamped record, 1 thereafter,
  including the host's high-byte reconstruction across a low-byte wrap.
- Standard/extended id shifting, RTR (no payload, no SRR trailer), the SRR
  echo trailer, and status/error records, all round-tripped through the host
  model.
- Batch overflow: refusal never writes past 64 bytes, a batch may be filled to
  exactly 64, and small records pack at the density the 2-then-1 timestamp
  rule implies.
- `decode_tx` against host-encoded batches, including a raw DLC of 9..15
  (`CAN_CTRLMODE_CC_LEN8_DLC`) decoding as an 8-byte frame while the records
  around it survive; truncation of every field; and never returning more
  frames than `max_out`.
- Two deterministic fuzz loops (fixed-seed LCG, 40k iterations each) over
  random and structurally-plausible batches. Each batch is placed flush
  against an **unmapped guard page**, so a read even one byte past the
  declared length segfaults — with or without a working sanitizer.

**`isotp`** — driven exactly as the bench task drives it: frames in through
`isotp_on_can_frame()`, time through a fake `millis()` the test steps by hand,
transmissions captured by a fake `can_tx()`.

- SF/FF/CF/FC happy paths, sequence numbers wrapping 15 -> 0, a short final CF,
  and rejection of a short mid-block CF.
- First Frame DLC validation (only a full 8-byte FF opens a transfer) and
  FF_DL that does not describe a multi-frame message.
- Block-size accounting for `bs` = 0, 1 and 2, counted in flow-control frames.
- N_Bs and N_Cr timeouts on the fake clock, including N_Cr being restarted by
  every CF rather than only by the FF; STmin pacing; the WAIT-frame budget.
- Functional addressing: a Flow Control, Consecutive Frame or First Frame on
  the broadcast id is ignored and **cannot** abort, stall or steer a
  physically addressed transfer, and a broadcast SingleFrame cannot clobber a
  reassembly in flight or an uncollected message.

**`uds`** — driven end to end over a real `isotp_link_t`, so requests arrive as
CAN frames and responses are reassembled back out of them.

- SecurityAccess: seed/key unlock, the level restriction (every unimplemented
  sub-function including the `0x00` and `0x80..0xFF` ranges), sendKey without
  a seed, and the lockout counter with its `requiredTimeDelayNotExpired` window
  ageing on the poll tick.
- An unlock is dropped when S3 expires in the DEFAULT session, and an explicit
  `10 01` re-locks even though the session did not change.
- TransferData block-sequence counter over a full `0x01..0xFF -> 0x00` wrap,
  the retransmission ack (and its refusal before any block has been accepted),
  and the declared transfer size as a ceiling.
- Functional-addressing NRC suppression (ISO 14229-1 §A.2) — silence for the
  NRCs that require it, a negative response for those that do not.
- ECUReset restarts only when its positive response actually reached the
  transport (or was legitimately suppressed), and never when the link refused
  it.
- Dispatcher basics: session gating, suppressPosRsp, the `0x78`
  ResponsePending re-poll pattern and its iteration cap.

## Stubs

`stubs/` holds the smallest headers that let the real sources compile
unmodified — `esp_timer.h`, `esp_system.h`, `esp_random.h`,
`freertos/FreeRTOS.h`, `freertos/task.h`. `stubs/esp_stubs.c` implements them:

- `esp_timer_get_time()` returns whatever the test set, so the timestamp
  engine can be walked across a 16-bit wrap without waiting 2.8 seconds.
- `esp_random()` is a fixed-seed LCG, so a SecurityAccess seed — and the key
  derived from it — is identical on every run.
- `esp_restart()` records the call and returns instead of rebooting, which is
  what lets a test assert that a reset did *not* happen.
- `vTaskDelay()` accumulates ticks instead of sleeping.

`stubs/esp_stubs.h` is the test-side control surface for all of that; the code
under test never sees it.

## Sanitizers

`sanprobe.sh` picks the strongest `-fsanitize` set the toolchain can both build
**and run**, falling back from `address,undefined` to `undefined` to none.
Linking is not a sufficient check: some hosts ship an ASan runtime that
deadlocks inside its own initialiser, which would hang the suite rather than
sanitise it. The chosen flags are printed before the run and cached in
`build/sanflags` (removed by `make clean`).

## Adding a test

A case is a `static void fn(void)` using the `CHECK*` / `REQUIRE*` macros from
`test.h`; register it with `RUN(fn)` in the suite function at the bottom of the
file. `CHECK` records a failure and carries on so one run reports everything
that is broken; `REQUIRE` returns from the case, for a precondition the rest of
it would read past.
