# Transmitting with BRS: findings and plan

Status: **resolved for this adapter** (BRS transmit at 2 Mbit/s is clean on the bench, on the vehicle bus and in a complete ECU flash; BRS is honored by default). The driver sends CAN FD frames without BRS by default (`TxBrs` unset); receiving
BRS frames works. This document records what is known and how to find out whether transmit BRS can
be made reliable on a candleLightFD (gs_usb), or has to stay off.

## What we know (measured)

* Adapter: candleLightFD `1d50:606f`, 40 MHz controller clock, features `0x5B3` (FD, BT_CONST_EXT,
  listen-only, timestamps; no termination or state reporting), `BT_CONST_EXT` stalls (an older
  firmware), firmware version not exposed (`bcdDevice 0.0.0`).
* Bus: a live CAN FD bus, 500 kbit/s nominal, 2 Mbit/s data phase. Receiving BRS frames is
  reliable (hundreds of thousands of frames, no losses).
* Transmitting with BRS at 2 Mbit/s fails: bit errors in the data phase, bus-off within
  milliseconds, and after a few of these the adapter hangs (`PassThruOpen` fails with a pipe
  error, a USB port restart does not help, a power cycle does).
* The same frames without BRS (FD at the nominal rate) are reliable: 40/40 in three runs, and a
  complete ECU flash (57,000 frames, no error) over the adapter.
* The failure is pattern dependent, not absolute: a few BRS frames got through, in one test 5/5 at a
  90 % sample point, then 4 bus-offs in the next 40.

| data timing (40 MHz) | result |
|---|---|
| BRP 2, 10 tq, sample point 80 %, SJW 2 (default) | first frame: bus-off after 3 ms, adapter hangs |
| BRP 1, 20 tq, sample point 90 %, SJW 2 | 5/5, then 4 bus-offs in 40 frames, hung again |

## Bench results (PCAN-USB FD as receiver and ACKer, two nodes, short cable)

`scripts/brs-bench.sh` sends pseudo-random frames from this driver; the PCAN verifies every byte
(sequence number plus reproducible payload) and the driver trace counts bus-offs and error frames.
All runs transmit with BRS (`GSUSB_J2534_TX_BRS=1`), 500 kbit/s nominal, default timing.

| data phase | frames | result |
|---|---|---|
| 1 Mbit/s | 2000 x 64 B | 2000 sent, 2000 received, no error |
| 2 Mbit/s | 3000 x 64 B, 2 ms apart | 3000 / 3000, no error |
| 2 Mbit/s | 2000 x 64 B back to back | 2000 / 2000, no error |
| 2 Mbit/s | 3000 x 8 B | 3000 / 3000, no error |
| 4 Mbit/s (sample point 70 %) | 2000 x 64 B | 6 sent, 0 received, bus-off, TEC 248, **adapter hangs** |
| 4 Mbit/s, sample point 90 % (`4,4,1,1,1`) | 2000 x 64 B, 1 ms apart | 2000 / 2000, no error |
| 4 Mbit/s, sample point 80 % (`3,4,2,2,1`) | 2000 x 64 B, 1 ms apart | 2000 / 2000, no error |
| 5 Mbit/s, sample point 87.5 % (`3,3,1,1,1`) | 2000 x 64 B | 5 sent, 0 received, bus-off, TEC 254, **adapter hangs** |

The failing 4 Mbit/s run used the computed default `[3,3,3,3,1]` (sample point 70 %, SJW 3); the
same rate with 80 % or 90 % is clean. So the sample point of the data phase decides, not the rate
alone. 5 Mbit/s (8 time quanta) is beyond what this adapter/bench does reliably.

## Vehicle bus results (second day, after the bench runs)

The same adapter on the live bus (500 kbit/s / 2 Mbit/s, ECU powered), frames of 64 bytes on an unused
id with BRS honored (`GSUSB_J2534_TX_BRS=1`), driver version of commit `e19e565`:

| data timing | frames | result |
|---|---|---|
| default (`3,4,2,2,2`, 80 %) | 40 + 1000 + 1500 + 1000 (8 B) + 1000 | all sent and confirmed; no bus-off; one run showed 32 receive error frames in its first ms (joining the bus), none afterwards |
| default | 3 x 10,000, 1 ms apart | 30,000 / 30,000, no error frame, no bus-off, no reordering |
| 95 % (`9,9,1,1,1`) | 100 + 1000 + 3000 | clean |
| 90 % (`8,9,2,2,1`) | 300 + 1000 | clean |
| default and 95 % | replay of 40 read-only UDS requests to the ECU (BRS) | 40/40 confirmed, ECU answered 36-37 (the rest are DIDs it does not know), no error |

So BRS transmit at 2 Mbit/s now works on the vehicle bus with the default timing. The earlier
failures (bus-off on the first frame, hang) cannot be reproduced. What changed since then is the driver:
the wait for bus sync after start, flow-control frames without BRS, the software TX queue and slot
handling; which one it was is not established (the transmit sequence of the old driver is gone). The
sample point still matters on the bench at 4 Mbit/s (70 % fails, 80/90 % clean), which fits missing TDC,
but 2 Mbit/s has enough margin on these buses.

### Flash with BRS honored

A complete ECU flash over the adapter with BRS honored (57,261 frames, all confirmed): 128 s, against
181 s for the same flash without BRS (about 30 % faster). No bus-off, no reordering, no frame loss; the
controller showed one excursion to error-passive (TEC 96 -> 128 within 1 ms, back to active after
0.2 s) right after a functional TesterPresent, with no frame affected. This closes the acceptance
list below, and BRS is now the default (`TxBrs=0` / `GSUSB_J2534_TX_BRS=0` restores stripping). The
default data sample point is also 80 % at 4 Mbit/s now (it was 75 %, which rounded to the failing 70 %).

Conclusions so far:

* On a short, clean bus BRS at 2 Mbit/s works. The failure on the vehicle bus is therefore not
  an inherent adapter problem but a margin problem on that bus (loop delay, stubs, transceivers).
* The failure mechanism (bit errors in the data phase, bus-off, then a hung adapter) is the same
  one seen on the vehicle bus, here at 4 Mbit/s. That fits a transmitter without effective
  delay compensation: its margin shrinks with the data rate and with the loop delay.
* A hung adapter after bus-off is reproducible and needs a power cycle.

## Hypotheses

1. **Missing or wrong transmitter delay compensation (TDC).** At data rates above about
   1 Mbit/s a transmitter must sample its own bit later than the nominal sample point, because the
   transceiver loop delay is a large part of a bit. The gs_usb host protocol has no TDC fields, so
   the host cannot set it; whether the firmware does it itself is unknown. PEAK's PCAN-USB FD uses
   the same 80 % sample point for 500 kbit/s / 2 Mbit/s but transmits BRS reliably on this bus.
2. **Timing resolution.** PEAK's table (PCAN-PassThru 05.00 manual, appendix A.4) uses 20 time
   quanta per data bit with SJW 4 (`BRP 2, TSEG1 15, TSEG2 4, SJW 4` at 80 MHz). The driver's
   default picks 10 time quanta with SJW 2 on this adapter (a tie-break in `calculate_timing`).
   The PEAK-like values (BRP 1 at 40 MHz, TSEG1 15, TSEG2 4, SJW 4) have **not** been tried.
3. **Bus and transceiver.** The loop delay (cable stubs, termination, transceiver) of this vehicle
   bus may simply exceed what this hardware tolerates at 2 Mbit/s without TDC.
4. **Old firmware.** Newer candleLight firmware may configure TDC or have other FD fixes; the
   adapter reports an old feature set.

## Plan

Do all risky steps on a **bench**: the adapter plus one other FD node (e.g. a PCAN-USB FD as ACKer
and monitor), short cable, 120 ohm termination at both ends, and no ECU. Expect to power cycle the
adapter after a failed run; use frames that nobody interprets (an unused id).

1. **Bench baseline.** Same test as on the vehicle (`make brs`-style replay with BRS, plus
   `make burst` with BRS), 3 runs. This separates the bus from the adapter:
   * works on the bench: the vehicle bus (loop delay, stubs) is the limiting factor; go to step 5.
   * fails on the bench: adapter timing or firmware; continue with steps 2-4.
2. **Timing matrix** (needs a small knob: allow `GSUSB_J2534_DATA_TIMING=prop,ph1,ph2,sjw,brp`,
   and make the tie-break prefer the smaller BRP). Test at 2 Mbit/s: BRP 1 and BRP 2; sample point
   70, 75, 80, 85, 90 %; SJW 1, 2, 4. Metric per cell: frames transmitted of 200, bus-offs, TEC
   at the end. Include PEAK's exact values first (cheapest experiment).
3. **Rate ladder.** Repeat the best timing at 1 Mbit/s, 1.5, 2, 2.5, 4 Mbit/s data. The highest
   rate that is clean gives the loop-delay margin, and shows whether hypothesis 1 fits (clean up
   to a threshold, then failing).
4. **Firmware.** Read the device descriptor and the DFU interface (the adapter exposes one),
   identify the firmware build, back it up (`dfu-util -U`), compare with the current candleLight
   firmware (does it enable TDC for the data phase?), update on the bench and repeat steps 1-3.
5. **Protocol support.** Check whether newer gs_usb (Linux driver, firmware) defines a TDC or
   "loop delay" feature bit. If it exists, implement it in `gsusb.rs` (feature detection,
   `DATA_BITTIMING` with TDC) and test.
6. **If it cannot be fixed on this hardware:** keep the default (strip BRS), document the
   supported combinations, and optionally decide per adapter from the firmware version whether
   BRS may be honored by default.

## Acceptance

* 10,000 BRS frames at 2 Mbit/s on the bench without a transmit error, bus-off or hang, in three
  consecutive runs, repeated on the vehicle bus (read-only requests) with the same result.
* A complete ECU flash over the adapter with BRS honored (`make brs-honor`): block time close to
  the PCAN reference (about 32 ms per 4095-byte block instead of 70-130 ms), no retries.
* Only then change the default of `TxBrs`, and only for the verified hardware/firmware.

## Open questions

* The first flash attempt over the adapter stopped at the first full block after a single transmit
  bit error (all 65 consecutive frames were sent, the ECU never answered); the cause is unexplained
  and may be related.
* Why some BRS frames pass and others fail (data pattern, position of the stuff bits, ACK timing).
* Whether the hang after bus-off is a firmware bug that should be reported upstream.
