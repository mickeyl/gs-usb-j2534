# gs-usb-j2534

A **SAE J2534 (PassThru) driver DLL for gs_usb / candleLight(FD) USB-CAN adapters**, with CAN FD
and ISO-TP, built for 64-bit and 32-bit applications. Register it once and every J2534 application
on the machine can use a cheap candleLightFD like a professional diagnostic interface.

## Why

Diagnostic and flashing tools on Windows talk J2534: they look in the registry for a PassThru
driver and load its DLL. Hardware that speaks the open **gs_usb** protocol (candleLight,
candleLightFD, many STM32/RP2040 boards) has no such driver, so closed-source tools cannot use it,
even though the hardware can do CAN FD at 2 Mbit/s for a fraction of the price.

The existing [j2534-bridge](../CANsole/packages/j2534-bridge) already speaks gs_usb natively, but it
is a *client library*, not a PassThru DLL: only the applications that embed it benefit. This project
closes the gap from the other side and turns the adapter into a regular PassThru device:

* any J2534 application can use it, 64-bit or 32-bit, no bridge process and no vendor driver;
* real CAN FD (BRS/ESI) and ISO 15765 on classic CAN and on CAN FD, including segmented messages;
* one small DLL with a static C runtime, MIT licensed, written in Rust.

## Status

| | |
|---|---|
| Protocols | `CAN`, `ISO15765`, `FD_CAN_PS` (0x8011), `FD_ISO15765_PS` (0x8012) |
| API | J2534-1 **04.04** (`PassThruOpen` … `PassThruIoctl`) |
| Builds | x64 DLL for 64-bit applications, x86 DLL for 32-bit applications (many diagnostic tools still are) |
| ISO-TP | segmentation both ways, flow control (BS/STmin), padding, extended addressing, timeouts |
| Robustness | automatic restart after bus-off, clean error when the adapter is unplugged |
| Output | `target\x86_64-pc-windows-msvc\release\gs_usb_j2534.dll`, `target\i686-pc-windows-msvc\release\gs_usb_j2534.dll` (no VC++ redistributable needed) |

**Known issue: transmitting with BRS.** Frames sent with the bit-rate switch at 2 Mbit/s made the
candleLightFD on the test bus run into bus-off and finally hang until it was power cycled, while the
same frames without BRS are reliable. The driver therefore sends FD frames at the nominal rate by
default (received BRS frames are fine). Findings, hypotheses and the test plan are in
[docs/brs-investigation.md](docs/brs-investigation.md); the switch is described under
[Behaviour and limits](#behaviour-and-limits).

Not (yet) supported: the 05.00 API variant, the GM `CAN_FD_PS` dialect, battery voltage (the hardware
has no ADC). Details under [Limits](#behaviour-and-limits).

## Quick start

Requirements: Windows 10/11 x64, a candleLight(FD) with a WinUSB driver bound (the default), Rust
with the `x86_64-pc-windows-msvc` and `i686-pc-windows-msvc` targets, GNU make in Git Bash.

```bash
make build        # build both DLLs (x64 + x86)
make check        # fmt, clippy, 34 tests per architecture (no hardware needed)
make install      # latest builds installed + registered for ALL users (UAC prompt)
make status       # what is registered, is it current, which program has it loaded
```

The adapter then shows up in J2534 applications as **“candleLightFD J2534 (gs-usb-j2534)”**. A
32-bit application only sees drivers registered in the 32-bit registry view and can only load a
32-bit DLL, which is why `make install` registers both flavours (`gs_usb_j2534_x64.dll` under
`%ProgramFiles%\GsUsbJ2534`, `gs_usb_j2534_x86.dll` under `%ProgramFiles(x86)%\GsUsbJ2534`).
`make` lists all targets, including hardware tests (`scan`, `listen`, `tp`, `uds`, `soak`, `cycle`,
`unplug`), `uninstall`, `trace-on`/`trace-off` and `dist` (zip with both DLLs, installer, README and
LICENSE).

Run `make` and `cargo` from a **Windows** shell (Git Bash, PowerShell or cmd), not from WSL: the
project builds for `x86_64-pc-windows-msvc` and needs the MSVC linker.

Without make: `scripts\install.ps1` (`-Status`, `-Uninstall`, `-InPlace`). The registration goes to
`HKLM\SOFTWARE\PassThruSupport.04.04` (64-bit view) and
`HKLM\SOFTWARE\WOW6432Node\PassThruSupport.04.04` (32-bit view).
A program that has the DLL loaded keeps it locked; the installer names it and asks you to close it.

**Only one program can own the adapter** (WinUSB allows a single handle): close CANcorder,
j2534-bridge native mode and similar tools before another one opens it, otherwise `PassThruOpen`
returns `ERR_DEVICE_IN_USE`.

## Using it

```text
PassThruOpen(NULL | "gs_usb:1d50:606f:SERIAL", &dev)
PassThruConnect(dev, FD_ISO15765_PS, CAN_ID_BOTH, 500000, &ch)          # nominal rate
PassThruIoctl(ch, SET_CONFIG, { FD_CAN_DATA_PHASE_RATE = 2000000, ISO15765_PAD_VALUE = 0xAA })
PassThruStartMsgFilter(ch, FLOW_CONTROL_FILTER, mask, pattern=<ECU id>, flow=<tester id>, &f)
PassThruWriteMsgs(ch, [<tester id> 3E 00])      # UDS TesterPresent
PassThruReadMsgs(ch, ...)                       # -> [<ECU id> 7E 00]
```

* The FD data phase defaults to 2 Mbit/s and is changed with `SET_CONFIG FD_CAN_DATA_PHASE_RATE`
  (the controller restarts briefly).
* Messages follow J2534 04.04: four identifier bytes (big endian, `CAN_29BIT_ID` in `TxFlags`), then
  the payload; with `ISO15765_ADDR_TYPE` an extended address byte follows. FD frames use
  `FD_CAN_FORMAT` (bit 20), `FD_CAN_BRS` (bit 19), `FD_CAN_ESI` (bit 21).
* ISO15765 frames are classic by default; set `FD_CAN_FORMAT` for FD frames and raise
  `FD_ISO15765_TX_DATA_LENGTH` (default 8) for larger ones.
* As the standard requires, nothing is received until a pass filter (raw CAN) or a flow-control
  filter (ISO15765) exists; loopback echoes obey the same filters.
* Several channels (for example `CAN` + `ISO15765`) can share the adapter at the same nominal rate.
* An FD-capable adapter always runs its controller in FD mode, also for classic `CAN`/`ISO15765`
  channels (data rate 2 Mbit/s unless set). A classic-only controller cannot decode FD frames and
  would flood an FD bus with error frames; classic frames are unaffected by FD mode.

## Verified

* **Tests** (`make test`, for x64 and x86): 17 unit tests (frame codec, bit timing, ISO-TP codec) and 17 end-to-end
  tests of bus + channels + ISO-TP against a simulated adapter with a fake ECU: single and
  multi-frame in both directions, BS/STmin pacing, FD 64-byte frames, extended addressing,
  N_Bs timeout, receiver overflow, filters, loopback ordering, periodic messages.
* **Real bus** (candleLightFD on a live CAN FD bus, 500 kbit/s / 2 Mbit/s):
  * ISO15765-FD TesterPresent (`02 3E 00`) to an ECU with 29-bit ids is answered with
    `02 7E 00 …`, also when sent with BRS from CANcorder.
  * Multi-frame UDS read (`22 F1 90`, 20-byte response): first frame, flow control and consecutive
    frames reassembled by the driver.
  * 20 open/connect/capture/close cycles, alternating with and without `PassThruDisconnect`.
  * 10-minute soak: passive capture of all frames plus a TesterPresent every 500 ms through a
    second channel on the same adapter. 1199 of 1199 answered (latency p50 11 ms, p95 16 ms,
    max 73 ms), 247,806 frames received (413/s), no overflow flag, no read errors. Five of six
    cyclic ids show one or two single gaps of 4-5 periods; the gaps differ per id, so there was
    no common host stall (none longer than ~43 ms). The cause on the sending side was not
    investigated.
* **Applications**: CANcorder (via j2534-bridge) and the bridge's `j2534-dump` open the driver
  through their PEAK-style SAE FD dialect (`0x805C`, `0x8001`, pass filters) and exchange frames.
  A commercial 32-bit diagnostic and flash tool talks UDS to an ECU over CAN FD through the
  x86 DLL: in a 38 s session 158 requests were sent and 266 ECU frames received, with no
  bus-off, no missing transmit confirmation and no error frame.
* **Flash tool attempts** (ECU reprogramming over the adapter, trace analysed):
  * Attempt 1 stopped at the first full 4095-byte TransferData block: all 65 consecutive frames
    were sent, one transmit bit error occurred during the block, and the ECU never answered. Not
    explained.
  * Attempt 2 transferred 199+ blocks of 4095 bytes (130-138 ms each, every one acknowledged by
    the ECU) and then stopped because `PassThruWriteMsgs` returned `ERR_BUFFER_FULL` in the middle
    of a consecutive-frame burst (all ten hardware slots were busy). The tool writes with timeout 0
    and does not retry a refused frame, so the block arrived incomplete. Fixed by the transmit
    queue (see below) and verified with `make burst`: 390 of 390 frames accepted, confirmed and in
    order, none refused. A complete flash over the adapter has not been repeated since.
  * For reference, the same flash over a PCAN-USB (CANcorder log) sends the 4095-byte blocks with
    BRS in 32 ms; without BRS they take 110-135 ms.
* **BRS on the test bus** (replay of 40 read-only `22 F1 xx` requests, 10 ms apart, candleLightFD on
  a 500 kbit/s / 2 Mbit/s vehicle bus):

  | transmit mode | result |
  |---|---|
  | BRS, data sample point 80 % | first frame: bus-off after 3 ms, controller hangs |
  | BRS, data sample point 90 % | 4 bus-offs in 40 frames, hung again in a second run |
  | no BRS (default) | 40/40 transmitted, no bus-off, reproduced in 3 runs |

Not verified on hardware: periodic messages, classic bit rates other than 500 kbit/s, data rates
above 2 Mbit/s.

## Behaviour and limits

* **One bus configuration per adapter.** Channels share the adapter if their nominal rate matches;
  rates cannot change while another channel is open. The first CAN channel of the adapter is used.
* **BRS is not transmitted by default.** On the test bench a transmission with BRS at 2 Mbit/s
  drove the candleLightFD into bus-off within milliseconds, and after a few of those the adapter
  hung (`PassThruOpen` fails with a pipe error) until it was power cycled; moving the data-phase
  sample point did not help. The cause is probably missing transmitter delay compensation, which
  the gs_usb protocol cannot configure. Received BRS frames are no problem. The driver therefore
  sends FD frames at the nominal rate even when the application sets `FD_CAN_BRS`; the ECU
  accepted them. Consequence: requests are slower (500 kbit/s for the data bytes), which is
  irrelevant for diagnostics but costs throughput for large FD transfers. With adapters or buses
  where BRS works, switch it on again: `make brs-honor` (registry `TxBrs=1`) or
  `GSUSB_J2534_TX_BRS=1` for one process; `make brs-strip` restores the default.
* **Transmit queue.** Raw CAN/FD writes go into a software queue of 4096 frames that feeds the ten
  hardware transmit slots strictly in order. `PassThruWriteMsgs` with timeout 0 therefore only
  returns `ERR_BUFFER_FULL` when that queue is full, not whenever the adapter is busy for a
  moment (applications such as flash tools write a whole ISO-TP block back to back and never retry).
  `CLEAR_TX_BUFFER` empties it.
* **Bus-off.** After a bus-off the controller is restarted automatically (up to 5 times in 5 s,
  then the channel reports `ERR_DEVICE_NOT_CONNECTED`). A bus-off that keeps recurring usually
  means a wrong bit rate or a bus fault.
* **Start-up.** Right after connecting, the controller needs a few tens of milliseconds to
  synchronise with a running bus; the first transmission waits (at most 250 ms) for the first valid
  frame.
* **Return semantics** follow the 05.00 text for 04.04 calls: `ReadMsgs` returns `ERR_BUFFER_EMPTY`
  when nothing arrived and `ERR_TIMEOUT` (with the partial count) when a non-zero timeout expires
  with fewer messages than requested. It waits for the whole timeout unless the requested count is
  reached, so request one message at a time when latency matters.
* `WriteMsgs` on ISO15765 with a non-zero timeout waits until the message is completely sent
  (including flow control); with `0` it only queues.
* Transmit confirmation uses the adapter's TX echo. A frame without echo for one second counts as
  failed (typically: nobody acknowledged it). Ten frames can be in flight.
* An unplugged adapter makes `ReadMsgs`/`WriteMsgs` return `ERR_DEVICE_NOT_CONNECTED`; disconnect,
  close and open again after plugging it back in.
* Not supported (`ERR_NOT_SUPPORTED`): `READ_VBATT`/programming voltage, K-line/J1850 protocols,
  `FIVE_BAUD_INIT`/`FAST_INIT`, periodic messages on ISO15765, RTR frames. J2534 defines no
  listen-only mode; use `GSUSB_J2534_LISTEN_ONLY=1` (see below).
* `PassThruScanForDevices` is deliberately **not** exported: j2534-bridge and other tools take that
  symbol as the marker of the incompatible 05.00 ABI (pointer-based `PASSTHRU_MSG`).
* `FD_ISO15765_TX_DATA_LENGTH = 0x805D` follows the contiguous J2534-2 numbering but could not be
  confirmed from a published source. `FD_CAN_DATA_PHASE_RATE` (0x805C), `J1962_PINS` (0x8001),
  `FD_CAN_PS` (0x8011) and `FD_ISO15765_PS` (0x8012) are verified.
* Bit timing: 80 % sample point (75 % at ≥ 4 Mbit/s), exact rates only. 4 Mbit/s BRS is not
  accepted on the candleLightFD bench (see the j2534-bridge notes), so it is never a default.

## Diagnostics

| Variable | Effect |
|---|---|
| `GSUSB_J2534_LOG=<file>` | trace of every API call, frame sent/received, error frame and recovery (one process) |
| registry `HKLM\SOFTWARE\GsUsbJ2534\TraceFile` | the same for **every** application; set with `make trace-on`, remove with `make trace-off` |
| `GSUSB_J2534_LISTEN_ONLY=1` | start the controller listen-only (no TX, no ACK): safe passive logging |
| `GSUSB_J2534_TX_BRS=1` / registry `HKLM\SOFTWARE\GsUsbJ2534\TxBrs`=1 | honor the application's BRS bit when sending (default: stripped, see above) |
| `GSUSB_J2534_DATA_TIMING=prop,ph1,ph2,sjw,brp` | explicit data-phase timing (checked against the controller limits and the bit rate); experiments only |
| `GSUSB_J2534_DATA_SAMPLE=<permille>` | data-phase sample point (default 800); a tuning knob, it did not cure the BRS failures above |

Set them before the application starts; programs that use j2534-bridge pass them on to the DLL.
Each frame sent is numbered (`TX seq=…`), its confirmation is logged with the delay (`TX echo seq=… after …`)
and flagged `REORDERED` if an earlier frame was still outstanding; controller error frames are decoded
(`protocol [bit1|while-transmitting] tec=8 rec=29`), and `bus stats: tx= echoes= reordered= error_frames= bus_off=`
is written when the bus shuts down. The trace grows quickly on a busy bus (about 60 kB/s), so turn the machine-wide trace off again;
`make trace-show` prints its tail.

Hardware tools (`examples/`), all driving the DLL through its exported API:

```bash
make scan                    # listen-only bit-rate scan, sends nothing
make tp                      # one ISO15765-FD TesterPresent (REQ/RESP, NOMINAL, DATA are variables)
make uds PAYLOAD=22F190      # one read-only UDS request, multi-frame responses included
make soak MINUTES=10         # passive capture + TesterPresent 2 Hz, gap analysis of cyclic ids
make cycle COUNT=20          # open/connect/close stress
make unplug                  # pull the adapter, plug it in again
make brs NOBRS=1             # replay of 40 read-only UDS requests (without NOBRS: BRS requested)
make burst BURSTS=6          # flash-tool-like transfer load: bursts of 65 x 64-byte frames, timeout 0
```

## Layout

```text
src/gsusb.rs    USB wire protocol, host-frame codec, bit timing        (CanAdapter trait)
src/bus.rs      reader thread, TX slots + echo tracking, fan-out, bus-off recovery
src/channel.rs  filters, RX queue, loopback, periodic messages, ISO-TP receive + transmit worker
src/isotp.rs    ISO 15765-2 codec / segmentation / reassembly (pure, unit-tested)
src/state.rs    device + channel registry, Connect, GET/SET_CONFIG
src/settings.rs machine-wide settings from the registry (trace file, TxBrs)
build.rs        x86 only: exports the undecorated stdcall names (`PassThruOpen`, not `_PassThruOpen@8`)
src/api.rs      exported PassThru* functions (panic-safe FFI boundary)
tests/e2e.rs    simulated adapter + fake ECU
examples/       probe.rs, stress.rs (hardware tools)
scripts/        install.ps1 (register / status / uninstall)
```

## Roadmap

* J2534-1 05.00 DLL (pointer-based `PASSTHRU_MSG`, `PassThruLogicalConnect`, `PassThruQueueMsgs`),
  a second DLL because the 04.04 and 05.00 symbols clash.
* GM `CAN_FD_PS` / `ISO15765_FD_PS` dialect (different protocol ids and flag bits).
* A standardised listen-only switch once its J2534-2 identifier is confirmed.

## License

[MIT](LICENSE). The J2534 identifiers were checked against SAE J2534-1 (05.00, 2022) and the
published J2534-2 values used by PEAK and Quantex/ScanDoc; no vendor code is used.
