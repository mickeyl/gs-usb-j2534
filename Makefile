# gs-usb-j2534: 64-bit J2534 PassThru driver for gs_usb / candleLight(FD) adapters.
#
# Run from Git Bash (GNU make). `make` lists the targets.

SHELL := bash

TARGET   := x86_64-pc-windows-msvc
OUT      := target/$(TARGET)/release
DLL      := $(OUT)/gs_usb_j2534.dll
TARGET32 := i686-pc-windows-msvc
DLL32    := target/$(TARGET32)/release/gs_usb_j2534.dll
PROBE    := $(OUT)/examples/probe.exe
STRESS   := $(OUT)/examples/stress.exe
VERSION  := $(shell sed -n 's/^version = "\(.*\)"/\1/p' Cargo.toml | head -1)
PS       := powershell.exe -NoProfile -ExecutionPolicy Bypass
INSTALL  := $(PS) -File scripts/install.ps1

# Bus settings for the hardware targets (override: make tp NOMINAL=250000 DATA=1000000)
NOMINAL  ?= 500000
DATA     ?= 2000000
REQ      ?= 18DA10F1
RESP     ?= 18DAF110
MINUTES  ?= 10
COUNT    ?= 20
BURSTS   ?= 6

.DEFAULT_GOAL := help
.PHONY: help build test lint check install reinstall uninstall status dist clean version \
        examples scan listen tp uds soak cycle unplug trace-on trace-off trace-show brs-honor brs-strip brs burst

help: ## list the targets
	@echo "gs-usb-j2534 $(VERSION)"
	@echo
	@grep -E '^[a-z-]+:.*## ' $(MAKEFILE_LIST) | sed -E 's/^([a-z-]+):.*## (.*)/  make \1|\2/' | column -t -s '|'
	@echo
	@echo "Hardware targets accept NOMINAL, DATA, REQ, RESP (current: $(NOMINAL)/$(DATA), $(REQ)/$(RESP))."

build: ## build both DLLs (x64 for 64-bit applications, x86 for 32-bit applications)
	cargo build --release
	cargo build --release --target $(TARGET32)

examples: ## build the hardware test tools (probe, stress)
	cargo build --release --examples

test: ## unit + simulated-ECU tests for x64 and x86 (no hardware needed)
	cargo test --release
	cargo test --release --target $(TARGET32)

lint: ## rustfmt check + clippy, warnings are errors
	cargo fmt --check
	cargo clippy --release --all-targets -- -D warnings

check: lint test ## everything that should pass before a push

install: build ## build, then install + register the latest x64 and x86 DLLs for ALL users (UAC prompt)
	$(INSTALL)

reinstall: install ## alias for install

uninstall: ## remove the registration and the installed DLL (UAC prompt)
	$(INSTALL) -Uninstall

status: ## what is registered, is it current, which program has the DLL loaded
	$(INSTALL) -Status

dist: build ## zip both DLLs, installer, README and LICENSE into dist/
	@mkdir -p dist/gs-usb-j2534-$(VERSION)-x64/scripts
	cp $(DLL) dist/gs-usb-j2534-$(VERSION)-x64/gs_usb_j2534_x64.dll
	cp $(DLL32) dist/gs-usb-j2534-$(VERSION)-x64/gs_usb_j2534_x86.dll
	cp scripts/install.ps1 dist/gs-usb-j2534-$(VERSION)-x64/scripts/
	cp README.md LICENSE dist/gs-usb-j2534-$(VERSION)-x64/
	$(PS) -Command "Compress-Archive -Force -Path 'dist/gs-usb-j2534-$(VERSION)-x64/*' -DestinationPath 'dist/gs-usb-j2534-$(VERSION)-x64.zip'"
	@echo "dist/gs-usb-j2534-$(VERSION)-x64.zip"

# ---- hardware: the adapter must be free (close CANcorder & co. first) -------------------------

scan: examples ## listen-only bit-rate scan (sends nothing)
	$(PROBE) scan

listen: examples ## passive capture for 5 s at NOMINAL/DATA (sends nothing)
	$(PROBE) listen --nominal $(NOMINAL) --data $(DATA) --secs 5

tp: examples ## one ISO15765-FD TesterPresent REQ -> RESP (transmits one frame)
	$(PROBE) tp --nominal $(NOMINAL) --data $(DATA) --req $(REQ) --resp $(RESP) --ext --pad AA

uds: examples ## one read-only UDS request, e.g. make uds PAYLOAD=22F190
	$(STRESS) uds --nominal $(NOMINAL) --data $(DATA) --req $(REQ) --resp $(RESP) --payload $(or $(PAYLOAD),22F190)

soak: examples ## long run: passive capture + TesterPresent 2 Hz (MINUTES=10)
	$(STRESS) soak --nominal $(NOMINAL) --data $(DATA) --req $(REQ) --resp $(RESP) --minutes $(MINUTES)

cycle: examples ## open/connect/capture/close COUNT times
	$(STRESS) cycle --nominal $(NOMINAL) --data $(DATA) --count $(COUNT)

unplug: examples ## capture until you pull the adapter, then wait for it to return
	$(STRESS) unplug --nominal $(NOMINAL) --data $(DATA)

TRACE ?= C:\ProgramData\GsUsbJ2534\trace.log

trace-on: ## trace EVERY application using the driver into TRACE (UAC prompt; remember trace-off)
	$(INSTALL) -TraceFile '$(TRACE)'

trace-off: ## switch the machine-wide driver trace off again (UAC prompt)
	$(INSTALL) -NoTrace

trace-show: ## print the last 40 lines of the machine-wide trace
	@tail -n 40 '$(shell cygpath -u '$(TRACE)')'

brs-honor: ## send FD frames with the application's BRS bit (risky on marginal buses, UAC prompt)
	$(INSTALL) -TxBrs honor

brs-strip: ## send FD frames at the nominal rate (default, UAC prompt)
	$(INSTALL) -TxBrs strip

brs: examples ## replay a flash-tool-like burst of 40 read-only UDS requests (adds --nobrs via NOBRS=1)
	$(STRESS) brs --nominal $(NOMINAL) --data $(DATA) --req $(REQ) --resp $(RESP) --frames 40 $(if $(NOBRS),--nobrs)

burst: examples ## flash-tool-like transfer load: BURSTS x 65 frames of 64 bytes back to back (timeout 0)
	$(STRESS) burst --nominal $(NOMINAL) --data $(DATA) --count $(BURSTS)

version: ## print the crate version
	@echo $(VERSION)

clean: ## remove build output and trace logs
	cargo clean
	rm -rf dist *.log
