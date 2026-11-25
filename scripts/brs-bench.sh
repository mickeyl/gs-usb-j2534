#!/usr/bin/env bash
# BRS bench: the gs_usb adapter (this driver) transmits, a PCAN-USB FD (PEAK J2534 DLL) receives
# and ACKs. Needs a bus with exactly these two nodes and termination; there is no other node that
# could acknowledge the frames, so the monitor is always started first and runs until the sender
# has finished.
#
#   scripts/brs-bench.sh                      # default matrix
#   scripts/brs-bench.sh label:timing:brs ... # own cells; timing "-" = computed default,
#                                             # else "prop,ph1,ph2,sjw,brp" (data phase, 40 MHz clock)
#
# Environment: FRAMES (300), LEN (64), GAP ms (3), NOMINAL (500000), DATA (2000000), OUT (bench-out),
# PEAK_DLL, PEAK_NAME. A failing cell can hang the adapter; the script stops then and asks for a
# power cycle.
set -u
cd "$(dirname "$0")/.."
T=target/x86_64-pc-windows-msvc/release
STRESS=$T/examples/stress.exe
GS=$T/gs_usb_j2534.dll
PEAK=${PEAK_DLL:-"C:/Program Files (x86)/PEAK-System/PCAN-PassThru API/04.04/x64/PCANPT32.dll"}
PEAK_NAME=${PEAK_NAME:-"J2534-2:PEAK 0x51"}
NOMINAL=${NOMINAL:-500000}
DATA=${DATA:-2000000}
FRAMES=${FRAMES:-300}
LEN=${LEN:-64}
GAP=${GAP:-3}
OUT=${OUT:-bench-out}
mkdir -p "$OUT"

field() { sed -n "s/.*\b$1=\([^ ]*\).*/\1/p" | head -1; }

header() {
  printf '%-18s %-12s %-3s | %-9s %-9s | %-6s %-7s %-7s | %-6s %-6s %-6s\n' \
    cell timing brs "written" "echoed" "pcan" "corrupt" "missing" "busoff" "errfr" "max-tec"
}

cell() {
  local label=$1 timing=$2 brs=$3
  local secs=$(( FRAMES * GAP / 1000 + 8 ))
  local mon=$OUT/$label.monitor tx=$OUT/$label.tx log=$OUT/$label.trace
  rm -f "$mon" "$tx" "$log"
  "$STRESS" monitor --dll "$PEAK" --name "$PEAK_NAME" --nominal "$NOMINAL" --data "$DATA" --secs "$secs" >"$mon" 2>&1 &
  local mp=$!
  for _ in $(seq 1 150); do grep -q READY "$mon" 2>/dev/null && break; sleep 0.1; done
  if ! grep -q READY "$mon"; then
    echo "$label: the PCAN monitor did not start:"; cat "$mon"; kill "$mp" 2>/dev/null; return 2
  fi
  local -a e=("GSUSB_J2534_TX_BRS=$brs" "GSUSB_J2534_LOG=$log")
  [ "$timing" != "-" ] && e+=("GSUSB_J2534_DATA_TIMING=$timing")
  local -a a=(--dll "$GS" --nominal "$NOMINAL" --data "$DATA" --frames "$FRAMES" --len "$LEN" --gap-ms "$GAP")
  [ "$brs" = 1 ] && a+=(--brs)
  env "${e[@]}" "$STRESS" benchtx "${a[@]}" >"$tx" 2>&1
  wait "$mp"
  local txl monl
  txl=$(grep BENCHTX "$tx" || true)
  monl=$(grep MONITOR "$mon" || true)
  if [ -z "$txl" ]; then
    echo "$label: no result from the transmitter (adapter hung?):"; tail -3 "$tx"; return 3
  fi
  local busoff errfr tec
  busoff=$(grep -c 'bus-off: restarting' "$log" 2>/dev/null || true)
  errfr=$(grep -c 'RX error frame' "$log" 2>/dev/null || true)
  tec=$(grep -o 'tec=[0-9]*' "$log" 2>/dev/null | cut -d= -f2 | sort -n | tail -1)
  printf '%-18s %-12s %-3s | %-9s %-9s | %-6s %-7s %-7s | %-6s %-6s %-6s\n' \
    "$label" "$timing" "$brs" "$(echo "$txl" | field written)" "$(echo "$txl" | field echoed)" \
    "$(echo "$monl" | field rx)" "$(echo "$monl" | field corrupt)" "$(echo "$monl" | field missing)" \
    "$busoff" "$errfr" "${tec:-0}"
  echo "$txl" | grep -q 'lost=true' && { echo "  -> the adapter reported a lost link"; return 3; }
  return 0
}

if [ $# -gt 0 ]; then
  cells=("$@")
else
  cells=(
    nobrs:-:0
    brs-default:-:1
    peak-80-sjw4:7,8,4,4,1:1
    brp1-80-sjw2:7,8,4,2,1:1
    brp1-80-sjw1:7,8,4,1,1:1
    brp1-75-sjw4:7,7,5,4,1:1
    brp1-70-sjw4:6,7,6,4,1:1
    brp1-85-sjw3:8,8,3,3,1:1
    brp1-90-sjw2:8,9,2,2,1:1
    brp1-60-sjw4:5,6,8,4,1:1
    brp2-70-sjw2:3,3,3,2,2:1
    brp2-90-sjw1:4,4,1,1,2:1
  )
fi
echo "frames=$FRAMES len=$LEN gap=${GAP}ms nominal=$NOMINAL data=$DATA"
header
for c in "${cells[@]}"; do
  IFS=: read -r label timing brs <<<"$c"
  cell "$label" "$timing" "$brs"
  rc=$?
  if [ $rc -ne 0 ]; then
    echo "stopping after '$label' (rc=$rc). If the adapter hangs, power cycle it, then rerun the remaining cells."
    exit $rc
  fi
  sleep 1
done
