#!/usr/bin/env bash
# Full-duration SD + USB dump test for VLF5 @ ~416 Hz.
set -euo pipefail

CLI="${ROCKET_CLI:-/home/pegasis/Projects/RocketryProjects/Rust_Monorepo/target/release/rocket-cli}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="${OUT_DIR:-/tmp/vlf5_duration_test}"
VALIDATOR="$SCRIPT_DIR/duration_storage_test.py"
LOG="$OUT_DIR/test.log"

mkdir -p "$OUT_DIR"
exec > >(tee -a "$LOG") 2>&1

list_records() {
  "$CLI" list-flight-log 2>&1 | awk -F: '/^  records/{gsub(/ /,"",$2); print $2}'
}

download_records() {
  local csv="$1"
  local out
  out=$("$CLI" download-flight-log "$csv" 2>&1)
  echo "$out"
  echo "$out" | awk '/^Wrote/{print $2, $4}'  # wrote N of M
}

run_phase() {
  local name="$1"
  local secs="$2"
  local csv="$OUT_DIR/${name}.csv"

  echo ""
  echo "========== PHASE: $name (${secs}s) =========="
  echo "[$(date -Is)] clearing log..."
  "$CLI" clear-flight-log
  sleep 3
  local baseline
  baseline="$(list_records)"
  echo "[$(date -Is)] baseline records after clear settle: ${baseline:-0}"

  echo "[$(date -Is)] logging for ${secs}s..."
  local t0
  t0=$(date +%s)
  sleep "$secs"
  local t1
  t1=$(date +%s)
  local wall=$((t1 - t0))
  echo "[$(date -Is)] wall time: ${wall}s"

  local listed
  listed="$(list_records)"
  local new_records=$((listed - baseline))
  echo "[$(date -Is)] SD record count: $listed (new since baseline: $new_records)"

  echo "[$(date -Is)] downloading to $csv ..."
  local dl_out wrote of
  dl_out=$(download_records "$csv")
  echo "$dl_out"
  read -r wrote of <<< "$(echo "$dl_out" | awk '/^Wrote/{print $2, $4}')"
  local lines
  lines=$(($(wc -l < "$csv") - 1))
  echo "[$(date -Is)] CSV data rows: $lines (device header: $of, wrote: $wrote)"

  if [[ "$wrote" != "$of" ]]; then
    echo "FAIL: incomplete download ($wrote of $of)"
    return 1
  fi
  if [[ "$lines" != "$wrote" ]]; then
    echo "FAIL: CSV rows ($lines) != wrote ($wrote)"
    return 1
  fi
  # Download includes baseline + logging during transfer; should be >= new_records.
  local logged_in_dump=$((lines - baseline))
  local diff=$((logged_in_dump > new_records ? logged_in_dump - new_records : new_records - logged_in_dump))
  local tol=100  # records logged during list+download (~0.5s)
  if [[ $logged_in_dump -lt $((new_records - tol)) ]]; then
    echo "FAIL: only $logged_in_dump new records in dump vs $new_records during wait (lost data?)"
    return 1
  fi
  echo "[$(date -Is)] dump covers $logged_in_dump records in test window (expected ~$new_records)"

  python3 "$VALIDATOR" "$csv" "$secs" "$baseline"
}

echo "VLF5 duration storage test — $(date -Is)"
echo "rocket-cli: $CLI"
echo "output: $OUT_DIR"

FAIL=0
run_phase "1min" 60 || FAIL=1
run_phase "5min" 300 || FAIL=1
run_phase "20min" 1200 || FAIL=1

echo ""
if [[ $FAIL -eq 0 ]]; then
  echo "ALL PHASES PASSED"
else
  echo "ONE OR MORE PHASES FAILED — see $LOG"
  exit 1
fi
