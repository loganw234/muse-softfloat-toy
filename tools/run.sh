#!/usr/bin/env bash
# Front door. Budgets: host | check | board
#   host  - build core+emulator, run both cartridges, render sheets
#   check - determinism: same hash streams twice (host), byte-diffed
#   board - delegate to board/run_toy.sh (flash + read + compare)
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="$ROOT/build"
CC="${CC:-gcc}"
PY="${MSF_PYTHON:-python3}"
LIB="$ROOT/libs/muse-softfloat"

build_emu() {
    mkdir -p "$BUILD/emu"
    # shellcheck disable=SC2046
    "$CC" -O2 -std=c11 -Wall -Wextra \
        -I"$LIB/include" -I"$ROOT/core" \
        "$ROOT"/core/*.c "$ROOT"/emu/*.c "$LIB"/src/*.c \
        -o "$BUILD/emu/toy_emu"
}

budget="${1:-host}"
case "$budget" in
host)
    build_emu
    for cart in fig8 pendulum; do
        out="$BUILD/emu/$cart-240"
        mkdir -p "$out"
        # pendulum needs more sim-time per frame for the 1-ulp split to show
        spf=40; [ "$cart" = pendulum ] && spf=500
        "$BUILD/emu/toy_emu" frames "$cart" 240 "$spf" 8 "$out" > "$out/hashes.txt"
        "$PY" "$ROOT/emu/sheet.py" "$out" "$out/sheet.png" 4 >/dev/null
        echo "== $cart (240)"; cat "$out/hashes.txt"
    done
    out="$BUILD/emu/fig8-360"
    mkdir -p "$out"
    "$BUILD/emu/toy_emu" frames fig8 360 40 4 "$out" > "$out/hashes.txt"
    "$PY" "$ROOT/emu/sheet.py" "$out" "$out/sheet.png" 4 >/dev/null
    echo "== fig8 (360)"; cat "$out/hashes.txt"
    echo "budget host: PASS (sheets under build/emu/)"
    ;;
check)
    build_emu
    for cart in fig8 pendulum; do
        "$BUILD/emu/toy_emu" run "$cart" 10000 1000 > "$BUILD/emu/$cart-run-a.txt"
        "$BUILD/emu/toy_emu" run "$cart" 10000 1000 > "$BUILD/emu/$cart-run-b.txt"
        cmp "$BUILD/emu/$cart-run-a.txt" "$BUILD/emu/$cart-run-b.txt"
        echo "== $cart determinism"; cat "$BUILD/emu/$cart-run-a.txt"
    done
    echo "budget check: PASS"
    ;;
board)
    exec bash "$ROOT/board/run_toy.sh"
    ;;
*)
    echo "usage: tools/run.sh [host|check|board]" >&2
    exit 2
    ;;
esac
