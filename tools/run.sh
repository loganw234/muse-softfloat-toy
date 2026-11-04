#!/usr/bin/env bash
# Front door. Budgets: host | check | site | board
#   host  - build core+emulator, run all cartridges, render sheets
#   check - determinism: same hash streams twice (host), byte-diffed
#   site  - build live WebAssembly emulator and verify pinned hashes
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
    for cart in $("$BUILD/emu/toy_emu" list | tr -d '\r'); do
        for size in 240 360; do
            out="$BUILD/emu/$cart-$size"
            mkdir -p "$out"
            # 0 asks the cartridge for its live-demo pacing.
            "$BUILD/emu/toy_emu" frames "$cart" "$size" 0 40 "$out" > "$out/hashes.txt"
            "$PY" "$ROOT/emu/sheet.py" "$out" "$out/sheet.png" 8 >/dev/null
            echo "== $cart ($size)"; cat "$out/hashes.txt"
        done
    done
    echo "budget host: PASS (sheets under build/emu/)"
    ;;
check)
    build_emu
    for cart in $("$BUILD/emu/toy_emu" list | tr -d '\r'); do
        "$BUILD/emu/toy_emu" run "$cart" 10000 1000 > "$BUILD/emu/$cart-run-a.txt"
        "$BUILD/emu/toy_emu" run "$cart" 10000 1000 > "$BUILD/emu/$cart-run-b.txt"
        cmp "$BUILD/emu/$cart-run-a.txt" "$BUILD/emu/$cart-run-b.txt"
        echo "== $cart determinism"; cat "$BUILD/emu/$cart-run-a.txt"
    done
    echo "budget check: PASS"
    ;;
site)
    "$PY" "$ROOT/site/build.py" --cc "${TOY_CC:-zig cc}"
    ;;
board)
    exec bash "$ROOT/board/run_toy.sh"
    ;;
*)
    echo "usage: tools/run.sh [host|check|site|board]" >&2
    exit 2
    ;;
esac
