#!/usr/bin/env bash
# run_toy.sh - board budget: build core-only firmware, flash the
# genuine Pico BY SERIAL under the shared workspace board lock,
# capture the hash stream, and diff it against the host emulator.
set -u
cd "$(dirname "$0")/.." || exit 9
ROOT="$PWD"
LEDGER_DIR="$(cd ../ledger 2>/dev/null && pwd)"
LOCKDIR="$LEDGER_DIR/board.lock.d"
LEGACY_LOCK="$LEDGER_DIR/board.lock"
PICOTOOL="${MSF_PICOTOOL:-/c/Users/logan/pico/picotool/picotool.exe}"
[ -n "${PICO_SDK_PATH:-}" ] || PICO_SDK_PATH="/c/Users/logan/pico/pico-sdk"
PY="${MSF_PYTHON:-../.venv/Scripts/python.exe}"
NINJA="${MSF_NINJA:-$(cygpath -w ../.venv/Scripts/ninja.exe 2>/dev/null | tr '\\' '/' || echo ../.venv/Scripts/ninja.exe)}"
BOARD_BUILD=build/board-fw
export TOY_BOARD_SERIAL="${TOY_BOARD_SERIAL:-E66440A2CB6F5C2B}" # the genuine Pico
mkdir -p "$BOARD_BUILD/tmp"
export TMPDIR="$ROOT/$BOARD_BUILD/tmp"
export TMP="$(cygpath -w "$TMPDIR" 2>/dev/null || echo "$TMPDIR")"
export TEMP="$TMP"

fail() { echo "BOARD: FAIL - $1"; exit 1; }

[ -d "$LEDGER_DIR" ] || fail "ledger dir not found at ../ledger"
[ -e "$LEGACY_LOCK" ] && fail "legacy board.lock held: $(cat "$LEGACY_LOCK")"
# Atomic lock: mkdir either succeeds or it does not (audit fix).
if ! mkdir "$LOCKDIR" 2>/dev/null; then
    fail "board lock held: $(cat "$LOCKDIR/holder" 2>/dev/null)"
fi
printf 'toy lead (Muse Code) pid %s host %s\n%s\n' "$$" "$(hostname)" "$(date '+%Y-%m-%d %H:%M:%S %z')" > "$LOCKDIR/holder"
trap 'rm -rf "$LOCKDIR"; echo "board lock released"' EXIT
echo "board lock taken (serial $TOY_BOARD_SERIAL)"

dir="$BOARD_BUILD/toy"
cmake -S board -B "$dir" -G Ninja -DCMAKE_MAKE_PROGRAM="$NINJA" \
    -DPICO_SDK_PATH="$PICO_SDK_PATH" -DCMAKE_BUILD_TYPE=Release \
    > "$dir-cmake.log" 2>&1 || { tail -20 "$dir-cmake.log"; fail "cmake configure"; }
cmake --build "$dir" > "$dir-build.log" 2>&1 || { tail -30 "$dir-build.log"; fail "firmware build"; }
UF2="$dir/toy_board.uf2"
echo "flashing $UF2 sha256=$(sha256sum "$UF2" | cut -d' ' -f1) -> serial $TOY_BOARD_SERIAL"
"$PICOTOOL" load --ser "$TOY_BOARD_SERIAL" -f -x "$UF2" > "$BOARD_BUILD/flash.log" 2>&1 \
    || { cat "$BOARD_BUILD/flash.log"; fail "picotool load (board may need BOOTSEL by hand)"; }
cat "$BOARD_BUILD/flash.log"
sleep 2

"$PY" board/read_toy.py TOY-ALL-DONE 900 > "$BOARD_BUILD/board-out.log" 2>&1 \
    || { cat "$BOARD_BUILD/board-out.log"; fail "board run"; }
cat "$BOARD_BUILD/board-out.log"

# Host side of the gate: identical step/hash lines, in order.
bash tools/run.sh host >/dev/null || fail "host build"
EMU="$ROOT/build/emu/toy_emu"
: > "$BOARD_BUILD/host-lines.txt"
for cart in $("$EMU" list); do
    "$EMU" proof "$cart" >> "$BOARD_BUILD/host-lines.txt"
done
grep -E '^(TOY-RUN|step|TOY-DONE)' "$BOARD_BUILD/board-out.log" > "$BOARD_BUILD/board-lines.txt"
if diff -u "$BOARD_BUILD/host-lines.txt" "$BOARD_BUILD/board-lines.txt"; then
    echo "board budget: PASS (host == board at every reported step)"
else
    fail "host/board hash streams differ"
fi
