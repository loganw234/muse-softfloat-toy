"""Build site/toy.wasm and prove it against pinned native hashes.

Toolchain: a clang targeting wasm32-freestanding. Developed with
`zig cc` 0.17.0. Pass --cc "..." or set TOY_CC / MSF_CC.
"""
import argparse
import os
import shlex
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SITE = Path(__file__).resolve().parent
LIB = ROOT / "libs" / "muse-softfloat"
LIB_SRC = ["msf16.c", "msf32.c", "msf64.c", "msf_alg.c", "msf_aug.c",
           "msf_flags.c", "msf_math.c", "msf_roundpack.c", "msf_string.c",
           "msf_wide.c"]
EXPORTS = [
    "toy_wasm_count", "toy_wasm_name", "toy_wasm_title", "toy_wasm_blurb",
    "toy_wasm_demo_steps_per_frame", "toy_wasm_proof_steps",
    "toy_wasm_proof_report_every", "toy_wasm_select", "toy_wasm_step",
    "toy_wasm_render", "toy_wasm_framebuffer", "toy_wasm_framebuffer_width",
    "toy_wasm_framebuffer_height", "toy_wasm_hash_lo", "toy_wasm_hash_hi",
    "toy_wasm_steps_lo", "toy_wasm_steps_hi", "toy_wasm_state_size",
]


def run(cmd: list[str], **kw) -> subprocess.CompletedProcess:
    print("+ " + " ".join(cmd), flush=True)
    return subprocess.run(cmd, **kw)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cc", default=os.environ.get("TOY_CC") or os.environ.get("MSF_CC", ""))
    ap.add_argument("--skip-check", action="store_true")
    args = ap.parse_args()
    if not args.cc:
        print('error: no compiler. Pass --cc "zig cc" (or set TOY_CC).', file=sys.stderr)
        return 2
    cc = shlex.split(args.cc, posix=False)

    core = sorted((ROOT / "core").glob("*.c"))
    cmd = cc + ["-target", "wasm32-freestanding", "-O2", "-nostdlib",
                "-Wl,--no-entry", "-Wl,-z,stack-size=1048576",
                "-Wl,--export-memory",
                f"-I{LIB / 'include'}", f"-I{ROOT / 'core'}"]
    cmd += [f"-Wl,--export={name}" for name in EXPORTS]
    cmd += [str(p) for p in core]
    cmd += [str(LIB / "src" / name) for name in LIB_SRC]
    cmd += [str(SITE / "wasm_glue.c"), "-o", str(SITE / "toy.wasm")]
    result = run(cmd)
    if result.returncode != 0:
        return result.returncode
    print(f"toy.wasm: {(SITE / 'toy.wasm').stat().st_size} bytes")

    if not args.skip_check:
        result = run(["node", str(SITE / "check_wasm.mjs"), str(SITE)])
        if result.returncode != 0:
            return result.returncode
    print("site build: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
