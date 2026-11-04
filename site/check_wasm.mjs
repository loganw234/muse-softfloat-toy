import { readFile } from "node:fs/promises";
import { join } from "node:path";

const siteDir = process.argv[2] || ".";
const bytes = await readFile(join(siteDir, "toy.wasm"));
const expected = JSON.parse(await readFile(join(siteDir, "expected-hashes.json"), "utf8"));
// Time-layer glue imports: report the clock UNSET, exactly the
// first-class state the deterministic checker must prove under
// (pinned hashes are clock-independent by construction).
const clockImports = {
  env: {
    toy_js_clock_valid: () => 0,
    toy_js_clock_tz_min: () => 0,
    toy_js_clock_utc_lo: () => 0,
    toy_js_clock_utc_hi: () => 0,
    toy_js_mono_lo: () => 0,
    toy_js_mono_hi: () => 0,
  },
};
const { instance } = await WebAssembly.instantiate(bytes, clockImports);
const w = instance.exports;
const dec = new TextDecoder();

function cstr(ptr) {
  const mem = new Uint8Array(w.memory.buffer);
  let end = ptr;
  while (mem[end] !== 0) end++;
  return dec.decode(mem.subarray(ptr, end));
}
function hash() {
  return ((BigInt(w.toy_wasm_hash_hi() >>> 0) << 32n) |
          BigInt(w.toy_wasm_hash_lo() >>> 0)).toString(16).padStart(16, "0");
}

let failures = 0;
for (let i = 0; i < w.toy_wasm_count(); i++) {
  const name = cstr(w.toy_wasm_name(i));
  if (!w.toy_wasm_select(i)) throw new Error(`select failed for ${name}`);
  const total = w.toy_wasm_proof_steps(i);
  const every = w.toy_wasm_proof_report_every(i);
  const want = expected[name];
  if (!want) throw new Error(`no expected hashes pinned for cartridge ${name}`);
  for (let step = 1; step <= total; step++) {
    w.toy_wasm_step(1);
    if (every && step % every === 0) {
      const got = hash();
      const exp = want[String(step)];
      const ok = got === exp;
      if (!ok) failures++;
      console.log(`${ok ? "PASS" : "FAIL"} ${name} step ${step} hash ${got}${ok ? "" : ` expected ${exp}`}`);
    }
  }
  for (const size of [240, 360]) {
    if (!w.toy_wasm_render(size)) throw new Error(`render failed for ${name} at ${size}`);
    const ptr = w.toy_wasm_framebuffer();
    const pixels = new Uint16Array(w.memory.buffer, ptr, size * size);
    let distinct = new Set();
    for (const p of pixels) distinct.add(p);
    const ok = distinct.size >= 4;
    if (!ok) failures++;
    console.log(`${ok ? "PASS" : "FAIL"} ${name} render ${size}: ${distinct.size} distinct colours`);
  }
}
{
  // Clock glue round-trip: exports exist and report unset.
  const ok = typeof w.toy_wasm_clock_valid === "function" &&
             w.toy_wasm_clock_valid() === 0;
  if (!ok) failures++;
  console.log(`${ok ? "PASS" : "FAIL"} clock glue: valid=0 (unset)`);
}
console.log(failures ? `WASM CHECK: FAIL (${failures})` : "WASM CHECK: PASS");
process.exit(failures ? 1 : 0);
