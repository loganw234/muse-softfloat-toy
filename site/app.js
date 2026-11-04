"use strict";

/* Web mock device: ONE live state presented on one round
 * panel of selectable size, plus the live input producers a
 * driver would own (pointer, dial, tilt, settings). Panel size
 * and browser pacing are presentation only - they never reset,
 * step or edit simulation state; the hash line is the proof.
 * Producers quantise to integers here or in C (see below) and
 * enter the sim only as events at step boundaries. Nothing on
 * this page simulates panel hardware. */

const $ = (id) => document.getElementById(id);
const statusEl = $("status");
const cartSelect = $("cart");
const playButton = $("play");
const resetButton = $("reset");
const speedInput = $("speed");
const speedValue = $("speed-value");
const panelSelect = $("panel");
const panelCustom = $("panel-custom");
const deviceEl = $("device");
const canvas = $("display");
const ctx = canvas.getContext("2d", { alpha: true });
const hashEl = $("hash");
const perfEl = $("perf");
const captionEl = $("panel-caption");

const PANEL_LABELS = { 240: "1.28″ RP2040", 360: "1.85″", 466: "1.75″ AMOLED", 480: "1.46″" };
const PANEL_MIN = 64, PANEL_MAX = 1024;
let panelSize = 360;
let image = ctx.createImageData(panelSize, panelSize);

/* Switching size re-renders the SAME state at a new backing
 * resolution (CSS scales the glass to fit). No select, no
 * step: the step/hash line must not move. */
function setPanelSize(size) {
  size = Math.min(PANEL_MAX, Math.max(PANEL_MIN, Math.round(Number(size) || 360)));
  panelSize = size;
  canvas.width = size;
  canvas.height = size;
  image = ctx.createImageData(size, size);
  panelCustom.value = String(size);
  panelSelect.value = PANEL_LABELS[size] ? String(size) : "";
  const label = PANEL_LABELS[size] ? ` · ${PANEL_LABELS[size]}` : " · custom";
  captionEl.textContent = `Mock device · ${size} × ${size}${label}`;
  if (wasm) render();
}
panelSelect.addEventListener("change", () => setPanelSize(Number(panelSelect.value)));
panelCustom.addEventListener("change", () => setPanelSize(Number(panelCustom.value)));

/* Time layer (glue only): JS owns the runner clock and feeds
 * WASM through env imports. It starts UNSET - a first-class
 * state; "Set clock" adopts the browser's current time + zone
 * offset as the manual set. The sim never reads it: pacing
 * below stays stepDebt -> integer step counts. */
const clockState = { valid: false, tzMin: -new Date().getTimezoneOffset() };
const clockEl = $("clock");
function clockUtcSeconds() {
  return clockState.valid ? Math.floor(Date.now() / 1000) : 0;
}
function refreshClockLabel() {
  if (!clockState.valid) { clockEl.textContent = "clock unset"; return; }
  let secs = clockUtcSeconds();
  if (wasm && typeof wasm.toy_wasm_clock_utc_lo === "function" &&
      wasm.toy_wasm_clock_valid()) {
    secs = Number((BigInt(wasm.toy_wasm_clock_utc_hi() >>> 0) << 32n) |
                  BigInt(wasm.toy_wasm_clock_utc_lo() >>> 0));
  }
  clockEl.textContent =
    `clock ${new Date(secs * 1000).toLocaleTimeString()} (UTC${clockState.tzMin >= 0 ? "+" : ""}${clockState.tzMin}m)`;
}
$("set-clock").addEventListener("click", () => {
  clockState.valid = true;
  clockState.tzMin = -new Date().getTimezoneOffset();
  refreshClockLabel();
});

let wasm = null;
let cartReady = false; /* a cartridge is selected (renderable) */
let selected = 0;
let playing = true;
let lastTime = 0;
let stepDebt = 0;
let lastRenderSecond = -1;
let framesThisSecond = 0;

function has(name) { return wasm && typeof wasm[name] === "function"; }

function cstr(ptr) {
  const mem = new Uint8Array(wasm.memory.buffer);
  let end = ptr;
  while (mem[end] !== 0) end++;
  return new TextDecoder().decode(mem.subarray(ptr, end));
}

function stateHash() {
  const value = (BigInt(wasm.toy_wasm_hash_hi() >>> 0) << 32n) |
                BigInt(wasm.toy_wasm_hash_lo() >>> 0);
  return value.toString(16).padStart(16, "0");
}

function stepsDone() {
  return (BigInt(wasm.toy_wasm_steps_hi() >>> 0) << 32n) |
         BigInt(wasm.toy_wasm_steps_lo() >>> 0);
}

/* Honest browser perf: measured ms/step inside this tab and
 * steps requested-but-not-executed at the chosen speed. These
 * numbers describe the browser only; board rates are measured
 * records elsewhere and are never scaled from this page. */
let msPerStep = null;
let perfRequested = 0;
let perfExecuted = 0;
function resetPerf() { msPerStep = null; perfRequested = 0; perfExecuted = 0; }
function perfLag() { return Math.max(0, Math.floor(perfRequested - perfExecuted)); }
function refreshPerf() {
  perfEl.textContent =
    `browser ${msPerStep === null ? "—" : msPerStep.toFixed(3)} ms/step · ` +
    `${perfLag()} steps behind requested speed · ` +
    `browser timing only — board rates are records, not scaled from this`;
}

function drawDisplay() {
  if (!cartReady) return; /* size may be chosen before first select */
  const size = panelSize;
  if (!wasm.toy_wasm_render(size)) throw new Error(`render failed at ${size}`);
  const ptr = wasm.toy_wasm_framebuffer();
  const pixels = new Uint16Array(wasm.memory.buffer, ptr, size * size);
  const rgba = image.data;
  const radius = size / 2;
  const r2 = radius * radius;
  for (let y = 0; y < size; y++) {
    const dy = y - radius + 0.5;
    for (let x = 0; x < size; x++) {
      const src = y * size + x;
      const dst = src * 4;
      const dx = x - radius + 0.5;
      if (dx * dx + dy * dy > r2) {
        rgba[dst + 3] = 0;
        continue;
      }
      const p = pixels[src];
      rgba[dst] = ((p >> 11) & 31) << 3;
      rgba[dst + 1] = ((p >> 5) & 63) << 2;
      rgba[dst + 2] = (p & 31) << 3;
      rgba[dst + 3] = 255;
    }
  }
  ctx.putImageData(image, 0, 0);
  hashEl.textContent = `step ${stepsDone()} · state ${stateHash()} · ${size}px glass (presentation only)`;
}

function render() { drawDisplay(); }

/* --- settings (SET_PARAM producer) ---
 * Controls are built from the cart's param table via the glue
 * introspection exports. Kinds (core/toy_input.h): 1=I32,
 * 2=ENUM, 3=F64 (there is no bool kind; a 0/1 ENUM covers it).
 * A change queues SET_PARAM immediately, except RESET_ONLY
 * params, which wait for the next reset and are then queued
 * in param order before step 1 (the step-0 setup slot carts
 * reseed in). Untouched params are never queued, so declared
 * defaults and compiled-in step-0 proof settings are exactly
 * what an untouched session runs. F64 values cross as the IEEE
 * bits of the number the control shows; the session log
 * records those bits, so replays are exact. */
const PARAM_I32 = 1, PARAM_ENUM = 2, PARAM_F64 = 3;
const PARAM_RESET_ONLY = 1;
const CAP_TILT = 1 << 4, CAP_DIAL = 1 << 8;
const paramsEl = $("params");
let paramState = [];
const f64View = new DataView(new ArrayBuffer(8));
function f64Bits(value) {
  f64View.setFloat64(0, value, false);
  const b = f64View.getBigUint64(0, false);
  return [Number(b & 0xffffffffn) >>> 0, Number(b >> 32n) >>> 0];
}

function queueParam(i) {
  if (!has("toy_wasm_queue_param")) return;
  const p = paramState[i];
  if (p.kind === PARAM_F64) {
    const [lo, hi] = f64Bits(p.value);
    wasm.toy_wasm_queue_param(i, 0, lo, hi);
  } else {
    wasm.toy_wasm_queue_param(i, p.value | 0, 0, 0);
  }
}

function refreshParamFlag(p) {
  if (!p.flagEl) return;
  p.flagEl.textContent = p.touched
    ? " (applies on reset · pending)" : " (applies on reset)";
}

function onParamInput(i, value) {
  const p = paramState[i];
  if (p.kind === PARAM_F64) {
    value = Math.min(p.max, Math.max(p.min, Number(value)));
    if (Number.isNaN(value)) return;
  } else {
    value = Math.min(p.max, Math.max(p.min, Math.round(Number(value)) || 0));
  }
  p.value = value;
  p.touched = true;
  if (p.rangeEl) p.rangeEl.value = String(value);
  if (p.numEl) p.numEl.value = String(value);
  if (p.selEl) p.selEl.value = String(value);
  if (p.flags & PARAM_RESET_ONLY) refreshParamFlag(p);
  else queueParam(i);
}

function buildParams() {
  paramsEl.textContent = "";
  paramState = [];
  const count = has("toy_wasm_param_count") ? wasm.toy_wasm_param_count(selected) : 0;
  if (!count) {
    const none = document.createElement("p");
    none.className = "no-settings";
    none.textContent = "no settings";
    paramsEl.append(none);
    return;
  }
  for (let i = 0; i < count; i++) {
    const kind = wasm.toy_wasm_param_kind(selected, i);
    const flags = wasm.toy_wasm_param_flags(selected, i);
    const label = cstr(wasm.toy_wasm_param_label(selected, i)) ||
                  cstr(wasm.toy_wasm_param_name(selected, i));
    const p = { kind, flags, touched: false, rangeEl: null, numEl: null, selEl: null, flagEl: null, value: 0, min: 0, max: 0 };
    const wrap = document.createElement("div");
    wrap.className = "param";
    const nameEl = document.createElement("span");
    nameEl.className = "pname";
    nameEl.textContent = label;
    if (flags & PARAM_RESET_ONLY) {
      p.flagEl = document.createElement("span");
      p.flagEl.className = "pflag";
      nameEl.append(p.flagEl);
      refreshParamFlag(p);
    }
    wrap.append(nameEl);
    const row = document.createElement("div");
    row.className = "prow";
    if (kind === PARAM_ENUM) {
      p.min = wasm.toy_wasm_param_i(selected, i, 0);
      p.max = wasm.toy_wasm_param_i(selected, i, 1);
      p.value = wasm.toy_wasm_param_i(selected, i, 2);
      p.selEl = document.createElement("select");
      p.selEl.setAttribute("aria-label", label);
      const n = wasm.toy_wasm_param_enum_count(selected, i);
      for (let e = 0; e < n; e++) {
        const opt = document.createElement("option");
        opt.value = String(e);
        opt.textContent = cstr(wasm.toy_wasm_param_enum_name(selected, i, e)) || String(e);
        p.selEl.append(opt);
      }
      p.selEl.value = String(p.value);
      p.selEl.addEventListener("change", () => onParamInput(i, Number(p.selEl.value)));
      row.append(p.selEl);
    } else if (kind === PARAM_F64) {
      p.min = Number(cstr(wasm.toy_wasm_param_f(selected, i, 0)));
      p.max = Number(cstr(wasm.toy_wasm_param_f(selected, i, 1)));
      p.value = Number(cstr(wasm.toy_wasm_param_f(selected, i, 2)));
      const step = (p.max - p.min) / 200 || 0.001;
      p.rangeEl = document.createElement("input");
      p.rangeEl.type = "range";
      p.rangeEl.min = String(p.min); p.rangeEl.max = String(p.max);
      p.rangeEl.step = String(step); p.rangeEl.value = String(p.value);
      p.rangeEl.setAttribute("aria-label", label);
      p.rangeEl.addEventListener("input", () => onParamInput(i, Number(p.rangeEl.value)));
      p.numEl = document.createElement("input");
      p.numEl.type = "number";
      p.numEl.min = String(p.min); p.numEl.max = String(p.max);
      p.numEl.step = String(step); p.numEl.value = String(p.value);
      p.numEl.setAttribute("aria-label", `${label} value`);
      p.numEl.addEventListener("change", () => onParamInput(i, Number(p.numEl.value)));
      row.append(p.rangeEl, p.numEl);
    } else { /* I32 (and any future int-like kind) */
      p.min = wasm.toy_wasm_param_i(selected, i, 0);
      p.max = wasm.toy_wasm_param_i(selected, i, 1);
      p.value = wasm.toy_wasm_param_i(selected, i, 2);
      p.rangeEl = document.createElement("input");
      p.rangeEl.type = "range";
      p.rangeEl.min = String(p.min); p.rangeEl.max = String(p.max);
      p.rangeEl.step = "1"; p.rangeEl.value = String(p.value);
      p.rangeEl.setAttribute("aria-label", label);
      p.rangeEl.addEventListener("input", () => onParamInput(i, Number(p.rangeEl.value)));
      p.numEl = document.createElement("input");
      p.numEl.type = "number";
      p.numEl.min = String(p.min); p.numEl.max = String(p.max);
      p.numEl.step = "1"; p.numEl.value = String(p.value);
      p.numEl.setAttribute("aria-label", `${label} value`);
      p.numEl.addEventListener("change", () => onParamInput(i, Number(p.numEl.value)));
      row.append(p.rangeEl, p.numEl);
    }
    wrap.append(row);
    paramsEl.append(wrap);
    paramState.push(p);
  }
}

/* After a reset, user-set values (live and RESET_ONLY alike)
 * are queued in param order, synchronously, so they drain
 * before step 1 - the step-0 slot. Settings therefore persist
 * across Reset like a device's would; a cart switch rebuilds
 * controls at defaults instead. */
function queueTouchedParams() {
  for (let i = 0; i < paramState.length; i++) {
    const p = paramState[i];
    if (!p.touched) continue;
    queueParam(i);
    if (p.flags & PARAM_RESET_ONLY) {
      p.flagEl.textContent = " (applies on reset · applied)";
    }
  }
}

function refreshCaps() {
  const caps = has("toy_wasm_input_caps") ? wasm.toy_wasm_input_caps(selected) : 0;
  $("dial-card").classList.toggle("ignored", !(caps & CAP_DIAL));
  $("dial-card").title = (caps & CAP_DIAL) ? "" : "This cartridge does not listen to the dial";
  $("tilt-card").classList.toggle("ignored", !(caps & CAP_TILT));
  $("tilt-card").title = (caps & CAP_TILT) ? "" : "This cartridge does not listen to tilt";
}

/* --- dial producer (TOY_EV_DIAL id 0) ---
 * Two sources, one integer currency: milliturns (1000 = one
 * full turn, clockwise positive). Only whole milliturns are
 * emitted; sub-milliturn remainders stay in the accumulators.
 *  - Wheel over the device: deltaY is normalised to pixels
 *    (deltaMode 1 lines x16, 2 pages x400), accumulated in
 *    milli-pixels; 4800 px of travel = 1 turn, so each emitted
 *    milliturn consumes exactly 4800 milli-px. Scroll-down
 *    (deltaY > 0) is positive (clockwise).
 *  - Bezel rim-drag: the pointer's angle about the glass
 *    centre (atan2 in screen coords, where +angle is visually
 *    clockwise) is unwrapped and accumulated in radians; each
 *    emitted milliturn consumes exactly 2pi/1000 rad. */
const dialReadout = $("dial-readout");
let dialTotal = 0;
let wheelAccMilliPx = 0;
let rimAccRad = 0;
const WHEEL_MP_PER_MILLITURN = 4800;
const RAD_PER_MILLITURN = (2 * Math.PI) / 1000;

function emitDial(milliturns) {
  if (!milliturns) return;
  dialTotal += milliturns;
  dialReadout.textContent =
    `dial ${dialTotal >= 0 ? "+" : ""}${dialTotal} milliturns (${(dialTotal / 1000).toFixed(3)} turns)`;
  if (has("toy_wasm_queue_dial")) wasm.toy_wasm_queue_dial(milliturns | 0);
}

deviceEl.addEventListener("wheel", (event) => {
  event.preventDefault();
  const scale = event.deltaMode === 1 ? 16 : event.deltaMode === 2 ? 400 : 1;
  wheelAccMilliPx += Math.round(event.deltaY * scale * 1000);
  const mt = Math.trunc(wheelAccMilliPx / WHEEL_MP_PER_MILLITURN);
  if (mt) {
    wheelAccMilliPx -= mt * WHEEL_MP_PER_MILLITURN;
    emitDial(mt);
  }
}, { passive: false });

/* --- device pointer: glass vs bezel rim ---
 * One pointer at a time on the device. Where it lands decides
 * what it is: inside the glass circle it is a POINTER_* contact
 * (raw integer panel pixels; C converts to milli-radius);
 * outside the circle - bezel ring or canvas corner - it is a
 * rim-drag turning the dial. The mode is fixed at pointerdown,
 * so a glass drag that strays outside stays a pointer drag. */
const EV_POINTER_DOWN = 1, EV_POINTER_MOVE = 2, EV_POINTER_UP = 3;
let devicePointer = null; /* {id, mode: "glass"|"rim", angle} */

function deviceGeometry(event) {
  const rect = canvas.getBoundingClientRect();
  const cx = rect.left + rect.width / 2;
  const cy = rect.top + rect.height / 2;
  const dx = event.clientX - cx;
  const dy = event.clientY - cy;
  return { rect, dx, dy, dist: Math.hypot(dx, dy), glassR: rect.width / 2 };
}

function queuePointer(type, event) {
  if (!has("toy_wasm_queue_pointer")) return;
  const { rect } = deviceGeometry(event);
  if (!rect.width || !rect.height) return;
  const px = Math.round((event.clientX - rect.left) * panelSize / rect.width);
  const py = Math.round((event.clientY - rect.top) * panelSize / rect.height);
  wasm.toy_wasm_queue_pointer(type, 0, px | 0, py | 0, panelSize);
}

deviceEl.addEventListener("pointerdown", (event) => {
  if (devicePointer !== null) return;
  const g = deviceGeometry(event);
  try { deviceEl.setPointerCapture(event.pointerId); } catch { /* inactive synthetic pointer */ }
  if (g.dist <= g.glassR) {
    devicePointer = { id: event.pointerId, mode: "glass", angle: 0 };
    queuePointer(EV_POINTER_DOWN, event);
  } else {
    devicePointer = { id: event.pointerId, mode: "rim", angle: Math.atan2(g.dy, g.dx) };
    deviceEl.classList.add("rim");
  }
});
deviceEl.addEventListener("pointermove", (event) => {
  if (!devicePointer || event.pointerId !== devicePointer.id) return;
  if (devicePointer.mode === "glass") {
    queuePointer(EV_POINTER_MOVE, event);
    return;
  }
  const g = deviceGeometry(event);
  const angle = Math.atan2(g.dy, g.dx);
  let d = angle - devicePointer.angle;
  if (d > Math.PI) d -= 2 * Math.PI;
  if (d < -Math.PI) d += 2 * Math.PI;
  devicePointer.angle = angle;
  rimAccRad += d;
  const mt = Math.trunc(rimAccRad / RAD_PER_MILLITURN);
  if (mt) {
    rimAccRad -= mt * RAD_PER_MILLITURN;
    emitDial(mt);
  }
});
const deviceRelease = (event) => {
  if (!devicePointer || event.pointerId !== devicePointer.id) return;
  if (devicePointer.mode === "glass") queuePointer(EV_POINTER_UP, event);
  devicePointer = null;
  deviceEl.classList.remove("rim");
};
deviceEl.addEventListener("pointerup", deviceRelease);
deviceEl.addEventListener("pointercancel", deviceRelease);

/* --- tilt producer (TOY_EV_TILT id 0) ---
 * The pad knob marks the direction things fall, in milli-g,
 * in the wire's display space: x right, y DOWN (core flips to
 * sim-up where needed; the pinned gravity proof's TILT
 * (300, 400) is "toward display lower-right"). Dragging up on
 * the pad therefore sends negative y. Each axis is round(pos
 * * 1000) clamped to +-1000; events are deduped by milli
 * value, so a held knob emits once. */
const tiltPad = $("tilt-pad");
const tiltKnob = $("tilt-knob");
const tiltReadout = $("tilt-readout");
let lastTilt = null;
let tiltPointer = null;

function sendTilt(mx, my) {
  tiltKnob.style.left = `${((mx / 1000) + 1) * 50}%`;
  tiltKnob.style.top = `${((my / 1000) + 1) * 50}%`;
  tiltReadout.textContent =
    `tilt x ${mx >= 0 ? "+" : ""}${mx} · y ${my >= 0 ? "+" : ""}${my} milli-g (y down)`;
  if (lastTilt && lastTilt[0] === mx && lastTilt[1] === my) return;
  lastTilt = [mx, my];
  if (has("toy_wasm_queue_tilt")) wasm.toy_wasm_queue_tilt(mx | 0, my | 0);
}
function tiltFromEvent(event) {
  const rect = tiltPad.getBoundingClientRect();
  if (!rect.width || !rect.height) return;
  const nx = Math.min(1, Math.max(-1, ((event.clientX - rect.left) / rect.width) * 2 - 1));
  const ny = Math.min(1, Math.max(-1, ((event.clientY - rect.top) / rect.height) * 2 - 1));
  sendTilt(Math.round(nx * 1000), Math.round(ny * 1000));
}
tiltPad.addEventListener("pointerdown", (event) => {
  if (tiltPointer !== null) return;
  tiltPointer = event.pointerId;
  try { tiltPad.setPointerCapture(event.pointerId); } catch { /* inactive synthetic pointer */ }
  tiltFromEvent(event);
});
tiltPad.addEventListener("pointermove", (event) => {
  if (event.pointerId === tiltPointer) tiltFromEvent(event);
});
const tiltRelease = (event) => {
  if (event.pointerId === tiltPointer) tiltPointer = null;
};
tiltPad.addEventListener("pointerup", tiltRelease);
tiltPad.addEventListener("pointercancel", tiltRelease);
$("tilt-level").addEventListener("click", () => sendTilt(0, 0));

/* --- session (TOYSESSION1) ---
 * The glue records every drained live event with the step it
 * applied at. Download saves exactly that text; the emulator
 * replays it bit-exactly. The readout counts recorded events
 * and says so loudly if the fixed cap truncated the log. */
const sessionReadout = $("session-readout");
function sessionText() {
  return has("toy_wasm_session_log") ? cstr(wasm.toy_wasm_session_log()) : "";
}
function refreshSessionReadout() {
  const log = sessionText();
  if (!log) { sessionReadout.textContent = "session log unavailable"; return; }
  const lines = log.split("\n").filter((l) => l &&
    !l.startsWith("TOYSESSION1") && !l.startsWith("END"));
  const truncated = lines.some((l) => l.startsWith("TRUNCATED"));
  const events = lines.filter((l) => !l.startsWith("TRUNCATED")).length;
  sessionReadout.textContent =
    `session: ${events} live event${events === 1 ? "" : "s"} recorded` +
    (truncated ? " · TRUNCATED at cap — download before continuing" : "");
}
$("download-session").addEventListener("click", () => {
  if (!wasm) return;
  const blob = new Blob([sessionText()], { type: "text/plain" });
  const a = document.createElement("a");
  a.href = URL.createObjectURL(blob);
  a.download = `toy-session-${cstr(wasm.toy_wasm_name(selected))}-step${stepsDone()}.txt`;
  a.click();
  URL.revokeObjectURL(a.href);
});
$("copy-hash").addEventListener("click", async () => {
  if (!wasm) return;
  const text = `${cstr(wasm.toy_wasm_name(selected))} step ${stepsDone()} state ${stateHash()}`;
  try {
    await navigator.clipboard.writeText(text);
    sessionReadout.textContent = `copied: ${text}`;
  } catch {
    sessionReadout.textContent = text;
  }
});

function selectCartridge(index, keepParams) {
  if (!wasm.toy_wasm_select(index)) throw new Error(`cartridge ${index} failed to load`);
  cartReady = true;
  selected = index;
  stepDebt = 0;
  resetPerf();
  /* The C session log reset with the select; reset every
   * producer accumulator to match, without queueing anything
   * (the fresh state already IS level/default). */
  dialTotal = 0; wheelAccMilliPx = 0; rimAccRad = 0;
  dialReadout.textContent = "dial 0 milliturns";
  lastTilt = null;
  tiltKnob.style.left = "50%"; tiltKnob.style.top = "50%";
  tiltReadout.textContent = "tilt x 0 · y 0 milli-g";
  $("blurb").textContent = cstr(wasm.toy_wasm_blurb(index));
  if (keepParams) queueTouchedParams();
  else buildParams();
  refreshCaps();
  refreshSessionReadout();
  render();
  refreshPerf();
}

function tick(now) {
  requestAnimationFrame(tick);
  if (!wasm) return;
  if (!lastTime) lastTime = now;
  const elapsed = Math.min(now - lastTime, 250);
  lastTime = now;
  framesThisSecond++;

  if (playing) {
    const nominalFps = 30;
    const multiplier = Number(speedInput.value) / 100;
    const stepsPerSecond = wasm.toy_wasm_demo_steps_per_frame(selected) * nominalFps * multiplier;
    const requested = elapsed * stepsPerSecond / 1000;
    perfRequested += requested;
    /* Debt past the 10000 cap is dropped, not banked; the lag
     * counter (requested - executed) shows exactly that loss
     * instead of silently resyncing. */
    stepDebt = Math.min(stepDebt + requested, 10000);
    const wholeSteps = Math.floor(stepDebt);
    if (wholeSteps > 0) {
      const t0 = performance.now();
      wasm.toy_wasm_step(wholeSteps);
      const dtMs = performance.now() - t0;
      const sample = dtMs / wholeSteps;
      msPerStep = msPerStep === null ? sample : msPerStep + (sample - msPerStep) * 0.1;
      perfExecuted += wholeSteps;
      stepDebt -= wholeSteps;
    }
    render();
  }

  const second = Math.floor(now / 1000);
  if (second !== lastRenderSecond) {
    if (lastRenderSecond >= 0 && playing) {
      statusEl.textContent =
        `LIVE WASM · ${cstr(wasm.toy_wasm_name(selected))} · step ${stepsDone()} · ${framesThisSecond} fps browser`;
    }
    framesThisSecond = 0;
    lastRenderSecond = second;
    refreshClockLabel();
    refreshPerf();
    refreshSessionReadout();
  }
}

async function init() {
  try {
    const response = await fetch("toy.wasm");
    if (!response.ok) throw new Error(`toy.wasm HTTP ${response.status}`);
    const bytes = await response.arrayBuffer();
    const { instance } = await WebAssembly.instantiate(bytes, {
      env: {
        toy_js_clock_valid: () => (clockState.valid ? 1 : 0),
        toy_js_clock_tz_min: () => clockState.tzMin | 0,
        toy_js_clock_utc_lo: () => clockUtcSeconds() >>> 0,
        toy_js_clock_utc_hi: () => Math.floor(clockUtcSeconds() / 4294967296) >>> 0,
        toy_js_mono_lo: () => Math.floor(performance.now()) >>> 0,
        toy_js_mono_hi: () => Math.floor(performance.now() / 4294967296) >>> 0,
      },
    });
    wasm = instance.exports;

    for (let i = 0; i < wasm.toy_wasm_count(); i++) {
      const option = document.createElement("option");
      option.value = String(i);
      option.textContent = `${cstr(wasm.toy_wasm_name(i))} — ${cstr(wasm.toy_wasm_title(i))}`;
      cartSelect.append(option);
    }
    const params = new URLSearchParams(location.search);
    let initial = 0;
    const wanted = params.get("cart");
    if (wanted) {
      for (let i = 0; i < wasm.toy_wasm_count(); i++) {
        if (cstr(wasm.toy_wasm_name(i)) === wanted) initial = i;
      }
    }
    cartSelect.value = String(initial);
    if (params.get("size")) setPanelSize(Number(params.get("size")));
    selectCartridge(initial, false);
    const preSteps = Math.min(Math.max(Number(params.get("steps")) || 0, 0), 1000000);
    if (preSteps > 0) {
      wasm.toy_wasm_step(preSteps);
      render();
    }
    if (params.get("pause") === "1") {
      playing = false;
      playButton.textContent = "Play";
      playButton.classList.remove("on");
    }
    statusEl.textContent = "LIVE WASM · simulation core ready";
    requestAnimationFrame(tick);
  } catch (error) {
    statusEl.textContent = `WebAssembly unavailable (${error.message}). No simulation is being shown.`;
    statusEl.classList.add("down");
    for (const el of [playButton, resetButton, cartSelect, panelSelect,
                      panelCustom, $("download-session"), $("copy-hash"),
                      $("tilt-level")]) el.disabled = true;
  }
}

cartSelect.addEventListener("change", () => selectCartridge(Number(cartSelect.value), false));
resetButton.addEventListener("click", () => selectCartridge(selected, true));
playButton.addEventListener("click", () => {
  playing = !playing;
  playButton.textContent = playing ? "Pause" : "Play";
  playButton.classList.toggle("on", playing);
  resetPerf();
  if (!playing && wasm) statusEl.textContent = `PAUSED · step ${stepsDone()} · state ${stateHash()}`;
});
speedInput.addEventListener("input", () => {
  speedValue.textContent = `${(Number(speedInput.value) / 100).toFixed(1)}×`;
  /* Rebase the lag accounting: a speed the user just chose
   * must not inherit "debt" from the old speed. */
  perfRequested = perfExecuted = 0;
  stepDebt = Math.min(stepDebt, 1);
});

setPanelSize(360);
init();
