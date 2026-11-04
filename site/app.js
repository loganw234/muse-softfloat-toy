"use strict";

const $ = (id) => document.getElementById(id);
const statusEl = $("status");
const cartSelect = $("cart");
const playButton = $("play");
const resetButton = $("reset");
const speedInput = $("speed");
const speedValue = $("speed-value");
const displaysInput = $("displays");
const displays = [
  { size: 240, canvas: $("display-240"), panel: $("panel-240"), hash: $("hash-240") },
  { size: 360, canvas: $("display-360"), panel: $("panel-360"), hash: $("hash-360") },
];
for (const d of displays) {
  d.ctx = d.canvas.getContext("2d", { alpha: true });
  d.image = d.ctx.createImageData(d.size, d.size);
}

let wasm = null;
let selected = 0;
let playing = true;
let lastTime = 0;
let stepDebt = 0;
let lastRenderSecond = -1;
let framesThisSecond = 0;

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

function selectCartridge(index) {
  if (!wasm.toy_wasm_select(index)) throw new Error(`cartridge ${index} failed to load`);
  selected = index;
  stepDebt = 0;
  $("blurb").textContent = cstr(wasm.toy_wasm_blurb(index));
  render();
}

function drawDisplay(d) {
  if (!wasm.toy_wasm_render(d.size)) throw new Error(`render failed at ${d.size}`);
  const ptr = wasm.toy_wasm_framebuffer();
  const pixels = new Uint16Array(wasm.memory.buffer, ptr, d.size * d.size);
  const rgba = d.image.data;
  const radius = d.size / 2;
  const r2 = radius * radius;
  for (let y = 0; y < d.size; y++) {
    const dy = y - radius + 0.5;
    for (let x = 0; x < d.size; x++) {
      const src = y * d.size + x;
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
  d.ctx.putImageData(d.image, 0, 0);
  d.hash.textContent = `step ${stepsDone()} · state ${stateHash()}`;
}

function visibleDisplays() {
  const mode = displaysInput.value;
  return displays.filter((d) => mode === "both" || String(d.size) === mode);
}

function render() {
  for (const d of visibleDisplays()) drawDisplay(d);
}

function applyDisplayMode() {
  const mode = displaysInput.value;
  for (const d of displays) {
    d.panel.hidden = !(mode === "both" || String(d.size) === mode);
  }
  if (wasm) render();
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
    stepDebt = Math.min(stepDebt + elapsed * stepsPerSecond / 1000, 10000);
    const wholeSteps = Math.floor(stepDebt);
    if (wholeSteps > 0) {
      wasm.toy_wasm_step(wholeSteps);
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
  }
}

async function init() {
  try {
    const response = await fetch("toy.wasm");
    if (!response.ok) throw new Error(`toy.wasm HTTP ${response.status}`);
    const bytes = await response.arrayBuffer();
    const { instance } = await WebAssembly.instantiate(bytes, {});
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
    selectCartridge(initial);
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
    playButton.disabled = true;
    resetButton.disabled = true;
    cartSelect.disabled = true;
  }
}

cartSelect.addEventListener("change", () => selectCartridge(Number(cartSelect.value)));
resetButton.addEventListener("click", () => selectCartridge(selected));
playButton.addEventListener("click", () => {
  playing = !playing;
  playButton.textContent = playing ? "Pause" : "Play";
  playButton.classList.toggle("on", playing);
  if (!playing && wasm) statusEl.textContent = `PAUSED · step ${stepsDone()} · state ${stateHash()}`;
});
speedInput.addEventListener("input", () => {
  speedValue.textContent = `${(Number(speedInput.value) / 100).toFixed(1)}×`;
});
displaysInput.addEventListener("change", applyDisplayMode);

applyDisplayMode();
init();
