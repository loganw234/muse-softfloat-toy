# Study: checkpoint format, versioning, and power-loss resume

Wave T1 study parcel `toy1-checkpoint` (see [docs/WAVE-T1.md](WAVE-T1.md):32).
Design only — no code changes. It closes the seam named in
[docs/CARTRIDGES.md](CARTRIDGES.md):80-82 ("No checkpoint serialization
beyond 'state bytes are the save file'") and implements nothing; §9 breaks
the work into T2 parcels.

Goal: the desk toy resumes bit-exact after power loss, and a checkpoint
taken on any runner (host emulator, RP2040/RP2350 board, WASM site)
resumes bit-exact on every other runner. That interchange is the headline
consequence of the deterministic design: state is a POD of bit patterns
([docs/CARTRIDGES.md](CARTRIDGES.md):19-22), `step()` is a pure function
of state ([docs/CARTRIDGES.md](CARTRIDGES.md):27-35), and the sim core is
already specified to "expose its state bytes for hashing/checkpointing"
([docs/TOY-0.md](TOY-0.md):60-61).

## 1. What exists today (evidence)

- State contract: one POD struct of bit-pattern integers, no pointers or
  heap, ≤ `TOY_MAX_STATE_SIZE` (64 KiB), ≤ 8-byte alignment, hosts zero
  the allocation before `reset()` and `reset()` memsets the whole struct
  so padding bytes are defined before hashing
  ([docs/CARTRIDGES.md](CARTRIDGES.md):19-26).
- The cap itself: `#define TOY_MAX_STATE_SIZE 65536u`
  ([core/toy_cart.h](../core/toy_cart.h):19), with the comment that a
  cartridge exceeding it "does not load anywhere"
  ([core/toy_cart.h](../core/toy_cart.h):17-18). The header comment
  already promises state is "hashable and checkpointable verbatim"
  ([core/toy_cart.h](../core/toy_cart.h):3-6).
- Descriptor: `name`, `title`, `blurb`, `state_size`, pacing/proof
  fields, `reset`/`step`/`render` pointers — **no version field**
  ([core/toy_cart.h](../core/toy_cart.h):21-35). Registry lookup is by
  name string ([core/toy_carts.c](../core/toy_carts.c):11-16).
- Proof hash: FNV-1a 64 over raw state bytes, offset basis
  1469598103934665603, prime 1099511628211
  ([core/toy_carts.c](../core/toy_carts.c):18-27); declared as "identical
  on host, RP2040 and RP2350 for identical step counts"
  ([core/toy_cart.h](../core/toy_cart.h):45-47).
- No runner persists anything. Host: `toy_emu` has only
  `list`/`proof`/`frames`/`run`, each `xcalloc` + `reset()` per
  invocation ([emu/main_emu.c](../emu/main_emu.c):15-20, 47-122).
  Board: one static `_Alignas(8)` buffer of `TOY_MAX_STATE_SIZE`,
  memset + reset per cartridge, hashes streamed over CDC, then an idle
  loop ([board/main_board.c](../board/main_board.c):19-37, 39-68).
  WASM: one static state buffer, `toy_wasm_select` always memsets +
  resets and zeroes its step counter; the only state exports are hash
  halves and `state_size` — there is no way to read or write state bytes
  from JS ([site/wasm_glue.c](../site/wasm_glue.c):14, 18, 64-73,
  97-111).
- Step counts live in the runner, not the descriptor: WASM keeps
  `toy_steps_done` ([site/wasm_glue.c](../site/wasm_glue.c):18, 75-81);
  host/board proof loops count locally
  ([emu/main_emu.c](../emu/main_emu.c):60-67,
  [board/main_board.c](../board/main_board.c):27-36). Additionally, both
  current cartridges embed their own `uint64_t steps` inside state and
  increment it in `step()` ([core/cart_fig8.c](../core/cart_fig8.c):24,
  90; [core/cart_pendulum.c](../core/cart_pendulum.c):20, 97). §7 defines
  which counter the header carries and why the duplication is harmless.
- Board flash: untouched by toy code. The board target links only
  `pico_stdlib` + `pico_unique_id` ([board/CMakeLists.txt](../board/CMakeLists.txt),
  `target_link_libraries` line) and enumerates cartridge sources
  explicitly. Target hardware is 16 MB flash on both ordered kits
  ([docs/TOY-0.md](TOY-0.md):23-39); the six existing dev boards are
  Pico / RP2040-Zero class, 2 MB-class flash
  ([docs/TOY-0.md](TOY-0.md):42-45). The RP2350 kit adds RTC + microSD
  ([docs/TOY-0.md](TOY-0.md):30-34); the RP2040 kit has no battery RTC.

### Measured state sizes (base 8f63c78)

Both registered cartridges ([core/toy_carts.c](../core/toy_carts.c):5-8)
are built exclusively from `f64` (a `uint64_t` bit pattern,
[core/toy_math.h](../core/toy_math.h):15) and `uint64_t`, so every field
is 8 bytes, 8-aligned, and there is no padding to hide:

| cartridge | composition | `sizeof` |
|---|---|---|
| fig8 | (x,v: 12 + w1,w0,dt: 3 + steps: 1)×8 = 128; trail 3×288×2×8 = 13,824; tperiods 8 — [core/cart_fig8.c](../core/cart_fig8.c):19-27 | **13,960 B** |
| pendulum | (th,w: 8 + dt,g: 2 + steps: 1)×8 = 88; tip 2×220×2×8 = 7,040 — [core/cart_pendulum.c](../core/cart_pendulum.c):14-22 | **7,128 B** |

History dominates: trails are 99% of fig8 and 98.8% of pendulum state.
That is deliberate (trails are simulation history, hence state —
[docs/CARTRIDGES.md](CARTRIDGES.md):55-57), and it means checkpoint size
is roughly "trail size", bounded only by the 64 KiB cap. Wave T1 carts
(cloud, carousel, …) land on sibling branches and are not visible from
this base; the format must therefore be sized against the cap, not
against today's two cartridges.

## 2. The checkpoint image (one byte format everywhere)

```
offset  size  field
------  ----  ---------------------------------------------------------
  0       8   magic: ASCII "TOYCKPT1"  (54 4F 59 43 4B 50 54 31)
  8       2   format_version  u16 LE = 1      (envelope version)
 10       2   header_size     u16 LE = 64     (payload starts here)
 12       4   flags           u32 LE = 0      (reserved; must be 0 in v1)
 16      16   cart_name: ASCII, NUL-padded, NUL within the field
 32       4   state_size      u32 LE  (1 .. TOY_MAX_STATE_SIZE)
 36       4   cart_state_version u32 LE (per-cartridge, §4)
 40       8   step_count      u64 LE  (runner steps since reset)
 48       8   state_hash      u64 LE  (toy_hash_state over payload)
 56       4   payload_crc32   u32 LE  (CRC-32/ISO-HDLC over payload)
 60       4   header_crc32    u32 LE  (CRC-32 over header bytes [0,60) )
 64   state_size   payload: state bytes, verbatim
```

Total size = `64 + state_size`, so **max 65,600 B** (65,536 + 64);
today: fig8 **14,024 B**, pendulum **7,192 B**. No compression in v1:
the budget is already small, RAM on RP2040 is 264 KB
([docs/TOY-0.md](TOY-0.md):38), and a codec is a new deterministic
surface for no requirement. File name convention: `<cart_name>.ckpt`.

Field decisions, argued:

**Magic + version + header_size.** Magic rejects foreign/truncated
files before any field is trusted. `header_size` is stored (not
implied by version) so a future v2 can widen the header while v1
readers fail cleanly on `format_version`, and so the payload offset is
never folklore. Readers must check `header_size == 64` for v1 and
`total_len == header_size + state_size` exactly — trailing bytes are
an error, not ignored (a torn or concatenated write must not parse).

**Cartridge identity: name string, not name hash.** Lookup is already
by name ([core/toy_carts.c](../core/toy_carts.c):11-16) and every
human-facing surface uses it (`toy_emu proof <cart>`,
`TOY-RUN cart=%s`, the site selector). A 64-bit FNV of the name would
save nothing (16 bytes are already there), would be opaque in a hex
dump of a stranded checkpoint, and adds a collision question the
registry doesn't have (names are unique by construction — two carts
with one name cannot be selected distinctly by `toy_cart_by_name`).
The name is not trusted alone: it is cross-checked against the
descriptor's `state_size` and `cart_state_version` before any byte is
loaded (§4). Cost of the choice: a **15-character ASCII cap** on
cartridge names (16th byte is the mandatory NUL). Current names:
`fig8`, `pendulum` ([core/cart_fig8.c](../core/cart_fig8.c):136,
[core/cart_pendulum.c](../core/cart_pendulum.c):145); Wave T1's longest
are `harmonograph`/`microscope`/`intervallens` (12–13). The cap must
become a registry rule in the ckpt-core parcel (§9), enforced by a
host test over `toy_carts[]`, not by convention. Owner question Q5.

**Step count** is the runner's count of `step()` calls since `reset()`
(the quantity proof streams print). It is metadata for display and
proof-stream continuation; it is not needed to *compute* the future
(that is fully determined by payload bytes), but a resumed proof
stream must number lines identically to an uninterrupted one (§7).
`u64`: at a fanciful 1 M steps/s it wraps in ~585,000 years; there is
no reason to be cleverer.

**`state_hash`: reuse `toy_hash_state` verbatim** — FNV-1a 64, offset
basis 1469598103934665603, prime 1099511628211, over exactly the
`state_size` payload bytes ([core/toy_carts.c](../core/toy_carts.c):18-27).
Its job in the header is **semantic identity**: it must equal the hash
the proof stream printed at `step_count`. A loader recomputes it;
mismatch means the payload is not the state the header claims, full
stop. It also makes a checkpoint self-describing to existing tooling:
the same 16 hex digits appear in `TOY-DONE` lines and in
[site/expected-hashes.json](../site/expected-hashes.json).

**Integrity: a separate CRC-32, because FNV is the wrong tool for
storage damage.** FNV-1a was chosen as a proof hook, not an
error-detecting code:

- It does not cover the header. A torn write that corrupts
  `state_size` or `step_count` but leaves payload intact passes an
  FNV-only check and then mis-slices the payload. `header_crc32`
  covers bytes [0,60) — every field that steers parsing.
- CRC-32 (reflected poly 0xEDB88320, init/xorout 0xFFFFFFFF) has
  *guaranteed* burst detection (any error burst ≤ 32 bits) — the
  actual failure shapes here are bursts: an erased 0xFF sector tail,
  a half-programmed 256 B page, a truncated SD write. FNV's
  multiplication mixes well statistically but promises nothing about
  specific burst patterns, and its per-byte multiply is *slower* than
  a table CRC on Cortex-M0+ anyway.
- Keeping both separates two different rejections a loader must make
  for different reasons: CRC fail = "storage damaged, try the other
  slot"; CRC pass + FNV fail = "intact bytes, wrong meaning — writer
  bug, mislabelled cartridge, or layout drift" (§3). Collapsing them
  into one field destroys that diagnosis for 4 saved bytes.

Neither is authentication. There is no key, no MAC, and a checkpoint
from an untrusted source is trusted exactly as far as "these bytes
make this deterministic machine draw this" — acceptable because
loading a state can only ever run the named cartridge's own `step()`
on bounded POD data (no pointers by contract). If checkpoints are
ever shared publicly as a feature, that threat model gets its own
paragraph then, not a retrofitted HMAC now.

**Endianness: little-endian, pinned in the spec, serialized
byte-wise.** All current targets are little-endian (x86-64 host,
Cortex-M0+/M33, wasm32), so a struct dump would *happen* to work
everywhere today. The format nevertheless defines every integer as
LE and requires writers/readers to go through explicit
`put_u32le`/`get_u32le`-style helpers. Reasons: (a) the header is the
interchange artifact — "happens to work" is how formats become
undebuggable when a big-endian or unusual-ABI host appears; (b) it
forbids the C-struct-dump implementation, whose padding is
compiler-defined and would change bytes without changing any field;
(c) byte-wise code is ~20 lines and testable against a golden byte
fixture (§8). Illustrative writer core:

```c
static void put_u64le(unsigned char *p, uint64_t v)
{ for (int i = 0; i < 8; i++) p[i] = (unsigned char)(v >> (8 * i)); }
/* header: memset(h,0,64); memcpy(h,"TOYCKPT1",8); put_u16le(h+8,1);
 * ... put_u32le(h+60, crc32(h,60)); payload appended verbatim. */
```

An in-memory `toy_ckpt_header_t` struct may exist for convenience in
T2, but the *bytes on disk* are defined by the table above, and the
struct must never be `fwrite`n directly.

**Payload alignment:** payload starts at file offset 64 (8-aligned in
a file) but loaders must not assume the payload pointer is usable as
a state pointer in place. Load = copy into the runner's existing
8-aligned state allocation (board's `_Alignas(8)` buffer,
[board/main_board.c](../board/main_board.c):19; WASM's,
[site/wasm_glue.c](../site/wasm_glue.c):14; host `xcalloc`,
[emu/main_emu.c](../emu/main_emu.c):15-20) — after validation, from a
scratch buffer, §7.

## 3. The payload-layout caveat (the load-bearing assumption)

"Verbatim payload" is only platform-independent if the C struct's
*byte layout* is. Today it is, accidentally-on-purpose: every state
field is `f64`/`uint64_t` (§1), alignment 8 everywhere we run
(x86-64 SysV/Win64, AAPCS for Cortex-M, wasm32), so offsets coincide.
A future cartridge mixing `uint32_t`/`uint16_t`/`uint8_t` fields
could pad differently across ABIs (the classic case: 32-bit x86
aligns `uint64_t` to 4), and a checkpoint would then silently mean
different states on different runners — FNV would still match the
bytes, so the failure would be invisible until trajectories diverged.

The format's answer is discipline, enforced by test, not hope
(T2 ckpt-core parcel):

1. Cartridge state structs order fields by descending alignment
   (8-byte, then 4, 2, 1) and group same-width fields, so the layout
   a compiler *would* choose is the layout on every ABI we target.
2. `reset()` already memsets the whole struct
   ([docs/CARTRIDGES.md](CARTRIDGES.md):24-26;
   [core/cart_fig8.c](../core/cart_fig8.c):62,
   [core/cart_pendulum.c](../core/cart_pendulum.c):77), so any padding
   that does exist is 0x00 and participates deterministically in both
   hashes.
3. Each cartridge gains a golden layout fixture in its own file —
   `_Static_assert(sizeof(state) == N)` plus `offsetof` asserts for
   the first field after any width change — so a compiler/ABI that
   pads differently **fails to build** instead of producing a
   divergent checkpoint. If a legitimate change alters `sizeof` or
   any offset, that is definitionally a `cart_state_version` bump
   (§4).
4. Hosts that cannot satisfy the asserts do not get checkpoint
   interchange; they may still run cartridges. This is the same
   "does not load anywhere" posture the size cap already takes
   ([core/toy_cart.h](../core/toy_cart.h):17-18).

Explicit per-field serialization (marshalling each field by name)
was considered and rejected for v1: it would require every cartridge
to grow serialize/deserialize code, breaking the parcel-ready
contract ([docs/CARTRIDGES.md](CARTRIDGES.md):5-15) to defend
against an ABI we do not ship on. If a genuinely foreign host ever
becomes a target, marshalling is what `format_version = 2` is for.

## 4. Versioning policy

Two versions, deliberately independent:

- `format_version` (envelope): the 64-byte header's shape. Changes
  only when the header changes. Readers accept exactly 1 in v1;
  anything else is rejected *before* other fields are read (beyond
  magic + this field).
- `cart_state_version` (payload): the byte-layout meaning of one
  cartridge's struct. Owned by the cartridge, stored in its
  descriptor. Bumped by the cartridge author on **any** change to
  the state struct: field added/removed/reordered/resized, trail
  length changed, a constant folded into state changed meaning, or
  `sizeof` changed for any reason. Cosmetic code changes to
  `render()` do not bump it; any change to what `step()` computes
  from the same bytes **does** (same bytes must always evolve
  identically — that is the entire product).

This requires one descriptor field. The implied diff to
[core/toy_cart.h](../core/toy_cart.h):21-35:

```diff
 typedef struct {
     const char *name;   /* short id, e.g. "fig8" */
     const char *title;  /* display title */
     const char *blurb;  /* one line: what to notice */
     size_t state_size;
+    /* Checkpoint payload-layout version (docs/study-checkpoint.md).
+     * Bump on ANY state-struct or step-semantics change; loaders
+     * reject mismatches rather than guess. Starts at 1. */
+    uint32_t state_version;
     uint32_t demo_steps_per_frame;
     ...
```

and one added value in each cartridge's positional initializer —
e.g. [core/cart_fig8.c](../core/cart_fig8.c):135-139 becomes
`sizeof(fig8_state), 1, 4, 2000, 500, …`. Because initializers are
positional today, the ckpt-core parcel touches every `core/cart_*.c`
exactly once; that is a seam parcel's job, after which new
cartridges follow the updated contract at no extra cost. (Switching
to designated initializers at the same moment is tempting and is
explicitly *not* recommended inside this parcel — it rewrites every
descriptor's shape for style during a semantics change.)

**Mismatch posture: reject, never migrate, never destroy.** A loader
rejects (in check order): bad magic; unknown `format_version`;
`header_size != 64`; `header_crc32` mismatch; unknown `cart_name`
(not in `toy_carts[]`); `flags != 0`; `state_size` zero, >
`TOY_MAX_STATE_SIZE`, or ≠ descriptor's; `cart_state_version` ≠
descriptor's; `total_len ≠ 64 + state_size`; `payload_crc32`
mismatch; recomputed FNV ≠ `state_hash`. On any rejection the runner
(a) keeps the rejected bytes available for export/inspection (host:
leaves the file alone; board: does not erase the slot until a *new*
valid checkpoint replaces it; browser: keeps the IndexedDB record),
and (b) starts the cartridge from `reset()` and says so on its
status surface. Silent reset-over-a-stranded-long-run is the one
unacceptable outcome; loud reset is merely sad.

No migration in v1. Wave T1 just landed ten cartridges whose states
are, per the contract doc itself, "still changing"
([docs/CARTRIDGES.md](CARTRIDGES.md):80-82) — writing N→N+1 state
converters for structs that may change again next wave buys nothing
and risks the worst failure mode (a converter that produces a
plausible-but-wrong state whose hash stream then diverges from every
honest history). The door is left open by shape, not by code: a
future stable cartridge may add an optional
`migrate(void *st, uint32_t from_version, size_t from_size)` hook,
at which point "reject unknown version" becomes "reject versions
with no migration path". Until a cartridge is declared stable by the
owner, firmware updates may orphan checkpoints, and the release
notes must say so per cartridge. Owner question Q3.

## 5. Storage backends

One byte image (§2), four homes. Only internal flash needs bytes
*around* the image; everywhere else the image is stored as-is, which
is what makes interchange trivially testable with `cmp`.

### 5.1 Internal QSPI flash (RP2040 / RP2350)

Assumed NOR geometry, standard for the W25Q-class parts on Pico and
Waveshare boards and the constants the Pico SDK exposes
(`FLASH_SECTOR_SIZE` 4096, `FLASH_PAGE_SIZE` 256): erase unit
**4 KiB sector** (erased state 0xFF), program unit **256 B page**,
programming only clears bits (1→0). Endurance assumed **100k
erase cycles/sector** (typical for the class). These three numbers
must be verified against both fitted kits before the board parcel
merges — they are the only external constants in this study, and
every sizing figure below is arithmetic on them. RP2350 additionally
has partition/boot concepts the RP2040 lacks; the slot layer must
use the SDK's safe-flash path (`flash_safe_execute`-style) on both,
with interrupts/XIP handled by the SDK, never by hand-rolled QSPI.

**Slot sizing.** A slot must hold the largest checkpoint this
firmware can produce, rounded up to whole sectors:

```
slot_bytes = ceil((64 + max_state_size) / 4096) * 4096
max today (fig8):     ceil(14024/4096)= 4 sectors = 16,384 B
pendulum alone:       ceil( 7192/4096)= 2 sectors =  8,192 B
cap (65536 state):    ceil(65600/4096)=17 sectors = 69,632 B
```

Size for the **cap**, not for today's largest cart: repartitioning
flash in the field to grow slots is exactly the migration pain §4
refuses elsewhere. Reserve from the **top of flash** (below the
SDK's own reserved regions as the board parcel determines):

```
A/B pair at cap size: 2 × 69,632 = 139,264 B
  = 6.64% of a 2 MiB part     (Pico / Zero dev boards)
  = 0.83% of a 16 MiB part    (both ordered kits, TOY-0 §2)
```

One A/B pair holds the checkpoint of the **currently running
cartridge only** (whichever it is — the header says which). Per-cart
pairs for a 12-cartridge gallery at cap size would cost
12 × 139,264 ≈ 1.6 MB, which does not fit a 2 MiB part beside
firmware, and on 16 MiB it spends flash to solve a problem microSD
solves better (§5.2). Owner question Q2 confirms this scope.

**A/B atomicity protocol.** The image alone cannot express
"generation" or "fully written" (a file gets those from its
filesystem), so on flash each slot prepends a 16-byte wrapper in
its first page; the image follows at slot offset 256 (one program
page reserved for the wrapper, keeping the image itself untouched
and page-aligned):

```
slot+0    8   slot_seq   u64 LE, monotonic across both slots
slot+8    4   commit     0xFFFFFFFF (erased/uncommitted)
                         -> 0x434F4D4D ("COMM") programmed LAST
slot+12   4   reserved = 0xFFFFFFFF
slot+256  ..  checkpoint image (§2), verbatim
```

Write to the *inactive* slot only: (1) erase inactive slot's
sectors; (2) program `slot_seq = last_seq + 1` with commit left
erased; (3) program the image pages; (4) read back and verify both
CRCs + FNV from flash (XIP read — cheap, and catches a dying
sector at write time rather than at 3 a.m. reboot); (5) program
`commit` — a single page program that only clears bits, the atomic
act of publication. Boot: validate both slots fully
(commit word, then §4's check list on the image), resume from the
valid slot with the higher `slot_seq`; if exactly one validates,
use it; if none, fresh `reset()` and report. Power loss at any
instant leaves either the old slot (loss during 1–4) or the new one
(after 5) — never a half-image as the best candidate, because
half-images fail CRC/FNV and the uncommitted wrapper fails before
that. `slot_seq` at u64 never wraps in practice (at one checkpoint
per minute, 3.5×10^13 years); do not synthesize ordering from
`step_count`, which *decreases* legitimately on cartridge switch.

The erase+program stall is real: programming a cap-size image is
`ceil(65600/256) = 257` page programs plus 17 sector erases
(fig8 today: 55 pages, 4 erases), order tens-to-hundreds of ms
with XIP stalled. Checkpoints are therefore taken between frames,
never inside `step()`/`render()`, and the display layer (when it
exists) should expect a one-frame hitch. The current board runner
is headless proof code ([board/main_board.c](../board/main_board.c):1-6),
so this costs nothing until the glass lands — but the T2 board
parcel must place the call accordingly from day one.

**Write-frequency policy (wear arithmetic).** Alternating A/B, each
slot is erased once per two checkpoints. Sector life at 100k
cycles, continuous running:

```
checkpoint every   per-slot erase every   time to 100k cycles
        1 s                 2 s                 2.3 days   ✗
       60 s               120 s               138.9 days   ✗
      300 s (5 min)       600 s                 1.9 years  ~
      900 s (15 min)     1800 s                 5.7 years  ✓
```

Per-step or per-frame checkpointing destroys the device; even
"every minute" is a 5-month fuse. Recommended triggers, in
priority order: (a) **events** — cartridge switch (checkpoint the
outgoing cart), sleep/power-down gesture, settings change, low-
battery warning (§6 — several of these hooks belong to the input/
timebank studies); (b) **periodic backstop** every 15 minutes of
*stepping*, skipped if `state_hash` equals the last written hash
(an idle or converged cart costs zero erases); (c) never more
often than once per 60 s regardless of event spam (coalesce). The
power-loss window is thus ≤15 min of steps in the worst case,
zero across planned sleeps/switches. Whether that window is
acceptable is owner question Q1 — it is a product decision
(long-run sanctity vs hardware life), not an engineering one.
If the owner wants a tighter window on the 16 MiB kits, widen to
a 4- or 8-slot ring (4×69,632 = 278,528 B = 1.66% of 16 MiB)
for a linear wear reduction per sector; the protocol above is
unchanged, only slot count and selection generalize. V1 ships
A/B; the ring is a named variant, not a redesign.

### 5.2 microSD (RP2350 kit)

The RP2350 kit's microSD slot exists precisely so "banked
trajectories and event snippets stop being storage-constrained"
([docs/TOY-0.md](TOY-0.md):33-34). Backing store: FAT (the
interchange lingua franca — the card must be readable by plugging
it into a PC, which is also the sneakernet interchange path).
Files are §2 images byte-for-byte. FAT rename-over-existing is
not atomic, so use **generation-named files**:
`<cart>-NNNNNN.ckpt` (NNNNNN = zero-padded write counter kept in
internal flash or derived by scanning the directory). Write the
new file, flush/close, re-open and verify CRCs, then delete all
but the two newest *valid* generations for that cart. Reader scans,
validates with §4's list, picks the highest valid generation.
Per-cartridge resume and a checkpoint *gallery/archive* both
become free here. There is no SD or FAT code in the repo today
(board links are core-only, §1) and driver selection (SPI vs SDIO,
FatFs licensing posture) is out of this study's scope: SD is a
deferred T2 parcel and internal-flash resume must not gate on it.

### 5.3 Host file (emulator)

`<cart>.ckpt`, §2 bytes, written via `<cart>.ckpt.tmp` + flush +
rename over the target (atomic replace on POSIX; on Windows,
`ReplaceFile`/rename-after-delete — the emu parcel picks the
platform detail, the *requirement* is: a killed `toy_emu` never
leaves a torn `.ckpt` under the real name). New subcommands are
T2's to name (`save`/`load` beside the existing four,
[emu/main_emu.c](../emu/main_emu.c):112-121); the important
property is that host is the reference implementation every other
backend is `cmp`-ed against.

### 5.4 Browser (WASM site)

Today the page holds state only in WASM linear memory and loses
it on reload; the glue cannot even expose the bytes (§1). Storage
choice, with the arithmetic against a 65,600 B max image
(base64: `ceil(65600/3)×4 = 87,468` chars; fig8 18,700; pendulum
9,592):

| option | fits? | verdict |
|---|---|---|
| localStorage | ~5 MiB-class per-origin quota (browser-dependent, string-only, key+value counted in UTF-16 code units) ⇒ max image as base64 ≈ 87,468 units ≈ 175 KB of quota, ~3% of 5 MiB | **No** as primary: synchronous (janks the rAF loop in [site/app.js](../site/app.js):99-129), string-only (forces base64 + its 33% tax), quota shared and evictable, no transactions. It *fits*, which is precisely the trap. Acceptable only for a tiny pointer ("last cart was fig8"), never the payload. |
| **IndexedDB** | Stores `ArrayBuffer` natively; quotas are a large fraction of disk (browser-dependent, typically hundreds of MB to GB per origin) ⇒ one 65.6 KB record is ~0.01–0.1% of even a conservative quota; a full 12-cart cap-size gallery is 787 KB | **Yes**: async, transactional (the write either commits or doesn't — the browser's own A/B), binary-clean, survives reload. Store one record per cartridge under key = name, value = the §2 `ArrayBuffer` plus a JS-side `{savedAt}` display field kept *outside* the image bytes. |
| File download/upload | n/a | **Yes, as the interchange path**: `Blob` download of the exact bytes (`<cart>.ckpt`), `<input type=file>` / drag-drop upload to load. This is how a desk-toy checkpoint (via SD or USB mass-storage later) resumes in the browser. |
| base64 string / URL | 87,468 chars max | Copy-paste transport only ("paste a checkpoint" textarea). Not in a URL for cap-size carts (URL length limits, logs, referrers). Whether the paste feature is wanted at all is owner question Q4. |

WASM glue additions needed (T2, [site/wasm_glue.c](../site/wasm_glue.c)):
a state-pointer export for JS to read `state_size` bytes (the
framebuffer is already exposed this way,
[site/wasm_glue.c](../site/wasm_glue.c):93), a load entry that
validates via the *shared C* implementation (§9, parcel 1 — JS
must not re-implement CRC/FNV checking divergently) then copies
into `toy_state` and sets `toy_steps_done`
([site/wasm_glue.c](../site/wasm_glue.c):18). Note the glue's
existing guard `c->state_size > sizeof toy_state`
([site/wasm_glue.c](../site/wasm_glue.c):67) is exactly the
loader's cap check — reuse the pattern.

**Interchange story, stated plainly:** the bytes in a host `.ckpt`
file, an SD `.ckpt` file, an IndexedDB record, and slots 256… of a
flash slot are *identical* for the same (cartridge, step). Export
from flash = strip the 256-byte wrapper page. Any runner that
cannot produce byte-identical output for a pinned fixture fails
its gate (§8) — this is the same host == board == WASM discipline
the proof hashes already enforce, extended from hashes to files.

## 6. Interplay with the sibling studies

**Settings (toy1-input): beside the checkpoint, not inside it.**
The §2 image contains exactly what determines future states and
nothing else; user settings (warp, brightness, parameters the
input study defines) must be a separate record with its own magic,
version, and CRC, stored alongside (its own flash sector pair, its
own IndexedDB record, a sibling file). Load order on boot:
settings first, then checkpoint. The boundary rule is sharp: the
instant a "setting" influences what `step()` computes (a user-
tunable physics parameter, a Julia constant), it stops being a
setting and becomes **state** — it must live in the state struct
(hence in the payload, the hashes, and `cart_state_version`), or
the same checkpoint would evolve differently under different
settings and the proof stream would fork silently. Purely
presentational settings (display size on the site, speed slider
— [site/app.js](../site/app.js):91-97, 184-186) never enter the
image. `flags` in the header stays 0 in v1 rather than growing a
settings trailer: co-versioning two independently-changing
records inside one blob is how both become unversionable.

**Input events (toy1-input): the one threat to "state is
everything".** The resume guarantee (§7) holds because `step()`
today reads nothing but state ([docs/CARTRIDGES.md](CARTRIDGES.md):34-35).
If the input seam lets touch/IMU events affect stepping from
*outside* state — a queue in the runner, a "pending impulse"
variable in glue — then a checkpoint no longer determines the
future, and two runners resuming it with different subsequent
touches diverge (correctly, but the "resume bit-exact" claim needs
the asterisk). Requirement handed to the input study: any event
that has been *consumed* by stepping must have its effect fully
inside state bytes at every step boundary (event counters and
queues, if any, live in the struct); events arriving after a
checkpoint are new history and need their own log if replay is
ever wanted — that log is an annex the input study owns, not a
field this format pre-allocates.

**Timebank (toy1-timebank): orthogonal by design.** Banked
trajectory samples are presentation history (decimated, possibly
interpolated on playback — and interpolation is "drawing, not a
state", [docs/CARTRIDGES.md](CARTRIDGES.md):58-59); a checkpoint
is the one exact present. The time layer's banking ring
([docs/TOY-0.md](TOY-0.md):64-72) is never checkpointed and never
restored: on resume, the bank restarts empty from the resumed
step. Wall-clock is likewise absent: there is no timestamp field
in §2 because (a) the RP2040 kit has no battery RTC, so a
timestamp would be incomparable across power cycles; (b) the
RP2350's RTC ([docs/TOY-0.md](TOY-0.md):30-31) belongs to the time
layer (active hours, morning report), and stamping it into the
interchange image would make the same (cart, step) produce
different bytes on different days — destroying the `cmp`-equality
§5.4 promises. If provenance ("when was this saved") is wanted,
it goes in backend metadata (file mtime, IndexedDB side field,
flash wrapper spare bytes) — never in the image. Pacing state is
presentation too: the site's fractional `stepDebt`
([site/app.js](../site/app.js):24, 111-116) is not restored; it
restarts at 0, which shifts wall-clock phase by <1 frame and sim
state by exactly nothing.

## 7. Resume semantics

**Restored:** the `state_size` payload bytes into the runner's
state allocation; the runner's step counter ← header
`step_count`; the selected cartridge ← header `cart_name`
(resume implies selection — a board booting to a fig8 checkpoint
runs fig8). **Not restored:** framebuffer (regenerated by the
first `render()`; it is `toy_fb_t` presentation memory,
[core/toy_fb.h](../core/toy_fb.h):14-17, and at 360×360×2 =
259,200 B would dwarf the checkpoint anyway); wall-clock/RTC
(§6); pacing fractions, display size, speed, banked samples (§6).
Trails *are* restored — they live in state
([core/cart_fig8.c](../core/cart_fig8.c):25,
[core/cart_pendulum.c](../core/cart_pendulum.c):21) — so the first
frame after resume already shows history, not a cold start.

**Load procedure (all runners, same order):**

1. Read the full image into a **scratch** buffer (never directly
   over live state). On board/WASM this reuses the existing
   64 KiB-class allocation only if the runner is not yet running
   a cartridge it would clobber; boot-time load is the common
   case and is clobber-free by definition.
2. Run §4's check list. Any failure → reject path (§4): state
   untouched, blob preserved, loud status.
3. `toy_memset(state, 0, state_size)` then copy payload — the
   memset mirrors the existing zero-before-use discipline
   ([board/main_board.c](../board/main_board.c):23,
   [site/wasm_glue.c](../site/wasm_glue.c):69) and guarantees
   bytes beyond a (hypothetically shorter) payload can never be
   stale. **Do not call `reset()`** — it memsets and
   reinitializes, which is precisely the state being replaced
   ([core/cart_fig8.c](../core/cart_fig8.c):59-82).
4. Set the runner step counter; recompute `toy_hash_state` once
   and assert equality with the header (already done in step 2
   over the scratch copy — the assert after copy guards the copy
   itself); render one frame; resume stepping.

**Why the proof hash stream continues identically, by
construction.** Let `S_n` be state bytes after n steps. `step()`
is a pure, platform-independent function `S_{n+1} = f(S_n)`: no
wall clock, no `rand()`, no heap, no platform branches, all FP via
the integer-only library ([docs/CARTRIDGES.md](CARTRIDGES.md):27-35;
[core/toy_math.h](../core/toy_math.h):3-8). The proof stream is
`(n, H(S_n))` with H = `toy_hash_state`
([emu/main_emu.c](../emu/main_emu.c):60-67 — same lines on board,
[board/main_board.c](../board/main_board.c):27-36). A checkpoint
at step N stores exactly `S_N` (payload) with `H(S_N)` in the
header as a checkable anchor. Resume installs `S_N` byte-for-byte
and counter N; the next step computes `f(S_N) = S_{N+1}` — the
identical bytes an uninterrupted run computes, hence identical
`H`, at the identical step number. Induction does the rest: the
resumed stream is *suffix-equal* to the uninterrupted stream, not
approximately. The two premises are the whole argument and both
are already project invariants: (i) all of the future is in the
state bytes (threat: §6's input caveat — hence its requirement);
(ii) byte layout means the same struct on every runner (§3 —
hence its fixtures). Where those hold, "checkpoint on the toy,
resume in the browser, hashes continue" is not a feature to
build so much as a theorem to not break.

**The duplicate step counters, resolved.** Both current states
embed `steps` and increment it themselves
([core/cart_fig8.c](../core/cart_fig8.c):24, 90;
[core/cart_pendulum.c](../core/cart_pendulum.c):20, 97) while
runners count externally. The header's `step_count` is defined
as the **runner's** count, because it exists for every cartridge
(including future ones without an internal counter) and is what
proof streams print. The embedded counter rides inside the
payload and stays consistent automatically *if* writer and
cartridge both counted honestly — but the loader does not (and
cannot, state being opaque) verify the inner value. If they ever
disagree, the payload's future evolution follows the *inner*
counter wherever the cartridge reads it (e.g. fig8's trail slot
arithmetic, [core/cart_fig8.c](../core/cart_fig8.c):91-92), while
stream numbering follows the header — a cosmetic misalignment,
not a divergent trajectory, and a writer bug the continuation
test (§8, A2) catches immediately. New cartridges should be
discouraged from embedding a counter the runner already keeps;
that guidance belongs in the CARTRIDGES.md update inside the
ckpt-core parcel.

## 8. Acceptance tests for the T2 implementation parcels

All must pass for every registered cartridge, driven from the
registry (`toy_emu list` pattern, [emu/main_emu.c](../emu/main_emu.c):49-53)
so new carts join automatically — the same property proof runs
have today ([docs/CARTRIDGES.md](CARTRIDGES.md):66-68).

**A. Core format (host, no hardware).**
A1 *Golden header*: a hand-built 64-byte fixture (known name/size/
versions/step/hash/CRCs as literal bytes in the test) parses to
exactly those field values — this pins endianness and offsets
independently of the writer (a writer+reader pair sharing a bug
must not be able to pass).
A2 *Round-trip + continuation*: run N steps, save, fresh process
load, run M more; the (step, hash) stream must `cmp`-equal an
uninterrupted N+M run. N ∈ {0, 1, proof_report_every,
proof_steps}, M ≥ proof_report_every. Also save→load→save must
be byte-identical.
A3 *Registry rules*: every cart's name ≤15 chars ASCII,
`state_size` ≤ cap, layout `_Static_assert`s compile (§3).

**B. Rejection (each must refuse, leave live state untouched, and
preserve the blob).**
B1 *Torn writes*: every prefix truncation of a valid image
(0..len−1 in steps incl. mid-header, at 60/63/64, mid-payload);
all-0xFF image (erased flash); all-0x00 image.
B2 *Bit damage*: single-bit flip in each header region (magic,
version, size, versions, step, hash, both CRCs) and in payload —
assert the *specific* rejection reason distinguishes CRC-fail
from FNV-fail where §2 says it must (flip in payload with CRCs
recomputed → FNV-fail path).
B3 *Version/identity*: `format_version` 0 and 2;
`cart_state_version` ±1; unknown name; valid image of cart X
presented while cart Y selected (must re-select-or-reject per
§7, never load into Y); `state_size` = cap+8; trailing garbage
byte; `flags` = 1.

**C. Flash slots (host-simulated fake NOR first, hardware
second).** The fake models erase→0xFF, program-clears-bits-only,
and a kill-switch callable between any two byte programs.
C1 Kill during each protocol phase (§5.1 steps 1–5): boot
selection always returns a fully valid slot (old or new), never
a torn one, over randomized kill points. C2 Both slots valid →
higher `slot_seq` wins; newest corrupted (post-commit bit flip)
→ older wins and the corrupt slot is not erased until replaced.
C3 Commit word absent → slot invisible even if image CRCs pass.
C4 Wear policy unit test with a simulated clock: no write before
the periodic interval with unchanged hash; event triggers write;
spam coalesces to the 60 s floor; erase counter per slot matches
the §5.1 model. Hardware pass (board parcel): save on board,
power-cycle, resume, proof-stream suffix matches host — run
under the normal board gate at merge, by the lead.

**D. Cross-runner interchange (the headline gate).**
D1 Pinned fixtures: for each cart, one `.ckpt` at a fixed step
(e.g. `proof_steps`) generated once by the host reference and
committed as a test fixture with its expected continuation
hashes. D2 Host loads fixture → continuation matches. WASM (node
harness beside [site/check_wasm.mjs](../site/check_wasm.mjs))
loads fixture → same continuation, and WASM-*produced* checkpoint
at the same step `cmp`-equals the fixture byte-for-byte.
D3 Board-produced image (read back over CDC or extracted from
flash) `cmp`-equals the fixture; board resumes the host fixture
to the same suffix. D4 Browser-storage smoke (manual or
headless): save, reload page, state+step+hash restored from
IndexedDB; export download → host loads it; host file → browser
import → identical hash readout in the page status line.

Failure of any D item is release-blocking for the responsible
parcel — byte interchange is the feature, not a stretch goal.

## 9. T2 parcel breakdown

Sequenced; each parcel is independently reviewable and leaves
`tools/run.sh check` green. Files are owned exclusively except
where noted.

| # | parcel | owns | delivers | depends on |
|---|---|---|---|---|
| 1 | `ckpt-core` | new `core/toy_ckpt.{h,c}`; `core/toy_cart.h` (add `state_version`); one-line edits in every `core/cart_*.c` (field + layout asserts); `docs/CARTRIDGES.md` checkpoint section; new host test in `emu/` or `tests/` | serialize / validate / load-into-scratch API used by *all* runners (single CRC+FNV implementation), §8A + §8B passing | — |
| 2 | `ckpt-emu` | `emu/main_emu.c`, `tools/run.sh` (new `ckpt` budget) | `save`/`load` subcommands, tmp+rename writes (§5.3), §8A2 wired as a budget, pinned fixtures (§8D1) | 1 |
| 3 | `ckpt-wasm` | `site/wasm_glue.c`, `site/app.js`, `site/check_wasm.mjs` (+ page markup in `site/index.html`) | state export/load exports via parcel 1's C code, IndexedDB persistence, `.ckpt` download/upload, §8D2 + §8D4 | 1, 2 (fixtures) |
| 4 | `ckpt-board-flash` | `board/main_board.c`, `board/CMakeLists.txt`, new `board/toy_flash.{h,c}`, fake-NOR host test | flash reserve, A/B slots (§5.1), wear policy, §8C; board proof gains a resume-suffix line | 1, 2 (fixtures). Verify fitted-part constants first (§5.1). Board budget at merge by lead. |
| 5 | `ckpt-sd` (deferred) | new `board/` SD/FAT layer | generation-named files (§5.2), per-cart SD resume | 4 + RP2350 kit in hand; driver choice may need its own mini-study |

Parcels 2 and 3 can run in parallel after 1 (3 needs 2's fixture
*files* only, which can be pre-generated by parcel 1's author if
scheduling demands). Parcel 4 is the only one touching hardware
risk; its fake-NOR simulation (§8C) must land and pass before any
real erase is issued — a torn-write bug found by killing power on
the one genuine Pico is a lesson the project can learn in RAM
instead.

## 10. Owner questions

- **Q1 (wear vs loss window).** Is "lose ≤15 min of steps on
  unplanned power loss, zero across planned sleep/switch"
  acceptable (§5.1 table)? If not, the 16 MiB kits can run a
  4–8-slot ring or SD-backed frequent checkpoints — at hardware
  cost the 2 MiB boards can't match, splitting the fleet's
  behaviour.
- **Q2 (resume scope).** One current-cartridge checkpoint in
  internal flash (fits everywhere), per-cartridge resume only
  where SD/16 MiB exists — acceptable split, or must switching
  carts on the small board also resume each where it left off
  (⇒ smaller per-cart slots, ⇒ a state-size sub-cap per cart)?
- **Q3 (versioning hardness).** Reject-and-loud-reset on any
  `cart_state_version` mismatch (blob preserved for export),
  migration only after a cart is declared stable — confirmed as
  policy while states churn?
- **Q4 (sharing surface).** Is "paste a base64 checkpoint into
  the website / copy one out" a wanted headline (18,700 chars
  for fig8), or is binary file download/upload sufficient?
- **Q5 (name cap).** Cartridge names capped at 15 ASCII chars
  as a registry rule (16-byte header field, §2) — any planned
  cartridge name longer?

## Appendix: worked sizes at a glance

```
state (fig8 / pendulum / cap)      13,960 / 7,128 / 65,536 B
checkpoint image (+64 header)      14,024 / 7,192 / 65,600 B
flash sectors @4 KiB               4 / 2 / 17  (slot 69,632 B at cap)
flash program pages @256 B         55 / 29 / 257
A/B reserve @cap                   139,264 B (6.64% of 2 MiB, 0.83% of 16 MiB)
base64 transport                   18,700 / 9,592 / 87,468 chars
IndexedDB per record               = image bytes (no transform)
sector life @100k, 15-min policy   ~5.7 years continuous (§5.1)
```

