# muse-softfloat-toy

A deterministic desk toy: a gallery of exact simulations (orbits,
chaos, precision demos) for a round-touch board (RP2350 target;
proven today on RP2040), every state re-derivable bit-for-bit on
a desktop. Built on
[muse-softfloat](https://github.com/loganw234/muse-softfloat)
(vendored as a pinned submodule at its sealed `v1.0.0` tag), whose
binary64 arithmetic is the simulation engine for every cartridge.
Thirteen cartridges so far, each proven host == board == browser.

Public, MIT-licensed (see [LICENSE](LICENSE)). The design of
record is
[docs/TOY-0.md](docs/TOY-0.md); new simulations follow
[docs/CARTRIDGES.md](docs/CARTRIDGES.md); the dated project record
is [docs/RECORD.md](docs/RECORD.md).

**Live emulator:** <https://loganw234.github.io/muse-softfloat-toy/>
— the real C sim core in WebAssembly, rendered at both glass sizes.
Exact states can be deep-linked, e.g.
`?cart=pendulum&steps=3600&pause=1`.

## Layout

| path | what |
|---|---|
| `core/` | the deterministic sim core: framebuffer contract, cartridge interface, cartridges |
| `emu/` | host screen emulator: renders the same framebuffer to PNG frames, prints state hashes |
| `site/` | permanent live browser emulator: the sim core compiled to WebAssembly, both glass sizes |
| `board/` | RP2040 firmware (core-only hash runner today; display/input when the kit arrives) |
| `libs/muse-softfloat/` | the library, pinned submodule at `v1.0.0` — never edited from here |
| `tools/run.sh` | the front door: `host`, `check`, `board` budgets |
| `docs/` | TOY-0 study, RECORD |

## Run it (host, today)

```
git submodule update --init
bash tools/run.sh host    # build + run cartridges, render frames to build/emu/
bash tools/run.sh check   # determinism: two runs, identical state hashes
bash tools/run.sh site    # build + verify the live WebAssembly emulator
bash tools/run.sh board   # flash the genuine Pico, compare state hashes host == board
```

## The rule this repo lives by

The stepped states are exact and identical everywhere; anything
drawn between stepped states (banked-playback interpolation) is
drawing, and is labelled as such. If host and board ever disagree
on a state hash, that is the bug, whatever the screen looks like.
