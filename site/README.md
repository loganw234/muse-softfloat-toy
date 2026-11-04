# Live screen emulator

This is the project's permanent browser emulator. It does not play
recorded frames: `app.js` instantiates `toy.wasm`, steps the same
registered cartridges as the native emulator and board firmware,
and renders one state at both target glass sizes.

Build and verify locally:

```
python site/build.py --cc "zig cc"
python -m http.server 8000 --directory site
```

Then open <http://localhost:8000/>.

`build.py` runs `check_wasm.mjs` under Node. The checker executes
every registered cartridge's descriptor-defined proof run and
compares each reported hash with `expected-hashes.json`; those
pinned values come from the native emulator and must also match
the board. A cartridge whose behaviour intentionally changes must
update its pinned hashes in the same reviewed commit that changes
its mathematics.

`toy.wasm` is generated and ignored locally; GitHub Actions rebuilds
it from source for deployment.
