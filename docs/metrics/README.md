# Toy metrics — plant ledger

Verification of parcels in this repo follows the planted-copy
pattern from the muse-softfloat library project, with its own
ledger here (the sealed library's ledger does not grow).

Per parcel, before merge:

1. The lead makes a squash copy of the parcel branch and plants
   five defects in it. The key (what was planted, where) stays
   in workspace scratch; only its SHA-256 is committed here,
   in `plants.jsonl`, **before** the verifier is dispatched.
2. An independent verifier reviews the planted copy cold
   (provenance gate: it must quote the copy's HEAD; this file
   is fenced from its brief by name).
3. After the verdict, the row is completed in place (records
   are restated pre-push, append-only after) with caught/missed
   per plant. New rows carry `checked_by: lead-merge`.

A plant row proves the verifier actually inspected the copy it
was given; a verifier that cannot find planted defects has not
verified the unplanted claims either.
