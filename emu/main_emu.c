/* main_emu.c - host screen emulator and hash runner.
 *
 *   toy_emu frames <cart> <size> <steps-per-frame> <frames> <outdir>
 *       Render frames to <outdir>/fNNN.ppm (circle-masked, as the
 *       round glass presents it) and print per-frame state hashes.
 *   toy_emu run <cart> <total-steps> <report-every>
 *       No rendering: print step/hash lines. This is the stream the
 *       board firmware must reproduce exactly.
 *   toy_emu save <cart> <steps> <file>
 *       Run <steps> from reset, then write a TOYCKPT1 checkpoint
 *       (core/toy_ckpt.h) to <file> via a .tmp sibling + rename.
 *   toy_emu load <file> <more-steps> <report-every>
 *       Read and validate a checkpoint, then continue stepping.
 *       Step numbers and hash lines continue the uninterrupted
 *       `run` stream: save at N then load for M prints the tail
 *       of `run <cart> <N+M>`.
 *   toy_emu bank <cart> <steps> <keyframe-every> [file]
 *       Time layer: step <steps> from reset storing sparse
 *       TOYBANK1 keyframes every <keyframe-every> steps (0 =
 *       start+final only) to [file] (default <cart>.bank).
 *       Wall labels come from the runner clock (emu/clock_emu.c,
 *       env TOY_CLOCK_UTC_SECONDS); unset banks fine, unlabelled.
 *   toy_emu replay <file>
 *       Re-step between keyframes VERIFYING every keyframe hash
 *       (core/toy_bank.h) - the bit-identical playback proof -
 *       then print the morning report as verified. Fails loudly
 *       on the first mismatched keyframe.
 *   toy_emu report <file>
 *       Print the morning report v1 for a bank file (no replay;
 *       playback_verified=0).
 *   toy_emu clock
 *       Print the runner clock: valid/unset, UTC seconds, zone
 *       offset, local minute-of-day, monotonic ms, and whether
 *       the default schedule considers now active. Setting the
 *       clock never changes any hash (tools/clock_test.c).
 *   toy_emu session <file>
 *       Replay a TOYSESSION1 interaction log exported by the web
 *       mock device ("TOYSESSION1 cart=<name>", one
 *       "<step> <type> <id> <x> <y> <v> <bits>" event per line,
 *       final "END <total_steps>") from reset, with the cart's
 *       proof signal source and proof script running exactly as
 *       in `proof` mode - the native re-proof of a browser
 *       session. An event's step is the 1-based proof-step
 *       number it applied before (see run_session).
 *   toy_emu feedrun <cart> <file.tsig> <steps>
 *       TSIG replay import: feed <cart> from a recorded TSIG v1
 *       byte stream (core/toy_signal.h) instead of its synthetic
 *       proof source, proof inputs and boundary order unchanged.
 *       The header must match the cart's sig_spec exactly
 *       (device, channels, sample rate; no resampling) and its
 *       payload FNV must verify, else exit 2 before any step -
 *       and a stream that ends early aborts mid-run at the
 *       step it starved. The TOY-RUN line names sig=replay and
 *       the file's stream id, so a replay hash is a distinct
 *       pinned hash, never a synthetic proof hash in disguise.
 *   toy_emu tsigout <cart> <steps> <file>
 *       Export the cart's synthetic proof stream (exactly
 *       <steps> x samples_per_step samples) as a TSIG v1 file,
 *       so the same bytes can be re-run via feedrun - and,
 *       come real captures, a detector can be run on the
 *       synthetic file and the recorded file side by side.
 *   toy_emu shell <size> <outdir>
 *       Scripted tour of the product shell (core/toy_shell.h):
 *       a compiled-in event sequence walks gallery -> settings
 *       -> running -> clock-set -> clockface at a fixed fake
 *       wall time, rendering fNNN.ppm frames and printing the
 *       shell action log plus a per-frame FNV-1a 64 over the
 *       framebuffer bytes (row-major, low byte then high byte
 *       of each RGB565 pixel). Fully reproducible: same size,
 *       same log, same fingerprints, every run.
 */
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "toy_cart.h"
#include "toy_ckpt.h"
#include "toy_bank.h"
/* Runner-side only (see core/toy_clock.h): the emu may read
 * wall time for bank labels/scheduling; cartridges never can. */
#define TOY_CLOCK_ALLOW 1
#include "toy_clock.h"
/* Runner shell (core/toy_shell.h): presentation layer over the
 * same registry; the emu is a runner, so it may drive it. */
#define TOY_SHELL_ALLOW 1
#include "toy_shell.h"

static void *xcalloc(size_t n)
{
    void *p = calloc(1, n ? n : 1);
    if (!p) { fprintf(stderr, "oom\n"); exit(2); }
    return p;
}

/* Input seam: proof/run/frames replay the descriptor's compiled-in
 * proof_inputs script at its recorded steps automatically (step 0
 * after reset, then before each numbered step), via the shared
 * helpers in toy_cart.h. Scriptless carts print byte-identical
 * lines to the pre-seam runner; scripted carts add inputs= and
 * input_hash= to their TOY-RUN line so host/board diffs prove the
 * same script ran. The emu has no live input device in this mode;
 * live events would enter through the shared toy_queue_t helpers.
 *
 * Signal seam (SIG-1): proof/run/frames instantiate the
 * descriptor's synthetic proof spec (toy_sig_proof_source_init)
 * and feed exactly samples_per_step frames BEFORE each step's
 * apply()/step(), the boundary order feed -> apply -> step.
 * Signal carts add sig=/stream= identity to TOY-RUN (study
 * section 7); signal-deaf carts print byte-identical lines and
 * the feed helper is a no-op. An underrun aborts the run here:
 * proofs never zero-fill (study section 4 rule 4). */
static void check_cart_input(const toy_cart_t *cart)
{
    int rc = toy_cart_check_input(cart);
    if (rc) { fprintf(stderr, "bad input descriptor for %s (code %d)\n", cart->name, rc); exit(2); }
    rc = toy_cart_check_signal(cart);
    if (rc) { fprintf(stderr, "bad signal descriptor for %s (code %d)\n", cart->name, rc); exit(2); }
}

static toy_sig_source_t emu_sig_src;
static int16_t emu_sig_frame[TOY_SIG_MAX_FRAME_VALUES];

/* Proof source for cart, or NULL when signal-deaf. Source
 * creation is the one place a future live mode would swap in
 * a driver-backed source; the feed loop would not change. */
static toy_sig_source_t *sig_setup(const toy_cart_t *cart)
{
    int created = toy_sig_proof_source_init(cart, &emu_sig_src);
    if (created < 0) {
        fprintf(stderr, "cannot instantiate proof signal for %s\n", cart->name);
        exit(2);
    }
    return created ? &emu_sig_src : NULL;
}

static void sig_feed_or_die(const toy_cart_t *cart, void *st,
                            toy_sig_source_t *src)
{
    int rc = toy_feed_step(cart, st, src, emu_sig_frame,
                           TOY_SIG_MAX_FRAME_VALUES);
    if (rc) {
        fprintf(stderr, "signal feed failed for %s (code %d): proof aborted\n",
                cart->name, rc);
        exit(2);
    }
}

static void print_run_line(const toy_cart_t *cart, const toy_sig_source_t *src)
{
    printf("TOY-RUN cart=%s state_size=%zu hash=fnv1a64", cart->name, cart->state_size);
    if (cart->proof_input_count)
        printf(" inputs=%u input_hash=%016llx", cart->proof_input_count,
               (unsigned long long)toy_hash_inputs(cart->proof_inputs,
                                                   cart->proof_input_count));
    if (src)
        printf(" sig=%s device=%u scenario=%u seed=0x%llx rate=%u per_step=%u stream=%016llx",
               toy_sig_kind_name(src->info.kind), src->info.device,
               src->info.scenario_id,
               (unsigned long long)src->info.seed,
               src->info.sample_rate_hz, cart->sig_spec.samples_per_step,
               (unsigned long long)src->info.stream_id);
    printf("\n");
}

/* --- TOYSESSION1 replay (`session` mode) ---
 * Step semantics mirror site/wasm_glue.c toy_wasm_step (lines
 * 111-129) and toy_wasm_select (93-109) exactly:
 *  - select: reset, step counter = 0, queue cleared, proof
 *    entries for step 0 applied (no feed);
 *  - each sub-step: feed FIRST, then proof entries due at
 *    (counter + 1), then the live queue drains with the shared
 *    cap TOY_MAX_EVENTS_PER_STEP minus the proof entries just
 *    applied (proof first, excess live events carry in order),
 *    then step(), then counter++.
 * A logged session step S is the 1-based proof-step number
 * (as in `proof`/`run`) of the sub-step it drained before:
 * the browser exporter stamps toy_steps_done + 1 at drain
 * (docs/CARTRIDGES.md "Web mock device" is normative). So
 * the replay loop below is the proof loop with, at emu step
 * s = done + 1, the session events stamped s pushed into a
 * real toy_queue_t (file order, so enqueue coalescing/drops
 * and drain carry are the queue's own) before feed -> proof
 * -> drain -> step. Pushing all of step s's events just
 * before that boundary is exact: in the browser they were
 * queued while the counter read s - 1, after the previous
 * boundary's drain and before this one, and nothing else
 * reads the queue in between.
 *
 * Valid stamps are 1..END total: stamp 0 has no boundary
 * (the step-0 slot is proof setup only, no live drain) and
 * stamps past total would drain only in a further sub-step
 * past the exported end. Likewise events still queued
 * (cap-carried) after the final step were never applied in
 * the session and reject rather than guess.
 *
 * The whole file is parsed and validated strictly BEFORE
 * the cart is touched, so a malformed log prints no hash
 * lines: exact header with a resolvable cart, exactly seven
 * decimal fields per event, nondecreasing steps, event type
 * in range and inside the cart's caps as the wasm queue push
 * gates them (deaf cart, missing cap, SET_PARAM id out of
 * range all reject), a final END line, and nothing after it.
 * Every rejection is a stderr message and exit 2, like the
 * other modes. */
static int sess_parse_i64(const char *tok, int64_t *out)
{
    if (!tok || !tok[0]) return -1;
    errno = 0;
    char *end = NULL;
    long long v = strtoll(tok, &end, 10);
    if (errno || end == tok || *end) return -1;
    *out = (int64_t)v;
    return 0;
}

static int sess_parse_u64(const char *tok, uint64_t *out)
{
    if (!tok || !tok[0] || tok[0] == '-') return -1;
    errno = 0;
    char *end = NULL;
    unsigned long long v = strtoull(tok, &end, 10);
    if (errno || end == tok || *end) return -1;
    *out = (uint64_t)v;
    return 0;
}

/* Split a line into whitespace tokens in place. Returns the
 * token count, or max+1 if there are more than max. */
static int sess_tokens(char *line, char **toks, int max)
{
    int n = 0;
    char *t = strtok(line, " \t");
    while (t) {
        if (n >= max) return max + 1;
        toks[n++] = t;
        t = strtok(NULL, " \t");
    }
    return n;
}

static int run_session(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "cannot read %s\n", path); return 2; }
    const toy_cart_t *cart = NULL;
    toy_proof_input_t *evs = NULL; /* same {step, ev} shape */
    size_t n = 0, capn = 0;
    uint64_t total = 0;
    int have_end = 0;
    int64_t prev_step = -1;
    long lineno = 0;
    char line[1024];
    while (fgets(line, sizeof line, f)) {
        lineno++;
        size_t len = strlen(line);
        if (len && line[len - 1] != '\n' && !feof(f)) {
            fprintf(stderr, "session %s:%ld: line too long\n", path, lineno);
            fclose(f); free(evs); return 2;
        }
        while (len && (line[len - 1] == '\n' || line[len - 1] == '\r'))
            line[--len] = '\0';
        if (have_end) {
            fprintf(stderr, "session %s:%ld: trailing content after END\n",
                    path, lineno);
            fclose(f); free(evs); return 2;
        }
        char *toks[9];
        int nt = sess_tokens(line, toks, 9);
        if (!cart) {
            if (nt != 2 || strcmp(toks[0], "TOYSESSION1") != 0 ||
                strncmp(toks[1], "cart=", 5) != 0 || !toks[1][5]) {
                fprintf(stderr,
                        "session %s:%ld: bad header (want"
                        " TOYSESSION1 cart=<name>)\n", path, lineno);
                fclose(f); return 2;
            }
            cart = toy_cart_by_name(toks[1] + 5);
            if (!cart) {
                fprintf(stderr, "unknown cart %s\n", toks[1] + 5);
                fclose(f); return 2;
            }
            check_cart_input(cart);
            continue;
        }
        if (nt == 2 && strcmp(toks[0], "END") == 0) {
            if (sess_parse_u64(toks[1], &total) != 0 ||
                total > 0xffffffffull) {
                fprintf(stderr, "session %s:%ld: bad END total\n",
                        path, lineno);
                fclose(f); free(evs); return 2;
            }
            have_end = 1;
            continue;
        }
        int64_t step, type, id, x, y, v;
        uint64_t bits;
        if (nt != 7 ||
            sess_parse_i64(toks[0], &step) != 0 ||
            sess_parse_i64(toks[1], &type) != 0 ||
            sess_parse_i64(toks[2], &id) != 0 ||
            sess_parse_i64(toks[3], &x) != 0 ||
            sess_parse_i64(toks[4], &y) != 0 ||
            sess_parse_i64(toks[5], &v) != 0 ||
            sess_parse_u64(toks[6], &bits) != 0) {
            fprintf(stderr,
                    "session %s:%ld: malformed event line (want"
                    " <step> <type> <id> <x> <y> <v> <bits>)\n",
                    path, lineno);
            fclose(f); free(evs); return 2;
        }
        if (step < 0 || step > 0xffffffffll) {
            fprintf(stderr, "session %s:%ld: step out of range\n",
                    path, lineno);
            fclose(f); free(evs); return 2;
        }
        if (step < prev_step) {
            fprintf(stderr, "session %s:%ld: decreasing step\n",
                    path, lineno);
            fclose(f); free(evs); return 2;
        }
        prev_step = step;
        if (type < 1 || type > TOY_EV_MAX_KNOWN) {
            fprintf(stderr, "session %s:%ld: event type out of range\n",
                    path, lineno);
            fclose(f); free(evs); return 2;
        }
        if (id < 0 || id > 0xffffffffll ||
            x < INT32_MIN || x > INT32_MAX ||
            y < INT32_MIN || y > INT32_MAX ||
            v < INT32_MIN || v > INT32_MAX) {
            fprintf(stderr, "session %s:%ld: event field out of range\n",
                    path, lineno);
            fclose(f); free(evs); return 2;
        }
        if (!cart->apply) {
            fprintf(stderr,
                    "session %s:%ld: event for deaf cart %s (no apply)\n",
                    path, lineno, cart->name);
            fclose(f); free(evs); return 2;
        }
        uint32_t need = toy_event_cap_mask((uint32_t)type);
        if (need && (cart->input_caps & need) == 0) {
            fprintf(stderr,
                    "session %s:%ld: event type %lld not in caps of %s\n",
                    path, lineno, (long long)type, cart->name);
            fclose(f); free(evs); return 2;
        }
        if (type == TOY_EV_SET_PARAM && (uint64_t)id >= cart->param_count) {
            fprintf(stderr,
                    "session %s:%ld: SET_PARAM id out of range for %s\n",
                    path, lineno, cart->name);
            fclose(f); free(evs); return 2;
        }
        if (n == capn) {
            size_t want = capn ? capn * 2 : 64;
            toy_proof_input_t *grown =
                (toy_proof_input_t *)realloc(evs, want * sizeof *evs);
            if (!grown) {
                fprintf(stderr, "oom\n");
                fclose(f); free(evs); return 2;
            }
            evs = grown;
            capn = want;
        }
        evs[n].step = (uint32_t)step;
        memset(&evs[n].ev, 0, sizeof evs[n].ev);
        evs[n].ev.type = (uint32_t)type;
        evs[n].ev.id = (uint32_t)id;
        evs[n].ev.x = (int32_t)x;
        evs[n].ev.y = (int32_t)y;
        evs[n].ev.v = (int32_t)v;
        evs[n].ev.bits = bits;
        n++;
    }
    if (ferror(f)) {
        fprintf(stderr, "cannot read %s\n", path);
        fclose(f); free(evs); return 2;
    }
    fclose(f);
    if (!cart) {
        fprintf(stderr, "session %s: bad header (empty file)\n", path);
        return 2;
    }
    if (!have_end) {
        fprintf(stderr, "session %s: missing END line\n", path);
        free(evs); return 2;
    }
    if (n && (evs[0].step < 1 || (uint64_t)evs[n - 1].step > total)) {
        fprintf(stderr,
                "session %s: event steps must be in 1..END total %llu"
                " (1-based proof-step stamps; got first %u last %u)\n",
                path, (unsigned long long)total,
                evs[0].step, evs[n - 1].step);
        free(evs); return 2;
    }
    toy_sig_source_t *sig = sig_setup(cart);
    void *st = xcalloc(cart->state_size);
    cart->reset(st);
    uint32_t cur = 0;
    toy_apply_proof_at(cart, st, 0, &cur); /* step-0 setup; no feed */
    toy_queue_t live;
    toy_queue_clear(&live);
    print_run_line(cart, sig);
    size_t idx = 0;
    uint64_t applied = 0;
    for (uint64_t done = 0; done < total; done++) {
        while (idx < n && (uint64_t)evs[idx].step == done + 1) {
            toy_queue_push(&live, &evs[idx].ev);
            idx++;
        }
        sig_feed_or_die(cart, st, sig); /* feed -> proof -> drain -> step */
        uint32_t before = cur;
        toy_apply_proof_at(cart, st, (uint32_t)(done + 1), &cur);
        applied += toy_apply_queue(cart, st, &live,
                                    TOY_MAX_EVENTS_PER_STEP - (cur - before));
        cart->step(st);
    }
    if (live.len) {
        fprintf(stderr,
                "session %s: %u event(s) still queued after END %llu"
                " (cap carry past the final step: never applied in"
                " the session)\n",
                path, live.len, (unsigned long long)total);
        free(evs); return 2;
    }
    uint64_t h = toy_hash_state(st, cart->state_size);
    printf("step %llu hash %016llx\n", (unsigned long long)total,
           (unsigned long long)h);
    printf("TOY-SESSION events %zu applied %llu\n", n,
           (unsigned long long)applied);
    printf("TOY-DONE steps %llu hash %016llx\n",
           (unsigned long long)total, (unsigned long long)h);
    free(evs);
    return 0;
}

static int write_ppm(const char *path, const toy_fb_t *fb)
{
    FILE *f = fopen(path, "wb");
    if (!f) return -1;
    fprintf(f, "P6\n%d %d\n255\n", fb->w, fb->h);
    int cx = fb->w / 2, cy = fb->h / 2, r = (fb->w < fb->h ? fb->w : fb->h) / 2;
    for (int y = 0; y < fb->h; y++) {
        for (int x = 0; x < fb->w; x++) {
            unsigned char rgb[3];
            int dx = x - cx, dy = y - cy;
            if (dx * dx + dy * dy > r * r) {
                rgb[0] = 14; rgb[1] = 14; rgb[2] = 20; /* off-glass */
            } else {
                unsigned p = fb->px[(size_t)y * (size_t)fb->w + (size_t)x];
                rgb[0] = (unsigned char)(((p >> 11) & 31u) << 3);
                rgb[1] = (unsigned char)(((p >> 5) & 63u) << 2);
                rgb[2] = (unsigned char)((p & 31u) << 3);
            }
            fwrite(rgb, 1, 3, f);
        }
    }
    fclose(f);
    return 0;
}

/* Write via <file>.tmp then rename, so a killed toy_emu never
 * leaves a torn checkpoint under the real name (study 5.3). */
static int write_file_atomic(const char *path, const unsigned char *buf, size_t len)
{
    char tmp[1024];
    if (snprintf(tmp, sizeof tmp, "%s.tmp", path) >= (int)sizeof tmp) return -1;
    FILE *f = fopen(tmp, "wb");
    if (!f) return -1;
    int ok = fwrite(buf, 1, len, f) == len && fflush(f) == 0;
    if (fclose(f) != 0) ok = 0;
    if (!ok) { remove(tmp); return -1; }
    if (rename(tmp, path) != 0) {
        /* Windows rename does not replace; drop the old file first. */
        remove(path);
        if (rename(tmp, path) != 0) { remove(tmp); return -1; }
    }
    return 0;
}

/* Read a whole checkpoint file. Reads at most cap bytes; a file
 * longer than cap yields len == cap (validation then rejects it
 * on exact length). Returns 0 on success. */
static int read_file(const char *path, unsigned char *buf, size_t cap, size_t *len)
{
    FILE *f = fopen(path, "rb");
    if (!f) return -1;
    size_t n = fread(buf, 1, cap, f);
    int bad = ferror(f);
    fclose(f);
    if (bad) return -1;
    *len = n;
    return 0;
}

/* Read a whole bank file (budget-bounded). One byte past the
 * budget is read so an oversized file reports at its true size
 * class and parse rejects it, rather than truncating. */
static unsigned char *read_bank_file(const char *path, size_t *len_out)
{
    size_t cap = (size_t)TOY_BANK_BUDGET_BYTES + 1u;
    unsigned char *buf = (unsigned char *)xcalloc(cap);
    FILE *f = fopen(path, "rb");
    if (!f) { free(buf); return NULL; }
    size_t n = fread(buf, 1, cap, f);
    int bad = ferror(f);
    fclose(f);
    if (bad) { free(buf); return NULL; }
    *len_out = n;
    return buf;
}

/* Morning report v1, text form (study-timebank section 5.2).
 * Wall time prints as raw UTC seconds + local HH:MM derived by
 * integer minute-of-day; calendar rendering is presentation
 * (device UI / JS Date), and an unset clock prints as unset -
 * never an invented time. */
static void print_morning_report(const toy_morning_report_t *r)
{
    printf("TOY-MORNING-REPORT v1\n");
    printf("cart %s\n", r->cart_name);
    printf("steps %llu -> %llu (banked %llu)\n",
           (unsigned long long)r->steps_at_start,
           (unsigned long long)r->steps_at_end,
           (unsigned long long)r->steps_banked);
    if (r->wall_valid)
        printf("steps_expected_1x %llu\n",
               (unsigned long long)r->steps_expected_1x);
    else
        printf("steps_expected_1x n/a (clock unset)\n");
    printf("keyframes %u every %u steps\n", r->keyframe_count,
           r->keyframe_every);
    printf("hash_start %016llx\n", (unsigned long long)r->hash_start);
    printf("hash_end %016llx\n", (unsigned long long)r->hash_end);
    if (r->wall_valid) {
        toy_time_t ws = { r->wall_start_s, r->tz_offset_min, true };
        toy_time_t we = { r->wall_end_s, r->tz_offset_min, true };
        unsigned ms = toy_clock_minutes_of_day(ws);
        unsigned me = toy_clock_minutes_of_day(we);
        printf("wall utc %lld -> %lld (span %lld s, tz %+d min, local %02u:%02u -> %02u:%02u)\n",
               (long long)r->wall_start_s, (long long)r->wall_end_s,
               (long long)(r->wall_end_s - r->wall_start_s),
               (int)r->tz_offset_min, ms / 60, ms % 60, me / 60, me % 60);
    } else {
        printf("wall clock unset - banked without wall labels\n");
    }
    printf("end_reason %s\n", toy_bank_end_name(r->end_reason));
    printf("playback_verified %u\n", (unsigned)r->playback_verified);
    printf("bank_bytes %zu\n", r->bank_bytes);
}

/* --- TSIG replay import/export (`feedrun` / `tsigout`) ---
 * The replay twin of the proof loops above: identical boundary
 * order (step-0 proof setup, then feed -> apply -> step), the
 * only swap being the source - a TSIG v1 file's bytes via
 * toy_sig_source_replay + toy_feed_step_bytes instead of the
 * descriptor's synthetic spec via sig_setup + toy_feed_step.
 * Validation is strict and happens BEFORE the cart is touched,
 * like `session`: a signal-deaf cart, a malformed header, an
 * inexact length, a payload whose FNV-1a 64 does not equal the
 * header's payload-hash field (the format's integrity check;
 * TSIG has no separate CRC field), or a header that disagrees
 * with sig_spec on device/channels/sample rate each exit 2
 * without printing a hash line.
 *
 * The rate must EQUAL the spec: resampling is forbidden, not
 * merely unimplemented. Every feed cart's per-sample filters
 * and step dt are built for the spec rate (an 8-sample detrend
 * is 80 ms only at 100 Hz), so silently accepting a 200 Hz
 * recording would run different physics under the same cart
 * name. study-signals section 4 rule 3 binds rates statically
 * for exactly this reason; a mismatched capture must be
 * re-recorded at rate, or resampled by a future proven core
 * decimator, never here.
 *
 * Identity: the file bytes are the stream. The TOY-RUN line
 * prints sig=replay, orig_kind rides in the TOY-TSIG line, and
 * stream= is the payload hash from the header (verified
 * against the payload before stepping). The hashed label in
 * cart state therefore records REPLAY + that id - by the
 * labelling rule (study-signals section 7) a feedrun final
 * hash pins the replay run itself and must differ from the
 * synthetic proof hash even when tsigout exported those exact
 * samples: the sample path is bit-exact, the provenance words
 * in hashed state honestly are not. */
static unsigned char *read_whole_file(const char *path, size_t *len_out)
{
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;
    size_t cap = 1u << 16, n = 0;
    unsigned char *buf = (unsigned char *)malloc(cap);
    if (!buf) { fclose(f); return NULL; }
    for (;;) {
        if (n == cap) {
            size_t want = cap * 2u;
            unsigned char *grown = (unsigned char *)realloc(buf, want);
            if (!grown) { free(buf); fclose(f); return NULL; }
            buf = grown;
            cap = want;
        }
        size_t r = fread(buf + n, 1, cap - n, f);
        n += r;
        if (r == 0) break;
    }
    int bad = ferror(f);
    fclose(f);
    if (bad) { free(buf); return NULL; }
    *len_out = n;
    return buf;
}

static int run_feedrun(const char *cart_name, const char *path,
                       const char *steps_text)
{
    uint64_t steps = 0;
    if (sess_parse_u64(steps_text, &steps) != 0 ||
        steps > 0xffffffffull) {
        fprintf(stderr, "feedrun: bad step count %s\n", steps_text);
        return 2;
    }
    const toy_cart_t *cart = toy_cart_by_name(cart_name);
    if (!cart) { fprintf(stderr, "unknown cart %s\n", cart_name); return 2; }
    check_cart_input(cart);
    if (!cart->feed) {
        fprintf(stderr,
                "feedrun: cart %s consumes no signal feed"
                " (signal-deaf); a TSIG stream has nowhere to go\n",
                cart->name);
        return 2;
    }
    size_t len = 0;
    unsigned char *bytes = read_whole_file(path, &len);
    if (!bytes) { fprintf(stderr, "cannot read %s\n", path); return 2; }
    if (toy_sig_source_replay(&emu_sig_src, bytes, len) != 0) {
        fprintf(stderr,
                "feedrun: %s is not a valid TSIG v1 stream"
                " (bad header, or payload shorter than the"
                " header claims)\n", path);
        free(bytes);
        return 2;
    }
    toy_sig_source_t *src = &emu_sig_src;
    const toy_sig_info_t *info = &src->info;
    uint64_t payload_bytes =
        info->n_samples * (uint64_t)info->channels * 2ull;
    if ((uint64_t)len != (uint64_t)TOY_SIG_HEADER_SIZE + payload_bytes) {
        fprintf(stderr,
                "feedrun: %s has %zu bytes but its header describes"
                " %llu (trailing bytes are not part of a stream's"
                " identity): rejected\n",
                path, len,
                (unsigned long long)((uint64_t)TOY_SIG_HEADER_SIZE +
                                     payload_bytes));
        free(bytes);
        return 2;
    }
    uint64_t computed = toy_sig_fnv1a64(bytes + TOY_SIG_HEADER_SIZE,
                                        (size_t)payload_bytes);
    if (computed != src->replay.payload_hash) {
        fprintf(stderr,
                "feedrun: %s payload hash mismatch: header says"
                " %016llx, payload computes %016llx; the file"
                " bytes are the identity and they do not agree"
                " with themselves\n",
                path, (unsigned long long)src->replay.payload_hash,
                (unsigned long long)computed);
        free(bytes);
        return 2;
    }
    const toy_sig_spec_t *spec = &cart->sig_spec;
    if (info->device != spec->device) {
        fprintf(stderr,
                "feedrun: %s is device %u but cart %s binds device"
                " %u; rejected\n",
                path, info->device, cart->name, spec->device);
        free(bytes);
        return 2;
    }
    if (info->channels != toy_sig_device_channels(spec->device)) {
        fprintf(stderr,
                "feedrun: %s has %u channels but cart %s's device"
                " %u layout is %u; rejected\n",
                path, info->channels, cart->name, spec->device,
                toy_sig_device_channels(spec->device));
        free(bytes);
        return 2;
    }
    if (info->sample_rate_hz != spec->sample_rate_hz) {
        fprintf(stderr,
                "feedrun: %s is %u Hz but cart %s binds %u Hz;"
                " resampling is forbidden (the cart's per-sample"
                " filters and dt assume the spec rate): rejected\n",
                path, info->sample_rate_hz, cart->name,
                spec->sample_rate_hz);
        free(bytes);
        return 2;
    }
    void *st = xcalloc(cart->state_size);
    cart->reset(st);
    uint32_t cur = 0;
    toy_apply_proof_at(cart, st, 0, &cur); /* step-0 setup; no feed */
    print_run_line(cart, src);
    printf("TOY-TSIG file=%s orig=%s samples=%llu scale=%u scale2=%u"
           " payload_hash=%016llx verified=1\n",
           path, toy_sig_kind_name(info->orig_kind),
           (unsigned long long)info->n_samples, info->scale_num,
           info->scale2_num,
           (unsigned long long)src->replay.payload_hash);
    for (uint64_t s = 1; s <= steps; s++) {
        int rc = toy_feed_step_bytes(cart, st, src, bytes, len,
                                     emu_sig_frame,
                                     TOY_SIG_MAX_FRAME_VALUES);
        if (rc != 0) {
            if (rc == 2)
                fprintf(stderr,
                        "feedrun: TSIG underrun at step %llu for %s:"
                        " stream exhausted (%llu samples in file,"
                        " %u needed per step); run aborted, no"
                        " samples invented\n",
                        (unsigned long long)s, cart->name,
                        (unsigned long long)info->n_samples,
                        spec->samples_per_step);
            else
                fprintf(stderr,
                        "feedrun: signal feed failed at step %llu"
                        " for %s (code %d); run aborted\n",
                        (unsigned long long)s, cart->name, rc);
            free(bytes);
            return 2;
        }
        toy_apply_proof_at(cart, st, (uint32_t)s, &cur);
        cart->step(st);
    }
    uint64_t h = toy_hash_state(st, cart->state_size);
    printf("step %llu hash %016llx\n", (unsigned long long)steps,
           (unsigned long long)h);
    printf("TOY-DONE steps %llu hash %016llx\n",
           (unsigned long long)steps, (unsigned long long)h);
    free(bytes);
    return 0;
}

/* Export exactly the samples a proof run of <steps> steps
 * would consume, polled per-step from a fresh proof source so
 * the sequence is the proof sequence by construction. The
 * header is the format builder's (toy_sig_write_header) over
 * the source info with n_samples filled in; orig_kind stays
 * SYNTHETIC, so the file never claims to be a recording. */
static int run_tsigout(const char *cart_name, const char *steps_text,
                       const char *path)
{
    uint64_t steps = 0;
    if (sess_parse_u64(steps_text, &steps) != 0 ||
        steps > 0xffffffffull) {
        fprintf(stderr, "tsigout: bad step count %s\n", steps_text);
        return 2;
    }
    const toy_cart_t *cart = toy_cart_by_name(cart_name);
    if (!cart) { fprintf(stderr, "unknown cart %s\n", cart_name); return 2; }
    check_cart_input(cart);
    if (!cart->feed) {
        fprintf(stderr,
                "tsigout: cart %s consumes no signal feed"
                " (signal-deaf); it has no proof stream to export\n",
                cart->name);
        return 2;
    }
    toy_sig_source_t *src = sig_setup(cart); /* exits 2 on failure */
    uint32_t per_step = cart->sig_spec.samples_per_step;
    uint32_t channels = src->info.channels;
    uint64_t n_samples = steps * (uint64_t)per_step;
    uint64_t values = n_samples * (uint64_t)channels;
    if (values > (uint64_t)(SIZE_MAX / sizeof(int16_t)) ||
        values * 2ull > (uint64_t)SIZE_MAX - TOY_SIG_HEADER_SIZE) {
        fprintf(stderr, "tsigout: %llu steps is too large to export\n",
                (unsigned long long)steps);
        return 2;
    }
    int16_t *samples =
        (int16_t *)xcalloc((size_t)values * sizeof(int16_t));
    for (uint64_t s = 0; s < steps; s++) {
        uint32_t flags = 0;
        uint32_t got = toy_sig_poll(src, NULL, 0,
                                    samples + (size_t)(s * per_step *
                                                       channels),
                                    per_step, &flags);
        if (got != per_step || flags != 0u) {
            fprintf(stderr,
                    "tsigout: synthetic proof source underran at"
                    " step %llu for %s (got %u, flags %u)\n",
                    (unsigned long long)(s + 1u), cart->name, got, flags);
            return 2;
        }
    }
    uint64_t payload_hash =
        toy_sig_hash_samples(samples, n_samples, channels);
    toy_sig_info_t export_info = src->info;
    export_info.n_samples = n_samples; /* unbounded -> this file */
    size_t total = (size_t)TOY_SIG_HEADER_SIZE + (size_t)values * 2u;
    unsigned char *bytes = (unsigned char *)xcalloc(total);
    toy_sig_write_header(bytes, &export_info, payload_hash);
    toy_sig_write_payload(bytes + TOY_SIG_HEADER_SIZE, samples,
                          n_samples, channels);
    if (write_file_atomic(path, bytes, total) != 0) {
        fprintf(stderr, "cannot write %s\n", path);
        return 2;
    }
    printf("TOY-TSIG-SAVED cart=%s steps %llu samples %llu"
           " payload_hash %016llx bytes %zu file %s\n",
           cart->name, (unsigned long long)steps,
           (unsigned long long)n_samples,
           (unsigned long long)payload_hash, total, path);
    return 0;
}

/* --- product shell tour (`shell` mode) ---
 * The emu playing runner exactly as board/main_board.c will on
 * Friday: every scripted event goes to toy_shell_event() first;
 * START_CART resets the cart and drains the shell's param
 * queue before step 1; FORWARD_TO_CART enqueues to the cart;
 * CLOCK_COMMIT rewrites the fake wall clock (same UTC date,
 * edited local hh:mm at the edited offset, seconds preserved);
 * RUNNING frames step+render the cart, other frames render the
 * shell with the fake now passed in. */
static toy_shell_t emu_sh;
static toy_fb_t emu_sh_fb;
static toy_time_t emu_fake_now;
static const toy_cart_t *emu_run_cart;
static void *emu_run_state;
static toy_sig_source_t *emu_run_sig;
static uint32_t emu_run_cur;
static long emu_run_abs;
static toy_queue_t emu_live_q;
static int emu_frame_no;
static const char *emu_sh_outdir;

static const char *shell_screen_name(uint8_t s)
{
    switch (s) {
    case TOY_SHELL_GALLERY: return "GALLERY";
    case TOY_SHELL_RUNNING: return "RUNNING";
    case TOY_SHELL_SETTINGS: return "SETTINGS";
    case TOY_SHELL_CLOCKSET: return "CLOCKSET";
    case TOY_SHELL_CLOCKFACE: return "CLOCKFACE";
    default: return "?";
    }
}

static void shell_log_actions(uint32_t act)
{
    printf(" act=0x%02x", act);
    if (act & TOY_SH_ACT_START_CART) printf("|START_CART");
    if (act & TOY_SH_ACT_EXIT_TO_GALLERY) printf("|EXIT_TO_GALLERY");
    if (act & TOY_SH_ACT_PARAM_QUEUED) printf("|PARAM_QUEUED");
    if (act & TOY_SH_ACT_CLOCK_COMMIT) printf("|CLOCK_COMMIT");
    if (act & TOY_SH_ACT_SCREEN) printf("|SCREEN");
    if (act & TOY_SH_ACT_FORWARD_TO_CART) printf("|FORWARD_TO_CART");
    if (!act) printf("|NONE");
    printf(" screen=%s current=%s sel=%d qlen=%u\n",
           shell_screen_name(toy_shell_screen(&emu_sh)),
           toy_shell_current(&emu_sh) ? toy_shell_current(&emu_sh)->name : "-",
           emu_sh.selected, emu_sh.param_queue.len);
}

static void shell_print_param_queue(const char *what)
{
    for (uint32_t i = 0; i < emu_sh.param_queue.len; i++) {
        const toy_event_t *e = &emu_sh.param_queue.ev[i];
        printf("  %s SET_PARAM idx=%u v=%d bits=%016llx\n", what, e->id,
               e->v, (unsigned long long)e->bits);
    }
}

static void shell_do_actions(uint32_t act, const toy_event_t *ev)
{
    if (act & TOY_SH_ACT_START_CART) {
        const toy_cart_t *cart = toy_shell_current(&emu_sh);
        if (!cart) { fprintf(stderr, "shell: START with no current cart\n"); exit(2); }
        check_cart_input(cart);
        emu_run_cart = cart;
        emu_run_sig = sig_setup(cart);
        memset(emu_run_state, 0, TOY_MAX_STATE_SIZE);
        emu_run_abs = 0;
        emu_run_cur = 0;
        toy_queue_clear(&emu_live_q);
        cart->reset(emu_run_state);
        toy_apply_proof_at(cart, emu_run_state, 0, &emu_run_cur); /* proof step-0, as site select */
        shell_print_param_queue("start-param");
        toy_apply_queue(cart, emu_run_state, &emu_sh.param_queue,
                        TOY_MAX_EVENTS_PER_STEP); /* user params, step-0 slot */
        printf("  runner: reset %s, start params drained, hash %016llx\n",
               cart->name,
               (unsigned long long)toy_hash_state(emu_run_state, cart->state_size));
    }
    if ((act & TOY_SH_ACT_PARAM_QUEUED) && !(act & TOY_SH_ACT_START_CART)) {
        shell_print_param_queue("live-param");
        while (emu_sh.param_queue.len) { /* move to the cart live queue */
            toy_queue_push(&emu_live_q, &emu_sh.param_queue.ev[0]);
            for (uint32_t i = 1; i < emu_sh.param_queue.len; i++)
                emu_sh.param_queue.ev[i - 1] = emu_sh.param_queue.ev[i];
            emu_sh.param_queue.len--;
        }
    }
    if (act & TOY_SH_ACT_EXIT_TO_GALLERY) {
        printf("  runner: stopped %s\n", emu_run_cart ? emu_run_cart->name : "-");
        emu_run_cart = NULL;
    }
    if (act & TOY_SH_ACT_CLOCK_COMMIT) {
        int64_t days = emu_fake_now.utc_seconds / 86400;
        int64_t sec = ((emu_fake_now.utc_seconds % 60) + 60) % 60;
        int64_t local_s = (int64_t)emu_sh.clock_hh * 3600 +
                          (int64_t)emu_sh.clock_mm * 60 + sec;
        int64_t utc_s = (local_s - (int64_t)emu_sh.clock_tz_offset_min * 60) % 86400;
        if (utc_s < 0) utc_s += 86400;
        emu_fake_now.utc_seconds = days * 86400 + utc_s;
        emu_fake_now.tz_offset_min = emu_sh.clock_tz_offset_min;
        emu_fake_now.valid = true;
        printf("  runner: toy_clock_set(utc=%lld, tz=%d) for local %02d:%02d\n",
               (long long)emu_fake_now.utc_seconds,
               (int)emu_fake_now.tz_offset_min,
               emu_sh.clock_hh, emu_sh.clock_mm);
    }
    if (act & TOY_SH_ACT_FORWARD_TO_CART) {
        toy_queue_push(&emu_live_q, ev);
        printf("  runner: forwarded type=%u to cart queue (len=%u)\n",
               ev->type, emu_live_q.len);
    }
}

static void shell_send(toy_event_t ev, const char *label)
{
    printf("ev %s type=%u id=%u x=%d y=%d v=%d ->", label, ev.type, ev.id,
           ev.x, ev.y, ev.v);
    uint32_t act = toy_shell_event(&emu_sh, &ev);
    shell_log_actions(act);
    shell_do_actions(act, &ev);
}

static toy_event_t shell_ev(uint32_t type, uint32_t id, int32_t x, int32_t y, int32_t v)
{
    toy_event_t e;
    memset(&e, 0, sizeof e);
    e.type = type; e.id = id; e.x = x; e.y = y; e.v = v;
    return e;
}

static uint64_t shell_fb_fnv(const toy_fb_t *fb)
{
    uint64_t h = 1469598103934665603ull;
    size_t n = (size_t)fb->w * (size_t)fb->h;
    for (size_t i = 0; i < n; i++) {
        h ^= (uint64_t)(fb->px[i] & 0xffu); h *= 1099511628211ull;
        h ^= (uint64_t)(fb->px[i] >> 8);    h *= 1099511628211ull;
    }
    return h;
}

static void shell_frame(const char *label, toy_time_t now)
{
    if (toy_shell_screen(&emu_sh) == TOY_SHELL_RUNNING && emu_run_cart) {
        const toy_cart_t *cart = emu_run_cart;
        for (uint32_t s = 0; s < cart->demo_steps_per_frame; s++) {
            emu_run_abs++;
            sig_feed_or_die(cart, emu_run_state, emu_run_sig);
            uint32_t before = emu_run_cur;
            toy_apply_proof_at(cart, emu_run_state, (uint32_t)emu_run_abs, &emu_run_cur);
            toy_apply_queue(cart, emu_run_state, &emu_live_q,
                            TOY_MAX_EVENTS_PER_STEP - (emu_run_cur - before));
            cart->step(emu_run_state);
        }
        cart->render(emu_run_state, &emu_sh_fb);
    } else {
        toy_shell_render(&emu_sh, &emu_sh_fb, now);
    }
    char path[1024];
    snprintf(path, sizeof path, "%s/f%03d.ppm", emu_sh_outdir, emu_frame_no);
    if (write_ppm(path, &emu_sh_fb) != 0) {
        fprintf(stderr, "cannot write %s\n", path);
        exit(2);
    }
    printf("frame %d %s screen=%s sel=%d scroll=%d fbhash=%016llx%s\n",
           emu_frame_no, label, shell_screen_name(toy_shell_screen(&emu_sh)),
           emu_sh.selected, emu_sh.scroll,
           (unsigned long long)shell_fb_fnv(&emu_sh_fb),
           toy_shell_screen(&emu_sh) == TOY_SHELL_RUNNING && emu_run_cart
               ? " (cart owns glass)" : "");
    emu_frame_no++;
}

static int run_shell_tour(int size, const char *outdir)
{
    if (size < 64 || size > 1024) {
        fprintf(stderr, "shell: size must be 64..1024\n");
        return 2;
    }
    emu_sh_fb.w = emu_sh_fb.h = size;
    emu_sh_fb.px = (uint16_t *)xcalloc((size_t)size * (size_t)size * sizeof(uint16_t));
    emu_run_state = xcalloc(TOY_MAX_STATE_SIZE);
    emu_sh_outdir = outdir;
    emu_frame_no = 0;
    emu_run_cart = NULL;
    /* Fixed fake now: local 10:10:30 at UTC+01:00 on an
     * arbitrary fixed UTC date (day 20000 since the epoch). */
    emu_fake_now.utc_seconds = 20000ll * 86400ll + (10ll * 3600 + 10 * 60 + 30 - 3600);
    emu_fake_now.tz_offset_min = 60;
    emu_fake_now.valid = true;
    toy_time_t unset;
    memset(&unset, 0, sizeof unset);
    toy_shell_init(&emu_sh, toy_carts, toy_cart_count);
    int32_t gravity = -1;
    for (int i = 0; i < toy_cart_count; i++)
        if (strcmp(toy_carts[i]->name, "gravity") == 0) gravity = i;
    if (gravity < 0) { fprintf(stderr, "shell: gravity cart missing\n"); return 2; }
    printf("TOY-SHELL size=%d carts=%d fake_utc=%lld fake_tz=%d (local 10:10:30)\n",
           size, toy_cart_count, (long long)emu_fake_now.utc_seconds,
           (int)emu_fake_now.tz_offset_min);
    shell_frame("gallery-top", emu_fake_now);
    shell_send(shell_ev(TOY_EV_DIAL, 0, 0, 0, 200), "dial+200");
    shell_frame("gallery-mid", emu_fake_now);
    shell_send(shell_ev(TOY_EV_DIAL, 0, 0, 0, 400), "dial+400-to-bottom");
    shell_frame("gallery-bottom", emu_fake_now);
    shell_send(shell_ev(TOY_EV_DIAL, 0, 0, 0, (gravity - emu_sh.selected) * 25), "dial-to-gravity");
    shell_frame("gallery-gravity", emu_fake_now);
    shell_send(shell_ev(TOY_EV_POINTER_DOWN, 0, 400, 800, 0), "tap-footer-set");
    shell_frame("settings-initial", emu_fake_now);
    shell_send(shell_ev(TOY_EV_POINTER_DOWN, 0, 500, -480, 0), "tap-row0-right-bodies+");
    shell_frame("settings-bodies-staged", emu_fake_now);
    shell_send(shell_ev(TOY_EV_KEY_DOWN, TOY_KEY_DOWN, 0, 0, 0), "key-down-softening");
    shell_send(shell_ev(TOY_EV_DIAL, 0, 0, 0, 100), "dial+100-softening");
    shell_frame("settings-softening", emu_fake_now);
    shell_send(shell_ev(TOY_EV_KEY_DOWN, TOY_KEY_DOWN, 0, 0, 0), "key-down-palette");
    shell_send(shell_ev(TOY_EV_KEY_DOWN, TOY_KEY_RIGHT, 0, 0, 0), "key-right-palette+");
    shell_frame("settings-palette", emu_fake_now);
    shell_send(shell_ev(TOY_EV_KEY_DOWN, TOY_KEY_ENTER, 0, 0, 0), "enter-start-with-staged");
    shell_frame("running-gravity", emu_fake_now);
    shell_send(shell_ev(TOY_EV_POINTER_DOWN, 0, 0, 0, 0), "pointer-centre-to-cart");
    shell_frame("running-forwarded", emu_fake_now);
    shell_send(shell_ev(TOY_EV_KEY_DOWN, TOY_KEY_X, 0, 0, 0), "key-x-settings");
    shell_send(shell_ev(TOY_EV_KEY_DOWN, TOY_KEY_RIGHT, 0, 0, 0), "key-right-palette-live");
    shell_frame("settings-live-param", emu_fake_now);
    shell_send(shell_ev(TOY_EV_KEY_DOWN, TOY_KEY_ESC, 0, 0, 0), "esc-resume");
    shell_frame("running-resumed", emu_fake_now);
    shell_send(shell_ev(TOY_EV_KEY_DOWN, TOY_KEY_ESC, 0, 0, 0), "esc-exit");
    shell_frame("gallery-return", emu_fake_now);
    shell_send(shell_ev(TOY_EV_DIAL, 0, 0, 0, (toy_cart_count - emu_sh.selected) * 25), "dial-to-set-clock");
    shell_send(shell_ev(TOY_EV_KEY_DOWN, TOY_KEY_ENTER, 0, 0, 0), "enter-clockset");
    shell_frame("clockset-seeded", emu_fake_now);
    shell_send(shell_ev(TOY_EV_KEY_DOWN, TOY_KEY_RIGHT, 0, 0, 0), "hh+1");
    shell_send(shell_ev(TOY_EV_KEY_DOWN, TOY_KEY_ENTER, 0, 0, 0), "enter-to-mm");
    shell_send(shell_ev(TOY_EV_DIAL, 0, 0, 0, 350), "dial+350-mm");
    shell_send(shell_ev(TOY_EV_KEY_DOWN, TOY_KEY_ENTER, 0, 0, 0), "enter-to-tz");
    shell_send(shell_ev(TOY_EV_DIAL, 0, 0, 0, 180), "dial+180-tz");
    shell_frame("clockset-edited", emu_fake_now);
    shell_send(shell_ev(TOY_EV_KEY_DOWN, TOY_KEY_ENTER, 0, 0, 0), "enter-to-set");
    shell_send(shell_ev(TOY_EV_KEY_DOWN, TOY_KEY_ENTER, 0, 0, 0), "enter-commit");
    shell_frame("clockface-set", emu_fake_now);
    shell_frame("clockface-unset", unset);
    shell_send(shell_ev(TOY_EV_KEY_DOWN, TOY_KEY_DOWN, 0, 0, 0), "key-wake-to-gallery");
    shell_frame("gallery-wake", emu_fake_now);
    emu_sh.idle_limit_frames = 5;
    uint32_t idle_act = TOY_SH_ACT_NONE;
    for (int i = 0; i < 5; i++) idle_act = toy_shell_tick(&emu_sh);
    printf("tick x5 (idle limit 5) ->");
    shell_log_actions(idle_act);
    shell_frame("clockface-idle", emu_fake_now);
    printf("TOY-SHELL-DONE frames=%d size=%d\n", emu_frame_no, size);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc >= 2 && strcmp(argv[1], "list") == 0 && argc == 2) {
        for (int i = 0; i < toy_cart_count; i++)
            printf("%s\n", toy_carts[i]->name);
        return 0;
    }
    if (argc >= 2 && strcmp(argv[1], "proof") == 0 && argc == 3) {
        const toy_cart_t *cart = toy_cart_by_name(argv[2]);
        if (!cart) { fprintf(stderr, "unknown cart %s\n", argv[2]); return 2; }
        check_cart_input(cart);
        toy_sig_source_t *sig = sig_setup(cart);
        void *st = xcalloc(cart->state_size);
        cart->reset(st);
        uint32_t cur = 0;
        toy_apply_proof_at(cart, st, 0, &cur); /* step-0 setup; no feed */
        print_run_line(cart, sig);
        for (uint32_t s = 1; s <= cart->proof_steps; s++) {
            sig_feed_or_die(cart, st, sig); /* feed -> apply -> step */
            toy_apply_proof_at(cart, st, s, &cur);
            cart->step(st);
            if (cart->proof_report_every && (s % cart->proof_report_every) == 0)
                printf("step %u hash %016llx\n", s,
                       (unsigned long long)toy_hash_state(st, cart->state_size));
        }
        printf("TOY-DONE steps %u hash %016llx\n", cart->proof_steps,
               (unsigned long long)toy_hash_state(st, cart->state_size));
        return 0;
    }
    if (argc >= 2 && strcmp(argv[1], "frames") == 0 && argc == 7) {
        const toy_cart_t *cart = toy_cart_by_name(argv[2]);
        if (!cart) { fprintf(stderr, "unknown cart %s\n", argv[2]); return 2; }
        check_cart_input(cart);
        toy_sig_source_t *sig = sig_setup(cart);
        int size = atoi(argv[3]);
        long spf = atol(argv[4]);
        if (spf == 0) spf = (long)cart->demo_steps_per_frame;
        long frames = atol(argv[5]);
        const char *outdir = argv[6];
        void *st = xcalloc(cart->state_size);
        toy_fb_t fb;
        fb.w = fb.h = size;
        fb.px = (uint16_t *)xcalloc((size_t)size * (size_t)size * sizeof(uint16_t));
        cart->reset(st);
        /* Proof entries apply at their absolute steps across frames,
         * so contact sheets show the scripted gesture. */
        uint32_t cur = 0;
        long abs_step = 0;
        toy_apply_proof_at(cart, st, 0, &cur);
        char path[1024];
        for (long fr = 0; fr < frames; fr++) {
            for (long s = 0; s < spf; s++) {
                abs_step++;
                sig_feed_or_die(cart, st, sig); /* feed -> apply -> step */
                toy_apply_proof_at(cart, st, (uint32_t)abs_step, &cur);
                cart->step(st);
            }
            cart->render(st, &fb);
            snprintf(path, sizeof path, "%s/f%03ld.ppm", outdir, fr);
            if (write_ppm(path, &fb) != 0) { fprintf(stderr, "cannot write %s\n", path); return 2; }
            printf("frame %ld hash %016llx\n", fr,
                   (unsigned long long)toy_hash_state(st, cart->state_size));
        }
        return 0;
    }
    if (argc >= 2 && strcmp(argv[1], "run") == 0 && argc == 5) {
        const toy_cart_t *cart = toy_cart_by_name(argv[2]);
        if (!cart) { fprintf(stderr, "unknown cart %s\n", argv[2]); return 2; }
        check_cart_input(cart);
        toy_sig_source_t *sig = sig_setup(cart);
        long total = atol(argv[3]);
        long every = atol(argv[4]);
        void *st = xcalloc(cart->state_size);
        cart->reset(st);
        uint32_t cur = 0;
        toy_apply_proof_at(cart, st, 0, &cur); /* step-0 setup; no feed */
        print_run_line(cart, sig);
        for (long s = 1; s <= total; s++) {
            sig_feed_or_die(cart, st, sig); /* feed -> apply -> step */
            toy_apply_proof_at(cart, st, (uint32_t)s, &cur);
            cart->step(st);
            if (every > 0 && (s % every) == 0)
                printf("step %ld hash %016llx\n", s,
                       (unsigned long long)toy_hash_state(st, cart->state_size));
        }
        printf("TOY-DONE steps %ld hash %016llx\n", total,
               (unsigned long long)toy_hash_state(st, cart->state_size));
        return 0;
    }
    if (argc >= 2 && strcmp(argv[1], "save") == 0 && argc == 5) {
        const toy_cart_t *cart = toy_cart_by_name(argv[2]);
        if (!cart) { fprintf(stderr, "unknown cart %s\n", argv[2]); return 2; }
        /* A signal source is host-owned and its position is not
         * in TOYCKPT1, so an unfed save/resume would silently
         * fork the stream. Refuse loudly until a restore parcel
         * defines fast-forwarding the proof source to the
         * header's step count x samples_per_step. */
        if (cart->feed) { fprintf(stderr, "cart %s consumes a signal feed: checkpoints not supported for feed carts yet\n", cart->name); return 2; }
        unsigned long long steps = strtoull(argv[3], NULL, 10);
        void *st = xcalloc(cart->state_size);
        cart->reset(st);
        for (unsigned long long s = 0; s < steps; s++) cart->step(st);
        size_t img_len = toy_ckpt_image_size(cart);
        if (img_len == 0) { fprintf(stderr, "cart %s cannot be checkpointed\n", cart->name); return 2; }
        unsigned char *img = (unsigned char *)xcalloc(img_len);
        size_t written = 0;
        toy_ckpt_err_t err = toy_ckpt_serialize(cart, st, (uint64_t)steps,
                                                img, img_len, &written);
        if (err != TOY_CKPT_OK) {
            fprintf(stderr, "checkpoint serialise failed: %s\n", toy_ckpt_strerror(err));
            return 2;
        }
        if (write_file_atomic(argv[4], img, written) != 0) {
            fprintf(stderr, "cannot write %s\n", argv[4]);
            return 2;
        }
        printf("TOY-CKPT-SAVED cart=%s steps %llu hash %016llx bytes %zu file %s\n",
               cart->name, steps,
               (unsigned long long)toy_hash_state(st, cart->state_size),
               written, argv[4]);
        return 0;
    }
    if (argc >= 2 && strcmp(argv[1], "load") == 0 && argc == 5) {
        unsigned long long more = strtoull(argv[3], NULL, 10);
        unsigned long long every = strtoull(argv[4], NULL, 10);
        /* One byte past the maximum image so an oversized or
         * trailing-garbage file is seen at its true (wrong) size
         * class rather than silently truncated to a valid one. */
        size_t cap = (size_t)TOY_CKPT_MAX_IMAGE_SIZE + 1u;
        unsigned char *img = (unsigned char *)xcalloc(cap);
        size_t len = 0;
        if (read_file(argv[2], img, cap, &len) != 0) {
            fprintf(stderr, "cannot read %s\n", argv[2]);
            return 2;
        }
        const toy_cart_t *cart = NULL;
        uint64_t base_steps = 0;
        void *st = xcalloc(TOY_MAX_STATE_SIZE);
        toy_ckpt_err_t err = toy_ckpt_load(img, len, NULL, st, TOY_MAX_STATE_SIZE,
                                           &cart, &base_steps);
        if (err != TOY_CKPT_OK) {
            fprintf(stderr, "checkpoint rejected: %s (file left unchanged)\n",
                    toy_ckpt_strerror(err));
            return 2;
        }
        if (cart->feed) { fprintf(stderr, "cart %s consumes a signal feed: checkpoints not supported for feed carts yet (file left unchanged)\n", cart->name); return 2; }
        printf("TOY-RUN cart=%s state_size=%zu hash=fnv1a64\n", cart->name, cart->state_size);
        printf("TOY-CKPT-LOADED cart=%s step %llu hash %016llx file %s\n",
               cart->name, (unsigned long long)base_steps,
               (unsigned long long)toy_hash_state(st, cart->state_size), argv[2]);
        for (unsigned long long s = 1; s <= more; s++) {
            cart->step(st);
            unsigned long long abs_step = (unsigned long long)base_steps + s;
            if (every > 0 && (abs_step % every) == 0)
                printf("step %llu hash %016llx\n", abs_step,
                       (unsigned long long)toy_hash_state(st, cart->state_size));
        }
        printf("TOY-DONE steps %llu hash %016llx\n",
               (unsigned long long)base_steps + more,
               (unsigned long long)toy_hash_state(st, cart->state_size));
        return 0;
    }
    if (argc >= 2 && strcmp(argv[1], "clock") == 0 && argc == 2) {
        toy_time_t now = toy_clock_now();
        toy_schedule_t sched = toy_schedule_default();
        printf("TOY-CLOCK valid=%d", now.valid ? 1 : 0);
        if (now.valid) {
            unsigned m = toy_clock_minutes_of_day(now);
            printf(" utc_seconds=%lld tz_offset_min=%d local %02u:%02u",
                   (long long)now.utc_seconds, (int)now.tz_offset_min,
                   m / 60, m % 60);
        } else {
            printf(" (unset: schedule features off, simulation unaffected;"
                   " set TOY_CLOCK_UTC_SECONDS)");
        }
        printf(" mono_ms=%llu schedule_active=%d live_warp_x100=%u"
               " playback_warp_x100=%u keyframe_interval_s=%u\n",
               (unsigned long long)toy_clock_mono_ms(),
               toy_clock_is_active(now, &sched) ? 1 : 0,
               (unsigned)sched.live_warp_x100,
               (unsigned)sched.playback_warp_x100,
               (unsigned)sched.keyframe_interval_s);
        return 0;
    }
    if (argc >= 2 && strcmp(argv[1], "bank") == 0 && (argc == 5 || argc == 6)) {
        const toy_cart_t *cart = toy_cart_by_name(argv[2]);
        if (!cart) { fprintf(stderr, "unknown cart %s\n", argv[2]); return 2; }
        check_cart_input(cart);
        unsigned long long total = strtoull(argv[3], NULL, 10);
        unsigned long every_ul = strtoul(argv[4], NULL, 10);
        if (every_ul > 0xfffffffful) {
            fprintf(stderr, "keyframe-every too large\n");
            return 2;
        }
        uint32_t every = (uint32_t)every_ul;
        char default_path[128];
        const char *path = argv[5];
        if (argc == 5) {
            snprintf(default_path, sizeof default_path, "%s.bank", cart->name);
            path = default_path;
        }
        uint32_t count = toy_bank_keyframe_count((uint64_t)total, every);
        size_t est = toy_bank_estimate_size(cart, count);
        if (est == 0) {
            fprintf(stderr,
                    "bank exceeds fixed budget: cart=%s steps=%llu every=%u"
                    " needs %u keyframes, budget %u bytes allows %u;"
                    " increase keyframe-every\n",
                    cart->name, total, every, count,
                    (unsigned)TOY_BANK_BUDGET_BYTES,
                    toy_bank_max_keyframes(cart));
            return 2;
        }
        unsigned char *buf = (unsigned char *)xcalloc(est);
        void *st = xcalloc(cart->state_size);
        toy_time_t wall0 = toy_clock_now();
        toy_bank_writer_t w;
        toy_bank_err_t err = toy_bank_begin(&w, cart, (uint64_t)total, every,
                                            wall0.utc_seconds,
                                            wall0.valid ? 1u : 0u,
                                            wall0.tz_offset_min, buf, est);
        if (err != TOY_BANK_OK) {
            fprintf(stderr, "bank begin failed: %s\n", toy_bank_strerror(err));
            return 2;
        }
        cart->reset(st);
        uint32_t cur = 0;
        toy_apply_proof_at(cart, st, 0, &cur); /* step-0 setup */
        uint32_t next_kf = 0;
        for (unsigned long long s = 0;; s++) {
            if (next_kf < count &&
                (uint64_t)s == toy_bank_keyframe_step((uint64_t)total, every,
                                                      next_kf)) {
                toy_time_t wl = toy_clock_now();
                err = toy_bank_add_keyframe(&w, st, (uint64_t)s, wl.utc_seconds);
                if (err != TOY_BANK_OK) {
                    fprintf(stderr, "bank keyframe failed: %s\n",
                            toy_bank_strerror(err));
                    return 2;
                }
                printf("keyframe %u step %llu hash %016llx\n", next_kf, s,
                       (unsigned long long)toy_hash_state(st, cart->state_size));
                next_kf++;
            }
            if (s >= total) break;
            toy_apply_proof_at(cart, st, (uint32_t)(s + 1), &cur);
            cart->step(st);
        }
        toy_time_t wall1 = toy_clock_now();
        size_t bank_len = 0;
        err = toy_bank_finish(&w, wall1.utc_seconds, TOY_BANK_END_COMPLETE,
                              &bank_len);
        if (err != TOY_BANK_OK) {
            fprintf(stderr, "bank finish failed: %s\n", toy_bank_strerror(err));
            return 2;
        }
        if (write_file_atomic(path, buf, bank_len) != 0) {
            fprintf(stderr, "cannot write %s\n", path);
            return 2;
        }
        printf("TOY-BANK-SAVED cart=%s steps %llu every %u keyframes %u"
               " hash %016llx bytes %zu wall_valid=%d file %s\n",
               cart->name, total, every, count,
               (unsigned long long)toy_hash_state(st, cart->state_size),
               bank_len, wall0.valid ? 1 : 0, path);
        return 0;
    }
    if (argc >= 2 && strcmp(argv[1], "replay") == 0 && argc == 3) {
        size_t len = 0;
        unsigned char *buf = read_bank_file(argv[2], &len);
        if (!buf) { fprintf(stderr, "cannot read %s\n", argv[2]); return 2; }
        toy_bank_info_t info;
        const toy_cart_t *cart = NULL;
        toy_bank_err_t err = toy_bank_parse(buf, len, NULL, &info, &cart);
        if (err != TOY_BANK_OK) {
            fprintf(stderr, "TOY-REPLAY-REJECTED file %s: %s\n",
                    argv[2], toy_bank_strerror(err));
            return 2;
        }
        printf("TOY-REPLAY cart=%s steps %llu every %u keyframes %u file %s\n",
               cart->name, (unsigned long long)info.total_steps,
               info.keyframe_every, info.keyframe_count, argv[2]);
        for (uint32_t i = 0; i < info.keyframe_count; i++) {
            toy_bank_keyframe_t kf;
            toy_bank_keyframe(buf, len, &info, i, &kf);
            printf("keyframe %u step %llu hash %016llx (stored)\n", i,
                   (unsigned long long)kf.step_index,
                   (unsigned long long)kf.state_hash);
        }
        void *scratch = xcalloc(TOY_MAX_STATE_SIZE);
        uint32_t verified = 0, fail_idx = 0;
        uint64_t fail_step = 0, want = 0, got = 0;
        err = toy_bank_verify_replay(buf, len, scratch, TOY_MAX_STATE_SIZE,
                                     &verified, &fail_idx, &fail_step,
                                     &want, &got);
        if (err != TOY_BANK_OK) {
            fprintf(stderr,
                    "TOY-REPLAY-FAILED file %s: %s at keyframe %u step %llu"
                    " want %016llx got %016llx (verified %u before failure)\n",
                    argv[2], toy_bank_strerror(err), fail_idx,
                    (unsigned long long)fail_step,
                    (unsigned long long)want, (unsigned long long)got,
                    verified);
            return 2;
        }
        printf("TOY-REPLAY-VERIFIED keyframes %u final step %llu hash %016llx\n",
               verified, (unsigned long long)info.total_steps,
               (unsigned long long)toy_hash_state(scratch, cart->state_size));
        toy_morning_report_t rep;
        err = toy_bank_morning_report(buf, len, 1u, &rep, NULL);
        if (err != TOY_BANK_OK) {
            fprintf(stderr, "morning report failed: %s\n",
                    toy_bank_strerror(err));
            return 2;
        }
        print_morning_report(&rep);
        return 0;
    }
    if (argc >= 2 && strcmp(argv[1], "report") == 0 && argc == 3) {
        size_t len = 0;
        unsigned char *buf = read_bank_file(argv[2], &len);
        if (!buf) { fprintf(stderr, "cannot read %s\n", argv[2]); return 2; }
        toy_morning_report_t rep;
        toy_bank_err_t err = toy_bank_morning_report(buf, len, 0u, &rep, NULL);
        if (err != TOY_BANK_OK) {
            fprintf(stderr, "TOY-REPORT-REJECTED file %s: %s\n",
                    argv[2], toy_bank_strerror(err));
            return 2;
        }
        print_morning_report(&rep);
        return 0;
    }
    if (argc >= 2 && strcmp(argv[1], "session") == 0 && argc == 3)
        return run_session(argv[2]);
    if (argc >= 2 && strcmp(argv[1], "feedrun") == 0 && argc == 5)
        return run_feedrun(argv[2], argv[3], argv[4]);
    if (argc >= 2 && strcmp(argv[1], "tsigout") == 0 && argc == 5)
        return run_tsigout(argv[2], argv[3], argv[4]);
    if (argc >= 2 && strcmp(argv[1], "shell") == 0 && argc == 4)
        return run_shell_tour(atoi(argv[2]), argv[3]);
    fprintf(stderr,
        "usage:\n"
        "  toy_emu list\n"
        "  toy_emu proof <cart>\n"
        "  toy_emu frames <cart> <size> <steps-per-frame|0=demo> <frames> <outdir>\n"
        "  toy_emu run <cart> <total-steps> <report-every>\n"
        "  toy_emu save <cart> <steps> <file>\n"
        "  toy_emu load <file> <more-steps> <report-every>\n"
        "  toy_emu bank <cart> <steps> <keyframe-every> [file]\n"
        "  toy_emu replay <file>\n"
        "  toy_emu report <file>\n"
        "  toy_emu session <file>\n"
        "  toy_emu feedrun <cart> <file.tsig> <steps>\n"
        "  toy_emu tsigout <cart> <steps> <file>\n"
        "  toy_emu shell <size> <outdir>\n"
        "  toy_emu clock\n"
        "carts:");
    for (int i = 0; i < toy_cart_count; i++) fprintf(stderr, " %s", toy_carts[i]->name);
    fprintf(stderr, "\n");
    return 2;
}
