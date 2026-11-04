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
 */
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
 * live events would enter through the shared toy_queue_t helpers. */
static void check_cart_input(const toy_cart_t *cart)
{
    int rc = toy_cart_check_input(cart);
    if (rc) { fprintf(stderr, "bad input descriptor for %s (code %d)\n", cart->name, rc); exit(2); }
}

static void print_run_line(const toy_cart_t *cart)
{
    printf("TOY-RUN cart=%s state_size=%zu hash=fnv1a64", cart->name, cart->state_size);
    if (cart->proof_input_count)
        printf(" inputs=%u input_hash=%016llx", cart->proof_input_count,
               (unsigned long long)toy_hash_inputs(cart->proof_inputs,
                                                   cart->proof_input_count));
    printf("\n");
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
        void *st = xcalloc(cart->state_size);
        cart->reset(st);
        uint32_t cur = 0;
        toy_apply_proof_at(cart, st, 0, &cur); /* step-0 setup */
        print_run_line(cart);
        for (uint32_t s = 1; s <= cart->proof_steps; s++) {
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
        long total = atol(argv[3]);
        long every = atol(argv[4]);
        void *st = xcalloc(cart->state_size);
        cart->reset(st);
        uint32_t cur = 0;
        toy_apply_proof_at(cart, st, 0, &cur); /* step-0 setup */
        print_run_line(cart);
        for (long s = 1; s <= total; s++) {
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
        "  toy_emu clock\n"
        "carts:");
    for (int i = 0; i < toy_cart_count; i++) fprintf(stderr, " %s", toy_carts[i]->name);
    fprintf(stderr, "\n");
    return 2;
}
