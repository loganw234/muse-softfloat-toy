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
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "toy_cart.h"
#include "toy_ckpt.h"

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
    fprintf(stderr,
        "usage:\n"
        "  toy_emu list\n"
        "  toy_emu proof <cart>\n"
        "  toy_emu frames <cart> <size> <steps-per-frame|0=demo> <frames> <outdir>\n"
        "  toy_emu run <cart> <total-steps> <report-every>\n"
        "  toy_emu save <cart> <steps> <file>\n"
        "  toy_emu load <file> <more-steps> <report-every>\n"
        "carts:");
    for (int i = 0; i < toy_cart_count; i++) fprintf(stderr, " %s", toy_carts[i]->name);
    fprintf(stderr, "\n");
    return 2;
}
