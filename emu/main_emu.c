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
        void *st = xcalloc(cart->state_size);
        cart->reset(st);
        printf("TOY-RUN cart=%s state_size=%zu hash=fnv1a64\n", cart->name, cart->state_size);
        for (uint32_t s = 1; s <= cart->proof_steps; s++) {
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
        char path[1024];
        for (long fr = 0; fr < frames; fr++) {
            for (long s = 0; s < spf; s++) cart->step(st);
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
        long total = atol(argv[3]);
        long every = atol(argv[4]);
        void *st = xcalloc(cart->state_size);
        cart->reset(st);
        printf("TOY-RUN cart=%s state_size=%zu hash=fnv1a64\n", cart->name, cart->state_size);
        for (long s = 1; s <= total; s++) {
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
