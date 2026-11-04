/* main_emu.c - host screen emulator and hash runner.
 *
 *   toy_emu frames <cart> <size> <steps-per-frame> <frames> <outdir>
 *       Render frames to <outdir>/fNNN.ppm (circle-masked, as the
 *       round glass presents it) and print per-frame state hashes.
 *   toy_emu run <cart> <total-steps> <report-every>
 *       No rendering: print step/hash lines. This is the stream the
 *       board firmware must reproduce exactly.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "toy_cart.h"

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

int main(int argc, char **argv)
{
    if (argc >= 2 && strcmp(argv[1], "frames") == 0 && argc == 7) {
        const toy_cart_t *cart = toy_cart_by_name(argv[2]);
        if (!cart) { fprintf(stderr, "unknown cart %s\n", argv[2]); return 2; }
        int size = atoi(argv[3]);
        long spf = atol(argv[4]);
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
    fprintf(stderr,
        "usage:\n"
        "  toy_emu frames <cart> <size> <steps-per-frame> <frames> <outdir>\n"
        "  toy_emu run <cart> <total-steps> <report-every>\n"
        "carts:");
    for (int i = 0; i < toy_cart_count; i++) fprintf(stderr, " %s", toy_carts[i]->name);
    fprintf(stderr, "\n");
    return 2;
}
