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
    fprintf(stderr,
        "usage:\n"
        "  toy_emu list\n"
        "  toy_emu proof <cart>\n"
        "  toy_emu frames <cart> <size> <steps-per-frame|0=demo> <frames> <outdir>\n"
        "  toy_emu run <cart> <total-steps> <report-every>\n"
        "carts:");
    for (int i = 0; i < toy_cart_count; i++) fprintf(stderr, " %s", toy_carts[i]->name);
    fprintf(stderr, "\n");
    return 2;
}
