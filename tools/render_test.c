/* render_test.c - pixel-identity harness for render-only changes.
 *
 * Own main, NOT wired into tools/run.sh. Steps solclock, pond and
 * attitude through their proof scripts (feed -> proof-apply ->
 * step, the emu proof order) and a no-input stretch, rendering at
 * 240, 360 and 466 into a framebuffer snapshot every SNAP_STRIDE
 * steps, FNV-1a 64 over the raw fb bytes. The per-(cart, run,
 * size) rollup + first/last snapshot hashes + end state hash are
 * pinned in PINNED below (recorded from the unmodified renderers
 * with `record`); the default mode fails loud on any drift, which
 * is the pixel-identical contract for a render-only parcel.
 *
 * A reset-invalidation pass renders sequence A, resets the SAME
 * buffer, runs a different sequence B (different params/inputs),
 * and requires the pixels to match a fresh buffer that only ever
 * ran B: no render cache may leak across reset/cart-switch.
 *
 * `bench` times render at 240/360/466 on representative
 * mid-proof states (host ms/frame; host has hardware double, so
 * treat board direction via the op counts in the parcel report).
 *
 * Build (from repo root, a strict-binary64 host compiler -
 * x86-64, or i686 with -mfpmath=sse -msse2; renderers convert
 * state to native double for display, and under x87 excess
 * precision solclock's pixels are not binary64 pixels, in the
 * unmodified code either):
 *   gcc -O2 -std=c11 -Wall -Wextra -Werror \
 *       -Ilibs/muse-softfloat/include -Icore \
 *       tools/render_test.c <all core and lib .c files> \
 *       -o build/render_test
 * Run: build/render_test [check|record|bench]
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "toy_cart.h"

#define SNAP_STRIDE 40u
#define NOINPUT_STEPS 240u
#define FB_MAX 466

static _Alignas(16) unsigned char state_a[TOY_MAX_STATE_SIZE];
static _Alignas(16) unsigned char state_b[TOY_MAX_STATE_SIZE];
static uint16_t fb_px[FB_MAX * FB_MAX];
static toy_sig_source_t sig_src;
static int16_t sig_frame[TOY_SIG_MAX_FRAME_VALUES];
static int failures;

static uint64_t fnv1a(const void *p, size_t n, uint64_t h)
{
    const unsigned char *b = (const unsigned char *)p;
    for (size_t i = 0; i < n; i++) {
        h ^= b[i];
        h *= UINT64_C(1099511628211);
    }
    return h;
}
#define FNV_SEED UINT64_C(14695981039346656037)

static uint64_t snap_hash(const toy_cart_t *cart, const void *st, int size)
{
    toy_fb_t fb = { size, size, fb_px };
    memset(fb_px, 0xA5, (size_t)size * (size_t)size * sizeof fb_px[0]);
    cart->render(st, &fb);
    return fnv1a(fb_px, (size_t)size * (size_t)size * sizeof fb_px[0],
                 FNV_SEED);
}

typedef struct {
    const char *cart;
    char run; /* 'P' proof, 'N' no-input */
    int size;
    uint32_t nsnap;
    uint64_t first, last, rollup, state_end;
} pin_t;

/* Recorded by `record` on the UNMODIFIED renderers (base
 * 54c83ce) before the toy5-render optimization; see parcel
 * report scratch/toy2/toy5-render.md for the full SNAP list
 * method (456 snapshots behind these 24 rollups). */
static const pin_t PINNED[] = {
    { "solclock", 'P', 240, 37u, UINT64_C(0xf50ba52c20383f1d), UINT64_C(0xf6bdf99ced7aa7a6), UINT64_C(0xc1a2075a20e3b2a5), UINT64_C(0xae7610758e99119e) },
    { "solclock", 'P', 360, 37u, UINT64_C(0x3a55c9211500e88c), UINT64_C(0xfda40c43a836172c), UINT64_C(0x2b57b33c7f2ee01d), UINT64_C(0xae7610758e99119e) },
    { "solclock", 'P', 466, 37u, UINT64_C(0x6911545e00495a7b), UINT64_C(0x19eccec421da2109), UINT64_C(0x68dbfec22e5123c7), UINT64_C(0xae7610758e99119e) },
    { "solclock", 'N', 240, 7u, UINT64_C(0xf50ba52c20383f1d), UINT64_C(0x3e1f26975d02de45), UINT64_C(0x3183d47c5ce28420), UINT64_C(0x25a203c2874c111c) },
    { "solclock", 'N', 360, 7u, UINT64_C(0x3a55c9211500e88c), UINT64_C(0x4e549cfde76ded88), UINT64_C(0x1dfec69b09522c5f), UINT64_C(0x25a203c2874c111c) },
    { "solclock", 'N', 466, 7u, UINT64_C(0x6911545e00495a7b), UINT64_C(0x651303592802a174), UINT64_C(0xa301a97c933f8aa5), UINT64_C(0x25a203c2874c111c) },
    { "pond", 'P', 240, 11u, UINT64_C(0x56742105423cb095), UINT64_C(0xb8a9873d65e53cec), UINT64_C(0x403947c33f334ccd), UINT64_C(0xd38a09749ff1eec4) },
    { "pond", 'P', 360, 11u, UINT64_C(0x7b158c3c268d6a8d), UINT64_C(0x4a3b7349c63e0b0e), UINT64_C(0xc0cfbe9ea4ccb988), UINT64_C(0xd38a09749ff1eec4) },
    { "pond", 'P', 466, 11u, UINT64_C(0x8aac5e1103b8e679), UINT64_C(0x28e830963eac57e0), UINT64_C(0x2a596a911852c093), UINT64_C(0xd38a09749ff1eec4) },
    { "pond", 'N', 240, 7u, UINT64_C(0x56742105423cb095), UINT64_C(0xd1d3dfcb0b39668d), UINT64_C(0x2e786bd32fd9b316), UINT64_C(0xc28b4efee9b30704) },
    { "pond", 'N', 360, 7u, UINT64_C(0x7b158c3c268d6a8d), UINT64_C(0x1840544498d72314), UINT64_C(0x858280732f6e0969), UINT64_C(0xc28b4efee9b30704) },
    { "pond", 'N', 466, 7u, UINT64_C(0x8aac5e1103b8e679), UINT64_C(0x14054415abac77d3), UINT64_C(0x3bc20254ec2bad83), UINT64_C(0xc28b4efee9b30704) },
    { "attitude", 'P', 240, 64u, UINT64_C(0x2a84ed79de4cce57), UINT64_C(0x9484114498374730), UINT64_C(0x79bb0ce61d3177cb), UINT64_C(0xbd0ec1ae39738955) },
    { "attitude", 'P', 360, 64u, UINT64_C(0xcc9156a65f0dfa45), UINT64_C(0x460e00ef4b9236eb), UINT64_C(0xb0158174150093ad), UINT64_C(0xbd0ec1ae39738955) },
    { "attitude", 'P', 466, 64u, UINT64_C(0xd49ca9a90ba84633), UINT64_C(0x08d1bd493983c1d6), UINT64_C(0xff9638b81f727f24), UINT64_C(0xbd0ec1ae39738955) },
    { "attitude", 'N', 240, 7u, UINT64_C(0x2a84ed79de4cce57), UINT64_C(0x2a84ed79de4cce57), UINT64_C(0xa5c10c3e93af5d86), UINT64_C(0xd88e88a16aa8bfa1) },
    { "attitude", 'N', 360, 7u, UINT64_C(0xcc9156a65f0dfa45), UINT64_C(0xcc9156a65f0dfa45), UINT64_C(0xab3bf997ad3e439f), UINT64_C(0xd88e88a16aa8bfa1) },
    { "attitude", 'N', 466, 7u, UINT64_C(0xd49ca9a90ba84633), UINT64_C(0xd49ca9a90ba84633), UINT64_C(0xb2e16667fc71ae5f), UINT64_C(0xd88e88a16aa8bfa1) },
    { "solclock", 'S', 240, 12u, UINT64_C(0x0121ad24d4ad80fa), UINT64_C(0x8c3201206ad745bb), UINT64_C(0x73c30a48218b451e), UINT64_C(0x082ae46c9541ff66) },
    { "solclock", 'S', 360, 12u, UINT64_C(0x8adbe0c5e01a920e), UINT64_C(0x35e1a30bb065a52b), UINT64_C(0x79609a9d938fca77), UINT64_C(0x082ae46c9541ff66) },
    { "solclock", 'S', 466, 12u, UINT64_C(0xfd5f96d3870ac0c3), UINT64_C(0x9ef5c95d19ed2f31), UINT64_C(0x4e62e4b784832b23), UINT64_C(0x082ae46c9541ff66) },
    { "attitude", 'L', 240, 7u, UINT64_C(0x56f9f2beb235cd08), UINT64_C(0x3eb7e95d005887c3), UINT64_C(0x9f82236cb39342c7), UINT64_C(0xbfa82af7c002d873) },
    { "attitude", 'L', 360, 7u, UINT64_C(0x0dbccbe21ea9e90a), UINT64_C(0x7452385428afd686), UINT64_C(0x7c1aa64d61198d74), UINT64_C(0xbfa82af7c002d873) },
    { "attitude", 'L', 466, 7u, UINT64_C(0xc2542163e846f71d), UINT64_C(0x1115af43f679bfd4), UINT64_C(0x747c02f11b1624c4), UINT64_C(0xbfa82af7c002d873) },
};

static toy_sig_source_t *sig_setup(const toy_cart_t *cart)
{
    int created = toy_sig_proof_source_init(cart, &sig_src);
    if (created < 0) {
        fprintf(stderr, "cannot instantiate proof signal for %s\n",
                cart->name);
        exit(2);
    }
    return created ? &sig_src : NULL;
}

static void feed_or_die(const toy_cart_t *cart, void *st,
                        toy_sig_source_t *src)
{
    int rc = toy_feed_step(cart, st, src, sig_frame,
                           TOY_SIG_MAX_FRAME_VALUES);
    if (rc) {
        fprintf(stderr, "feed failed for %s (code %d)\n", cart->name, rc);
        exit(2);
    }
}

/* Accumulate one snapshot step (all three sizes) into acc[3]. */
static pin_t acc[3];
static void take_snapshot(const toy_cart_t *cart, const void *st,
                          char run, uint32_t step, int record)
{
    static const int sizes[3] = { 240, 360, 466 };
    for (int i = 0; i < 3; i++) {
        uint64_t h = snap_hash(cart, st, sizes[i]);
        if (record)
            printf("SNAP %s %c step %u size %d hash %016llx\n",
                   cart->name, run, step, sizes[i],
                   (unsigned long long)h);
        acc[i].cart = cart->name;
        acc[i].run = run;
        acc[i].size = sizes[i];
        if (acc[i].nsnap == 0) {
            acc[i].first = h;
            acc[i].rollup = FNV_SEED;
        }
        acc[i].rollup = fnv1a(&step, sizeof step, acc[i].rollup);
        acc[i].rollup = fnv1a(&h, sizeof h, acc[i].rollup);
        acc[i].last = h;
        acc[i].nsnap++;
    }
}

static void finish_run(const toy_cart_t *cart, const void *st, int record)
{
    uint64_t se = toy_hash_state(st, cart->state_size);
    for (int i = 0; i < 3; i++) {
        acc[i].state_end = se;
        if (record) {
            printf("    { \"%s\", '%c', %d, %uu, UINT64_C(0x%016llx), "
                   "UINT64_C(0x%016llx), UINT64_C(0x%016llx), "
                   "UINT64_C(0x%016llx) },\n",
                   acc[i].cart, acc[i].run, acc[i].size, acc[i].nsnap,
                   (unsigned long long)acc[i].first,
                   (unsigned long long)acc[i].last,
                   (unsigned long long)acc[i].rollup,
                   (unsigned long long)acc[i].state_end);
        } else {
            const pin_t *p = NULL;
            for (size_t k = 0; k < sizeof PINNED / sizeof PINNED[0]; k++)
                if (PINNED[k].cart && !strcmp(PINNED[k].cart, acc[i].cart) &&
                    PINNED[k].run == acc[i].run &&
                    PINNED[k].size == acc[i].size)
                    p = &PINNED[k];
            int ok = p && p->nsnap == acc[i].nsnap &&
                     p->first == acc[i].first && p->last == acc[i].last &&
                     p->rollup == acc[i].rollup &&
                     p->state_end == acc[i].state_end;
            printf("%s %s %c size %d snaps %u rollup %016llx state %016llx\n",
                   ok ? "PASS" : "FAIL", acc[i].cart, acc[i].run,
                   acc[i].size, acc[i].nsnap,
                   (unsigned long long)acc[i].rollup,
                   (unsigned long long)acc[i].state_end);
            if (!ok) failures++;
        }
    }
    memset(acc, 0, sizeof acc);
}

static void run_proof(const toy_cart_t *cart, void *st, int record)
{
    toy_sig_source_t *sig = sig_setup(cart);
    memset(st, 0, cart->state_size);
    cart->reset(st);
    uint32_t cur = 0;
    toy_apply_proof_at(cart, st, 0, &cur);
    take_snapshot(cart, st, 'P', 0, record);
    for (uint32_t s = 1; s <= cart->proof_steps; s++) {
        feed_or_die(cart, st, sig);
        toy_apply_proof_at(cart, st, s, &cur);
        cart->step(st);
        if (s % SNAP_STRIDE == 0 || s == cart->proof_steps)
            take_snapshot(cart, st, 'P', s, record);
    }
    finish_run(cart, st, record);
}

static void run_noinput(const toy_cart_t *cart, void *st, int record);

/* solclock param sweep 'S': 12 spread dates through the step-0
 * SET_PARAM path, so moon phase (sunx/sunz full circle), EoT sign
 * and declination extremes all land in snapshots. */
static void apply_ev(const toy_cart_t *cart, void *st, uint32_t type,
                     uint32_t id, int32_t x, int32_t y, int32_t v);
static void run_sweep_sc(const toy_cart_t *cart, void *st, int record)
{
    memset(st, 0, cart->state_size);
    cart->reset(st);
    for (uint32_t k = 0; k < 12; k++) {
        apply_ev(cart, st, TOY_EV_SET_PARAM, 0, 0, 0, 2025 + (int32_t)(k % 3));
        apply_ev(cart, st, TOY_EV_SET_PARAM, 1, 0, 0, 1 + (int32_t)k);
        apply_ev(cart, st, TOY_EV_SET_PARAM, 2, 0, 0,
                 1 + (int32_t)((k * 7) % 28));
        apply_ev(cart, st, TOY_EV_SET_PARAM, 3, 0, 0,
                 (int32_t)((k * 5) % 24));
        apply_ev(cart, st, TOY_EV_SET_PARAM, 4, 0, 0,
                 (int32_t)((k * 23) % 60));
        apply_ev(cart, st, TOY_EV_SET_PARAM, 5, 0, 0,
                 -720 + (int32_t)k * 120);
        for (int s = 0; s < 37; s++) cart->step(st);
        take_snapshot(cart, st, 'S', k, record);
    }
    finish_run(cart, st, record);
}

/* attitude crafted states 'L': level, axis-aligned splits
 * (ux == 0 / uy == 0), diagonal, just-above-threshold and a
 * negative-quadrant tilt, fed directly (first sample loads the
 * EMA outright, so the state tilt direction is exact). */
static void run_crafted_at(const toy_cart_t *cart, void *st, int record)
{
    static const int16_t frames[][3] = {
        {     0,     0, 16384 }, /* level: tilt 0 */
        {     0,  8192, 14189 }, /* ux == 0, down = screen up */
        { -8192,     0, 14189 }, /* uy == 0, down = screen right */
        { -2896,  2896, 15800 }, /* diagonal */
        {  -400,   300, 16370 }, /* tilt ~0.031 g, just split */
        {  5000, -6000, 14000 }, /* negative quadrant */
        {  8192,  8192, 11585 }, /* down past the 0.50 g rim dir */
    };
    (void)sig_setup(cart);
    for (uint32_t k = 0; k < sizeof frames / sizeof frames[0]; k++) {
        int16_t fr[6] = { frames[k][0], frames[k][1], frames[k][2],
                          0, 0, 0 };
        memset(st, 0, cart->state_size);
        cart->reset(st);
        /* 60 identical fed frames: EMA settled, trail populated */
        for (int s = 0; s < 60; s++) {
            cart->feed(st, fr, 1, 6, &sig_src.info, 0);
            cart->step(st);
        }
        take_snapshot(cart, st, 'L', k, record);
    }
    finish_run(cart, st, record);
}

/* --- reset invalidation: seq A primes any cache, reset, seq B on
 * the same buffer must pixel-match seq B on a fresh buffer. --- */
static void apply_ev(const toy_cart_t *cart, void *st, uint32_t type,
                     uint32_t id, int32_t x, int32_t y, int32_t v)
{
    toy_event_t ev = { 0 };
    ev.type = type; ev.id = id; ev.x = x; ev.y = y; ev.v = v;
    if (cart->apply) cart->apply(st, &ev);
}

static void steps(const toy_cart_t *cart, void *st, toy_sig_source_t *sig,
                  uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) {
        feed_or_die(cart, st, sig);
        cart->step(st);
    }
}

/* Sequence B per cart (the post-reset identity that must match). */
static void seq_b(const toy_cart_t *cart, void *st)
{
    toy_sig_source_t *sig = sig_setup(cart);
    memset(st, 0, cart->state_size);
    cart->reset(st);
    if (!strcmp(cart->name, "solclock")) {
        /* different params via the step-0 SET_PARAM path */
        apply_ev(cart, st, TOY_EV_SET_PARAM, 0, 0, 0, 2031);
        apply_ev(cart, st, TOY_EV_SET_PARAM, 1, 0, 0, 6);
        apply_ev(cart, st, TOY_EV_SET_PARAM, 2, 0, 0, 21);
        apply_ev(cart, st, TOY_EV_SET_PARAM, 3, 0, 0, 3);
        apply_ev(cart, st, TOY_EV_SET_PARAM, 4, 0, 0, 45);
        apply_ev(cart, st, TOY_EV_SET_PARAM, 5, 0, 0, 330);
        steps(cart, st, sig, 500);
    } else if (!strcmp(cart->name, "pond")) {
        apply_ev(cart, st, TOY_EV_TILT, 0, 300, 400, 0);
        steps(cart, st, sig, 50);
        apply_ev(cart, st, TOY_EV_POINTER_DOWN, 0, -333, 0, 0);
        steps(cart, st, sig, 80);
    } else { /* attitude: fed stream from a fresh source */
        steps(cart, st, sig, 400);
    }
}

static void run_invalidation(const toy_cart_t *cart)
{
    static const int sizes[2] = { 240, 360 };
    /* fresh buffer: B only */
    seq_b(cart, state_b);
    /* reused buffer: A (defaults/proof head, rendered), reset, B */
    toy_sig_source_t *sig = sig_setup(cart);
    memset(state_a, 0, cart->state_size);
    cart->reset(state_a);
    uint32_t cur = 0;
    toy_apply_proof_at(cart, state_a, 0, &cur);
    for (uint32_t s = 1; s <= 120; s++) {
        feed_or_die(cart, state_a, sig);
        toy_apply_proof_at(cart, state_a, s, &cur);
        cart->step(state_a);
    }
    (void)snap_hash(cart, state_a, 360); /* prime any render cache */
    seq_b(cart, state_a);
    for (int i = 0; i < 2; i++) {
        uint64_t ha = snap_hash(cart, state_a, sizes[i]);
        uint64_t hb = snap_hash(cart, state_b, sizes[i]);
        int ok = ha == hb &&
                 !memcmp(state_a, state_b, cart->state_size);
        printf("%s %s reset-invalidation size %d hash %016llx\n",
               ok ? "PASS" : "FAIL", cart->name, sizes[i],
               (unsigned long long)ha);
        if (!ok) failures++;
    }
}

/* --- bench: representative mid-proof states, host ms/frame --- */
static volatile unsigned bench_sink;
static void bench_one(const toy_cart_t *cart, const void *st,
                      const char *tag)
{
    static const int sizes[3] = { 240, 360, 466 };
    const int reps = 300;
    for (int i = 0; i < 3; i++) {
        toy_fb_t fb = { sizes[i], sizes[i], fb_px };
        clock_t t0 = clock();
        for (int r = 0; r < reps; r++) {
            cart->render(st, &fb);
            bench_sink += fb_px[(size_t)r % ((size_t)sizes[i] * sizes[i])];
        }
        double ms = 1000.0 * (double)(clock() - t0) /
                    ((double)CLOCKS_PER_SEC * reps);
        printf("BENCH %s %s size %d %.4f ms/frame (host, %d reps)\n",
               cart->name, tag, sizes[i], ms, reps);
    }
}

static void bench_cart(const toy_cart_t *cart, uint32_t to_step)
{
    toy_sig_source_t *sig = sig_setup(cart);
    memset(state_a, 0, cart->state_size);
    cart->reset(state_a);
    uint32_t cur = 0;
    toy_apply_proof_at(cart, state_a, 0, &cur);
    if (to_step == 0) {
        bench_one(cart, state_a, "reset-state");
        return;
    }
    for (uint32_t s = 1; s <= to_step; s++) {
        feed_or_die(cart, state_a, sig);
        toy_apply_proof_at(cart, state_a, s, &cur);
        cart->step(state_a);
    }
    bench_one(cart, state_a, "mid-proof");
}

int main(int argc, char **argv)
{
    const char *mode = argc > 1 ? argv[1] : "check";
    const toy_cart_t *carts[3];
    carts[0] = toy_cart_by_name("solclock");
    carts[1] = toy_cart_by_name("pond");
    carts[2] = toy_cart_by_name("attitude");
    for (int i = 0; i < 3; i++)
        if (!carts[i]) { fprintf(stderr, "cart missing\n"); return 2; }
    if (!strcmp(mode, "bench")) {
        bench_cart(carts[0], 720);
        bench_cart(carts[1], 200);
        bench_cart(carts[2], 1250);
        bench_cart(carts[2], 0); /* NO SIGNAL state */
        printf("bench sink %u\n", bench_sink);
        return 0;
    }
    int record = !strcmp(mode, "record");
    if (record)
        printf("static const pin_t PINNED[] = {\n");
    for (int i = 0; i < 3; i++) {
        run_proof(carts[i], state_a, record);
        run_noinput(carts[i], state_a, record);
    }
    run_sweep_sc(carts[0], state_a, record);
    run_crafted_at(carts[2], state_a, record);
    if (record) {
        printf("};\n");
        return 0;
    }
    for (int i = 0; i < 3; i++)
        run_invalidation(carts[i]);
    printf("render_test: %s (%d failure(s))\n",
           failures ? "FAIL" : "PASS", failures);
    return failures ? 1 : 0;
}

static void run_noinput(const toy_cart_t *cart, void *st, int record)
{
    memset(st, 0, cart->state_size);
    cart->reset(st);
    take_snapshot(cart, st, 'N', 0, record);
    for (uint32_t s = 1; s <= NOINPUT_STEPS; s++) {
        cart->step(st);
        if (s % SNAP_STRIDE == 0 || s == NOINPUT_STEPS)
            take_snapshot(cart, st, 'N', s, record);
    }
    finish_run(cart, st, record);
}
