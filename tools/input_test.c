/* input_test.c - standalone proof that the input seam works.
 *
 * Own main, NOT wired into tools/run.sh. Defines a minimal
 * listening test cartridge in this file only (never registered)
 * and checks, with printed hashes:
 *   1. deaf-cart path untouched (helpers are no-ops on fig8)
 *   2. scripted proof_inputs change state deterministically
 *      (hash with script != without, twice-identical)
 *   3. SET_PARAM affects step maths via stored settings
 *   4. dial/touch events quantise to step boundaries
 *   5. over-cap drain CARRIES to the next boundary in order
 *   6. queue coalescing + descriptor validation
 *
 * Build (from repo root, Git Bash):
 *   gcc -O2 -std=c11 -Wall -Wextra -Ilibs/muse-softfloat/include
 *     -Icore tools/input_test.c <all core and lib .c files>
 *     -o build/input_test
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "toy_cart.h"
#include "toy_math.h"
#include "toy_util.h"

static int failures;
static void expect(int ok, const char *what)
{
    printf("%s %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) failures++;
}

/* --- minimal listening test cart (this file only) --- */

typedef struct {
    f64 acc;          /* rate integrated by step() */
    f64 rate;         /* effective "gain" param, as softfloat */
    f64 cursor;       /* pointer x, converted from milli at apply */
    f64 scale;        /* effective "scale" F64 param (raw bits) */
    f64 dt;
    uint64_t steps;
    int32_t gain;     /* stored I32 param */
    int32_t dial_mt;  /* summed DIAL milliturns */
    int32_t last_x, last_y;
    uint32_t events_seen;
    uint32_t down;    /* 0/1 touch state */
    uint64_t applied_before_step; /* steps value seen by apply() */
} test_state;

static void test_reset(void *st_)
{
    test_state *s = (test_state *)st_;
    toy_memset(s, 0, sizeof *s);
    s->gain = 2; /* declared default for param 0 */
    s->rate = (f64)toy_f64_bits_from_i32(s->gain);
    s->scale = f64_dec("1.0"); /* declared default for param 1 */
    s->dt = f64_dec("0.01");
}

static void test_apply(void *st_, const toy_event_t *ev)
{
    test_state *s = (test_state *)st_;
    s->events_seen++;
    s->applied_before_step = s->steps; /* quantisation witness */
    switch (ev->type) {
    case TOY_EV_POINTER_DOWN: s->down = 1; /* fall through */
    case TOY_EV_POINTER_MOVE:
        s->last_x = ev->x;
        s->last_y = ev->y;
        s->cursor = (f64)toy_f64_bits_from_milli(ev->x);
        break;
    case TOY_EV_POINTER_UP: s->down = 0; break;
    case TOY_EV_DIAL: s->dial_mt += ev->v; break;
    case TOY_EV_SET_PARAM:
        if (ev->id == 0) { /* gain, declared 1..10 */
            int32_t v = ev->v;
            if (v < 1) v = 1;
            if (v > 10) v = 10;
            s->gain = v;
            s->rate = (f64)toy_f64_bits_from_i32(v);
        } else if (ev->id == 1) { /* scale, F64 bits */
            s->scale = (f64)ev->bits;
        }
        break;
    default: break; /* ignore the rest */
    }
}

static void test_step(void *st_)
{
    test_state *s = (test_state *)st_;
    /* acc += (rate + cursor*scale) * dt: settings and pointer
     * both feed step maths only through stored state. */
    f64 drive = f64_add(s->rate, f64_mul(s->cursor, s->scale));
    s->acc = f64_fma(drive, s->dt, s->acc);
    s->steps++;
}

static void test_render(const void *st_, toy_fb_t *fb)
{
    (void)st_;
    toy_fb_clear(fb, 0);
}

static const toy_param_t test_params[] = {
    { "gain",  "Gain",  TOY_PARAM_I32, 0, 1, 10, 2, 0, 0, 0, 0 },
    { "scale", "Scale", TOY_PARAM_F64, 0, 0, 0, 0, "0.5", "2.0", "1.0", 0 },
};

static const toy_proof_input_t test_proof[] = {
    {   0, { TOY_EV_SET_PARAM,    0,    0,    0, 5,   0, 0 } },
    {  10, { TOY_EV_POINTER_DOWN, 0,  500, -250, 0,   0, 0 } },
    {  50, { TOY_EV_DIAL,         0,    0,    0, 250, 0, 0 } },
    {  60, { TOY_EV_POINTER_UP,   0,  500, -250, 0,   0, 0 } },
};

static const toy_cart_t test_cart = {
    "inputtest", "Input Test", "test only",
    sizeof(test_state), 1, 200, 100,
    test_reset, test_step, test_render,
    test_apply, test_params, 2, test_proof, 4,
    TOY_CAP_POINTER | TOY_CAP_PARAMS | TOY_CAP_DIAL
};

static uint64_t run_test_cart(int with_script, test_state *out)
{
    test_state s;
    memset(&s, 0, sizeof s);
    test_reset(&s);
    uint32_t cur = 0;
    if (with_script) toy_apply_proof_at(&test_cart, &s, 0, &cur);
    for (uint32_t step = 1; step <= 200; step++) {
        if (with_script) toy_apply_proof_at(&test_cart, &s, step, &cur);
        test_step(&s);
    }
    if (out) *out = s;
    return toy_hash_state(&s, sizeof s);
}

int main(void)
{
    printf("sizeof toy_event_t = %zu\n", sizeof(toy_event_t));
    expect(sizeof(toy_event_t) == 32, "event is 32 bytes");
    expect(toy_cart_check_input(&test_cart) == 0,
           "test cart descriptor valid");

    /* 1. Deaf path: helpers must not touch fig8. */
    const toy_cart_t *fig8 = toy_cart_by_name("fig8");
    expect(fig8 && fig8->apply == 0 && fig8->proof_input_count == 0,
           "fig8 is deaf (zero-filled tail)");
    expect(toy_cart_check_input(fig8) == 0, "fig8 descriptor valid");
    {
        void *a = calloc(1, fig8->state_size);
        void *b = calloc(1, fig8->state_size);
        if (!a || !b) return 2;
        fig8->reset(a);
        fig8->reset(b);
        uint32_t cur = 0;
        toy_queue_t q;
        toy_queue_clear(&q);
        toy_event_t poke = { TOY_EV_POINTER_DOWN, 0, 100, 100, 0, 0, 0 };
        toy_queue_push(&q, &poke);
        for (uint32_t s = 1; s <= 500; s++) {
            toy_apply_proof_at(fig8, b, s, &cur);
            toy_apply_queue(fig8, b, &q, TOY_MAX_EVENTS_PER_STEP);
            fig8->step(a);
            fig8->step(b);
        }
        uint64_t ha = toy_hash_state(a, fig8->state_size);
        uint64_t hb = toy_hash_state(b, fig8->state_size);
        printf("fig8 plain %016llx via-helpers %016llx\n",
               (unsigned long long)ha, (unsigned long long)hb);
        expect(ha == hb, "deaf fig8 hash untouched by seam helpers");
        free(a);
        free(b);
    }

    /* 2. Proof script: changes state, twice-identical. */
    test_state scripted;
    uint64_t h_none = run_test_cart(0, 0);
    uint64_t h_scr1 = run_test_cart(1, &scripted);
    uint64_t h_scr2 = run_test_cart(1, 0);
    printf("test no-script %016llx script %016llx script2 %016llx\n",
           (unsigned long long)h_none, (unsigned long long)h_scr1,
           (unsigned long long)h_scr2);
    printf("input_hash %016llx\n",
           (unsigned long long)toy_hash_inputs(test_proof, 4));
    expect(h_scr1 != h_none, "proof script changes state");
    expect(h_scr1 == h_scr2, "proof script replay is twice-identical");
    expect(scripted.gain == 5 && scripted.dial_mt == 250 &&
           scripted.last_x == 500 && scripted.down == 0 &&
           scripted.events_seen == 4,
           "script effects all landed in state");

    /* 3. SET_PARAM step-0 affects step maths (gain 5 vs default 2). */
    {
        test_state s;
        memset(&s, 0, sizeof s);
        test_reset(&s);
        toy_event_t ev;
        expect(toy_param_event(&test_cart, 0, 5, 0, &ev) == 0,
               "toy_param_event builds I32 SET_PARAM");
        test_apply(&s, &ev); /* step-0 application */
        for (int i = 0; i < 200; i++) test_step(&s);
        uint64_t h5 = toy_hash_state(&s, sizeof s);
        printf("gain5-only %016llx (default run %016llx)\n",
               (unsigned long long)h5, (unsigned long long)h_none);
        expect(h5 != h_none, "SET_PARAM gain changes step maths");
        expect(toy_param_event(&test_cart, 1, 0, "1.5", &ev) == 0 &&
               ev.bits == (uint64_t)f64_dec("1.5"),
               "toy_param_event F64 carries decimal bits");
        expect(toy_param_event(&test_cart, 9, 0, 0, &ev) == -1,
               "toy_param_event rejects bad index");
    }

    /* 4. Quantisation: a queued touch applies at the boundary
     * before step 11, never mid-step; px->milli is integer. */
    {
        test_state s;
        memset(&s, 0, sizeof s);
        test_reset(&s);
        for (int i = 0; i < 10; i++) test_step(&s);
        toy_queue_t q;
        toy_queue_clear(&q);
        /* canvas 360, pixel (270,90) -> milli (+500,-500) */
        expect(toy_queue_push_pointer(&q, TOY_EV_POINTER_DOWN, 0,
                                      270, 90, 360) == 1,
               "pointer push queues");
        toy_apply_queue(&test_cart, &s, &q, TOY_MAX_EVENTS_PER_STEP);
        expect(s.applied_before_step == 10,
               "apply ran at boundary before step 11");
        expect(s.last_x == 500 && s.last_y == -500,
               "px->milli conversion (270,90)@360 = (500,-500)");
        printf("quantise witness applied_before_step=%llu hash %016llx\n",
               (unsigned long long)s.applied_before_step,
               (unsigned long long)toy_hash_state(&s, sizeof s));
    }

    /* 5. Over-cap CARRY: 20 dial events, cap 16 per boundary. */
    {
        test_state s;
        memset(&s, 0, sizeof s);
        test_reset(&s);
        toy_queue_t q;
        toy_queue_clear(&q);
        for (int i = 0; i < 20; i++) {
            toy_event_t d = { TOY_EV_DIAL, 0, 0, 0, 10, 0, 0 };
            toy_queue_push(&q, &d);
        }
        uint32_t n1 = toy_apply_queue(&test_cart, &s, &q,
                                      TOY_MAX_EVENTS_PER_STEP);
        int after1 = (s.events_seen == 16 && q.len == 4);
        test_step(&s);
        uint32_t n2 = toy_apply_queue(&test_cart, &s, &q,
                                      TOY_MAX_EVENTS_PER_STEP);
        printf("carry drain1=%u drain2=%u dial_mt=%d pending=%u\n",
               n1, n2, s.dial_mt, q.len);
        expect(after1 && n1 == 16 && n2 == 4 && s.dial_mt == 200 &&
               q.len == 0,
               "over-cap events carry to next boundary in order");
    }

    /* 6. Coalescing + validation of a broken descriptor. */
    {
        toy_queue_t q;
        toy_queue_clear(&q);
        toy_event_t m1 = { TOY_EV_POINTER_MOVE, 0, 100, 0, 0, 0, 0 };
        toy_event_t m2 = { TOY_EV_POINTER_MOVE, 0, 900, 0, 0, 0, 0 };
        toy_queue_push(&q, &m1);
        toy_queue_push(&q, &m2);
        expect(q.len == 1 && q.ev[0].x == 900,
               "MOVE coalesces in place, latest wins");

        toy_cart_t bad = test_cart;
        bad.apply = 0; /* script without apply must be rejected */
        expect(toy_cart_check_input(&bad) != 0,
               "broken descriptor rejected");
    }

    printf(failures ? "input_test: FAIL (%d)\n" : "input_test: PASS\n",
           failures);
    return failures ? 1 : 0;
}
