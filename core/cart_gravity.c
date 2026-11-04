/* cart_gravity.c - N-body gravity sandbox (drag/fling + tilt).
 *
 * The second interactive cartridge (study-input section 11.2,
 * refined here): bodies attract each other by Newtonian gravity
 * in muse-softfloat binary64; the user flings new bodies into
 * the system with a press-drag-release gesture, and device tilt
 * acts as a uniform gravity direction (a desk toy you can tip).
 * apply() is pre-step integer->state only (seam rules: softfloat
 * conversions through toy_input.h helpers, no allocation, no
 * I/O, no wall clock); step() itself stays input-free.
 *
 * Simulation. Direct O(n^2) summation in fixed index order,
 * Plummer-style softened: a_i = sum_j G*m_j*(x_j-x_i) /
 * (|x_j-x_i|^2 + eps^2)^(3/2), plus the uniform tilt term.
 * Integrator: kick-drift-kick leapfrog (velocity Verlet) at
 * fixed dt = 0.004 with the acceleration cached in state, so
 * one step costs one acceleration evaluation. Leapfrog is
 * symplectic and time-reversible: energy error stays bounded
 * instead of drifting secularly, which is what a run-forever
 * desk toy needs; softening bounds the force at close range so
 * a fixed step cannot meet a singularity. There are NO
 * collisions and no merging - softened bodies pass through
 * each other; merging would need an arbitrary radius and
 * would destroy the proof-friendly property that every body
 * keeps its mass and slot. The plane is open: escaped bodies
 * keep integrating off-glass until the ring recycles them.
 *
 * Coordinates. Sim space is the glass disc: x right, y UP,
 * unit radius nominal. Pointer events arrive in milli-radius
 * display space (+y down); apply() converts with
 * toy_f64_bits_from_milli and flips y in integers.
 *
 * Fling (section 11.2, one argued refinement). DOWN records
 * the grab point and the state step counter; MOVE updates the
 * aim point; UP updates aim from its own coordinates (the
 * release point) and spawns a body AT THE GRAB POINT with
 * velocity (aim-grab)/(steps_held*dt) * fling_scale, computed
 * in softfloat from stored step counts, never wall time. The
 * sketch's raw (aim-grab)/(steps held) is a per-step rate;
 * converting it to a per-unit-time velocity at dt=0.004
 * multiplies by 250, so any deliberate drag would eject the
 * body at many times escape speed. fling_scale = 0.1 maps a
 * 10-step, third-of-the-glass drag to roughly circular speed
 * (~0.8-0.9). Components clamp to +/-2 so a 1-step flick
 * cannot produce an unbounded velocity.
 *
 * Tilt. A TILT event overwrites the stored gravity-direction
 * vector with the milli-g event values (x right, y flipped to
 * sim-up), scaled by tilt_scale = 0.4 into a uniform
 * acceleration applied to every body. At 1 g that is about a
 * quarter of the central body's pull at r=0.55: the system
 * visibly sloshes and re-centres rather than emptying.
 *
 * Slots. At most GV_MAX (12) bodies, always in slots
 * 0..n_active-1. reset() seeds a deterministic starter system
 * (central mass 1.0 plus companions on near-circular orbits,
 * speeds sqrt(G*M/r) computed in softfloat at reset) so the
 * untouched cart is alive. A spawn fills the next free slot;
 * when full it overwrites a ring cursor that cycles through
 * slots 1..GV_MAX-1 - slot 0 (the starter's central anchor)
 * is never recycled, so the system keeps its centre of
 * attraction no matter how many bodies the user flings.
 * Flung bodies get the fixed spawn_mass.
 *
 * Settings (SET_PARAM only): "bodies" (I32, RESET_ONLY,
 * 1..8 starter bodies, default 5) reseeds when it arrives at
 * step 0 and is stored without live effect afterwards, per
 * the RESET_ONLY contract (the runner re-resets); "softening"
 * (F64, live, 0.005..0.2, default 0.03) is eps above; a live
 * change recomputes the cached acceleration. "palette"
 * (ENUM, live) is render-only but stored in state so hashes
 * say what was shown. Caps are POINTER|TILT|PARAMS only: the
 * sketch's SHAKE/KEYS have no semantics here (pause stays
 * runner chrome), and a cart should not advertise inputs it
 * ignores.
 */
#include "toy_cart.h"
#include "toy_math.h"
#include "toy_util.h"

#define GV_MAX           12
#define GV_STARTER_MAX   8
#define GV_TRAIL         32
#define GV_PARAM_BODIES  0u
#define GV_PARAM_SOFT    1u
#define GV_PARAM_PALETTE 2u
#define GV_BODIES_DEF    5

/* State-size arithmetic, descending alignment, no hidden
 * padding: x/v/a 3*12*2*8 = 576, mass 12*8 = 96, trail
 * 12*32*2*8 = 6144, thirteen scalar f64 = 104, two u64 = 16,
 * eight 32-bit fields = 32. Total 6968 bytes (~11% of the
 * 64 KiB TOY_MAX_STATE_SIZE), a multiple of 8. The cached
 * acceleration a[] is part of the leapfrog state: at every
 * step boundary a[] == accel(x), which is what checkpoints
 * need to resume a KDK sequence bit-exactly. */
typedef struct {
    f64 x[GV_MAX][2];
    f64 v[GV_MAX][2];
    f64 a[GV_MAX][2];
    f64 mass[GV_MAX];
    f64 trail[GV_MAX][GV_TRAIL][2];
    f64 G, dt, softening, soft2;
    f64 tilt_x, tilt_y, tilt_scale;
    f64 fling_scale, spawn_mass;
    f64 grab_x, grab_y, aim_x, aim_y;
    uint64_t steps;
    uint64_t down_step;       /* s->steps when DOWN landed */
    int32_t bodies_param;     /* "bodies" RESET_ONLY effect */
    uint32_t n_active;
    uint32_t next_slot;       /* ring cursor, 1..GV_MAX-1 */
    uint32_t palette;         /* "palette" enum effect */
    uint32_t down;            /* 1 while the contact is held */
    uint32_t contact_id;
    uint32_t spawns;          /* total flung bodies (display) */
    uint32_t reserved;
} gravity_state;

/* Starter table: position and CCW tangent in milli units
 * (sim space, y up), mass as a decimal string. Slot 0 is the
 * central anchor; companions come in opposite pairs so every
 * even prefix has near-zero total momentum. The "bodies"
 * param activates a prefix of this table. */
static const struct {
    int32_t px, py, tx, ty;
    const char *mass;
} gv_starter[GV_STARTER_MAX] = {
    {    0,    0,     0,     0, "1.0"  },
    {  550,    0,     0,  1000, "0.02" },
    { -550,    0,     0, -1000, "0.02" },
    {    0,  850, -1000,     0, "0.02" },
    {    0, -850,  1000,     0, "0.02" },
    {  300,    0,     0,  1000, "0.015" },
    { -300,    0,     0, -1000, "0.015" },
    {  600,  600,  -707,   707, "0.02" },
};

static inline f64 gv_milli(int32_t m) { return (f64)toy_f64_bits_from_milli(m); }
static inline int gv_gt(f64 a, f64 b) { msf_flags_t f = 0; return msf64_compare_quiet_greater(a, b, &f); }
static inline int gv_lt(f64 a, f64 b) { msf_flags_t f = 0; return msf64_compare_quiet_less(a, b, &f); }
static inline int gv_ge(f64 a, f64 b) { msf_flags_t f = 0; return msf64_compare_quiet_greater_equal(a, b, &f); }

static int32_t gv_clamp_milli(int32_t v)
{
    if (v > 1000) return 1000;
    if (v < -1000) return -1000;
    return v;
}

static uint32_t gv_n(const gravity_state *s)
{
    return s->n_active > (uint32_t)GV_MAX ? (uint32_t)GV_MAX : s->n_active;
}

/* Softened pairwise gravity + uniform tilt into out[][].
 * Fixed i,j order, full ordered-pair loop like fig8's accel:
 * the summation order is part of the determinism contract. */
static void gv_accel(const gravity_state *s, f64 out[GV_MAX][2])
{
    uint32_t n = gv_n(s);
    f64 gx = f64_mul(s->tilt_x, s->tilt_scale);
    f64 gy = f64_mul(s->tilt_y, s->tilt_scale);
    for (int i = 0; i < GV_MAX; i++) out[i][0] = out[i][1] = F64_ZERO;
    for (uint32_t i = 0; i < n; i++) {
        f64 ax = gx, ay = gy;
        for (uint32_t j = 0; j < n; j++) {
            if (j == i) continue;
            f64 dx = f64_sub(s->x[j][0], s->x[i][0]);
            f64 dy = f64_sub(s->x[j][1], s->x[i][1]);
            f64 r2 = f64_add(f64_fma(dx, dx, f64_mul(dy, dy)), s->soft2);
            f64 inv = f64_div(F64_ONE, f64_mul(r2, f64_sqrt(r2)));
            f64 f = f64_mul(f64_mul(s->G, s->mass[j]), inv);
            ax = f64_fma(dx, f, ax);
            ay = f64_fma(dy, f, ay);
        }
        out[i][0] = ax;
        out[i][1] = ay;
    }
}

static void gv_fill_trail(gravity_state *s, uint32_t i)
{
    for (int t = 0; t < GV_TRAIL; t++) {
        s->trail[i][t][0] = s->x[i][0];
        s->trail[i][t][1] = s->x[i][1];
    }
}

/* (Re)seed the starter system with `count` bodies. Called by
 * reset() (through the param-default events) and by a step-0
 * "bodies" SET_PARAM. Companion speeds are the circular
 * speed about the central mass, sqrt(G*M/r), in softfloat;
 * companion-companion forces are a ~2% perturbation, so the
 * starter orbits stay near-circular for many periods. */
static void gv_seed(gravity_state *s, int32_t count)
{
    if (count < 1) count = 1;
    if (count > GV_STARTER_MAX) count = GV_STARTER_MAX;
    s->bodies_param = count;
    s->n_active = (uint32_t)count;
    s->next_slot = 1;
    s->down = 0;
    s->spawns = 0;
    f64 mcentral = f64_dec(gv_starter[0].mass);
    for (int i = 0; i < GV_MAX; i++) {
        s->x[i][0] = s->x[i][1] = F64_ZERO;
        s->v[i][0] = s->v[i][1] = F64_ZERO;
        s->a[i][0] = s->a[i][1] = F64_ZERO;
        s->mass[i] = F64_ZERO;
        if (i < count) {
            s->x[i][0] = gv_milli(gv_starter[i].px);
            s->x[i][1] = gv_milli(gv_starter[i].py);
            s->mass[i] = f64_dec(gv_starter[i].mass);
            if (i > 0) {
                f64 r = f64_sqrt(f64_fma(s->x[i][0], s->x[i][0],
                                         f64_mul(s->x[i][1], s->x[i][1])));
                f64 vc = f64_sqrt(f64_div(f64_mul(s->G, mcentral), r));
                s->v[i][0] = f64_mul(vc, gv_milli(gv_starter[i].tx));
                s->v[i][1] = f64_mul(vc, gv_milli(gv_starter[i].ty));
            }
        }
        gv_fill_trail(s, (uint32_t)i);
    }
    gv_accel(s, s->a);
}

/* Spawn the flung body: next free slot, else the ring cursor
 * over slots 1..GV_MAX-1 (slot 0 is never recycled). */
static void gv_spawn(gravity_state *s, f64 vx, f64 vy)
{
    uint32_t slot;
    if (gv_n(s) < (uint32_t)GV_MAX) {
        slot = s->n_active;
        s->n_active++;
    } else {
        slot = s->next_slot;
        if (slot < 1u || slot >= (uint32_t)GV_MAX) slot = 1;
        s->next_slot = slot + 1u >= (uint32_t)GV_MAX ? 1u : slot + 1u;
    }
    s->x[slot][0] = s->grab_x;
    s->x[slot][1] = s->grab_y;
    s->v[slot][0] = vx;
    s->v[slot][1] = vy;
    s->mass[slot] = s->spawn_mass;
    gv_fill_trail(s, slot);
    s->spawns++;
    gv_accel(s, s->a);
}

static f64 gv_clamp_vel(f64 v)
{
    if (gv_gt(v, F64_TWO)) return F64_TWO;
    if (gv_lt(v, f64_neg(F64_TWO))) return f64_neg(F64_TWO);
    return v;
}

/* apply(): events -> state effects only (seam rules). */
static void gravity_apply(void *st_, const toy_event_t *ev)
{
    gravity_state *s = (gravity_state *)st_;
    switch (ev->type) {
    case TOY_EV_POINTER_DOWN:
        /* A fresh DOWN (re)starts the grab, even mid-contact. */
        s->down = 1;
        s->contact_id = ev->id;
        s->grab_x = s->aim_x = gv_milli(gv_clamp_milli(ev->x));
        s->grab_y = s->aim_y = gv_milli(-gv_clamp_milli(ev->y));
        s->down_step = s->steps;
        break;
    case TOY_EV_POINTER_MOVE:
        if (s->down && ev->id == s->contact_id) {
            s->aim_x = gv_milli(gv_clamp_milli(ev->x));
            s->aim_y = gv_milli(-gv_clamp_milli(ev->y));
        }
        break;
    case TOY_EV_POINTER_UP:
        if (s->down && ev->id == s->contact_id) {
            s->aim_x = gv_milli(gv_clamp_milli(ev->x));
            s->aim_y = gv_milli(-gv_clamp_milli(ev->y));
            uint64_t held = s->steps - s->down_step;
            if (held < 1u) held = 1u;
            if (held > 1048576u) held = 1048576u;
            f64 held_f = (f64)toy_f64_bits_from_i32((int32_t)held);
            f64 denom = f64_mul(held_f, s->dt);
            f64 vx = f64_mul(f64_div(f64_sub(s->aim_x, s->grab_x), denom),
                             s->fling_scale);
            f64 vy = f64_mul(f64_div(f64_sub(s->aim_y, s->grab_y), denom),
                             s->fling_scale);
            gv_spawn(s, gv_clamp_vel(vx), gv_clamp_vel(vy));
            s->down = 0;
        }
        break;
    case TOY_EV_TILT:
        s->tilt_x = gv_milli(gv_clamp_milli(ev->x));
        s->tilt_y = gv_milli(-gv_clamp_milli(ev->y));
        gv_accel(s, s->a); /* cached accel keeps a[] == accel(x) */
        break;
    case TOY_EV_SET_PARAM:
        if (ev->id == GV_PARAM_BODIES) {
            int32_t v = ev->v;
            if (v < 1) v = 1;
            if (v > GV_STARTER_MAX) v = GV_STARTER_MAX;
            s->bodies_param = v;
            /* RESET_ONLY: reseed only in the step-0 setup
             * slot; live, the value is stored and the runner
             * re-resets (study-input section 7.3). */
            if (s->steps == 0) gv_seed(s, v);
        } else if (ev->id == GV_PARAM_SOFT) {
            f64 v = (f64)ev->bits;
            f64 lo = f64_dec("0.005"), hi = f64_dec("0.2");
            if (!gv_ge(v, lo)) v = lo; /* also routes NaN to lo */
            if (gv_gt(v, hi)) v = hi;
            if (v != s->softening) {
                s->softening = v;
                s->soft2 = f64_mul(v, v);
                gv_accel(s, s->a);
            }
        } else if (ev->id == GV_PARAM_PALETTE) {
            int32_t v = ev->v;
            if (v < 0) v = 0;
            if (v > 2) v = 2;
            s->palette = (uint32_t)v;
        }
        break; /* out-of-range ids ignored, per the seam */
    default:
        break; /* unknown types ignored, per the seam */
    }
}

static void gravity_reset(void *st_)
{
    gravity_state *s = (gravity_state *)st_;
    toy_memset(s, 0, sizeof *s);
    s->G = f64_dec("0.5");
    s->dt = f64_dec("0.004");
    s->softening = f64_dec("0.03");
    s->soft2 = f64_mul(s->softening, s->softening);
    s->tilt_scale = f64_dec("0.4");
    s->fling_scale = f64_dec("0.1");
    s->spawn_mass = f64_dec("0.04");
    s->next_slot = 1;
    /* Param defaults through the same step-0 SET_PARAM path a
     * runner uses for user settings (julia's pattern): the
     * "bodies" default seeds the starter system, because
     * reset runs with steps == 0. */
    for (uint32_t i = 0; i < toy_cart_gravity.param_count; i++) {
        toy_event_t ev;
        if (toy_param_event(&toy_cart_gravity, i,
                            toy_cart_gravity.params[i].i_def, 0, &ev) == 0)
            gravity_apply(s, &ev);
    }
    if (s->n_active == 0) gv_seed(s, GV_BODIES_DEF); /* unreachable guard */
}

static void gravity_step(void *st_)
{
    gravity_state *s = (gravity_state *)st_;
    uint32_t n = gv_n(s);
    f64 anew[GV_MAX][2];
    f64 hh = f64_mul(s->dt, F64_HALF);
    for (uint32_t i = 0; i < n; i++)
        for (int k = 0; k < 2; k++)
            s->v[i][k] = f64_fma(s->a[i][k], hh, s->v[i][k]);
    for (uint32_t i = 0; i < n; i++)
        for (int k = 0; k < 2; k++)
            s->x[i][k] = f64_fma(s->v[i][k], s->dt, s->x[i][k]);
    gv_accel(s, anew);
    for (uint32_t i = 0; i < n; i++)
        for (int k = 0; k < 2; k++) {
            s->v[i][k] = f64_fma(anew[i][k], hh, s->v[i][k]);
            s->a[i][k] = anew[i][k];
        }
    s->steps++;
    if ((s->steps % 2u) == 0) {
        size_t slot = (size_t)((s->steps / 2u) % GV_TRAIL);
        for (uint32_t i = 0; i < n; i++) {
            s->trail[i][slot][0] = s->x[i][0];
            s->trail[i][slot][1] = s->x[i][1];
        }
    }
}

/* Display conversion only (never feeds state). */
static double gv_d64(f64 a) { union { f64 u; double d; } v; v.u = a; return v.d; }

/* Three palettes, six body colours each; the palette index
 * is state ("palette" param), so the hash says what was
 * shown. Slot 0 always takes entry 0 (the bright anchor). */
static const unsigned char gv_palettes[3][6][3] = {
    { {255, 230, 160}, {80, 200, 255}, {255, 140, 90},
      {150, 255, 170}, {230, 130, 255}, {120, 160, 255} },
    { {255, 200, 110}, {255, 110, 50}, {240, 70, 40},
      {255, 160, 80}, {220, 90, 120}, {255, 230, 140} },
    { {245, 245, 245}, {200, 200, 210}, {160, 165, 180},
      {220, 220, 225}, {140, 145, 160}, {185, 190, 200} }
};
static const char *const gv_palette_names[] = { "nebula", "ember", "mono" };

static void gravity_render(const void *st_, toy_fb_t *fb)
{
    const gravity_state *s = (const gravity_state *)st_;
    int cx = toy_cx(fb), cy = toy_cy(fb), r = toy_r(fb);
    double scale = (double)r * 0.82 / 1.15; /* sim half-width 1.15 */
    uint32_t pal = s->palette > 2u ? 2u : s->palette;
    uint32_t n = gv_n(s);
    toy_fb_clear(fb, toy_rgb565(5, 7, 18));
    toy_fb_ring(fb, cx, cy, r - 2, toy_rgb565(28, 34, 54));

    /* Tilt direction: a dim teal spoke from the centre. */
    if ((s->tilt_x & ~F64_SIGN) != 0 || (s->tilt_y & ~F64_SIGN) != 0) {
        int tx = cx + (int)(gv_d64(s->tilt_x) * (double)r * 0.30);
        int ty = cy - (int)(gv_d64(s->tilt_y) * (double)r * 0.30);
        toy_fb_line(fb, cx, cy, tx, ty, toy_rgb565(30, 120, 120));
        toy_fb_disc(fb, tx, ty, 2, toy_rgb565(60, 200, 190));
    }

    size_t head = (size_t)((s->steps / 2u) % GV_TRAIL);
    for (uint32_t i = 0; i < n; i++) {
        const unsigned char *c8 = gv_palettes[pal][i % 6u];
        uint16_t body = toy_rgb565(c8[0], c8[1], c8[2]);
        for (int t = 0; t < GV_TRAIL; t++) {
            size_t idx = (head + GV_TRAIL - (size_t)t) % GV_TRAIL;
            int x = cx + (int)(gv_d64(s->trail[i][idx][0]) * scale);
            int y = cy - (int)(gv_d64(s->trail[i][idx][1]) * scale);
            unsigned fade = 255u - (unsigned)(t * 200 / GV_TRAIL);
            unsigned p = body;
            uint16_t c = (uint16_t)(((((p >> 11) & 31u) * fade) >> 8 << 11) |
                                    ((((p >> 5) & 63u) * fade) >> 8 << 5) |
                                    (((p & 31u) * fade) >> 8));
            toy_fb_px(fb, x, y, c);
        }
        int x = cx + (int)(gv_d64(s->x[i][0]) * scale);
        int y = cy - (int)(gv_d64(s->x[i][1]) * scale);
        int rad = 2 + (int)(gv_d64(s->mass[i]) * 6.0);
        if (rad < 2) rad = 2;
        if (rad > 2 + r / 18) rad = 2 + r / 18;
        toy_fb_disc(fb, x, y, rad, body);
        toy_fb_disc(fb, x, y, rad > 2 ? 1 : 0, 0xffffu);
    }

    /* Held contact: the fling vector being aimed. */
    if (s->down) {
        int gx = cx + (int)(gv_d64(s->grab_x) * scale);
        int gy = cy - (int)(gv_d64(s->grab_y) * scale);
        int ax = cx + (int)(gv_d64(s->aim_x) * scale);
        int ay = cy - (int)(gv_d64(s->aim_y) * scale);
        uint16_t gold = toy_rgb565(255, 210, 80);
        toy_fb_line(fb, gx, gy, ax, ay, gold);
        toy_fb_ring(fb, gx, gy, 3 + r / 60, gold);
        toy_fb_px(fb, ax, ay, gold);
    }

    int sc = r >= 150 ? 2 : 1;
    toy_fb_text(fb, cx - 14 * sc, cy - r + 10 * sc, sc,
                toy_rgb565(150, 170, 200), "GRAVITY");
    char buf[8];
    buf[0] = 'N'; buf[1] = '=';
    buf[2] = (char)('0' + (n / 10u) % 10u);
    buf[3] = (char)('0' + n % 10u);
    buf[4] = '\0';
    toy_fb_text(fb, cx - 8 * sc, cy + r - 16 * sc, sc,
                toy_rgb565(150, 170, 200), buf);
}

static const toy_param_t gravity_params[] = {
    { "bodies", "Starter bodies", TOY_PARAM_I32, TOY_PARAM_RESET_ONLY,
      1, GV_STARTER_MAX, GV_BODIES_DEF, 0, 0, 0, 0 },
    { "softening", "Softening", TOY_PARAM_F64, 0,
      0, 0, 0, "0.005", "0.2", "0.03", 0 },
    { "palette", "Palette", TOY_PARAM_ENUM, 0,
      0, 2, 0, 0, 0, 0, gv_palette_names },
};

/* Proof script: 800 steps, hashes every 100, each phase
 * landing between its own pair of hash boundaries. Pointer
 * milli below are display space (+y down); sim notes are
 * (x, y-up). Both flings are held exactly 10 steps, so the
 * spawn velocity is displacement*2.5 (see the fling note).
 *   1..200   idle starter system, no events: the untouched
 *            attract state (5 bodies on near-circular orbits).
 *   201-211  fling 1: grab sim (-0.50,+0.25), aim sim
 *            (-0.65,-0.05), velocity (-0.375,-0.75) - close
 *            to circular there; body 6 joins as an ellipse.
 *   301-311  fling 2: grab sim (+0.65,-0.35), aim sim
 *            (+0.85,-0.05), velocity (+0.50,+0.75); body 7.
 *   401      TILT (300, 400) milli-g: uniform acceleration
 *            toward display lower-right; the system sloshes.
 *   501      SET_PARAM softening -> 0.08 (live F64; bits are
 *            exactly f64_dec("0.08"), the correctly rounded
 *            binary64 value a runner would build from the
 *            decimal via toy_param_event - a compiled-in
 *            script cannot call the parser, so the bits are
 *            spelled out and pinned by the parcel's probe).
 *   601      SET_PARAM palette -> "ember" (live ENUM).
 * Sorted by step, at most 1 entry at any step. */
static const toy_proof_input_t gravity_proof[] = {
    { 201, { TOY_EV_POINTER_DOWN, 0, -500, -250, 0, 0, 0 } },
    { 206, { TOY_EV_POINTER_MOVE, 0, -650,   50, 0, 0, 0 } },
    { 211, { TOY_EV_POINTER_UP,   0, -650,   50, 0, 0, 0 } },
    { 301, { TOY_EV_POINTER_DOWN, 0,  650,  350, 0, 0, 0 } },
    { 306, { TOY_EV_POINTER_MOVE, 0,  850,   50, 0, 0, 0 } },
    { 311, { TOY_EV_POINTER_UP,   0,  850,   50, 0, 0, 0 } },
    { 401, { TOY_EV_TILT,         0,  300,  400, 0, 0, 0 } },
    { 501, { TOY_EV_SET_PARAM, GV_PARAM_SOFT, 0, 0, 0, 0,
             0x3fb47ae147ae147bull } }, /* "0.08" */
    { 601, { TOY_EV_SET_PARAM, GV_PARAM_PALETTE, 0, 0, 1, 0, 0 } },
};

/* Pacing/proof. One step is dt=0.004; the r=0.55 starter
 * orbit has period ~3.6 time units (~900 steps). Demo 6
 * steps/frame at a nominal 30 fps is 180 steps/s: one inner
 * orbit in ~5 s, calm enough to aim a fling against, and a
 * 10-step press is ~2 frames of drag. Proof 800 steps covers
 * ~0.9 starter periods plus every input phase above.
 *
 * Board costing (ESTIMATE from measured wave factors; the
 * measurements are in the parcel report): one step is one
 * acceleration evaluation, n*(n-1) softened pair terms with
 * n = 5 rising to 7 over the proof (20-42 sqrt+div pairs).
 * Projected with the mandelbrot ~181x host-to-RP2040 factor,
 * the 800-step proof lands far under the ~3-minute routine
 * board budget even at the full GV_MAX = 12 (132 pairs). */
const toy_cart_t toy_cart_gravity = {
    "gravity", "Gravity Sandbox",
    "Press, drag, release to fling a body. Tip the toy to tip gravity.",
    sizeof(gravity_state), 6, 800, 100,
    gravity_reset, gravity_step, gravity_render,
    /* explicit WAVE-T2 tail: apply, params, param_count,
     * proof_inputs, proof_input_count, input_caps,
     * state_version, feed, sig_spec (signal-deaf) */
    gravity_apply, gravity_params, 3, gravity_proof, 9,
    TOY_CAP_POINTER | TOY_CAP_TILT | TOY_CAP_PARAMS, 0, 0, { 0 }
};
