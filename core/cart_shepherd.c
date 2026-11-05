/* cart_shepherd.c - Shepherd: the gameified gravity cart.
 *
 * The owner asked for gravity with stakes: "a new object is
 * thrown in and the goal is to collect as many as you can while
 * others potentially lose their orbit from moving too fast or
 * collisions." Shepherd keeps gravity's honest physics (the
 * same softfloat direct O(n^2) N-body class, cached-acceleration
 * KDK leapfrog, Plummer softening, fixed dt) and adds a game
 * layer that is itself part of hashed state: deterministic
 * spawns, capture/loss rules, collisions-as-merges, score.
 * apply() is pre-step integer->state only (seam rules: softfloat
 * conversions through toy_input.h helpers, no allocation, no
 * I/O, no wall clock); step() itself stays input-free. The
 * spawn schedule and spawn states derive only from an in-state
 * seeded LCG and step counters, so a scripted playthrough has
 * an exact final score on host, board and WASM alike.
 *
 * Game design (as landed).
 *
 * - Anchor. Slot 0 is a heavy central anchor PINNED at the
 *   origin (mass 1.0, never integrated, immune to tilt and to
 *   grabs). Pinning is a deliberate deviation from gravity,
 *   whose anchor drifts: capture is defined by orbital energy
 *   about the anchor, and a fixed anchor makes that energy
 *   exact and the pen stationary on the glass.
 * - Sheep. Slots 1..SH_MAX-1 (9) hold spawned bodies ("sheep",
 *   mass 0.02). A sheep is WILD until captured, then CAPTURED
 *   forever (the score is banked; the body stays in the sim
 *   and can still be lost - see below). reset() pre-seeds one
 *   wild sheep on a circular orbit at r = 0.55 so the untouched
 *   cart shows a first capture on its own.
 * - Spawns. First at step 100, then every spawn_interval
 *   steps into the lowest empty slot (skipped, without
 *   consuming LCG draws, when the pen is full). Spawn radius
 *   0.80, LCG angle (1000 milli-turn positions), CCW tangential
 *   velocity f * v_circ(0.80) with the LCG factor f in a
 *   level-dependent band (see the ramp). Spawn-band maths:
 *   a tangential launch at r0 has semi-major axis
 *   a = r0/(2 - f^2), so f > ~1.155 puts the apocentre past
 *   the escape radius 1.60 (lost by leaving the pen, bound or
 *   not) and f >= sqrt(2) ~ 1.414 is unbound outright. The
 *   level-0 band 0.780..1.480 therefore deals roughly half
 *   pen-keeping orbits and half strays before N-body
 *   perturbations - the no-input baseline captures some sheep
 *   and loses others, which is the game.
 * - Capture (exact). Each step, after integration, a wild
 *   sheep's dwell counter increments while BOTH hold: its
 *   specific orbital energy about the anchor
 *   E = |v|^2/2 - G*M/sqrt(r^2+eps^2) is negative, and its
 *   radius is inside the pen band [0.35, 0.95]. Any step
 *   failing either test resets dwell to 0. At 300 consecutive
 *   dwell steps (1.2 time units, about a fifth of a
 *   spawn-radius period - long enough that a pen-exiting
 *   eccentric orbit, which leaves the band well under 300
 *   steps from spawn, cannot capture by merely passing
 *   through) the sheep is CAPTURED: captured_total++,
 *   streak++, score += 10 + 5*(streak-1) (streak is the
 *   post-increment run of captures with no loss between).
 * - Loss (exact). A sheep (wild or captured) whose radius
 *   exceeds escape_r = 1.60, or falls inside crash_r = 0.12 of
 *   the anchor, is removed: lost_total++, streak = 0. Score
 *   and captured_total never decrease (a captured sheep that
 *   later escapes was still collected; the loss costs the
 *   streak, not the bank).
 * - Collisions (exact). Two sheep closer than coll_r = 0.07
 *   MERGE in fixed slot order: the lower slot survives with
 *   summed mass and mass-weighted position/velocity, so total
 *   momentum is conserved in softfloat; dwell resets; the
 *   merged body is captured iff either parent was. A merge is
 *   neither a capture nor a loss (merges++ only) - crowding
 *   the pen consolidates the flock instead of destroying it,
 *   but the heavier body perturbs its neighbours harder.
 * - Difficulty ramp (deterministic). Level =
 *   min(captured_total/3, 5). Spawn interval shrinks
 *   400,340,290,250,215,185 steps and the speed-factor band
 *   widens from 0.780..1.480 to 0.665..1.605 by level: the
 *   better you shepherd, the faster and wilder the throws.
 *
 * Player verbs (gravity's, re-aimed). Shepherd cannot create
 * sheep - the schedule throws them in - so the pointer rescues
 * instead of spawning: POINTER_DOWN grabs the nearest sheep
 * within grab_r = 0.22 of the press (softfloat nearest-pick
 * in apply(), which may read state; the anchor and empty
 * glass grab nothing), MOVE aims, and UP SLINGSHOTS the
 * grabbed sheep by REPLACING its velocity with gravity's
 * fling formula (aim-grab)/(steps_held*dt) * 0.1, components
 * clamped to +/-2, held time from step counters only. Replace
 * (not delta-v) is the coherent choice: the drag vector reads
 * directly as the new velocity the player is aiming for, a
 * tap stops a sheep dead (it then falls anchorward), and a
 * scripted rescue has exact, computable maths. The sheep
 * keeps integrating while held - you catch a moving body, not
 * a frozen one - and its dwell resets (a flung sheep must
 * re-earn capture). TILT is gravity's uniform acceleration
 * (tilt_scale 0.4) applied to every sheep: it shepherds the
 * whole flock at once, coarsely, where the slingshot is the
 * precision tool. Caps are POINTER|TILT|PARAMS, as gravity.
 *
 * Integrator and the a[] invariant. KDK leapfrog at
 * dt = 0.004, G = 0.5, one acceleration evaluation per step
 * via the cached a[]: at every step boundary a[] ==
 * accel(x). Every mutation path maintains it: spawn, merge
 * and loss recompute a[] inside step(); tilt and softening
 * recompute it in apply(); a slingshot changes only v, which
 * accel(x) does not read, so the invariant holds untouched.
 * Game updates run inside step() after the KDK update, in
 * fixed order: spawn (if due), merges (slot order), then
 * capture/loss per slot; the step counter increments first,
 * so the first spawn lands at the end of step 100 exactly.
 *
 * Coordinates. Sim space is the glass disc: x right, y UP,
 * unit radius nominal, anchor at the origin. Pointer events
 * arrive in milli-radius display space (+y down); apply()
 * converts with toy_f64_bits_from_milli and flips y in
 * integers.
 *
 * Settings (SET_PARAM only): "seed" (I32, RESET_ONLY,
 * 0..9999, default 7) selects the LCG stream, reseeding the
 * flock when it arrives at step 0 and stored without live
 * effect afterwards; "softening" (F64, live, 0.005..0.2,
 * default 0.03); "palette" (ENUM, live, render-only, stored
 * in state so hashes say what was shown).
 */
#include "toy_cart.h"
#include "toy_math.h"
#include "toy_util.h"

#define SH_MAX            10
#define SH_TRAIL          24
#define SH_PARAM_SEED     0u
#define SH_PARAM_SOFT     1u
#define SH_PARAM_PALETTE  2u
#define SH_SEED_DEF       7
#define SH_EMPTY          0u
#define SH_WILD           1u
#define SH_CAPTURED       2u
#define SH_DWELL_CAPTURE  300u
#define SH_FIRST_SPAWN    100u

/* State-size arithmetic, descending alignment, no hidden
 * padding: x/v/a 3*10*2*8 = 480, mass 10*8 = 80, trail
 * 10*24*2*8 = 3840, twenty scalar f64 = 160, three u64 = 24,
 * status+dwell 20*4 = 80, fourteen 32-bit fields = 56.
 * Total 4720 bytes (gravity's 6968 class, ~7% of the 64 KiB
 * TOY_MAX_STATE_SIZE), a multiple of 8. */
typedef struct {
    f64 x[SH_MAX][2];
    f64 v[SH_MAX][2];
    f64 a[SH_MAX][2];
    f64 mass[SH_MAX];
    f64 trail[SH_MAX][SH_TRAIL][2];
    f64 G, dt, softening, soft2;
    f64 tilt_x, tilt_y, tilt_scale;
    f64 fling_scale, spawn_mass;
    f64 grab_x, grab_y, aim_x, aim_y;
    f64 rmin, rmax, escape_r, coll_r, spawn_r, grab_r, crash_r;
    uint64_t steps;
    uint64_t down_step;       /* s->steps when DOWN landed */
    uint64_t next_spawn;      /* step count of the next spawn */
    uint32_t status[SH_MAX];  /* sheep slots; slot 0 unused */
    uint32_t dwell[SH_MAX];   /* consecutive capture-band steps */
    uint32_t rng;             /* LCG state (seeded at seed) */
    uint32_t score;
    uint32_t streak;
    uint32_t best_streak;
    uint32_t captured_total;
    uint32_t lost_total;
    uint32_t spawned_total;   /* scheduled spawns (not starter) */
    uint32_t merges;
    int32_t seed_param;       /* "seed" RESET_ONLY effect */
    int32_t grab_slot;        /* -1 = no sheep grabbed */
    uint32_t palette;         /* "palette" enum effect */
    uint32_t down;            /* 1 while the contact is held */
    uint32_t contact_id;
    uint32_t reserved;
} shepherd_state;

/* Difficulty ramp: level = min(captured_total/3, 5). */
static const uint32_t sh_spawn_interval_tab[6] = {
    400u, 340u, 290u, 250u, 215u, 185u
};
static const int32_t sh_factor_lo_milli[6] = {
    780, 760, 740, 715, 690, 665
};
static const int32_t sh_factor_hi_milli[6] = {
    1480, 1505, 1530, 1555, 1580, 1605
};

static inline f64 sh_milli(int32_t m) { return (f64)toy_f64_bits_from_milli(m); }
static inline int sh_gt(f64 a, f64 b) { msf_flags_t f = 0; return msf64_compare_quiet_greater(a, b, &f); }
static inline int sh_lt(f64 a, f64 b) { msf_flags_t f = 0; return msf64_compare_quiet_less(a, b, &f); }
static inline int sh_ge(f64 a, f64 b) { msf_flags_t f = 0; return msf64_compare_quiet_greater_equal(a, b, &f); }
static inline int sh_le(f64 a, f64 b) { msf_flags_t f = 0; return msf64_compare_quiet_less_equal(a, b, &f); }

static int32_t sh_clamp_milli(int32_t v)
{
    if (v > 1000) return 1000;
    if (v < -1000) return -1000;
    return v;
}

/* Slot 0 (the pinned anchor) is always active; a sheep slot
 * is active iff its status is WILD or CAPTURED. */
static int sh_active(const shepherd_state *s, uint32_t i)
{
    if (i == 0u) return 1;
    return s->status[i] == SH_WILD || s->status[i] == SH_CAPTURED;
}

static uint32_t sh_level(const shepherd_state *s)
{
    uint32_t l = s->captured_total / 3u;
    return l > 5u ? 5u : l;
}

/* Numerical Recipes LCG, in state: the spawn stream is a
 * function of (seed, spawn count) only. */
static uint32_t sh_rng_next(shepherd_state *s)
{
    s->rng = s->rng * 1664525u + 1013904223u;
    return s->rng;
}

/* Softened pairwise gravity + uniform tilt into out[][] for
 * the sheep; the anchor's row stays zero (it is never
 * integrated). Fixed i,j order, full ordered-pair loop like
 * gravity's accel: the summation order is part of the
 * determinism contract. */
static void sh_accel(const shepherd_state *s, f64 out[SH_MAX][2])
{
    f64 gx = f64_mul(s->tilt_x, s->tilt_scale);
    f64 gy = f64_mul(s->tilt_y, s->tilt_scale);
    for (int i = 0; i < SH_MAX; i++) out[i][0] = out[i][1] = F64_ZERO;
    for (uint32_t i = 1; i < (uint32_t)SH_MAX; i++) {
        if (!sh_active(s, i)) continue;
        f64 ax = gx, ay = gy;
        for (uint32_t j = 0; j < (uint32_t)SH_MAX; j++) {
            if (j == i || !sh_active(s, j)) continue;
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

static void sh_fill_trail(shepherd_state *s, uint32_t i)
{
    for (int t = 0; t < SH_TRAIL; t++) {
        s->trail[i][t][0] = s->x[i][0];
        s->trail[i][t][1] = s->x[i][1];
    }
}

static void sh_clear_slot(shepherd_state *s, uint32_t i)
{
    /* A grabbed sheep that is lost or merged away mid-hold
     * releases the grab: the later UP must not fling whatever
     * new sheep reuses the slot. */
    if (s->grab_slot == (int32_t)i) s->grab_slot = -1;
    s->status[i] = SH_EMPTY;
    s->dwell[i] = 0;
    s->x[i][0] = s->x[i][1] = F64_ZERO;
    s->v[i][0] = s->v[i][1] = F64_ZERO;
    s->a[i][0] = s->a[i][1] = F64_ZERO;
    s->mass[i] = F64_ZERO;
    for (int t = 0; t < SH_TRAIL; t++)
        s->trail[i][t][0] = s->trail[i][t][1] = F64_ZERO;
}

/* (Re)seed the flock: pinned anchor + one starter sheep on a
 * circular orbit at r = 0.55, counters cleared, LCG seeded
 * from the "seed" param. Called by reset() (through the
 * param-default events) and by a step-0 "seed" SET_PARAM. */
static void sh_seed(shepherd_state *s, int32_t seed)
{
    if (seed < 0) seed = 0;
    if (seed > 9999) seed = 9999;
    s->seed_param = seed;
    s->rng = 0x2545F491u + (uint32_t)seed * 0x9E3779B9u;
    if (s->rng == 0u) s->rng = 1u;
    s->score = 0;
    s->streak = 0;
    s->best_streak = 0;
    s->captured_total = 0;
    s->lost_total = 0;
    s->spawned_total = 0;
    s->merges = 0;
    s->down = 0;
    s->grab_slot = -1;
    s->next_spawn = (uint64_t)SH_FIRST_SPAWN;
    for (uint32_t i = 0; i < (uint32_t)SH_MAX; i++) sh_clear_slot(s, i);
    s->x[0][0] = s->x[0][1] = F64_ZERO;
    s->v[0][0] = s->v[0][1] = F64_ZERO;
    s->mass[0] = F64_ONE;
    sh_fill_trail(s, 0u);
    s->x[1][0] = sh_milli(550);
    s->x[1][1] = F64_ZERO;
    s->mass[1] = s->spawn_mass;
    s->status[1] = SH_WILD;
    f64 r = f64_sqrt(f64_fma(s->x[1][0], s->x[1][0],
                             f64_mul(s->x[1][1], s->x[1][1])));
    f64 vc = f64_sqrt(f64_div(f64_mul(s->G, s->mass[0]), r));
    s->v[1][0] = F64_ZERO;
    s->v[1][1] = vc;
    sh_fill_trail(s, 1u);
    sh_accel(s, s->a);
}

/* Throw in one sheep at spawn_r with an LCG angle and a CCW
 * tangential speed f * v_circ(spawn_r), f from the level
 * band. Fills the lowest empty slot; returns 0 without
 * consuming LCG draws when the pen is full. */
static int sh_spawn(shepherd_state *s)
{
    uint32_t slot = 0;
    for (uint32_t i = 1; i < (uint32_t)SH_MAX; i++) {
        if (s->status[i] == SH_EMPTY) { slot = i; break; }
    }
    if (slot == 0u) return 0;
    uint32_t level = sh_level(s);
    int32_t angle_mt = (int32_t)((sh_rng_next(s) >> 16) % 1000u);
    uint32_t u = (sh_rng_next(s) >> 16) % 1000u;
    int32_t lo = sh_factor_lo_milli[level];
    int32_t hi = sh_factor_hi_milli[level];
    int32_t factor_milli = lo + (int32_t)((u * (uint32_t)(hi - lo)) / 999u);
    f64 argpi = f64_mul(sh_milli(angle_mt), F64_TWO);
    f64 c = f64_cos_pi(argpi);
    f64 sn = f64_sin_pi(argpi);
    f64 vc = f64_sqrt(f64_div(f64_mul(s->G, s->mass[0]), s->spawn_r));
    f64 speed = f64_mul(vc, sh_milli(factor_milli));
    s->x[slot][0] = f64_mul(s->spawn_r, c);
    s->x[slot][1] = f64_mul(s->spawn_r, sn);
    s->v[slot][0] = f64_mul(f64_neg(sn), speed);
    s->v[slot][1] = f64_mul(c, speed);
    s->mass[slot] = s->spawn_mass;
    s->status[slot] = SH_WILD;
    s->dwell[slot] = 0;
    sh_fill_trail(s, slot);
    s->spawned_total++;
    sh_accel(s, s->a);
    return 1;
}

static void sh_lose(shepherd_state *s, uint32_t i)
{
    s->lost_total++;
    s->streak = 0;
    sh_clear_slot(s, i);
}

static void sh_capture(shepherd_state *s, uint32_t i)
{
    s->status[i] = SH_CAPTURED;
    s->dwell[i] = 0;
    s->captured_total++;
    s->streak++;
    if (s->streak > s->best_streak) s->best_streak = s->streak;
    s->score += 10u + 5u * (s->streak - 1u);
}

/* Specific orbital energy about the anchor < 0 ? */
static int sh_bound(const shepherd_state *s, uint32_t i, f64 r2)
{
    f64 v2 = f64_fma(s->v[i][0], s->v[i][0],
                     f64_mul(s->v[i][1], s->v[i][1]));
    f64 e = f64_sub(f64_mul(v2, F64_HALF),
                    f64_div(f64_mul(s->G, s->mass[0]),
                            f64_sqrt(f64_add(r2, s->soft2))));
    return sh_lt(e, F64_ZERO);
}

static f64 sh_clamp_vel(f64 v)
{
    if (sh_gt(v, F64_TWO)) return F64_TWO;
    if (sh_lt(v, f64_neg(F64_TWO))) return f64_neg(F64_TWO);
    return v;
}

/* apply(): events -> state effects only (seam rules). */
static void shepherd_apply(void *st_, const toy_event_t *ev)
{
    shepherd_state *s = (shepherd_state *)st_;
    switch (ev->type) {
    case TOY_EV_POINTER_DOWN: {
        /* (Re)start the grab: nearest sheep within grab_r of
         * the press (slot order breaks exact ties). */
        s->down = 1;
        s->contact_id = ev->id;
        s->grab_x = s->aim_x = sh_milli(sh_clamp_milli(ev->x));
        s->grab_y = s->aim_y = sh_milli(-sh_clamp_milli(ev->y));
        s->down_step = s->steps;
        s->grab_slot = -1;
        f64 grab2 = f64_mul(s->grab_r, s->grab_r);
        f64 best = grab2;
        for (uint32_t i = 1; i < (uint32_t)SH_MAX; i++) {
            if (!sh_active(s, i)) continue;
            f64 dx = f64_sub(s->x[i][0], s->grab_x);
            f64 dy = f64_sub(s->x[i][1], s->grab_y);
            f64 d2 = f64_fma(dx, dx, f64_mul(dy, dy));
            if (sh_le(d2, grab2) &&
                (s->grab_slot < 0 || sh_lt(d2, best))) {
                best = d2;
                s->grab_slot = (int32_t)i;
            }
        }
        break;
    }
    case TOY_EV_POINTER_MOVE:
        if (s->down && ev->id == s->contact_id) {
            s->aim_x = sh_milli(sh_clamp_milli(ev->x));
            s->aim_y = sh_milli(-sh_clamp_milli(ev->y));
        }
        break;
    case TOY_EV_POINTER_UP:
        if (s->down && ev->id == s->contact_id) {
            s->aim_x = sh_milli(sh_clamp_milli(ev->x));
            s->aim_y = sh_milli(-sh_clamp_milli(ev->y));
            if (s->grab_slot >= 1 && s->grab_slot < SH_MAX &&
                sh_active(s, (uint32_t)s->grab_slot)) {
                uint64_t held = s->steps - s->down_step;
                if (held < 1u) held = 1u;
                if (held > 1048576u) held = 1048576u;
                f64 held_f = (f64)toy_f64_bits_from_i32((int32_t)held);
                f64 denom = f64_mul(held_f, s->dt);
                f64 vx = f64_mul(f64_div(f64_sub(s->aim_x, s->grab_x), denom),
                                 s->fling_scale);
                f64 vy = f64_mul(f64_div(f64_sub(s->aim_y, s->grab_y), denom),
                                 s->fling_scale);
                uint32_t g = (uint32_t)s->grab_slot;
                s->v[g][0] = sh_clamp_vel(vx);
                s->v[g][1] = sh_clamp_vel(vy);
                s->dwell[g] = 0; /* must re-earn capture */
                /* v-only change: a[] == accel(x) still holds. */
            }
            s->down = 0;
            s->grab_slot = -1;
        }
        break;
    case TOY_EV_TILT:
        s->tilt_x = sh_milli(sh_clamp_milli(ev->x));
        s->tilt_y = sh_milli(-sh_clamp_milli(ev->y));
        sh_accel(s, s->a); /* cached accel keeps a[] == accel(x) */
        break;
    case TOY_EV_SET_PARAM:
        if (ev->id == SH_PARAM_SEED) {
            int32_t v = ev->v;
            if (v < 0) v = 0;
            if (v > 9999) v = 9999;
            s->seed_param = v;
            /* RESET_ONLY: reseed only in the step-0 setup
             * slot; live, the value is stored and the runner
             * re-resets (study-input section 7.3). */
            if (s->steps == 0) sh_seed(s, v);
        } else if (ev->id == SH_PARAM_SOFT) {
            f64 v = (f64)ev->bits;
            f64 lo = f64_dec("0.005"), hi = f64_dec("0.2");
            if (!sh_ge(v, lo)) v = lo; /* also routes NaN to lo */
            if (sh_gt(v, hi)) v = hi;
            if (v != s->softening) {
                s->softening = v;
                s->soft2 = f64_mul(v, v);
                sh_accel(s, s->a);
            }
        } else if (ev->id == SH_PARAM_PALETTE) {
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

static void shepherd_reset(void *st_)
{
    shepherd_state *s = (shepherd_state *)st_;
    toy_memset(s, 0, sizeof *s);
    s->G = f64_dec("0.5");
    s->dt = f64_dec("0.004");
    s->softening = f64_dec("0.03");
    s->soft2 = f64_mul(s->softening, s->softening);
    s->tilt_scale = f64_dec("0.4");
    s->fling_scale = f64_dec("0.1");
    s->spawn_mass = f64_dec("0.02");
    s->rmin = f64_dec("0.35");
    s->rmax = f64_dec("0.95");
    s->escape_r = f64_dec("1.6");
    s->coll_r = f64_dec("0.07");
    s->spawn_r = f64_dec("0.8");
    s->grab_r = f64_dec("0.22");
    s->crash_r = f64_dec("0.12");
    s->grab_slot = -1;
    /* Param defaults through the same step-0 SET_PARAM path a
     * runner uses for user settings (gravity's pattern): the
     * "seed" default seeds the flock, because reset runs with
     * steps == 0. */
    for (uint32_t i = 0; i < toy_cart_shepherd.param_count; i++) {
        toy_event_t ev;
        if (toy_param_event(&toy_cart_shepherd, i,
                            toy_cart_shepherd.params[i].i_def, 0, &ev) == 0)
            shepherd_apply(s, &ev);
    }
    if (s->mass[0] == F64_ZERO) sh_seed(s, SH_SEED_DEF); /* unreachable guard */
}

static void shepherd_step(void *st_)
{
    shepherd_state *s = (shepherd_state *)st_;
    f64 anew[SH_MAX][2];
    f64 hh = f64_mul(s->dt, F64_HALF);
    for (uint32_t i = 1; i < (uint32_t)SH_MAX; i++) {
        if (!sh_active(s, i)) continue;
        for (int k = 0; k < 2; k++)
            s->v[i][k] = f64_fma(s->a[i][k], hh, s->v[i][k]);
    }
    for (uint32_t i = 1; i < (uint32_t)SH_MAX; i++) {
        if (!sh_active(s, i)) continue;
        for (int k = 0; k < 2; k++)
            s->x[i][k] = f64_fma(s->v[i][k], s->dt, s->x[i][k]);
    }
    sh_accel(s, anew);
    for (uint32_t i = 1; i < (uint32_t)SH_MAX; i++) {
        if (!sh_active(s, i)) continue;
        for (int k = 0; k < 2; k++) {
            s->v[i][k] = f64_fma(anew[i][k], hh, s->v[i][k]);
            s->a[i][k] = anew[i][k];
        }
    }
    s->steps++;

    /* Game updates, fixed order: spawn, merges, capture/loss.
     * Any position/mass change recomputes a[] so the boundary
     * invariant a[] == accel(x) holds for checkpoints. */
    int dirty = 0;
    if (s->steps >= s->next_spawn) {
        (void)sh_spawn(s); /* recomputes a[] itself on success */
        s->next_spawn = s->steps +
            (uint64_t)sh_spawn_interval_tab[sh_level(s)];
    }
    f64 coll2 = f64_mul(s->coll_r, s->coll_r);
    for (uint32_t i = 1; i < (uint32_t)SH_MAX; i++) {
        if (!sh_active(s, i)) continue;
        for (uint32_t j = i + 1u; j < (uint32_t)SH_MAX; j++) {
            if (!sh_active(s, j)) continue;
            f64 dx = f64_sub(s->x[j][0], s->x[i][0]);
            f64 dy = f64_sub(s->x[j][1], s->x[i][1]);
            f64 d2 = f64_fma(dx, dx, f64_mul(dy, dy));
            if (!sh_lt(d2, coll2)) continue;
            /* Merge j into i: summed mass, mass-weighted
             * position and velocity (momentum conserved). */
            f64 m = f64_add(s->mass[i], s->mass[j]);
            for (int k = 0; k < 2; k++) {
                s->x[i][k] = f64_div(
                    f64_fma(s->mass[j], s->x[j][k],
                            f64_mul(s->mass[i], s->x[i][k])), m);
                s->v[i][k] = f64_div(
                    f64_fma(s->mass[j], s->v[j][k],
                            f64_mul(s->mass[i], s->v[i][k])), m);
            }
            if (s->status[j] == SH_CAPTURED) s->status[i] = SH_CAPTURED;
            s->mass[i] = m;
            s->dwell[i] = 0;
            s->merges++;
            sh_clear_slot(s, j);
            sh_fill_trail(s, i);
            dirty = 1;
        }
    }
    f64 rmin2 = f64_mul(s->rmin, s->rmin);
    f64 rmax2 = f64_mul(s->rmax, s->rmax);
    f64 escape2 = f64_mul(s->escape_r, s->escape_r);
    f64 crash2 = f64_mul(s->crash_r, s->crash_r);
    for (uint32_t i = 1; i < (uint32_t)SH_MAX; i++) {
        if (!sh_active(s, i)) continue;
        f64 r2 = f64_fma(s->x[i][0], s->x[i][0],
                         f64_mul(s->x[i][1], s->x[i][1]));
        if (sh_gt(r2, escape2) || sh_lt(r2, crash2)) {
            sh_lose(s, i);
            dirty = 1;
            continue;
        }
        if (s->status[i] == SH_WILD) {
            if (sh_ge(r2, rmin2) && sh_le(r2, rmax2) &&
                sh_bound(s, i, r2)) {
                s->dwell[i]++;
                if (s->dwell[i] >= SH_DWELL_CAPTURE) sh_capture(s, i);
            } else {
                s->dwell[i] = 0;
            }
        }
    }
    if (dirty) sh_accel(s, s->a);

    if ((s->steps % 2u) == 0) {
        size_t slot = (size_t)((s->steps / 2u) % SH_TRAIL);
        for (uint32_t i = 0; i < (uint32_t)SH_MAX; i++) {
            if (!sh_active(s, i)) continue;
            s->trail[i][slot][0] = s->x[i][0];
            s->trail[i][slot][1] = s->x[i][1];
        }
    }
}

/* Display conversion only (never feeds state). */
static double sh_d64(f64 a) { union { f64 u; double d; } v; v.u = a; return v.d; }

/* Three palettes for WILD sheep (slot-coloured), as gravity.
 * CAPTURED sheep ignore the palette: always mint with a white
 * lock ring, so the banked flock reads distinctly in every
 * palette, mono included. */
static const unsigned char sh_palettes[3][6][3] = {
    { {255, 230, 160}, {80, 200, 255}, {255, 140, 90},
      {150, 255, 170}, {230, 130, 255}, {120, 160, 255} },
    { {255, 200, 110}, {255, 110, 50}, {240, 70, 40},
      {255, 160, 80}, {220, 90, 120}, {255, 230, 140} },
    { {245, 245, 245}, {200, 200, 210}, {160, 165, 180},
      {220, 220, 225}, {140, 145, 160}, {185, 190, 200} }
};
static const char *const sh_palette_names[] = { "pasture", "ember", "mono" };

static void shepherd_render(const void *st_, toy_fb_t *fb)
{
    const shepherd_state *s = (const shepherd_state *)st_;
    int cx = toy_cx(fb), cy = toy_cy(fb), r = toy_r(fb);
    double scale = (double)r * 0.82 / 1.15; /* sim half-width 1.15 */
    uint32_t pal = s->palette > 2u ? 2u : s->palette;
    toy_fb_clear(fb, toy_rgb565(5, 7, 18));
    toy_fb_ring(fb, cx, cy, r - 2, toy_rgb565(28, 34, 54));

    /* The pen: capture band [rmin, rmax] as two green rings. */
    toy_fb_ring(fb, cx, cy, (int)(sh_d64(s->rmin) * scale),
                toy_rgb565(24, 70, 44));
    toy_fb_ring(fb, cx, cy, (int)(sh_d64(s->rmax) * scale),
                toy_rgb565(32, 96, 56));

    size_t head = (size_t)((s->steps / 2u) % SH_TRAIL);
    for (uint32_t i = 1; i < (uint32_t)SH_MAX; i++) {
        if (!sh_active(s, i)) continue;
        uint16_t body;
        if (s->status[i] == SH_CAPTURED) {
            body = toy_rgb565(120, 255, 150);
        } else {
            const unsigned char *c8 = sh_palettes[pal][i % 6u];
            body = toy_rgb565(c8[0], c8[1], c8[2]);
        }
        for (int t = 0; t < SH_TRAIL; t++) {
            size_t idx = (head + SH_TRAIL - (size_t)t) % SH_TRAIL;
            int x = cx + (int)(sh_d64(s->trail[i][idx][0]) * scale);
            int y = cy - (int)(sh_d64(s->trail[i][idx][1]) * scale);
            unsigned fade = 255u - (unsigned)(t * 200 / SH_TRAIL);
            unsigned p = body;
            uint16_t c = (uint16_t)(((((p >> 11) & 31u) * fade) >> 8 << 11) |
                                    ((((p >> 5) & 63u) * fade) >> 8 << 5) |
                                    (((p & 31u) * fade) >> 8));
            toy_fb_px(fb, x, y, c);
        }
        int x = cx + (int)(sh_d64(s->x[i][0]) * scale);
        int y = cy - (int)(sh_d64(s->x[i][1]) * scale);
        int rad = 2 + (int)(sh_d64(s->mass[i]) * 40.0);
        if (rad < 2) rad = 2;
        if (rad > 2 + r / 18) rad = 2 + r / 18;
        toy_fb_disc(fb, x, y, rad, body);
        toy_fb_disc(fb, x, y, 1, 0xffffu);
        if (s->status[i] == SH_CAPTURED)
            toy_fb_ring(fb, x, y, rad + 2, toy_rgb565(230, 255, 235));
        else if (s->dwell[i] >= SH_DWELL_CAPTURE / 2u)
            toy_fb_ring(fb, x, y, rad + 2, toy_rgb565(190, 160, 60));
    }

    /* The pinned anchor: a large gold disc at the centre. */
    {
        int rad = 3 + r / 24;
        toy_fb_disc(fb, cx, cy, rad, toy_rgb565(255, 214, 110));
        toy_fb_disc(fb, cx, cy, rad > 3 ? 2 : 1, 0xffffu);
    }

    /* Tilt direction: a teal spoke from the centre, drawn
     * after the anchor so a gentle tilt is not hidden under
     * the anchor disc. */
    if ((s->tilt_x & ~F64_SIGN) != 0 || (s->tilt_y & ~F64_SIGN) != 0) {
        int tx = cx + (int)(sh_d64(s->tilt_x) * (double)r * 0.30);
        int ty = cy - (int)(sh_d64(s->tilt_y) * (double)r * 0.30);
        toy_fb_line(fb, cx, cy, tx, ty, toy_rgb565(40, 170, 170));
        toy_fb_disc(fb, tx, ty, 2, toy_rgb565(80, 230, 220));
    }

    /* Held contact: ring the grabbed sheep and show the
     * slingshot vector; a missed grab rings the press point. */
    if (s->down) {
        uint16_t gold = toy_rgb565(255, 210, 80);
        int ax = cx + (int)(sh_d64(s->aim_x) * scale);
        int ay = cy - (int)(sh_d64(s->aim_y) * scale);
        if (s->grab_slot >= 1 && s->grab_slot < SH_MAX &&
            sh_active(s, (uint32_t)s->grab_slot)) {
            uint32_t g = (uint32_t)s->grab_slot;
            int gx = cx + (int)(sh_d64(s->x[g][0]) * scale);
            int gy = cy - (int)(sh_d64(s->x[g][1]) * scale);
            toy_fb_line(fb, gx, gy, ax, ay, gold);
            toy_fb_ring(fb, gx, gy, 5 + r / 60, gold);
        } else {
            int gx = cx + (int)(sh_d64(s->grab_x) * scale);
            int gy = cy - (int)(sh_d64(s->grab_y) * scale);
            toy_fb_ring(fb, gx, gy, 3 + r / 60, toy_rgb565(120, 120, 130));
        }
        toy_fb_px(fb, ax, ay, gold);
    }

    int sc = r >= 150 ? 2 : 1;
    uint16_t ink = toy_rgb565(170, 210, 180);
    toy_fb_text(fb, cx - 16 * sc, cy - r + 10 * sc, sc, ink, "SHEPHERD");
    char buf[12];
    uint32_t sc4 = s->score % 10000u;
    buf[0] = 'S'; buf[1] = 'C'; buf[2] = ' ';
    buf[3] = (char)('0' + (sc4 / 1000u) % 10u);
    buf[4] = (char)('0' + (sc4 / 100u) % 10u);
    buf[5] = (char)('0' + (sc4 / 10u) % 10u);
    buf[6] = (char)('0' + sc4 % 10u);
    buf[7] = '\0';
    toy_fb_text(fb, cx - 14 * sc, cy - r + 18 * sc, sc,
                toy_rgb565(255, 230, 150), buf);
    uint32_t cap2 = s->captured_total % 100u;
    uint32_t lost2 = s->lost_total % 100u;
    buf[0] = 'C';
    buf[1] = (char)('0' + (cap2 / 10u) % 10u);
    buf[2] = (char)('0' + cap2 % 10u);
    buf[3] = ' ';
    buf[4] = 'L';
    buf[5] = (char)('0' + (lost2 / 10u) % 10u);
    buf[6] = (char)('0' + lost2 % 10u);
    buf[7] = '\0';
    toy_fb_text(fb, cx - 14 * sc, cy + r - 16 * sc, sc, ink, buf);
    uint32_t st2 = s->streak % 100u;
    buf[0] = 'S'; buf[1] = 'T';
    buf[2] = (char)('0' + (st2 / 10u) % 10u);
    buf[3] = (char)('0' + st2 % 10u);
    buf[4] = '\0';
    toy_fb_text(fb, cx - 8 * sc, cy + r - 24 * sc, sc, ink, buf);
}

static const toy_param_t shepherd_params[] = {
    { "seed", "Flock seed", TOY_PARAM_I32, TOY_PARAM_RESET_ONLY,
      0, 9999, SH_SEED_DEF, 0, 0, 0, 0 },
    { "softening", "Softening", TOY_PARAM_F64, 0,
      0, 0, 0, "0.005", "0.2", "0.03", 0 },
    { "palette", "Palette", TOY_PARAM_ENUM, 0,
      0, 2, 0, 0, 0, 0, sh_palette_names },
};

/* Proof script: a scripted playthrough with an exact final
 * score (100: captured 5, lost 2, spawned 9, merges 5 at
 * step 3200 - the parcel report quotes the state-reading
 * probe per phase). Pointer milli are display space (+y
 * down); sim notes are (x, y-up). Grabs are held exactly 10
 * steps, so the slingshot velocity is displacement*2.5
 * (gravity's fling arithmetic). Sorted by step, at most 1
 * entry at any step.
 *   1..160   idle: starter dwells; spawn 1 at step 100 is an
 *            unbound stray (factor 1.471, E = +0.052).
 *   161-171  rescue: grab that stray at sim (0.803,-0.193),
 *            slingshot to near-circular v ~ (0.14, 0.76);
 *            E flips to -0.30 and it captures at step 470.
 *            (The starter captures at 300.)
 *   1701     TILT (-120, -160) milli-g: uniform accel sim
 *            (-0.048, +0.064), braking the pen-exiting
 *            factor-1.268 sheep thrown in at 1640; 2151
 *            levels. In the no-tilt counterfactual that
 *            sheep is lost at 2379; here it survives the
 *            proof, and the 1980 spawn captures at 2279
 *            (score 100, streak 5).
 *   2501-11  deliberate loss: grab a CAPTURED sheep at sim
 *            (-0.462, +0.448) and blast it radially out,
 *            v ~ (-1.29, +1.12), E = +0.68; it escapes at
 *            step 2668 (score stays 100, streak 5 -> 0).
 *   2601     SET_PARAM softening -> 0.08 (live F64; bits are
 *            exactly f64_dec("0.08"), pinned by the probe -
 *            a compiled-in script cannot call the parser).
 *   2801     SET_PARAM palette -> "ember" (live ENUM). */
static const toy_proof_input_t shepherd_proof[] = {
    { 161, { TOY_EV_POINTER_DOWN, 0, 803, 193, 0, 0, 0 } },
    { 166, { TOY_EV_POINTER_MOVE, 0, 859, -112, 0, 0, 0 } },
    { 171, { TOY_EV_POINTER_UP,   0, 859, -112, 0, 0, 0 } },
    { 1701, { TOY_EV_TILT, 0, -120, -160, 0, 0, 0 } },
    { 2151, { TOY_EV_TILT, 0, 0, 0, 0, 0, 0 } },
    { 2501, { TOY_EV_POINTER_DOWN, 0, -462, -448, 0, 0, 0 } },
    { 2506, { TOY_EV_POINTER_MOVE, 0, -977, -895, 0, 0, 0 } },
    { 2511, { TOY_EV_POINTER_UP,   0, -977, -895, 0, 0, 0 } },
    { 2601, { TOY_EV_SET_PARAM, SH_PARAM_SOFT, 0, 0, 0, 0,
              0x3fb47ae147ae147bull } }, /* "0.08" */
    { 2801, { TOY_EV_SET_PARAM, SH_PARAM_PALETTE, 0, 0, 1, 0, 0 } },
};

/* Pacing/proof. One step is dt=0.004; a spawn-radius orbit
 * has period ~6.4 time units (~1600 steps). Demo 6
 * steps/frame at a nominal 30 fps is 180 steps/s: a sheep
 * crosses the pen in seconds, calm enough to aim a slingshot
 * against, and a 10-step press is ~2 frames of drag. Proof
 * 3200 steps covers ~8 spawn intervals (at the starting
 * level) plus every play phase.
 *
 * Board costing (ESTIMATE from measured wave factors; the
 * measurements are in the parcel report): one step is one
 * acceleration evaluation, at most 9*9 sheep/anchor pair
 * terms. Projected with the mandelbrot ~181x host-to-RP2040
 * factor, the 3200-step proof lands under the ~3-minute
 * routine board budget even at the full SH_MAX pen. */
const toy_cart_t toy_cart_shepherd = {
    "shepherd", "Shepherd",
    "Grab a stray and slingshot it into the pen band. Tip the toy to shepherd the flock.",
    sizeof(shepherd_state), 6, 3200, 200,
    shepherd_reset, shepherd_step, shepherd_render,
    /* explicit WAVE-T2 tail: apply, params, param_count,
     * proof_inputs, proof_input_count, input_caps,
     * state_version, feed, sig_spec (signal-deaf) */
    shepherd_apply, shepherd_params, 3, shepherd_proof, 10,
    TOY_CAP_POINTER | TOY_CAP_TILT | TOY_CAP_PARAMS, 0, 0, { 0 }
};






