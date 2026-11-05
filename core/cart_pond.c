/* cart_pond.c - a shallow-water pond you can tip and drop stones in.
 *
 * Binding sketch: docs/study-fluid.md section 7 (toy4 fluid
 * study), landed here as briefed. A 48x48 binary64 wave /
 * shallow-water heightfield on a disc: one height grid h and
 * one velocity grid v, stepped by the fixed two-pass
 * symplectic-Euler wave update below. All state arithmetic
 * is muse-softfloat binary64 via toy_math.h, so the pond is
 * bit-identical on host, board and WASM. Derived from the
 * wave equation from first principles; no third-party code.
 *
 * Step (fixed counts, ZERO div/sqrt/trig - the division in
 * the seam's milli conversion happens only in apply(), and
 * the one coordinate division only in reset()):
 *   pass 1 (velocity): over disc-interior cells,
 *     lap(i) = h[l]+h[r]+h[u]+h[d] - 4*h[i]
 *     v[i]   = damp * (v[i] + k*lap(i) + tilt_k*(plane - h[i]))
 *   pass 2 (height):   h[i] += v[i]
 * with plane(x,y) = gx*xn[x] + gy*yn[y], the plane gravity
 * would make flat under the stored tilt, so tilt both
 * sloshes the pond and re-levels it. 11 soft ops per cell
 * (the study prototype's 12th, gy*yn, is hoisted per row).
 * Boundary cells (the span edge) are never written: they
 * stay at reset height 0, which is the pond wall.
 *
 * Stability. For one Fourier mode with mu = k*lambda +
 * tilt_k (lambda in [0,8] an eigenvalue of -lap; 8 is the
 * checkerboard), the state matrix for (h,v) has trace
 * 1 + damp - damp*mu and determinant damp. The Jury
 * conditions then give stability iff mu < 2*(1+damp)/damp
 * (~4.01 at damp 0.996), i.e. k <= ~0.5 - the study's limit.
 * Defaults k = 0.18, damp = 0.996, tilt_k = 0.002 give
 * mu_max = 1.442, deep inside. The wavespeed parameter is
 * range-capped at 0.30 in apply() (see below), so even the
 * worst legal settings give mu_max = 8*0.30 + 0.01 = 2.41:
 * the declared range IS the blow-up guard, and no input
 * path can install an unstable k. There is no division in
 * the step, so no divide-by-anything path exists either.
 *
 * Domain. A per-row integer span table built at reset()
 * (cell in iff (2x-47)^2 + (2y-47)^2 <= 45^2, in integers)
 * clips the square grid to the round glass; loop bounds
 * come from the spans, so the work is data-independent.
 * Coordinates: column 0 left, row 0 at the TOP of the
 * glass. Stored xn runs -1..+1 left to right, yn runs
 * +1..-1 top to bottom (sim y-up), matching the gravity
 * cartridge's tilt convention: a TILT event (milli-g,
 * +y display-down) is stored once as (gx, gy) with
 * gy = -ev.y, and the plane then rides high on the side
 * water falls toward (TILT (300,400) piles water toward
 * the display lower-right).
 *
 * Interaction (apply() only, step() stays input-free):
 * - POINTER_DOWN: a full-strength droplet added to h at
 *   the nearest cell (integer milli mapping, clamped to
 *   the span interior): centre +1.0, Chebyshev ring 1
 *   +0.4, ring 2 (large size only) +0.15. POINTER_MOVE
 *   adds the same shape at half strength - dragging stirs.
 *   Dedupe: a MOVE into the same clamped cell as the
 *   previous droplet is ignored, so wiggling a finger
 *   inside one cell cannot pump the pond without bound
 *   (julia's same-milli dedupe, one cell wide).
 * - DIAL chooses the droplet size (the brief's choice,
 *   documented here): a quarter turn (250 milliturns,
 *   integer-accumulated with the remainder kept) steps
 *   the size small (1 cell) -> medium (3x3, default) ->
 *   large (5x5), clamped at both ends.
 * - Attract: until the first pointer event ever lands
 *   (touched == 0 - tilt alone does not count, tipping
 *   the desk is not touching the water), step() rains a
 *   fixed medium droplet every 120 steps, the first at
 *   step 40, walking four fixed interior cells. Purely
 *   step-count driven, like julia's drift loop.
 *
 * Settings (SET_PARAM only, all live F64, clamped to the
 * declared range in apply(); NaN routes to the low bound
 * exactly like gravity's softening, because !ge(NaN, lo)):
 * "wavespeed" 0.05..0.30 def 0.18 (the k above - the 0.30
 * cap keeps the scheme far inside its 0.5 stability
 * limit), "damping" 0.98..0.9995 def 0.996 (per-step
 * velocity survival), "tilt_k" 0..0.01 def 0.002 (how
 * fast the surface hunts the tilted plane). reset()
 * installs the declared defaults through the step-0
 * SET_PARAM sequence every runner uses (julia's pattern).
 */
#include "toy_cart.h"
#include "toy_math.h"
#include "toy_util.h"

#define PN              48
#define PN_PARAM_WAVE   0u
#define PN_PARAM_DAMP   1u
#define PN_PARAM_TILTK  2u
#define PN_ATTRACT_FIRST 40u
#define PN_ATTRACT_EVERY 120u
#define PN_DIAL_STEP    250 /* milliturns per size step */

/* State-size arithmetic, descending alignment, no hidden
 * padding: h + v = 2*2304*8 = 36864, xn + yn = 768, nine
 * scalar f64 = 72, steps = 8, spans = 2*48*4 = 384, ten
 * 32-bit fields = 40. Total 38136 bytes of the 64 KiB
 * TOY_MAX_STATE_SIZE (58%), a multiple of 8. The ~27 KiB
 * of headroom is the study's deliberate reserve: exactly
 * one more f64 grid (a future dye field) with room over. */
typedef struct {
    f64 h[PN * PN];
    f64 v[PN * PN];
    f64 xn[PN];          /* -1..+1, left to right */
    f64 yn[PN];          /* +1..-1, top row to bottom (sim up) */
    f64 k;               /* wavespeed ("wavespeed" param) */
    f64 damp;            /* "damping" param */
    f64 tilt_k;          /* "tilt_k" param */
    f64 gx, gy;          /* last tilt, sim-up, unit = 1 g */
    f64 four;            /* 4, parsed at reset */
    f64 drop_c;          /* droplet amplitudes: centre 1.0 */
    f64 drop_r1;         /* ring 1: 0.4 */
    f64 drop_r2;         /* ring 2: 0.15 */
    uint64_t steps;
    int32_t span0[PN];   /* first in-disc cell per row (48 = empty) */
    int32_t span1[PN];   /* last in-disc cell per row (-1 = empty) */
    uint32_t drops;      /* droplets added (display) */
    uint32_t touched;    /* 1 once any pointer event landed */
    uint32_t down;       /* 1 while a contact is held */
    uint32_t contact_id;
    int32_t last_cx;     /* last droplet centre, -1 = none */
    int32_t last_cy;
    uint32_t drop_level; /* DIAL droplet size, 0..2 (radius) */
    int32_t dial_acc;    /* sub-step milliturn remainder */
    uint32_t attract_idx;
    uint32_t reserved;
} pond_state;

_Static_assert(sizeof(pond_state) == 38136, "pond_state layout pinned");
_Static_assert(sizeof(pond_state) <= TOY_MAX_STATE_SIZE, "pond state fits");

static inline f64 pn_milli(int32_t m) { return (f64)toy_f64_bits_from_milli(m); }
static inline int pn_ge(f64 a, f64 b) { msf_flags_t f = 0; return msf64_compare_quiet_greater_equal(a, b, &f); }
static inline int pn_gt(f64 a, f64 b) { msf_flags_t f = 0; return msf64_compare_quiet_greater(a, b, &f); }

static int32_t pn_clamp_milli(int32_t v)
{
    if (v > 1000) return 1000;
    if (v < -1000) return -1000;
    return v;
}

/* Clamp an f64 param into [lo, hi] (decimal strings, parsed
 * per call like gravity's softening). !ge also catches NaN
 * and -inf; the high test catches +inf. */
static f64 pn_clamp_f64(f64 v, const char *lo_s, const char *hi_s)
{
    f64 lo = f64_dec(lo_s), hi = f64_dec(hi_s);
    if (!pn_ge(v, lo)) return lo;
    if (pn_gt(v, hi)) return hi;
    return v;
}

/* Milli-radius (display, +y down) -> nearest grid cell,
 * integer only. Cell x centres sit at milli -1000 +
 * 2000*x/47, so the nearest x rounds (m+1000)*47/2000. */
static int pn_milli_to_cell(int32_t m)
{
    int32_t c = ((m + 1000) * (PN - 1) + 1000) / 2000;
    if (c < 0) c = 0;
    if (c > PN - 1) c = PN - 1;
    return (int)c;
}

/* Clamp a droplet centre into the stepped interior of its
 * row. Returns 0 (no droplet) when the row has no interior
 * - a touch off the pond disc itself. */
static int pn_clamp_center(const pond_state *s, int cx, int cy,
                           int *ocx, int *ocy)
{
    if (cy < 1) cy = 1;
    if (cy > PN - 2) cy = PN - 2;
    int lo = (int)s->span0[cy] + 1, hi = (int)s->span1[cy] - 1;
    if (lo > hi) return 0;
    if (cx < lo) cx = lo;
    if (cx > hi) cx = hi;
    *ocx = cx;
    *ocy = cy;
    return 1;
}

/* Add one droplet to h at an already-clamped centre: the
 * stored amplitudes over the Chebyshev rings 0..level,
 * optionally halved (pointer drags). Every written cell
 * is re-tested against its own row's interior span, so
 * the wall ring is never written. Bounded: at most 25
 * cells. */
static void pn_droplet_at(pond_state *s, int cx, int cy,
                          uint32_t level, int half)
{
    int r = (int)(level > 2u ? 2u : level);
    for (int dy = -r; dy <= r; dy++) {
        int y = cy + dy;
        if (y < 1 || y > PN - 2) continue;
        int lo = (int)s->span0[y] + 1, hi = (int)s->span1[y] - 1;
        for (int dx = -r; dx <= r; dx++) {
            int x = cx + dx;
            if (x < lo || x > hi) continue;
            int ring = dx < 0 ? -dx : dx;
            int ady = dy < 0 ? -dy : dy;
            if (ady > ring) ring = ady;
            f64 amp = ring == 0 ? s->drop_c
                      : ring == 1 ? s->drop_r1 : s->drop_r2;
            if (half) amp = f64_mul(amp, F64_HALF);
            int i = y * PN + x;
            s->h[i] = f64_add(s->h[i], amp);
        }
    }
    s->drops++;
}

/* Pointer droplet with the one-cell dedupe: a MOVE whose
 * clamped centre equals the previous droplet's centre is
 * a no-op. DOWN passes dedupe = 0 (a fresh press always
 * drops) and records the centre either way. */
static void pn_pointer_droplet(pond_state *s, int32_t mx, int32_t my,
                               int half, int dedupe)
{
    int cx, cy;
    if (!pn_clamp_center(s, pn_milli_to_cell(mx), pn_milli_to_cell(my),
                         &cx, &cy))
        return;
    if (dedupe && cx == s->last_cx && cy == s->last_cy) return;
    pn_droplet_at(s, cx, cy, s->drop_level, half);
    s->last_cx = cx;
    s->last_cy = cy;
}

/* apply(): events -> state effects only (seam rules). */
static void pond_apply(void *st_, const toy_event_t *ev)
{
    pond_state *s = (pond_state *)st_;
    switch (ev->type) {
    case TOY_EV_POINTER_DOWN:
        s->touched = 1;
        s->down = 1;
        s->contact_id = ev->id;
        pn_pointer_droplet(s, pn_clamp_milli(ev->x), pn_clamp_milli(ev->y),
                           0, 0);
        break;
    case TOY_EV_POINTER_MOVE:
        /* Stirring works on hover (mouse) and drag (touch)
         * alike, julia-style; the dedupe bounds the pump. */
        s->touched = 1;
        pn_pointer_droplet(s, pn_clamp_milli(ev->x), pn_clamp_milli(ev->y),
                           1, 1);
        break;
    case TOY_EV_POINTER_UP:
        if (s->down && ev->id == s->contact_id) {
            s->down = 0;
            s->last_cx = -1;
            s->last_cy = -1;
        }
        break;
    case TOY_EV_TILT:
        /* Stored milli vector -> sim-up unit-g (gravity's
         * convention). Tilt does not set touched. */
        s->gx = pn_milli(pn_clamp_milli(ev->x));
        s->gy = pn_milli(-pn_clamp_milli(ev->y));
        break;
    case TOY_EV_DIAL: {
        /* Droplet size: 250 milliturns per step, integer
         * accumulator with remainder kept, levels clamped
         * 0..2 (turns past an end are consumed, so turning
         * back answers at once). Per-event delta clamped
         * so no event can overflow the accumulator. */
        int32_t dv = ev->v;
        if (dv > 2000) dv = 2000;
        if (dv < -2000) dv = -2000;
        int32_t acc = s->dial_acc + dv;
        uint32_t lvl = s->drop_level > 2u ? 2u : s->drop_level;
        while (acc >= PN_DIAL_STEP) {
            acc -= PN_DIAL_STEP;
            if (lvl < 2u) lvl++;
        }
        while (acc <= -PN_DIAL_STEP) {
            acc += PN_DIAL_STEP;
            if (lvl > 0u) lvl--;
        }
        s->dial_acc = acc;
        s->drop_level = lvl;
        break;
    }
    case TOY_EV_SET_PARAM:
        if (ev->id == PN_PARAM_WAVE)
            s->k = pn_clamp_f64((f64)ev->bits, "0.05", "0.30");
        else if (ev->id == PN_PARAM_DAMP)
            s->damp = pn_clamp_f64((f64)ev->bits, "0.98", "0.9995");
        else if (ev->id == PN_PARAM_TILTK)
            s->tilt_k = pn_clamp_f64((f64)ev->bits, "0", "0.01");
        break; /* out-of-range ids ignored, per the seam */
    default:
        break; /* unknown types ignored, per the seam */
    }
}

static void pond_reset(void *st_)
{
    pond_state *s = (pond_state *)st_;
    toy_memset(s, 0, sizeof *s);
    /* Disc spans, integers only (the prototype's test):
     * (2x-47)^2 + (2y-47)^2 <= 45^2. */
    for (int y = 0; y < PN; y++) {
        int best0 = PN, best1 = -1;
        for (int x = 0; x < PN; x++) {
            int dx = 2 * x - (PN - 1), dy = 2 * y - (PN - 1);
            if (dx * dx + dy * dy <= (PN - 3) * (PN - 3)) {
                if (x < best0) best0 = x;
                if (x > best1) best1 = x;
            }
        }
        s->span0[y] = best0;
        s->span1[y] = best1;
    }
    /* Cell coordinates -1..+1: ONE softfloat division,
     * here at reset only; the step divides by nothing.
     * (The study prototype used a 64-agnostic 2/63 pitch;
     * the cartridge spans its own 48 cells exactly.) */
    f64 cstep = f64_div(F64_TWO, f64_dec("47"));
    f64 cx = f64_neg(F64_ONE), cy = F64_ONE;
    for (int i = 0; i < PN; i++) {
        s->xn[i] = cx;
        cx = f64_add(cx, cstep);
        s->yn[i] = cy; /* row 0 is the top of the glass: +1 */
        cy = f64_sub(cy, cstep);
    }
    s->four = f64_dec("4");
    s->drop_c = F64_ONE; /* structural constant: +1.0 */
    s->drop_r1 = f64_dec("0.4");
    s->drop_r2 = f64_dec("0.15");
    s->last_cx = -1;
    s->last_cy = -1;
    s->drop_level = 1; /* medium 3x3 */
    /* Param defaults through the same step-0 SET_PARAM
     * sequence a runner uses for user settings (julia's
     * pattern): reset() provably means the declared
     * defaults wavespeed 0.18 / damping 0.996 /
     * tilt_k 0.002. h and v stay zero: a flat, still pond
     * until the step-40 attract rain. */
    for (uint32_t i = 0; i < toy_cart_pond.param_count; i++) {
        toy_event_t ev;
        if (toy_param_event(&toy_cart_pond, i,
                            toy_cart_pond.params[i].i_def, 0, &ev) == 0)
            pond_apply(s, &ev);
    }
    /* gx, gy, steps, drops, touched, down, dial_acc,
     * attract_idx: zero from the memset. */
}

/* Attract rain cells (column, row), all well inside the
 * disc; the first is the proof droplet's own cell. */
static const int pn_attract[4][2] = {
    { 16, 24 }, { 32, 16 }, { 24, 33 }, { 30, 27 }
};

static void pond_step(void *st_)
{
    pond_state *s = (pond_state *)st_;
    s->steps++;
    if (!s->touched && s->steps >= PN_ATTRACT_FIRST &&
        ((s->steps - PN_ATTRACT_FIRST) % PN_ATTRACT_EVERY) == 0) {
        uint32_t a = s->attract_idx & 3u;
        s->attract_idx++;
        pn_droplet_at(s, pn_attract[a][0], pn_attract[a][1], 1, 0);
    }
    /* Pass 1: velocity from the height Laplacian plus the
     * tilt-plane hunt, then damping. */
    for (int y = 1; y < PN - 1; y++) {
        int lo = (int)s->span0[y] + 1, hi = (int)s->span1[y] - 1;
        f64 gy_yn = f64_mul(s->gy, s->yn[y]);
        for (int x = lo; x <= hi; x++) {
            int i = y * PN + x;
            f64 lap = f64_sub(
                f64_add(f64_add(s->h[i - 1], s->h[i + 1]),
                        f64_add(s->h[i - PN], s->h[i + PN])),
                f64_mul(s->four, s->h[i]));
            f64 plane = f64_fma(s->gx, s->xn[x], gy_yn);
            f64 v = f64_fma(s->k, lap, s->v[i]);
            v = f64_fma(s->tilt_k, f64_sub(plane, s->h[i]), v);
            s->v[i] = f64_mul(s->damp, v);
        }
    }
    /* Pass 2: heights ride the fresh velocities. */
    for (int y = 1; y < PN - 1; y++) {
        int lo = (int)s->span0[y] + 1, hi = (int)s->span1[y] - 1;
        for (int x = lo; x <= hi; x++) {
            int i = y * PN + x;
            s->h[i] = f64_add(s->h[i], s->v[i]);
        }
    }
}

/* Display conversion only (never feeds state). */
static double pn_d64(f64 a) { union { f64 u; double d; } v; v.u = a; return v.d; }

/* Water shading from stored heights: height lifts the
 * colour from deep navy toward bright cyan, and a fake
 * light from the upper-left reads the slope (neighbour
 * differences of state, native arithmetic in the renderer
 * only, as the rendering rules allow). The value is
 * clamped before any cast, so a (never expected) non-
 * finite height renders as flat water, not UB. */
static uint16_t pn_water(const pond_state *s, int x, int y)
{
    int i = y * PN + x;
    double hv = pn_d64(s->h[i]);
    double sx = pn_d64(s->h[i + 1]) - pn_d64(s->h[i - 1]);
    double sy = pn_d64(s->h[i + PN]) - pn_d64(s->h[i - PN]);
    double t = hv * 1.2 - (sx + sy) * 2.5;
    if (!(t >= -1.0)) t = t < -1.0 ? -1.0 : 0.0; /* NaN -> flat */
    if (t > 1.0) t = 1.0;
    unsigned r, g, b;
    if (t >= 0.0) { /* still water (14,58,110) -> crest (225,250,255) */
        unsigned u = (unsigned)(t * 255.0);
        r = 14u + (211u * u) / 255u;
        g = 58u + (192u * u) / 255u;
        b = 110u + (145u * u) / 255u;
    } else { /* trough (4,18,48) -> still water */
        unsigned u = (unsigned)(-t * 255.0);
        r = 14u - (10u * u) / 255u;
        g = 58u - (40u * u) / 255u;
        b = 110u - (62u * u) / 255u;
    }
    return toy_rgb565(r, g, b);
}

static void pond_render(const void *st_, toy_fb_t *fb)
{
    const pond_state *s = (const pond_state *)st_;
    int cx = toy_cx(fb), cy = toy_cy(fb), r = toy_r(fb);
    toy_fb_clear(fb, toy_rgb565(5, 8, 20));
    toy_fb_ring(fb, cx, cy, r - 2, toy_rgb565(28, 44, 70));

    /* Per-pixel cells over the pond square (half-width
     * r-4, so the stored coordinates span the glass and
     * the disc spans clip to the water). Boundary cells
     * are the wall: still water at h = 0 by construction;
     * pn_water reads their interior neighbours only via
     * cells that exist, so shrink-wrap the slope reads by
     * drawing boundary cells from the flat colour. */
    int half = r - 4;
    if (half < PN) half = PN;
    uint16_t flat = toy_rgb565(14, 58, 110);
    for (int py = cy - half; py < cy + half; py++) {
        int gj = ((py - (cy - half)) * PN) / (2 * half);
        if (gj < 0 || gj >= PN) continue;
        int lo = (int)s->span0[gj], hi = (int)s->span1[gj];
        if (hi < lo) continue;
        for (int px = cx - half; px < cx + half; px++) {
            int gi = ((px - (cx - half)) * PN) / (2 * half);
            if (gi < lo || gi > hi) continue;
            uint16_t c = flat;
            if (gj >= 1 && gj <= PN - 2 && gi >= lo + 1 && gi <= hi - 1)
                c = pn_water(s, gi, gj);
            toy_fb_px(fb, px, py, c);
        }
    }

    /* Tilt spoke (gravity's idiom): from the centre toward
     * where the water falls, in display space. The surface
     * shading carries the same story; the spoke names it. */
    if ((s->gx & ~F64_SIGN) != 0 || (s->gy & ~F64_SIGN) != 0) {
        int tx = cx + (int)(pn_d64(s->gx) * (double)r * 0.30);
        int ty = cy - (int)(pn_d64(s->gy) * (double)r * 0.30);
        toy_fb_line(fb, cx, cy, tx, ty, toy_rgb565(255, 210, 80));
        toy_fb_disc(fb, tx, ty, 2, toy_rgb565(255, 230, 140));
    }

    int sc = r >= 150 ? 2 : 1;
    toy_fb_text(fb, cx - 8 * sc, cy - r + 10 * sc, sc,
                toy_rgb565(160, 200, 235), "POND");
    char buf[14];
    buf[0] = 'D'; buf[1] = 'R'; buf[2] = 'O'; buf[3] = 'P';
    buf[4] = 'S'; buf[5] = ' ';
    uint32_t d = s->drops > 999u ? 999u : s->drops; /* label width */
    int n = 6;
    if (d >= 100u) buf[n++] = (char)('0' + (d / 100u) % 10u);
    if (d >= 10u) buf[n++] = (char)('0' + (d / 10u) % 10u);
    buf[n++] = (char)('0' + d % 10u);
    buf[n] = '\0';
    toy_fb_text(fb, cx - (n * 4 * sc) / 2, cy + r - 16 * sc, sc,
                toy_rgb565(160, 200, 235), buf);
}

static const toy_param_t pond_params[] = {
    { "wavespeed", "Wave speed", TOY_PARAM_F64, 0,
      0, 0, 0, "0.05", "0.30", "0.18", 0 },
    { "damping", "Damping", TOY_PARAM_F64, 0,
      0, 0, 0, "0.98", "0.9995", "0.996", 0 },
    { "tilt_k", "Tilt pull", TOY_PARAM_F64, 0,
      0, 0, 0, "0", "0.01", "0.002", 0 },
};

/* Proof script (study section 7 concept): 400 steps,
 * hashes every 100, declared defaults throughout (no
 * step-0 params - the defaults ARE the proof). Pointer
 * and tilt milli are display space (+y down).
 *   1..119  flat pond; step 20 TILT (300,400) starts the
 *           slosh toward display lower-right; the step-40
 *           attract droplet (proof runs untouched until
 *           120) rings across the tilted surface.
 *   120     droplet tap at (-333, 0) - cell (16,24), the
 *           attract's own first cell - ends the attract
 *           rain; rings cross in ~60 steps (wavespeed
 *           sqrt(0.18) ~0.42 cells/step).
 *   220     TILT (-400,-100): slosh back toward display
 *           upper-left over the returning rings.
 *   300     droplet tap at (250,-250) - interference with
 *           the returning slosh, the money frame.
 *   340     TILT (0,0): settle toward flat to step 400.
 * Each tap is DOWN + UP one step apart (a released touch,
 * as julia/gravity model it). Sorted by step, 1 per step. */
static const toy_proof_input_t pond_proof[] = {
    {  20, { TOY_EV_TILT,         0,  300,  400, 0, 0, 0 } },
    { 120, { TOY_EV_POINTER_DOWN, 0, -333,    0, 0, 0, 0 } },
    { 121, { TOY_EV_POINTER_UP,   0, -333,    0, 0, 0, 0 } },
    { 220, { TOY_EV_TILT,         0, -400, -100, 0, 0, 0 } },
    { 300, { TOY_EV_POINTER_DOWN, 0,  250, -250, 0, 0, 0 } },
    { 301, { TOY_EV_POINTER_UP,   0,  250, -250, 0, 0, 0 } },
    { 340, { TOY_EV_TILT,         0,    0,    0, 0, 0, 0 } },
};

/* Pacing/proof. Demo 2 steps/frame at a nominal 30 fps
 * (study section 7): droplet rings cross the pond in
 * roughly a second of wall time where sustained. Honest
 * sustain note (parcel report carries the measurements):
 * the host step is ~1.7-1.9 ms, so the browser is
 * expected live at this rate; on today's RP2040 proof
 * board the same step costs cloud-class time (ESTIMATE
 * ~0.25-0.33 s/step from the study's x140-170 host-to-
 * board band - nothing fluid-shaped has run on the
 * board yet), so the Pico shows this pond in slow
 * motion - the blurb says so. RP2350 re-measurement is
 * future work. Proof 400 steps report 100: board
 * ESTIMATE ~1.6-2.2 min by that band; if the first board
 * timing lands high, trim the horizon (cloud precedent),
 * never the maths. */
const toy_cart_t toy_cart_pond = {
    "pond", "Pond",
    "Touch drops a stone, tilt sloshes. Browser-live; the Pico runs it in slow motion.",
    sizeof(pond_state), 2, 400, 100,
    pond_reset, pond_step, pond_render,
    /* explicit WAVE-T2 tail: apply, params, param_count,
     * proof_inputs, proof_input_count, input_caps,
     * state_version, feed, sig_spec (signal-deaf) */
    pond_apply, pond_params, 3, pond_proof, 7,
    TOY_CAP_POINTER | TOY_CAP_TILT | TOY_CAP_PARAMS | TOY_CAP_DIAL, 0, 0, { 0 }
};


