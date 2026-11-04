/* cart_julia.c - Julia set for a touch-chosen constant c.
 *
 * The first interactive cartridge: the filled Julia set of
 * z -> z^2 + c, computed incrementally exactly like mandelbrot
 * (one 80-pixel grid row per step, escape counts in state, a
 * short hold on completion), but c is user-controlled through
 * the input seam (docs/CARTRIDGES.md, "Input & settings seam").
 * apply() is pre-step integer->state only: it never calls
 * step(), never allocates, and converts event integers to
 * binary64 state exclusively through the softfloat plumbing
 * in toy_input.h/c. step() itself stays input-free.
 *
 * Pointer -> c mapping (touchscreen-first). Pointer events
 * carry milli-radius coordinates (origin at the glass centre,
 * +x right, +y DOWN, nominal disc x,y in [-1000,1000]). The
 * mapping is kept in integer milli units end to end:
 *
 *     c_re = (x - 250) / 1000      range [-1.25, +0.75]
 *     c_im = -y / 1000             range [-1.00, +1.00]
 *
 * (the y sign flips display-down to mathematical-up). That
 * rectangle circumscribes the interesting part of the
 * c-plane: the main cardioid (cusp at +0.25, left edge -0.75,
 * |Im| up to about 0.65), the period-2 bulb (centre -1,
 * radius 0.25, reaching -1.25 on the real axis), Seahorse
 * Valley between them, and the period-3 rabbit buds near
 * -0.123 +- 0.745i. For a quadratic polynomial the filled
 * Julia set is connected exactly when c lies in the
 * Mandelbrot set (the Fatou-Julia dichotomy: connected iff
 * the critical orbit of 0 stays bounded), so dragging a
 * finger across this box crosses the connected <-> Cantor-
 * dust transition again and again; the discarded rectangle
 * corners outside the touch disc are almost all dust. Milli
 * quantisation makes c resolution exactly 0.001, which is
 * also why every scripted c below has three decimals: each
 * is reached exactly by one integer pointer event.
 *
 * Idle behaviour: an attract loop. Until the first pointer
 * event ever arrives (touched == 0), each completed grid is
 * followed by a state-driven move of c to the next point of
 * an 8-point milli circle of radius 0.040 around the default
 * c, so the untouched desk toy keeps gently morphing - no
 * wall clock anywhere, the phase is the drift_idx state
 * field. The first touch ends the attract loop for good: a
 * c the user chose is never taken away from them, and a
 * completed user grid simply holds (steps stay near-free).
 *
 * Settings (params table, entered only as SET_PARAM events):
 * "iter" is the escape cap (32..160, default 128) and "zoom"
 * is an enum of z-plane half-widths 1.80/1.50/1.20/0.90
 * (default 1.50, the classic textbook window). Either
 * changing means the grid no longer describes the state, so
 * apply() restarts the row cursor deterministically.
 *
 * Escape maths is binary64 via toy_math.h, mirroring
 * cart_mandelbrot.c's 8-op loop (mul xy, fma 2xy+c_im,
 * sub x2-y2, add +c_re, two squares, add, compare) - except
 * z starts at the pixel instead of at 0, and c is the state
 * constant instead of the pixel.
 */
#include "toy_cart.h"
#include "toy_math.h"
#include "toy_util.h"

#define JL_GW            80
#define JL_GH            80
#define JL_ROWS_PER_STEP 1
#define JL_HOLD_STEPS    16u
#define JL_ITER_MIN      32
#define JL_ITER_MAX      160
#define JL_PARAM_ITER    0u
#define JL_PARAM_ZOOM    1u

/* State-size arithmetic: 3 f64 + 1 u64 = 32 bytes, ten u32
 * fields (one explicit reserved pad) = 40 bytes, header 72;
 * grid 80*80*2 = 12800 bytes; total 12872 bytes, 20% of the
 * 64 KiB TOY_MAX_STATE_SIZE, a multiple of 8 with fields in
 * descending-alignment order, no hidden padding. An 80x80
 * grid (not mandelbrot's 96x96) bounds one full recompute
 * to 80 row-steps at a cap of at most 160: the proof below
 * covers four complete grids in 320 compute steps, sized
 * against mandelbrot's measured on-device row cost (see the
 * costing note at the descriptor). */
typedef struct {
    f64 c_re, c_im;           /* current c, binary64 bits */
    f64 halfwidth;            /* z-plane half-width (zoom param) */
    uint64_t steps;
    int32_t c_re_milli;       /* exact milli c (label + dedupe) */
    int32_t c_im_milli;
    int32_t max_iter;         /* escape cap ("iter" param effect) */
    uint32_t zoom_idx;        /* "zoom" enum effect, 0..3 */
    uint32_t cursor;          /* next linear pixel index, 0..GW*GH */
    uint32_t hold;            /* steps left showing a completed grid */
    uint32_t touched;         /* 1 once any pointer event landed */
    uint32_t down;            /* 1 while a contact is on the glass */
    uint32_t drift_idx;       /* attract-loop phase, 0..7 */
    uint32_t reserved;        /* explicit pad, zeroed by reset memset */
    uint16_t grid[JL_GH][JL_GW]; /* escape counts 1..max_iter */
} julia_state;

/* Default c at reset: "-0.800" + "0.156"i, the swirling-
 * branches example from the picture gallery in Wikipedia,
 * "Julia set", Examples section (one of its canonical
 * plotted constants). Milli: (-800, 156). */
#define JL_DEF_RE_MILLI (-800)
#define JL_DEF_IM_MILLI 156

/* Attract loop: absolute milli c values on a radius-40
 * circle around the default (40 on the axes, 28 on the
 * diagonals), starting due east and running anticlockwise
 * in mathematical orientation. Visited only while
 * touched == 0, one point per completed grid. */
static const int32_t jl_drift[8][2] = {
    { -760, 156 }, { -772, 184 }, { -800, 196 }, { -828, 184 },
    { -840, 156 }, { -828, 128 }, { -800, 116 }, { -772, 128 }
};

/* Zoom enum names double as the decimal strings the
 * halfwidth is parsed from: one source for UI and state. */
static const char *const jl_zoom_names[] = { "1.80", "1.50", "1.20", "0.90" };
static const int32_t jl_zoom_milli[] = { 1800, 1500, 1200, 900 };

static inline f64 jl_from_i32(int32_t a) { return (f64)toy_f64_bits_from_i32(a); }
static inline int jl_gt(f64 a, f64 b) { msf_flags_t f = 0; return msf64_compare_quiet_greater(a, b, &f); }

static int32_t jl_clamp_coord(int32_t v)
{
    if (v > 1000) return 1000;
    if (v < -1000) return -1000;
    return v;
}

/* Store a new c (milli integers -> binary64 bits through the
 * seam helper, one softfloat divide by 1000 each) and restart
 * the grid. A repeat of the current milli c is not a change:
 * dragging within one 0.001 cell must not restart the sweep. */
static void jl_set_c_milli(julia_state *s, int32_t re_milli, int32_t im_milli)
{
    if (re_milli == s->c_re_milli && im_milli == s->c_im_milli) return;
    s->c_re_milli = re_milli;
    s->c_im_milli = im_milli;
    s->c_re = (f64)toy_f64_bits_from_milli(re_milli);
    s->c_im = (f64)toy_f64_bits_from_milli(im_milli);
    s->cursor = 0;
    s->hold = 0;
}

/* apply(): events -> state effects only (seam rules). */
static void julia_apply(void *st_, const toy_event_t *ev)
{
    julia_state *s = (julia_state *)st_;
    switch (ev->type) {
    case TOY_EV_POINTER_DOWN:
        s->down = 1;
        s->touched = 1;
        jl_set_c_milli(s, jl_clamp_coord(ev->x) - 250, -jl_clamp_coord(ev->y));
        break;
    case TOY_EV_POINTER_MOVE:
        /* Position sets c whether the runner sends moves only
         * during a contact (touch) or also on hover (mouse). */
        s->touched = 1;
        jl_set_c_milli(s, jl_clamp_coord(ev->x) - 250, -jl_clamp_coord(ev->y));
        break;
    case TOY_EV_POINTER_UP:
        s->down = 0; /* c stays where the finger left it */
        break;
    case TOY_EV_SET_PARAM:
        if (ev->id == JL_PARAM_ITER) {
            int32_t v = ev->v;
            if (v < JL_ITER_MIN) v = JL_ITER_MIN;
            if (v > JL_ITER_MAX) v = JL_ITER_MAX;
            if (v != s->max_iter) {
                s->max_iter = v;
                s->cursor = 0;
                s->hold = 0;
            }
        } else if (ev->id == JL_PARAM_ZOOM) {
            int32_t v = ev->v;
            if (v < 0) v = 0;
            if (v > 3) v = 3;
            if ((uint32_t)v != s->zoom_idx || s->halfwidth == F64_ZERO) {
                s->zoom_idx = (uint32_t)v;
                s->halfwidth = f64_dec(jl_zoom_names[v]);
                s->cursor = 0;
                s->hold = 0;
            }
        }
        break; /* out-of-range ids ignored, per the seam */
    default:
        break; /* unknown types ignored, per the seam */
    }
}

static void julia_reset(void *st_)
{
    julia_state *s = (julia_state *)st_;
    toy_memset(s, 0, sizeof *s);
    s->c_re_milli = JL_DEF_RE_MILLI;
    s->c_im_milli = JL_DEF_IM_MILLI;
    s->c_re = (f64)toy_f64_bits_from_milli(JL_DEF_RE_MILLI);
    s->c_im = (f64)toy_f64_bits_from_milli(JL_DEF_IM_MILLI);
    /* Then the descriptor's param defaults, applied through
     * the same step-0 sequence a runner uses for user
     * settings (study-input 5.5): one SET_PARAM event per
     * param, in index order, built by toy_param_event and
     * landed by julia_apply, so reset() provably means the
     * declared defaults and nothing else. The halfwidth==
     * ZERO guard in apply lets the zoom default land even
     * though a zeroed zoom_idx could equal the default
     * index in a future edit. */
    for (uint32_t i = 0; i < toy_cart_julia.param_count; i++) {
        toy_event_t ev;
        if (toy_param_event(&toy_cart_julia, i,
                            toy_cart_julia.params[i].i_def, 0, &ev) == 0)
            julia_apply(s, &ev);
    }
    /* cursor, hold, touched, down, drift_idx, steps: zero. */
}

/* Escape iterations for z -> z^2 + c from z = (zx, zy),
 * binary64, mandelbrot's 8-op shape plus the two initial
 * squares. Returns the 1-based iteration on which |z|^2 > 4,
 * else cap (pixel counted interior at this cap). */
static uint16_t jl_escape(f64 zx, f64 zy, f64 cre, f64 cim, f64 four,
                          int32_t cap)
{
    f64 x = zx, y = zy;
    f64 x2 = f64_mul(x, x), y2 = f64_mul(y, y);
    for (int32_t n = 1; n <= cap; n++) {
        f64 xy = f64_mul(x, y);
        y = f64_fma(F64_TWO, xy, cim);
        x = f64_add(f64_sub(x2, y2), cre);
        x2 = f64_mul(x, x);
        y2 = f64_mul(y, y);
        if (jl_gt(f64_add(x2, y2), four)) return (uint16_t)n;
    }
    return (uint16_t)cap;
}

/* One full grid row j into state (bounded: JL_GW pixels).
 * Pixel (i, j) is z0 = ((2i-(W-1)) * unit, ((H-1)-2j) * unit)
 * with unit = halfwidth / W, +Im up, all softfloat. */
static void jl_compute_row(julia_state *s, uint32_t j)
{
    int32_t cap = s->max_iter;
    if (cap < 1) cap = 1;
    if (cap > JL_ITER_MAX) cap = JL_ITER_MAX;
    f64 four = f64_mul(F64_TWO, F64_TWO);
    f64 unit = f64_div(s->halfwidth, jl_from_i32(JL_GW));
    int32_t tim = (int32_t)(JL_GH - 1) - 2 * (int32_t)j;
    f64 zy = f64_mul(jl_from_i32(tim), unit);
    for (int i = 0; i < JL_GW; i++) {
        f64 zx = f64_mul(jl_from_i32(2 * i - (JL_GW - 1)), unit);
        s->grid[j][i] = jl_escape(zx, zy, s->c_re, s->c_im, four, cap);
    }
}

static void julia_step(void *st_)
{
    julia_state *s = (julia_state *)st_;
    s->steps++;
    if (s->hold > 0) {
        /* Show the completed grid. Bounded: no pixels. */
        s->hold--;
        return;
    }
    if (s->cursor >= (uint32_t)(JL_GW * JL_GH)) {
        if (s->touched) return; /* user's grid holds indefinitely */
        /* Attract loop: next milli point, then fall through
         * and compute row 0 in the same step (mandelbrot's
         * stage-change shape) so no frame shows a stale grid
         * under a new c label. */
        uint32_t d = s->drift_idx & 7u;
        s->drift_idx = (d + 1u) & 7u;
        jl_set_c_milli(s, jl_drift[d][0], jl_drift[d][1]);
        s->cursor = 0; /* set_c also restarts; explicit for safety */
    }
    uint32_t row = s->cursor / (uint32_t)JL_GW;
    jl_compute_row(s, row);
    s->cursor += (uint32_t)(JL_ROWS_PER_STEP * JL_GW);
    if (s->cursor >= (uint32_t)(JL_GW * JL_GH))
        s->hold = JL_HOLD_STEPS;
}

/* Integer-only colour: mandelbrot's 48-step cyclic gradient,
 * with interior decided against the live cap. */
static uint16_t jl_colour(uint16_t n, int32_t cap)
{
    if ((int32_t)n >= cap) return toy_rgb565(2, 2, 6); /* interior */
    static const unsigned char stops[7][3] = {
        {10, 16, 90}, {0, 170, 230}, {20, 220, 120}, {250, 210, 60},
        {240, 90, 30}, {200, 40, 180}, {10, 16, 90}
    };
    unsigned ph = n % 48u, seg = ph / 8u, fr = ph % 8u;
    unsigned r = (stops[seg][0] * (8u - fr) + stops[seg + 1][0] * fr) / 8u;
    unsigned g = (stops[seg][1] * (8u - fr) + stops[seg + 1][1] * fr) / 8u;
    unsigned b = (stops[seg][2] * (8u - fr) + stops[seg + 1][2] * fr) / 8u;
    return toy_rgb565(r, g, b);
}

static void jl_fill_rect(toy_fb_t *fb, int x, int y, int w, int h, uint16_t c)
{
    for (int yy = y; yy < y + h; yy++)
        for (int xx = x; xx < x + w; xx++) toy_fb_px(fb, xx, yy, c);
}

/* Append unsigned decimal of v at buf[k]; return new k. */
static int jl_put_u32(char *buf, int k, uint32_t v)
{
    char tmp[10];
    int t = 0;
    if (v == 0) tmp[t++] = '0';
    while (v > 0) { tmp[t++] = (char)('0' + (v % 10u)); v /= 10u; }
    while (t > 0) buf[k++] = tmp[--t];
    return k;
}

/* Append a milli value as -0.123 (plus when forced), integer
 * formatting only - the c on the label is the exact milli c
 * in state, never a float round-trip. */
static int jl_put_milli(char *buf, int k, int32_t v, int force_plus)
{
    uint32_t a;
    if (v < 0) { buf[k++] = '-'; a = (uint32_t)(-v); }
    else { if (force_plus) buf[k++] = '+'; a = (uint32_t)v; }
    k = jl_put_u32(buf, k, a / 1000u);
    buf[k++] = '.';
    uint32_t f = a % 1000u;
    buf[k++] = (char)('0' + (f / 100u));
    buf[k++] = (char)('0' + ((f / 10u) % 10u));
    buf[k++] = (char)('0' + (f % 10u));
    return k;
}

static void julia_render(const void *st_, toy_fb_t *fb)
{
    const julia_state *s = (const julia_state *)st_;
    int cx = toy_cx(fb), cy = toy_cy(fb), r = toy_r(fb);
    uint16_t bg = toy_rgb565(8, 10, 22);
    uint16_t plate = toy_rgb565(10, 12, 28);
    uint16_t ink = toy_rgb565(190, 210, 235);
    toy_fb_clear(fb, bg);
    toy_fb_ring(fb, cx, cy, r - 2, toy_rgb565(28, 34, 54));

    /* Grid inscribed in the round glass: side ~1.40*r. */
    int side = (r * 140) / 100;
    int x0 = cx - side / 2, y0 = cy - side / 2;
    uint16_t uncomp = toy_rgb565(22, 26, 48);
    int32_t cap = s->max_iter < 1 ? 1 : s->max_iter;
    for (int y = y0; y < y0 + side; y++) {
        int gj = ((y - y0) * JL_GH) / side;
        for (int x = x0; x < x0 + side; x++) {
            int gi = ((x - x0) * JL_GW) / side;
            uint32_t idx = (uint32_t)(gj * JL_GW + gi);
            toy_fb_px(fb, x, y, idx < s->cursor || s->hold > 0
                                    ? jl_colour(s->grid[gj][gi], cap) : uncomp);
        }
    }

    /* Computation cursor scanline (only while computing). */
    if (s->hold == 0 && s->cursor < (uint32_t)(JL_GW * JL_GH)) {
        int crow = (int)(s->cursor / (uint32_t)JL_GW);
        int ycur = y0 + (crow * side) / JL_GH;
        toy_fb_line(fb, x0, ycur, x0 + side - 1, ycur, toy_rgb565(255, 210, 80));
    }

    /* Where c itself sits, as a point of its own z-plane:
     * a small cross, gold while a finger is down, skipped
     * when the zoomed window does not contain it. Integer
     * milli placement against the zoom enum's milli width. */
    {
        uint32_t zi = s->zoom_idx > 3u ? 3u : s->zoom_idx;
        int32_t hw = jl_zoom_milli[zi];
        if (s->c_re_milli > -hw && s->c_re_milli < hw &&
            s->c_im_milli > -hw && s->c_im_milli < hw) {
            int mx = x0 + (int)(((int64_t)(s->c_re_milli + hw) * side) / (2 * (int64_t)hw));
            int my = y0 + (int)(((int64_t)(hw - s->c_im_milli) * side) / (2 * (int64_t)hw));
            uint16_t mc = s->down ? toy_rgb565(255, 210, 80)
                                  : toy_rgb565(240, 245, 255);
            int arm = 3 + r / 60;
            toy_fb_line(fb, mx - arm, my, mx + arm, my, mc);
            toy_fb_line(fb, mx, my - arm, mx, my + arm, mc);
        }
    }

    int sc = r >= 150 ? 2 : 1;
    int tw = 4 * sc; /* per glyph advance */

    /* c label on a plate inside the grid's bottom edge, well
     * within the inscribed circle (checked at 240 and 360):
     * "C=-0.123+0.745I", at most 15 glyphs. */
    char buf[20];
    int k = 0;
    buf[k++] = 'C'; buf[k++] = '=';
    k = jl_put_milli(buf, k, s->c_re_milli, 0);
    k = jl_put_milli(buf, k, s->c_im_milli, 1);
    buf[k++] = 'I';
    buf[k] = '\0';
    int bw = k * tw + 2;
    jl_fill_rect(fb, cx - bw / 2, y0 + side - 5 * sc - 6, bw, 5 * sc + 4, plate);
    toy_fb_text(fb, cx - bw / 2 + 1, y0 + side - 5 * sc - 4, sc, ink, buf);

    toy_fb_text(fb, cx - (5 * tw) / 2, cy - r + 8 * sc, sc, ink, "JULIA");
}

static const toy_param_t julia_params[] = {
    { "iter", "Max iterations", TOY_PARAM_I32, 0,
      JL_ITER_MIN, JL_ITER_MAX, 128, 0, 0, 0, 0 },
    { "zoom", "View half-width", TOY_PARAM_ENUM, 0,
      0, 3, 1, 0, 0, 0, jl_zoom_names },
};

/* Proof script: four complete grids, one per 96-step cycle
 * (80 row-steps + 16 hold), hashes at every cycle boundary.
 *   cycle 1 (1..96):    idle default c "-0.800"+"0.156"i,
 *     no events - the untouched attract start.
 *   cycle 2 (97..192):  attract drift to "-0.760"+"0.156"i
 *     inside step 97 - proves the idle path moves c with no
 *     input at all.
 *   cycle 3 (193..288): POINTER_DOWN drags c to the Douady
 *     rabbit "-0.123"+"0.745"i (Wikipedia, "Douady rabbit":
 *     its plotted parameter c=-0.123+0.745i, near the centre
 *     of a period-3 bud). Pointer milli (127,-745):
 *     re = 127-250 = -123, im = -(-745) = 745.
 *   cycle 4 (289..384): SET_PARAM zoom -> "1.20", then the
 *     same contact POINTER_MOVEs c to "-0.400"+"0.600"i
 *     (Wikipedia, "Julia set", Examples: its first plotted
 *     constant), POINTER_UP at 297 ends the contact without
 *     disturbing the sweep. Pointer milli (-150,-600).
 * Step 0 sets "iter" to 96 for the proof (the declared
 * default 128 stays the live default): the step-0 settings
 * path is exercised and total proof compute is bounded.
 * Entries sorted by step, at most 2 at one step. */
static const toy_proof_input_t julia_proof[] = {
    {   0, { TOY_EV_SET_PARAM,    JL_PARAM_ITER,    0,    0, 96, 0, 0 } },
    { 193, { TOY_EV_POINTER_DOWN, 0,  127, -745, 0, 0, 0 } },
    { 289, { TOY_EV_SET_PARAM,    JL_PARAM_ZOOM,    0,    0, 2,  0, 0 } },
    { 289, { TOY_EV_POINTER_MOVE, 0, -150, -600, 0, 0, 0 } },
    { 297, { TOY_EV_POINTER_UP,   0, -150, -600, 0, 0, 0 } },
};

/* Pacing/proof: one step is one 80-pixel row, demo 8 steps
 * per frame sweeps a full grid in 10 nominal frames (plus
 * 2 hold frames), fast enough to follow a dragging finger
 * with a partial preview. proof 384 = exactly 4 cycles of
 * 96, report at each boundary.
 *
 * Board costing (estimate, measured-wave factors): the
 * proof is 320 compute steps + 64 near-free hold steps.
 * mandelbrot's row-step (96 px, cap 256, mixed f32/f64)
 * measured 1.2-2.0 s on-device at ~181x host; this row is
 * 80 px at proof cap 96, f64 only, and most z0 in a Julia
 * window escape in a handful of iterations. Measured host
 * cost is ~1.0 ms per compute step (mandelbrot ~9.2 ms on
 * the same host, ratio 0.11; hold steps are a counter
 * increment), which projects the 320-step proof compute at
 * roughly 40-70 s on RP2040 by either the 181x factor or
 * the mandelbrot-ratio route - an estimate, with the
 * measurements in the parcel report. */
const toy_cart_t toy_cart_julia = {
    "julia", "Julia by Touch",
    "Drag on the glass to choose c. The set redraws in binary64.",
    sizeof(julia_state), 8, 384, 96,
    julia_reset, julia_step, julia_render,
    /* explicit WAVE-T2 tail: apply, params, param_count,
     * proof_inputs, proof_input_count, input_caps,
     * state_version */
    julia_apply, julia_params, 2, julia_proof, 5,
    TOY_CAP_POINTER | TOY_CAP_PARAMS, 0
};
