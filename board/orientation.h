/* orientation.h - gravity-quadrant auto-rotation (toy5-rotate):
 * pure orientation tracker + the square-rotation helpers the
 * presenter and the input seams share.
 *
 * Why: rotating the device to scroll (device-as-dial) felt
 * wrong because the picture did not turn with the device.
 * Now the picture, touch coordinates and cart tilt vectors
 * all rotate together by gravity quadrant: the canonical
 * 360 framebuffer and all cart/shell milli coordinates stay
 * canonical; only presentation (presenter.c sampling) and
 * input production (product_input.h) apply the rotation.
 * Proof hashes and the emulator never see this header.
 *
 * --- Frames and measured facts ---------------------------------
 *
 * Glass coordinates: +x right, +y down (display space, the
 * toy_px_to_milli / TILT convention). Canonical coords are
 * the framebuffer / milli frame carts and the shell think
 * in; at quadrant 0 canonical == glass.
 *
 * Sensor facts (hardware-proven, board/SENSORS.md and
 * product_input.h): flat screen-up the QMI8658 reads
 * az ~ -1000 mg, i.e. sensor +Z points away from the viewer
 * through the back of the board, so at rest the reported
 * accel vector points opposite gravity (support force).
 * The in-plane alignment product_input.h already ships is
 * sensor ax -> glass +x, ay -> glass +y, and TILT folds the
 * anti-gravity sign per axis (TOY_PROD_TILT_X/Y_SIGN = -1):
 * the falling direction in glass space is f = (-ax, -ay).
 * live_feed.h states the same frame for feed carts (its
 * falling direction is (-feed_ax, -feed_ay)). This header's
 * candidate rule uses that same f; if the owner ever flips
 * a tilt sign define, the fold here must flip with it.
 *
 * --- Quadrant semantics (EXACT) --------------------------------
 *
 * quadrant q = how far the DEVICE has been rotated clockwise
 * (as the viewer sees the glass) from its canonical upright,
 * in 90-degree steps. Equivalently: which glass edge gravity
 * falls toward when the device is held still:
 *
 *   q=0  upright.            f ~ (0,+1000): falls toward glass
 *        bottom edge. Rest accel: ay ~ -1000, ax ~ 0.
 *   q=1  device 90 CW (canonical top edge now on the viewer's
 *        right). f ~ (+1000,0): falls toward glass right edge.
 *        Rest: ax ~ -1000.
 *   q=2  upside down.       f ~ (0,-1000): glass top edge.
 *        Rest: ay ~ +1000.
 *   q=3  device 270 CW / 90 CCW (canonical top on viewer's
 *        left). f ~ (-1000,0): glass left edge. Rest: ax ~ +1000.
 *
 * Presentation draws canonical pixel c at glass pixel
 * R_q(c) (toy_orient_fwd_point) so the canonical top always
 * appears at the viewer's top. Touch glass points and tilt
 * vectors are mapped back with R_q^-1 (toy_orient_inv_point
 * / toy_orient_inv_vec), so a finger on a drawn widget hits
 * that widget, and pond/gravity "down" in canonical coords
 * is the viewer's down. Check q=1: canonical top-centre
 * (cy=0) maps to glass (0, middle) - the glass left edge,
 * which is the viewer's top once the device sits 90 CW.
 *
 * Rotation formulas (square N, indices 0..N-1, +y down):
 *   fwd (canon->glass): q0 (x,y)          q1 (y, N-1-x)
 *                       q2 (N-1-x, N-1-y) q3 (N-1-y, x)
 *   inv (glass->canon): q0 (x,y)          q1 (N-1-y, x)
 *                       q2 (N-1-x, N-1-y) q3 (y, N-1-x)
 * Vectors (centred, no offset):
 *   fwd: q0 (x,y) q1 (y,-x) q2 (-x,-y) q3 (-y,x)
 *   inv: q0 (x,y) q1 (-y,x) q2 (-x,-y) q3 (y,-x)
 *
 * --- Tracker: hysteresis + dwell -------------------------------
 *
 * A candidate quadrant exists only when the in-plane falling
 * vector is decisive: |f| >= TOY_ORIENT_MIN_MG (below that
 * the device is near-flat, az dominant, and the last
 * quadrant is kept) AND the dominant axis beats the other
 * by >= TOY_ORIENT_MARGIN_MG (at |f| ~ 1 g that is roughly
 * a 5:3 dominance; the 45-degree diagonals, |fx| == |fy|,
 * are deep inside the no-candidate band, so slow tilts
 * across a boundary cannot flap between quadrants). A
 * candidate different from the current quadrant must then
 * persist continuously for TOY_ORIENT_DWELL_MS before the
 * quadrant changes; any other reading (no candidate,
 * a different candidate, or the current quadrant's own
 * candidate) clears the pending timer. Boot quadrant is 0.
 * now_ms is monotonic (toy_clock_mono_ms domain); a
 * backwards clock just restarts the dwell.
 */
#ifndef TOY_ORIENTATION_H
#define TOY_ORIENTATION_H

#include <stdbool.h>
#include <stdint.h>

#define TOY_ORIENT_MIN_MG 350     /* |f_xy| below this: flat, keep */
#define TOY_ORIENT_MARGIN_MG 250  /* dominant-other needed (mg) */
#define TOY_ORIENT_DWELL_MS 500u  /* candidate must persist this long */

static inline int toy_orient_norm(int q)
{
    return ((q % 4) + 4) % 4;
}

/* --- square point/vector rotations (see header table) ---------- */

static inline void toy_orient_fwd_point(int q, int32_t x, int32_t y,
                                        int32_t n, int32_t *ox,
                                        int32_t *oy)
{
    switch (toy_orient_norm(q)) {
    case 1:  *ox = y;         *oy = n - 1 - x; break;
    case 2:  *ox = n - 1 - x; *oy = n - 1 - y; break;
    case 3:  *ox = n - 1 - y; *oy = x;         break;
    default: *ox = x;         *oy = y;         break;
    }
}

static inline void toy_orient_inv_point(int q, int32_t x, int32_t y,
                                        int32_t n, int32_t *ox,
                                        int32_t *oy)
{
    switch (toy_orient_norm(q)) {
    case 1:  *ox = n - 1 - y; *oy = x;         break;
    case 2:  *ox = n - 1 - x; *oy = n - 1 - y; break;
    case 3:  *ox = y;         *oy = n - 1 - x; break;
    default: *ox = x;         *oy = y;         break;
    }
}

/* Canonical-vector -> glass-vector (R_q). */
static inline void toy_orient_fwd_vec(int q, int32_t x, int32_t y,
                                      int32_t *ox, int32_t *oy)
{
    switch (toy_orient_norm(q)) {
    case 1:  *ox = y;  *oy = -x; break;
    case 2:  *ox = -x; *oy = -y; break;
    case 3:  *ox = -y; *oy = x;  break;
    default: *ox = x;  *oy = y;  break;
    }
}

/* Glass-vector -> canonical-vector (R_q^-1): the tilt/feed map. */
static inline void toy_orient_inv_vec(int q, int32_t x, int32_t y,
                                      int32_t *ox, int32_t *oy)
{
    switch (toy_orient_norm(q)) {
    case 1:  *ox = -y; *oy = x;  break;
    case 2:  *ox = -x; *oy = -y; break;
    case 3:  *ox = y;  *oy = -x; break;
    default: *ox = x;  *oy = y;  break;
    }
}

/* --- candidate + tracker --------------------------------------- */

/* Quadrant this accel sample votes for, or -1 for none
 * (near-flat, or in the diagonal ambiguity band). ax_mg /
 * ay_mg are raw sensor milli-g; the falling direction in
 * glass space is (-ax, -ay) per the header frame note. */
static inline int toy_orient_candidate(int32_t ax_mg, int32_t ay_mg)
{
    int64_t fx = -(int64_t)ax_mg;
    int64_t fy = -(int64_t)ay_mg;
    int64_t ax_ = fx < 0 ? -fx : fx;
    int64_t ay_ = fy < 0 ? -fy : fy;
    if (fx * fx + fy * fy <
        (int64_t)TOY_ORIENT_MIN_MG * TOY_ORIENT_MIN_MG)
        return -1;
    if (ax_ - ay_ >= TOY_ORIENT_MARGIN_MG)
        return fx > 0 ? 1 : 3;
    if (ay_ - ax_ >= TOY_ORIENT_MARGIN_MG)
        return fy > 0 ? 0 : 2;
    return -1;
}

typedef struct {
    int quadrant;          /* current, 0..3 (boot 0) */
    int pending;           /* candidate being dwelt on, -1 none */
    uint64_t pending_since_ms;
} toy_orient_t;

static inline void toy_orient_init(toy_orient_t *o)
{
    o->quadrant = 0;
    o->pending = -1;
    o->pending_since_ms = 0;
}

/* Feed one frame's accel sample. Returns true exactly on the
 * call where the quadrant changes (o->quadrant already new). */
static inline bool toy_orient_update(toy_orient_t *o, int32_t ax_mg,
                                     int32_t ay_mg, uint64_t now_ms)
{
    int cand = toy_orient_candidate(ax_mg, ay_mg);
    if (cand < 0 || cand == o->quadrant) {
        o->pending = -1;
        return false;
    }
    if (cand != o->pending || now_ms < o->pending_since_ms) {
        o->pending = cand;
        o->pending_since_ms = now_ms;
        return false;
    }
    if (now_ms - o->pending_since_ms >= TOY_ORIENT_DWELL_MS) {
        o->quadrant = cand;
        o->pending = -1;
        return true;
    }
    return false;
}

#endif
