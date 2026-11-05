/* toy_shell.c - product runner shell. See toy_shell.h for the
 * placement rule and the runner loop contract. Integer-only
 * except softfloat F64-param interpolation (toy_math, the same
 * library path toy_param_event uses); no allocation, no libc
 * beyond toy_util, so host, RP2040 and WASM builds all link it
 * unchanged. Renders size-parametrically from centre/radius
 * fractions exactly like a cartridge. */
#define TOY_SHELL_ALLOW 1
#define TOY_CLOCK_ALLOW 1
#include "toy_shell.h"
#include "toy_math.h"
#include "toy_util.h"

/* --- layout in milli-radius (shared by render + hit tests) --- */
#define ML_TITLE_Y    (-850)
#define ML_G_ROW_Y    (-620)
#define ML_G_ROW_H    155
#define ML_DETAIL_Y   360
#define ML_FOOTER_Y   740
#define ML_S_ROW_Y    (-560)
#define ML_S_ROW_H    165
#define ML_LEFT       (-680)
#define ML_RIGHT      680

static int32_t clampi(int32_t v, int32_t lo, int32_t hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}
static int32_t wrapi(int32_t v, int32_t n) /* n > 0 */
{
    v %= n;
    return v < 0 ? v + n : v;
}

/* --- param values --- */

static uint64_t shell_f64_bits_at(const toy_param_t *p, int32_t permille)
{
    if (permille <= 0) return toy_f64_bits_from_dec(p->f_min);
    if (permille >= 1000) return toy_f64_bits_from_dec(p->f_max);
    f64 lo = f64_dec(p->f_min), hi = f64_dec(p->f_max);
    return (uint64_t)f64_add(lo, f64_mul(f64_sub(hi, lo),
                                         (f64)toy_f64_bits_from_milli(permille)));
}

static int32_t shell_f64_default_permille(const toy_param_t *p)
{
    f64 lo = f64_dec(p->f_min), hi = f64_dec(p->f_max), def = f64_dec(p->f_def);
    f64 span = f64_sub(hi, lo);
    if ((uint64_t)span == 0ull) return 0;
    msf_flags_t fl = 0;
    f64 milli = f64_mul(f64_div(f64_sub(def, lo), span), f64_dec("1000"));
    return clampi(msf64_to_i32_ties_to_even(milli, &fl), 0, 1000);
}

static void shell_load_params(toy_shell_t *sh, const toy_cart_t *cart)
{
    toy_memset(sh->params, 0, sizeof sh->params);
    sh->param_sel = 0;
    sh->param_scroll = 0;
    toy_queue_clear(&sh->param_queue);
    if (!cart) return;
    uint32_t n = cart->param_count;
    if (n > TOY_MAX_PARAMS) n = TOY_MAX_PARAMS;
    for (uint32_t i = 0; i < n; i++) {
        const toy_param_t *p = &cart->params[i];
        if (p->kind == TOY_PARAM_F64) {
            sh->params[i].i_val = shell_f64_default_permille(p);
            sh->params[i].f_bits = toy_f64_bits_from_dec(p->f_def); /* exact until edited */
        } else {
            sh->params[i].i_val = p->i_def;
        }
    }
}

/* Make cart idx the settings/running target; a different cart
 * forgets the previous cart's values (persistence across carts
 * is a runner storage policy, not shell state). */
static void shell_focus(toy_shell_t *sh, int32_t idx)
{
    if (idx == sh->current) return;
    sh->current = idx;
    sh->running_valid = 0;
    shell_load_params(sh, toy_shell_current(sh));
}

static void shell_queue_param(toy_shell_t *sh, uint32_t idx)
{
    toy_event_t ev;
    toy_memset(&ev, 0, sizeof ev);
    ev.type = TOY_EV_SET_PARAM;
    ev.id = idx;
    const toy_cart_t *cart = toy_shell_current(sh);
    if (cart && cart->params[idx].kind == TOY_PARAM_F64)
        ev.bits = sh->params[idx].f_bits;
    else
        ev.v = sh->params[idx].i_val;
    toy_queue_push(&sh->param_queue, &ev);
}

uint32_t toy_shell_queue_start_params(toy_shell_t *sh)
{
    const toy_cart_t *cart = toy_shell_current(sh);
    toy_queue_clear(&sh->param_queue);
    if (!cart) return 0;
    uint32_t queued = 0;
    uint32_t n = cart->param_count;
    if (n > TOY_MAX_PARAMS) n = TOY_MAX_PARAMS;
    for (uint32_t i = 0; i < n; i++) { /* descriptor order */
        sh->params[i].staged = 0;
        if (sh->params[i].touched) { shell_queue_param(sh, i); queued++; }
    }
    return queued;
}

/* Adjust the selected param by `units` detents. Returns
 * TOY_SH_ACT_PARAM_QUEUED for a live change, else 0 (a staged
 * RESET_ONLY / not-yet-running change queues nothing). */
static uint32_t shell_adjust_param(toy_shell_t *sh, int32_t units)
{
    const toy_cart_t *cart = toy_shell_current(sh);
    if (!cart || !units || sh->param_sel < 0 ||
        (uint32_t)sh->param_sel >= cart->param_count) return 0;
    uint32_t idx = (uint32_t)sh->param_sel;
    const toy_param_t *p = &cart->params[idx];
    toy_shell_param_t *v = &sh->params[idx];
    int32_t old = v->i_val;
    if (p->kind == TOY_PARAM_ENUM) {
        v->i_val = wrapi(old + units, p->i_max + 1);
    } else if (p->kind == TOY_PARAM_F64) {
        v->i_val = clampi(old + units * 10, 0, TOY_SHELL_F64_STEPS);
        if (v->i_val != old) v->f_bits = shell_f64_bits_at(p, v->i_val);
    } else { /* I32: one detent = 1, coarse ranges step range/100 */
        int64_t span = (int64_t)p->i_max - (int64_t)p->i_min;
        int64_t step = span / 100;
        if (step < 1) step = 1;
        int64_t nv = (int64_t)old + (int64_t)units * step;
        v->i_val = clampi((int32_t)nv, p->i_min, p->i_max);
    }
    if (v->i_val == old) return 0;
    v->touched = 1;
    if ((p->flags & TOY_PARAM_RESET_ONLY) || !sh->running_valid) {
        v->staged = 1;
        return 0;
    }
    shell_queue_param(sh, idx);
    return TOY_SH_ACT_PARAM_QUEUED;
}

static int shell_any_staged(const toy_shell_t *sh)
{
    const toy_cart_t *cart = toy_shell_current(sh);
    if (!cart) return 0;
    uint32_t n = cart->param_count < TOY_MAX_PARAMS ? cart->param_count : TOY_MAX_PARAMS;
    for (uint32_t i = 0; i < n; i++) if (sh->params[i].staged) return 1;
    return 0;
}

/* --- event classification --- */

static int ev_is_key(const toy_event_t *ev, uint32_t key)
{
    return ev->type == TOY_EV_KEY_DOWN && ev->id == key;
}
static int ev_is_back(const toy_event_t *ev) /* shell screens only */
{
    return ev_is_key(ev, TOY_KEY_ESC) || ev_is_key(ev, TOY_KEY_B) ||
           (ev->type == TOY_EV_BUTTON_DOWN && ev->id == TOY_SHELL_BUTTON_EXIT);
}
static int ev_is_exit(const toy_event_t *ev) /* RUNNING: narrower, see header */
{
    return ev_is_key(ev, TOY_KEY_ESC) ||
           (ev->type == TOY_EV_BUTTON_DOWN && ev->id == TOY_SHELL_BUTTON_EXIT);
}
static int ev_is_settings(const toy_event_t *ev)
{
    return ev_is_key(ev, TOY_KEY_X) ||
           (ev->type == TOY_EV_BUTTON_DOWN && ev->id == TOY_SHELL_BUTTON_SETTINGS);
}
static int ev_is_enter(const toy_event_t *ev)
{
    return ev_is_key(ev, TOY_KEY_ENTER) || ev_is_key(ev, TOY_KEY_A);
}
/* Signed detents from a navigation/adjust event, 0 if none.
 * Dial: 25 milliturns per selection detent (40/turn). */
static int32_t ev_detents(const toy_event_t *ev)
{
    if (ev->type == TOY_EV_ENCODER) return ev->v;
    if (ev->type == TOY_EV_DIAL) {
        int32_t d = ev->v / 25;
        if (!d && ev->v) d = ev->v < 0 ? -1 : 1;
        return d;
    }
    return 0;
}
/* Finer detents for value adjustment: 10 milliturns each. */
static int32_t ev_adjust_detents(const toy_event_t *ev)
{
    if (ev->type == TOY_EV_ENCODER) return ev->v;
    if (ev->type == TOY_EV_DIAL) {
        int32_t d = ev->v / 10;
        if (!d && ev->v) d = ev->v < 0 ? -1 : 1;
        return d;
    }
    if (ev_is_key(ev, TOY_KEY_RIGHT) || ev_is_key(ev, TOY_KEY_PLUS)) return 1;
    if (ev_is_key(ev, TOY_KEY_LEFT) || ev_is_key(ev, TOY_KEY_MINUS)) return -1;
    return 0;
}

static void shell_scroll_follow(int32_t sel, int32_t total, int32_t *scroll)
{
    int32_t vis = (int32_t)TOY_SHELL_VISIBLE_ROWS;
    if (sel < *scroll) *scroll = sel;
    if (sel >= *scroll + vis) *scroll = sel - vis + 1;
    int32_t max_scroll = total - vis;
    if (max_scroll < 0) max_scroll = 0;
    *scroll = clampi(*scroll, 0, max_scroll);
}

static uint32_t shell_start_current(toy_shell_t *sh)
{
    uint32_t n = toy_shell_queue_start_params(sh);
    sh->running_valid = 1;
    sh->screen = TOY_SHELL_RUNNING;
    return TOY_SH_ACT_START_CART | TOY_SH_ACT_SCREEN |
           (n ? TOY_SH_ACT_PARAM_QUEUED : 0u);
}

static uint32_t shell_open_settings(toy_shell_t *sh, uint8_t from)
{
    if (from == TOY_SHELL_GALLERY && sh->selected < sh->cart_count)
        shell_focus(sh, sh->selected);
    if (sh->current < 0) return TOY_SH_ACT_NONE;
    sh->return_screen = from;
    sh->screen = TOY_SHELL_SETTINGS;
    return TOY_SH_ACT_SCREEN;
}

static uint32_t shell_gallery_event(toy_shell_t *sh, const toy_event_t *ev)
{
    int32_t total = sh->cart_count + 2; /* + SET CLOCK / CLOCK FACE rows */
    if (ev_is_back(ev)) { sh->screen = TOY_SHELL_CLOCKFACE; return TOY_SH_ACT_SCREEN; }
    if (ev_is_settings(ev))
        return sh->selected < sh->cart_count
                   ? shell_open_settings(sh, TOY_SHELL_GALLERY) : TOY_SH_ACT_NONE;
    int32_t move = ev_detents(ev);
    if (ev_is_key(ev, TOY_KEY_DOWN)) move = 1;
    if (ev_is_key(ev, TOY_KEY_UP)) move = -1;
    if (move) {
        sh->selected = clampi(sh->selected + move, 0, total - 1);
        shell_scroll_follow(sh->selected, total, &sh->scroll);
        return TOY_SH_ACT_NONE;
    }
    int activate = ev_is_enter(ev);
    if (ev->type == TOY_EV_POINTER_DOWN) {
        if (ev->y >= ML_FOOTER_Y) {
            if (ev->x >= 0 && sh->selected < sh->cart_count)
                return shell_open_settings(sh, TOY_SHELL_GALLERY);
            activate = 1;
        } else if (ev->y >= ML_G_ROW_Y &&
                   ev->y < ML_G_ROW_Y + (int32_t)TOY_SHELL_VISIBLE_ROWS * ML_G_ROW_H) {
            int32_t row = sh->scroll + (ev->y - ML_G_ROW_Y) / ML_G_ROW_H;
            if (row >= 0 && row < total) {
                if (row == sh->selected) activate = 1;
                else { sh->selected = row; return TOY_SH_ACT_NONE; }
            }
        }
    }
    if (!activate) return TOY_SH_ACT_NONE;
    if (sh->selected < sh->cart_count) {
        shell_focus(sh, sh->selected);
        return shell_start_current(sh);
    }
    if (sh->selected == sh->cart_count) {
        sh->return_screen = TOY_SHELL_GALLERY;
        toy_shell_clockset_begin(sh, sh->last_now);
        return TOY_SH_ACT_SCREEN;
    }
    sh->screen = TOY_SHELL_CLOCKFACE;
    return TOY_SH_ACT_SCREEN;
}

static uint32_t shell_settings_event(toy_shell_t *sh, const toy_event_t *ev)
{
    const toy_cart_t *cart = toy_shell_current(sh);
    int32_t n = cart ? (int32_t)cart->param_count : 0;
    if (ev_is_back(ev) || (ev_is_settings(ev) && sh->running_valid)) {
        sh->screen = (sh->return_screen == TOY_SHELL_RUNNING && sh->running_valid)
                         ? TOY_SHELL_RUNNING : TOY_SHELL_GALLERY;
        return TOY_SH_ACT_SCREEN;
    }
    if (ev_is_enter(ev) ||
        (ev->type == TOY_EV_POINTER_DOWN && ev->y >= ML_FOOTER_Y)) {
        if (!sh->running_valid || shell_any_staged(sh))
            return shell_start_current(sh); /* fresh start applies staged */
        sh->screen = TOY_SHELL_RUNNING; /* nothing staged: resume */
        return TOY_SH_ACT_SCREEN;
    }
    if (!n) return TOY_SH_ACT_NONE;
    int32_t move = 0;
    if (ev_is_key(ev, TOY_KEY_DOWN)) move = 1;
    if (ev_is_key(ev, TOY_KEY_UP)) move = -1;
    if (move) {
        sh->param_sel = clampi(sh->param_sel + move, 0, n - 1);
        shell_scroll_follow(sh->param_sel, n, &sh->param_scroll);
        return TOY_SH_ACT_NONE;
    }
    if (ev->type == TOY_EV_POINTER_DOWN && ev->y >= ML_S_ROW_Y &&
        ev->y < ML_DETAIL_Y) {
        int32_t row = sh->param_scroll + (ev->y - ML_S_ROW_Y) / ML_S_ROW_H;
        if (row >= 0 && row < n) {
            if (row != sh->param_sel) {
                sh->param_sel = row;
                shell_scroll_follow(row, n, &sh->param_scroll);
                return TOY_SH_ACT_NONE;
            }
            if (ev->x < -220) return shell_adjust_param(sh, -1);
            if (ev->x > 220) return shell_adjust_param(sh, 1);
        }
        return TOY_SH_ACT_NONE;
    }
    int32_t adj = ev_adjust_detents(ev);
    if (adj) return shell_adjust_param(sh, adj);
    /* Dial motion below one detent still reaches adjust via
     * ev_adjust_detents' sign fallback; vertical dial scroll
     * of the selection is keys/taps (documented v1). */
    return TOY_SH_ACT_NONE;
}

static uint32_t shell_clock_commit(toy_shell_t *sh)
{
    sh->screen = TOY_SHELL_CLOCKFACE;
    return TOY_SH_ACT_CLOCK_COMMIT | TOY_SH_ACT_SCREEN;
}

static uint32_t shell_clockset_event(toy_shell_t *sh, const toy_event_t *ev)
{
    if (ev_is_back(ev)) { sh->screen = sh->return_screen; return TOY_SH_ACT_SCREEN; }
    if (ev->type == TOY_EV_POINTER_DOWN) {
        if (ev->y >= 540) { sh->screen = sh->return_screen; return TOY_SH_ACT_SCREEN; }
        if (ev->y >= 320) return shell_clock_commit(sh);
        if (ev->y >= -50) sh->clock_field = 2;
        else sh->clock_field = ev->x < 0 ? 0 : 1;
        return TOY_SH_ACT_NONE;
    }
    if (ev_is_enter(ev)) {
        if (sh->clock_field == 3) return shell_clock_commit(sh);
        if (sh->clock_field == 4) { sh->screen = sh->return_screen; return TOY_SH_ACT_SCREEN; }
        sh->clock_field++;
        return TOY_SH_ACT_NONE;
    }
    int32_t move = 0;
    if (ev_is_key(ev, TOY_KEY_DOWN)) move = 1;
    if (ev_is_key(ev, TOY_KEY_UP)) move = -1;
    if (move) { sh->clock_field = clampi(sh->clock_field + move, 0, 4); return TOY_SH_ACT_NONE; }
    int32_t adj = ev_adjust_detents(ev);
    if (!adj) return TOY_SH_ACT_NONE;
    switch (sh->clock_field) {
    case 0: sh->clock_hh = wrapi(sh->clock_hh + adj, 24); break;
    case 1: sh->clock_mm = wrapi(sh->clock_mm + adj, 60); break;
    case 2: sh->clock_tz_offset_min = clampi(sh->clock_tz_offset_min + adj * 15, -720, 840); break;
    default: break;
    }
    return TOY_SH_ACT_NONE;
}

static uint32_t shell_clockface_event(toy_shell_t *sh, const toy_event_t *ev)
{
    if (ev_is_settings(ev)) {
        sh->return_screen = TOY_SHELL_CLOCKFACE;
        toy_shell_clockset_begin(sh, sh->last_now);
        return TOY_SH_ACT_SCREEN;
    }
    if ((ev_is_enter(ev) || ev->type == TOY_EV_POINTER_DOWN) && !sh->last_now.valid) {
        sh->return_screen = TOY_SHELL_CLOCKFACE;
        toy_shell_clockset_begin(sh, sh->last_now);
        return TOY_SH_ACT_SCREEN;
    }
    if (ev->type == TOY_EV_KEY_DOWN || ev->type == TOY_EV_BUTTON_DOWN ||
        ev->type == TOY_EV_POINTER_DOWN || ev->type == TOY_EV_ENCODER ||
        ev->type == TOY_EV_DIAL) {
        sh->screen = TOY_SHELL_GALLERY;
        return TOY_SH_ACT_SCREEN;
    }
    return TOY_SH_ACT_NONE;
}

/* --- public event/API --- */

void toy_shell_init(toy_shell_t *sh, const toy_cart_t *const *carts, int32_t count)
{
    toy_memset(sh, 0, sizeof *sh);
    sh->carts = carts;
    sh->cart_count = count < 0 ? 0 : count;
    sh->current = -1;
    sh->screen = TOY_SHELL_GALLERY;
    sh->schedule = toy_schedule_default();
    sh->idle_limit_frames = TOY_SHELL_IDLE_FRAMES_DEFAULT;
    sh->clock_hh = 12;
}

const toy_cart_t *toy_shell_current(const toy_shell_t *sh)
{
    if (!sh->carts || sh->current < 0 || sh->current >= sh->cart_count) return 0;
    return sh->carts[sh->current];
}

uint32_t toy_shell_event(toy_shell_t *sh, const toy_event_t *ev)
{
    if (!ev || ev->type == TOY_EV_NONE) return TOY_SH_ACT_NONE;
    sh->idle_frames = 0;
    switch (sh->screen) {
    case TOY_SHELL_RUNNING:
        if (ev_is_exit(ev)) {
            sh->running_valid = 0;
            sh->screen = TOY_SHELL_GALLERY;
            return TOY_SH_ACT_EXIT_TO_GALLERY | TOY_SH_ACT_SCREEN;
        }
        if (ev_is_settings(ev)) return shell_open_settings(sh, TOY_SHELL_RUNNING);
        return TOY_SH_ACT_FORWARD_TO_CART;
    case TOY_SHELL_SETTINGS: return shell_settings_event(sh, ev);
    case TOY_SHELL_CLOCKSET: return shell_clockset_event(sh, ev);
    case TOY_SHELL_CLOCKFACE: return shell_clockface_event(sh, ev);
    default: return shell_gallery_event(sh, ev);
    }
}

uint32_t toy_shell_tick(toy_shell_t *sh)
{
    if (sh->screen != TOY_SHELL_GALLERY || !sh->idle_limit_frames)
        return TOY_SH_ACT_NONE;
    if (++sh->idle_frames >= sh->idle_limit_frames) {
        sh->idle_frames = 0;
        sh->screen = TOY_SHELL_CLOCKFACE;
        return TOY_SH_ACT_SCREEN;
    }
    return TOY_SH_ACT_NONE;
}

void toy_shell_enter(toy_shell_t *sh, uint8_t screen)
{
    if (screen > TOY_SHELL_CLOCKFACE) return;
    if (screen == TOY_SHELL_CLOCKSET) { toy_shell_clockset_begin(sh, sh->last_now); return; }
    sh->screen = screen;
    sh->idle_frames = 0;
}

void toy_shell_clockset_begin(toy_shell_t *sh, toy_time_t now)
{
    if (sh->screen != TOY_SHELL_CLOCKSET && sh->screen != TOY_SHELL_CLOCKFACE)
        sh->return_screen = sh->screen <= TOY_SHELL_RUNNING ? TOY_SHELL_GALLERY : sh->screen;
    sh->last_now = now;
    if (now.valid) {
        uint32_t m = toy_clock_minutes_of_day(now);
        sh->clock_hh = (int32_t)(m / 60u);
        sh->clock_mm = (int32_t)(m % 60u);
        sh->clock_tz_offset_min = now.tz_offset_min;
    } else {
        sh->clock_hh = 12; sh->clock_mm = 0; sh->clock_tz_offset_min = 0;
    }
    sh->clock_field = 0;
    sh->screen = TOY_SHELL_CLOCKSET;
    sh->idle_frames = 0;
}

/* --- rendering --- */

static int txt_scale(const toy_fb_t *fb) { return toy_r(fb) >= 170 ? 2 : 1; }
static int milli_x(const toy_fb_t *fb, int32_t m) { return toy_cx(fb) + (int)(m * toy_r(fb) / 1000); }
static int milli_y(const toy_fb_t *fb, int32_t m) { return toy_cy(fb) + (int)(m * toy_r(fb) / 1000); }

/* Uppercase + font-safe copy (the 3x5 font has A-Z 0-9 + marks). */
static void upper_copy(char *dst, const char *src, size_t cap)
{
    size_t i = 0;
    if (!cap) return;
    for (; src && src[i] && i + 1 < cap; i++) {
        char c = src[i];
        if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
        else if (!((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
                   c == ' ' || c == '.' || c == '-' || c == ':' || c == '+' ||
                   c == '/' || c == '=' || c == '%' || c == '(' || c == ')' ||
                   c == '#' || c == '*' || c == '<' || c == '>' || c == '_'))
            c = ' ';
        dst[i] = c;
    }
    dst[i] = '\0';
}

static size_t s_len(const char *s) { size_t n = 0; while (s[n]) n++; return n; }
static int text_w(size_t len, int sc) { return len ? (int)len * 4 * sc - sc : 0; }

static void draw_centered(toy_fb_t *fb, int32_t my, int sc, uint16_t col, const char *s)
{
    toy_fb_text(fb, toy_cx(fb) - text_w(s_len(s), sc) / 2, milli_y(fb, my), sc, col, s);
}
static void draw_at(toy_fb_t *fb, int32_t mx, int32_t my, int sc, uint16_t col, const char *s)
{
    toy_fb_text(fb, milli_x(fb, mx), milli_y(fb, my), sc, col, s);
}
static void draw_right(toy_fb_t *fb, int32_t mx, int32_t my, int sc, uint16_t col, const char *s)
{
    toy_fb_text(fb, milli_x(fb, mx) - text_w(s_len(s), sc), milli_y(fb, my), sc, col, s);
}
static void fill_band(toy_fb_t *fb, int32_t my0, int32_t my1, uint16_t col)
{
    int x0 = milli_x(fb, ML_LEFT - 40), x1 = milli_x(fb, ML_RIGHT + 40);
    int y0 = milli_y(fb, my0), y1 = milli_y(fb, my1);
    for (int y = y0; y < y1; y++) toy_fb_line(fb, x0, y, x1, y, col);
}

static int chars_per_line(const toy_fb_t *fb, int sc)
{
    int avail = (toy_r(fb) * (ML_RIGHT - ML_LEFT)) / 1000;
    int n = avail / (4 * sc);
    return n < 8 ? 8 : n;
}

/* Word-wrap an uppercased string into up to max_lines lines. */
static void draw_wrapped(toy_fb_t *fb, const char *s, int32_t my, int sc,
                         uint16_t col, int max_lines)
{
    int per = chars_per_line(fb, sc);
    if (per > 47) per = 47;
    char line[48];
    const char *p = s;
    for (int ln = 0; ln < max_lines && *p; ln++) {
        while (*p == ' ') p++;
        int n = 0, last_space = -1;
        while (p[n] && n < per) { if (p[n] == ' ') last_space = n; n++; }
        if (p[n] && last_space > 0) n = last_space;
        for (int i = 0; i < n; i++) line[i] = p[i];
        line[n < 47 ? n : 47] = '\0';
        draw_at(fb, ML_LEFT, my, sc, col, line);
        my += 92;
        p += n;
    }
}

static void fmt_i32(char *dst, size_t cap, int32_t v)
{
    char tmp[16];
    int n = 0, neg = v < 0;
    uint32_t u = neg ? (uint32_t)(-(v + 1)) + 1u : (uint32_t)v;
    do { tmp[n++] = (char)('0' + (int)(u % 10u)); u /= 10u; } while (u);
    size_t o = 0;
    if (neg && o + 1 < cap) dst[o++] = '-';
    while (n > 0 && o + 1 < cap) dst[o++] = tmp[--n];
    if (cap) dst[o] = '\0';
}
static void fmt2(char *dst, int32_t v) /* "07", v in 0..99 */
{
    dst[0] = (char)('0' + (v / 10) % 10);
    dst[1] = (char)('0' + v % 10);
    dst[2] = '\0';
}
static void fmt_tz(char *dst, size_t cap, int32_t tz_min)
{
    /* "UTC+05:30" */
    char hh[4], mm[4];
    int32_t a = tz_min < 0 ? -tz_min : tz_min;
    fmt2(hh, a / 60); fmt2(mm, a % 60);
    const char *parts[5] = { "UTC", tz_min < 0 ? "-" : "+", hh, ":", mm };
    size_t o = 0;
    for (int i = 0; i < 5; i++)
        for (const char *p = parts[i]; *p && o + 1 < cap; p++) dst[o++] = *p;
    if (cap) dst[o] = '\0';
}
static void fmt_join3(char *dst, size_t cap, const char *a, const char *b, const char *c)
{
    size_t o = 0;
    const char *parts[3] = { a, b, c };
    for (int i = 0; i < 3; i++)
        for (const char *p = parts[i]; p && *p && o + 1 < cap; p++) dst[o++] = *p;
    if (cap) dst[o] = '\0';
}

#define COL_BG     toy_rgb565(10, 12, 24)
#define COL_FG     toy_rgb565(205, 225, 255)
#define COL_DIM    toy_rgb565(110, 130, 165)
#define COL_ACCENT toy_rgb565(255, 200, 90)
#define COL_SEL    toy_rgb565(32, 52, 95)

static void shell_draw_header(toy_fb_t *fb, const char *title, const char *right)
{
    int sc = txt_scale(fb);
    draw_centered(fb, ML_TITLE_Y, sc, COL_FG, title);
    /* the chord at the title height is only ~0.53 R half-wide */
    if (right) draw_right(fb, 470, ML_TITLE_Y, 1, COL_DIM, right);
    toy_fb_line(fb, milli_x(fb, ML_LEFT - 40), milli_y(fb, ML_TITLE_Y + 95),
                milli_x(fb, ML_RIGHT + 40), milli_y(fb, ML_TITLE_Y + 95), COL_DIM);
}

static void shell_render_gallery(toy_shell_t *sh, toy_fb_t *fb)
{
    int sc = txt_scale(fb);
    int32_t total = sh->cart_count + 2;
    char pos[12], a[8], b[8];
    fmt_i32(a, sizeof a, sh->selected + 1);
    fmt_i32(b, sizeof b, total);
    fmt_join3(pos, sizeof pos, a, "/", b);
    shell_draw_header(fb, "GALLERY", pos);
    int per = chars_per_line(fb, sc);
    for (int32_t r = 0; r < (int32_t)TOY_SHELL_VISIBLE_ROWS; r++) {
        int32_t idx = sh->scroll + r;
        if (idx >= total) break;
        int32_t my = ML_G_ROW_Y + r * ML_G_ROW_H;
        char label[64], up[64];
        if (idx < sh->cart_count) {
            upper_copy(up, sh->carts[idx]->name, sizeof up);
            char title[48];
            upper_copy(title, sh->carts[idx]->title, sizeof title);
            fmt_join3(label, sizeof label, up, "  ", title);
        } else {
            fmt_join3(label, sizeof label,
                      idx == sh->cart_count ? "SET CLOCK" : "CLOCK FACE", "", "");
        }
        if ((int)s_len(label) > per - 2) label[per - 2 > 0 ? per - 2 : 0] = '\0';
        if (idx == sh->selected) {
            fill_band(fb, my - 28, my + ML_G_ROW_H - 55, COL_SEL);
            draw_at(fb, ML_LEFT - 5, my, sc, COL_ACCENT, ">");
        }
        draw_at(fb, ML_LEFT + 55, my, sc,
                idx == sh->selected ? COL_FG : COL_DIM, label);
    }
    /* scrollbar thumb on the right chord */
    {
        int y0 = milli_y(fb, ML_G_ROW_Y), y1 = milli_y(fb, ML_DETAIL_Y - 40);
        int x = milli_x(fb, ML_RIGHT + 95);
        toy_fb_line(fb, x, y0, x, y1, COL_DIM);
        if (total > 0) {
            int ty = y0 + (y1 - y0) * (int)sh->selected / (int)total;
            toy_fb_disc(fb, x, ty, toy_r(fb) / 40 + 1, COL_ACCENT);
        }
    }
    /* selected detail: title + blurb */
    char title[64], blurb[160];
    if (sh->selected < sh->cart_count) {
        upper_copy(title, sh->carts[sh->selected]->title, sizeof title);
        upper_copy(blurb, sh->carts[sh->selected]->blurb, sizeof blurb);
    } else if (sh->selected == sh->cart_count) {
        fmt_join3(title, sizeof title, "SET CLOCK", "", "");
        fmt_join3(blurb, sizeof blurb, "SET LOCAL TIME AND UTC OFFSET. THE SIM NEVER READS IT.", "", "");
    } else {
        fmt_join3(title, sizeof title, "CLOCK FACE", "", "");
        fmt_join3(blurb, sizeof blurb, "IDLE CLOCK. SHOWS THE RUNNER WALL TIME, SET BY HAND.", "", "");
    }
    if ((int)s_len(title) > per) title[per] = '\0';
    draw_at(fb, ML_LEFT, ML_DETAIL_Y, sc, COL_ACCENT, title);
    draw_wrapped(fb, blurb, ML_DETAIL_Y + 95, 1, COL_FG, 3);
    /* footer buttons */
    toy_fb_line(fb, milli_x(fb, ML_LEFT - 40), milli_y(fb, ML_FOOTER_Y - 35),
                milli_x(fb, ML_RIGHT + 40), milli_y(fb, ML_FOOTER_Y - 35), COL_DIM);
    /* footer chord is ~0.6 R half-wide: keep labels inside it */
    draw_at(fb, -520, ML_FOOTER_Y + 20, sc, COL_FG, "RUN >");
    if (sh->selected < sh->cart_count)
        draw_right(fb, 520, ML_FOOTER_Y + 20, sc, COL_FG, "SET >");
}

static void shell_param_value_text(const toy_shell_t *sh, const toy_cart_t *cart,
                                   uint32_t idx, char *dst, size_t cap)
{
    const toy_param_t *p = &cart->params[idx];
    const toy_shell_param_t *v = &sh->params[idx];
    char num[20];
    if (p->kind == TOY_PARAM_ENUM) {
        const char *nm = (p->enum_names && v->i_val >= 0 && v->i_val <= p->i_max)
                             ? p->enum_names[v->i_val] : "?";
        upper_copy(dst, nm, cap);
    } else if (p->kind == TOY_PARAM_F64) {
        if (!v->touched) {
            upper_copy(dst, p->f_def, cap); /* exact declared default */
        } else {
            fmt_i32(num, sizeof num, v->i_val / 10);
            fmt_join3(dst, cap, num, "% RNG", "");
        }
    } else {
        fmt_i32(dst, cap, v->i_val);
    }
    if (v->staged) {
        size_t n = s_len(dst);
        if (n + 2 < cap) { dst[n] = '*'; dst[n + 1] = '\0'; }
    }
}

static void shell_render_settings(toy_shell_t *sh, toy_fb_t *fb)
{
    int sc = txt_scale(fb);
    const toy_cart_t *cart = toy_shell_current(sh);
    char head[48], up[32];
    upper_copy(up, cart ? cart->name : "-", sizeof up);
    fmt_join3(head, sizeof head, up, " SET", "");
    shell_draw_header(fb, head, 0);
    if (!cart || !cart->param_count) {
        draw_centered(fb, -80, sc, COL_FG, "NO SETTINGS");
        draw_centered(fb, ML_FOOTER_Y, 1, COL_DIM, "ESC BACK");
        return;
    }
    int per = chars_per_line(fb, sc);
    int32_t n = (int32_t)cart->param_count;
    for (int32_t r = 0; r < (int32_t)TOY_SHELL_VISIBLE_ROWS; r++) {
        int32_t idx = sh->param_scroll + r;
        if (idx >= n) break;
        int32_t my = ML_S_ROW_Y + r * ML_S_ROW_H;
        char label[48], value[32];
        upper_copy(label, cart->params[idx].label ? cart->params[idx].label
                                                  : cart->params[idx].name,
                   sizeof label);
        if ((int)s_len(label) > per / 2) label[per / 2] = '\0';
        shell_param_value_text(sh, cart, (uint32_t)idx, value, sizeof value);
        if (idx == sh->param_sel) {
            fill_band(fb, my - 28, my + ML_S_ROW_H - 60, COL_SEL);
            draw_at(fb, ML_LEFT - 5, my, sc, COL_ACCENT, ">");
            draw_right(fb, ML_RIGHT, my, sc, COL_ACCENT, value);
        } else {
            draw_right(fb, ML_RIGHT, my, sc, COL_DIM, value);
        }
        draw_at(fb, ML_LEFT + 55, my, sc,
                idx == sh->param_sel ? COL_FG : COL_DIM, label);
    }
    /* detail for the selected param */
    const toy_param_t *p = &cart->params[sh->param_sel];
    char rng[64], x1[24], x2[24];
    if (p->kind == TOY_PARAM_F64) {
        upper_copy(x1, p->f_min, sizeof x1);
        upper_copy(x2, p->f_max, sizeof x2);
    } else {
        fmt_i32(x1, sizeof x1, p->i_min);
        fmt_i32(x2, sizeof x2, p->i_max);
    }
    fmt_join3(rng, sizeof rng, x1, " .. ", x2);
    draw_at(fb, ML_LEFT, 500, 1, COL_DIM, rng);
    const char *mode = (p->flags & TOY_PARAM_RESET_ONLY)
                           ? "RESET ONLY: APPLIES ON RESTART"
                           : (sh->params[sh->param_sel].staged ? "STAGED: APPLIES ON START"
                                                               : "LIVE: APPLIES AT NEXT STEP");
    if (sh->params[sh->param_sel].staged) mode = "STAGED *: APPLIES ON (RE)START";
    draw_at(fb, ML_LEFT, 590, 1, sh->params[sh->param_sel].staged ? COL_ACCENT : COL_DIM, mode);
    draw_centered(fb, ML_FOOTER_Y + 20, 1, COL_FG, "ENTER START/RESTART  ESC BACK");
    draw_centered(fb, ML_FOOTER_Y + 105, 1, COL_DIM, "DIAL/</> ADJUST  TAP EDGE -/+");
}

static void shell_render_clockset(toy_shell_t *sh, toy_fb_t *fb)
{
    int sc = txt_scale(fb);
    int big = sc * 3;
    shell_draw_header(fb, "SET CLOCK", sh->last_now.valid ? 0 : "UNSET");
    char hh[4], mm[4], hm[8];
    fmt2(hh, sh->clock_hh); fmt2(mm, sh->clock_mm);
    fmt_join3(hm, sizeof hm, hh, ":", mm);
    int x0 = toy_cx(fb) - text_w(5, big) / 2;
    int y0 = milli_y(fb, -420);
    toy_fb_text(fb, x0, y0, big, sh->clock_field == 0 ? COL_ACCENT : COL_FG, hh);
    toy_fb_text(fb, x0 + 2 * 4 * big, y0, big, COL_FG, ":");
    toy_fb_text(fb, x0 + 3 * 4 * big, y0, big, sh->clock_field == 1 ? COL_ACCENT : COL_FG, mm);
    int uy = y0 + 6 * big;
    if (sh->clock_field == 0) toy_fb_line(fb, x0, uy, x0 + text_w(2, big), uy, COL_ACCENT);
    if (sh->clock_field == 1) toy_fb_line(fb, x0 + 3 * 4 * big, uy, x0 + 3 * 4 * big + text_w(2, big), uy, COL_ACCENT);
    char tz[16];
    fmt_tz(tz, sizeof tz, sh->clock_tz_offset_min);
    draw_centered(fb, -40, sc * 2, sh->clock_field == 2 ? COL_ACCENT : COL_FG, tz);
    draw_centered(fb, 170, 1, COL_DIM, "WALL TIME ONLY: THE SIM NEVER READS IT");
    if (sh->clock_field == 3) fill_band(fb, 310, 500, COL_SEL);
    if (sh->clock_field == 4) fill_band(fb, 530, 720, COL_SEL);
    draw_centered(fb, 365, sc, sh->clock_field == 3 ? COL_ACCENT : COL_FG, "SET");
    draw_centered(fb, 585, sc, sh->clock_field == 4 ? COL_ACCENT : COL_FG, "CANCEL");
    draw_centered(fb, ML_FOOTER_Y + 60, 1, COL_DIM, "DIAL/</>: EDIT  ENTER: NEXT/SET");
}

/* sin/cos per minute-step, milli units (integer clock hands). */
static const int16_t SIN60[60] = {
    0,105,208,309,407,500,588,669,743,809,866,914,951,978,995,1000,995,978,951,914,
    866,809,743,669,588,500,407,309,208,105,0,-105,-208,-309,-407,-500,-588,-669,-743,-809,
    -866,-914,-951,-978,-995,-1000,-995,-978,-951,-914,-866,-809,-743,-669,-588,-500,-407,-309,-208,-105
};
static const int16_t COS60[60] = {
    1000,995,978,951,914,866,809,743,669,588,500,407,309,208,105,0,-105,-208,-309,-407,
    -500,-588,-669,-743,-809,-866,-914,-951,-978,-995,-1000,-995,-978,-951,-914,-866,-809,-743,-669,-588,
    -500,-407,-309,-208,-105,0,105,208,309,407,500,588,669,743,809,866,914,951,978,995
};

static void shell_hand(toy_fb_t *fb, int step60, int len_milli, int thick, uint16_t col)
{
    int r = toy_r(fb);
    int len = r * len_milli / 1000;
    step60 %= 60;
    if (step60 < 0) step60 += 60;
    int x = toy_cx(fb) + SIN60[step60] * len / 1000;
    int y = toy_cy(fb) - COS60[step60] * len / 1000;
    for (int t = -thick; t <= thick; t++)
        toy_fb_line(fb, toy_cx(fb) + t, toy_cy(fb), x + t, y, col);
}

static void shell_render_clockface(toy_shell_t *sh, toy_fb_t *fb, toy_time_t now)
{
    int sc = txt_scale(fb);
    int r = toy_r(fb);
    toy_fb_ring(fb, toy_cx(fb), toy_cy(fb), r * 88 / 100, now.valid ? COL_DIM : toy_rgb565(60, 70, 95));
    for (int i = 0; i < 60; i += 5) {
        int len = r * 80 / 100;
        int x = toy_cx(fb) + SIN60[i] * len / 1000;
        int y = toy_cy(fb) - COS60[i] * len / 1000;
        toy_fb_disc(fb, x, y, i % 15 == 0 ? r / 45 + 1 : r / 70 + 1,
                    now.valid ? COL_FG : COL_DIM);
    }
    if (!now.valid) {
        draw_centered(fb, -260, sc * 3, COL_FG, "--:--");
        draw_centered(fb, 60, sc, COL_ACCENT, "SET CLOCK");
        draw_centered(fb, 260, 1, COL_DIM, "ENTER OR TAP TO SET");
        (void)sh;
        return;
    }
    int64_t local = toy_clock_local_seconds(now) % 86400;
    if (local < 0) local += 86400;
    int hh = (int)(local / 3600), mm = (int)((local / 60) % 60), ss = (int)(local % 60);
    shell_hand(fb, ((hh % 12) * 60 + mm) / 12, 480, 1, COL_FG);
    shell_hand(fb, mm, 720, 1, COL_FG);
    shell_hand(fb, ss, 800, 0, COL_ACCENT);
    toy_fb_disc(fb, toy_cx(fb), toy_cy(fb), r / 35 + 1, COL_ACCENT);
    char hms[12], h2[4], m2[4], s2[4], tmp[10], tz[16];
    fmt2(h2, hh); fmt2(m2, mm); fmt2(s2, ss);
    fmt_join3(tmp, sizeof tmp, h2, ":", m2);
    fmt_join3(hms, sizeof hms, tmp, ":", s2);
    draw_centered(fb, 300, sc * 2, COL_FG, hms);
    fmt_tz(tz, sizeof tz, now.tz_offset_min);
    draw_centered(fb, 520, 1, COL_DIM, tz);
    draw_centered(fb, -700, 1, COL_DIM, "TOY");
}

void toy_shell_render(toy_shell_t *sh, toy_fb_t *fb, toy_time_t now)
{
    sh->last_now = now; /* runner-supplied; seeds the clock editor */
    if (sh->screen == TOY_SHELL_RUNNING) return; /* cart owns glass */
    toy_fb_clear(fb, COL_BG);
    switch (sh->screen) {
    case TOY_SHELL_SETTINGS: shell_render_settings(sh, fb); break;
    case TOY_SHELL_CLOCKSET: shell_render_clockset(sh, fb); break;
    case TOY_SHELL_CLOCKFACE: shell_render_clockface(sh, fb, now); break;
    default: shell_render_gallery(sh, fb); break;
    }
}
