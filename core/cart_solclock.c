/* cart_solclock.c - the science clock: mean vs apparent solar
 * time, equation of time, analemma, declination and moon phase,
 * computed from a runner-supplied calendar date.
 *
 * Firewall (docs/TOY-0.md section 3, docs/CARTRIDGES.md "Time
 * layer"): cartridges NEVER read wall time. The runner owns the
 * RTC and hands this cart plain integers - year, month, day,
 * hour, minute, UTC offset - as SET_PARAM events at step
 * boundaries (the shell is expected to re-set them from its RTC
 * about once a minute; the web mock's sliders do the same job in
 * rehearsal). Everything astronomical below is computed from
 * those integers inside hashed state, in muse-softfloat
 * binary64. Between settings the cart free-runs: one step is
 * exactly 60 sim-seconds (the params' own minute resolution, so
 * a runner re-setting each wall minute and the free-run agree
 * tick for tick), and the minute hand advances one minute per
 * step. At the nominal 30 fps demo pacing that is a 1800x
 * fast-forward - the orrery precedent (half a day per step):
 * an untouched desk toy should visibly live, and a live runner
 * re-syncs it anyway.
 *
 * MODEL (derived here from the two physical causes of the
 * equation of time; no astronomy code was copied - the only
 * borrowed numbers are published constants, cited where used):
 *
 * Calendar: proleptic Gregorian, exact integer arithmetic (leap
 * rule: divisible by 4, except centuries not divisible by 400).
 * Days are counted from 2000-01-01; J2000.0 is that date at
 * 12:00. Model time is kept as integer minutes, so date, hands
 * and readouts never pass through a float.
 *
 * Sun: Earth's approximate Keplerian elements from Standish &
 * Williams as tabulated by JPL SSD (the orrery's source,
 * Table 1, EM Bary row) give the mean longitude L and mean
 * anomaly M as linear functions of Julian centuries past
 * J2000; the geocentric mean longitude of the Sun is Earth's
 * heliocentric longitude + 180 degrees. The true longitude is
 * lambda = L + C, where C is the equation of centre, the
 * standard first-principles expansion of Kepler's equation in
 * powers of eccentricity (e = 0.0167):
 *     C = 2e sin M + (5/4) e^2 sin 2M      [radians; O(e^3) cut]
 * The dropped O(e^3) terms are below 5e-6 rad (~0.001 min of
 * time), far under the model's error floor.
 *
 * Equation of time (apparent - mean solar time, the sundial
 * convention: positive means the sundial is fast). Solar time
 * is an hour angle, H = sidereal angle - right ascension, so
 * EoT as an angle is (mean RA) - (apparent RA) = L - alpha,
 * with alpha the Sun's right ascension. Split at lambda:
 *     L - alpha = -(lambda - L) + (lambda - alpha) = -C + R
 * The first term is the eccentricity effect above. The second
 * is the obliquity effect (the "reduction to the equator"):
 * projecting ecliptic longitude onto the equator through the
 * axial tilt eps gives tan(alpha) = cos(eps) tan(lambda),
 * whose expansion in q = tan^2(eps/2) is
 *     R = q sin 2lambda - (q^2/2) sin 4lambda   [radians]
 * (checked term by term against the closed form while writing
 * this: the two-term R tracks atan2 to <0.01 min all year).
 * eps = 23.4392911 deg is the mean obliquity at J2000 (a
 * published constant); its slow drift (-47 arcsec/century) is
 * below this model's floor and not carried. Angles become
 * minutes of time at 1440 min per turn.
 *
 * Declination: sin(delta) = sin(eps) sin(lambda), solved for
 * delta by fixed-count Newton iteration on sinPi (the orrery's
 * Kepler pattern), residual stored.
 *
 * Moon: mean phase only. Age is counted from a pinned known
 * new moon, 2000-01-06 18:14 UTC (chosen epoch constant, a
 * tabulated event), modulo the mean synodic month
 * 29.530588853 d (published mean value). No lunar anomaly
 * correction is applied, so the age is the MEAN age: real
 * lunations vary by hours around the mean and the plotted
 * phase can be off by up to roughly half a day of age. That is
 * the stated accuracy class, not an almanac.
 *
 * ACCURACY CLASS (honest bounds, all checked by the offline
 * probe in the parcel report against tabulated values):
 * EoT is the two-cause series above, +/-30 s class versus an
 * almanac (the four yearly extremes - Feb ~-14 min, May ~+4,
 * Jul ~-6, Nov ~+16 - reproduce within seconds, the zero
 * crossings within ~0.15 min; aberration, nutation and the
 * elements' own arcminute-level fit error are unmodelled).
 * Declination is good to a few hundredths of a degree within
 * the elements' 1800-2050 validity. Moon age ~+/-0.5 d.
 * Apparent solar time here means local mean time + EoT, i.e.
 * the sundial reading ON the time-zone meridian: the cart has
 * no longitude parameter (six-param budget spent on the
 * calendar), so a longitude-within-zone correction, like DST,
 * is runner policy - the runner can fold it into the offset
 * it sends. Likewise leap seconds: civil minutes are taken as
 * given; TT-UTC shifts the Sun by ~0.001 deg, invisible here.
 *
 * Analemma: 36 (EoT, declination) points, one every 10 days of
 * the current year at local mean noon, recomputed in state on
 * any param change (and on a free-run year change). Bounded:
 * 36 evaluations of the same series, only at those boundaries,
 * never per step. Per step the cart re-evaluates the series
 * once for "now" (a handful of sinPi/cosPi calls).
 */
#include "toy_cart.h"
#include "toy_math.h"
#include "toy_util.h"

#define SC_ANA_N        36   /* analemma points, one per 10 days */
#define SC_ASIN_ITERS   6    /* fixed Newton count, declination */
#define SC_PARAM_YEAR   0u
#define SC_PARAM_MONTH  1u
#define SC_PARAM_DAY    2u
#define SC_PARAM_HOUR   3u
#define SC_PARAM_MINUTE 4u
#define SC_PARAM_UTCOFF 5u
#define SC_YEAR_MIN     1900
#define SC_YEAR_MAX     2199

/* State, descending alignment: binary64 bit patterns, then
 * 64-bit counters, then 32-bit words. Analemma 2*36 f64 = 576 B
 * dominates; whole state ~1 KiB of the 64 KiB budget. */
typedef struct {
    /* computed astronomy for "now" (all softfloat) */
    f64 eot_min;          /* equation of time, minutes (app-mean) */
    f64 decl_deg;         /* solar declination, degrees */
    f64 decl_resid;       /* asin Newton residual, diagnostic */
    f64 mean_long_cyc;    /* Sun mean longitude, cycles [0,1) */
    f64 true_long_cyc;    /* Sun true longitude, cycles [0,1) */
    f64 mean_anom_cyc;    /* Earth mean anomaly, cycles [0,1) */
    f64 moon_phase_cyc;   /* 0 new, 0.5 full, cycles of synodic */
    f64 moon_age_d;       /* mean moon age, days */
    f64 moon_illum;       /* illuminated fraction [0,1] */
    f64 moon_sun_x;       /* sun dir for shading: sin(2 pi phase) */
    f64 moon_sun_z;       /*                  -cos(2 pi phase) */
    /* hand direction unit vectors, x right / y up, softfloat */
    f64 min_dx, min_dy;   /* mean-time minute hand */
    f64 hr_dx, hr_dy;     /* mean-time hour hand */
    f64 app_dx, app_dy;   /* apparent solar (sundial) hour hand */
    /* precomputed analemma for ana_year */
    f64 ana_eot[SC_ANA_N];   /* minutes */
    f64 ana_decl[SC_ANA_N];  /* degrees */
    /* constants parsed once at reset (decimal, via f64_dec) */
    f64 el_l0, el_ldot;         /* Earth mean longitude, deg, deg/cy */
    f64 el_varpi0, el_varpidot; /* longitude of perihelion, deg */
    f64 el_e0, el_edot;         /* eccentricity */
    f64 sun_l0, mean_m0, mean_mdot; /* derived geocentric forms */
    f64 sin_eps, obl_q, obl_q2half; /* obliquity series constants */
    f64 synodic_d, newmoon_d;   /* moon: period, epoch (days) */
    f64 two_pi, c360, c1440, days_per_cy, c125;
    /* 64-bit integer time */
    uint64_t steps;
    int64_t base_local_min; /* setting as local minutes past
                               2000-01-01 00:00 local */
    int64_t elapsed_min;    /* free-run minutes since the setting */
    /* 32-bit words: the setting (runner integers, clamped) */
    int32_t set_year, set_month, set_day;
    int32_t set_hour, set_minute, set_utc_off;
    /* current civil reading derived in integers from model time */
    int32_t cur_year, cur_month, cur_day, cur_doy;
    int32_t cur_min_day;    /* local minute of day 0..1439 */
    int32_t ana_year;       /* year the analemma belongs to */
    /* integer readouts for render (rounded from state by the
     * library converter in compute, never from a host float) */
    int32_t eot_min_i;      /* EoT minutes, nearest */
    int32_t decl_tenths_i;  /* declination, tenths of a degree */
    int32_t moon_age_tenths_i;
    uint32_t reserved;      /* explicit pad, kept zero by memset */
} solclock_state;

/* --- small softfloat helpers local to this cart --- */
static f64 sc_from_i32(int32_t a) { return (f64)toy_f64_bits_from_i32(a); }
static f64 sc_floor(f64 a)
{
    msf_flags_t f = 0;
    return msf64_round_to_integral_toward_negative(a, &f);
}
static f64 sc_frac(f64 a) { return f64_sub(a, sc_floor(a)); }
static int32_t sc_nearest_i32(f64 a)
{
    msf_flags_t f = 0;
    return msf64_to_i32_ties_to_even(a, &f);
}

static int32_t sc_clamp(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* --- exact integer calendar (proleptic Gregorian) --- */
static int sc_is_leap(int32_t y)
{
    return (y % 4 == 0 && y % 100 != 0) || y % 400 == 0;
}

/* Days from 2000-01-01 to y-m-d. Month is clamped to 1..12 for
 * partially-set state during the reset defaults sequence; the
 * day may overflow its month (e.g. Feb 31) and normalises
 * arithmetically here, so a runner setting parameters in any
 * order reaches the same absolute day once all have landed,
 * and the current date read back is always a real date. The
 * year loop is bounded by the clamped parameter range. */
static int32_t sc_days_since_2000(int32_t y, int32_t m, int32_t d)
{
    static const int32_t mcum[12] = {
        0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334
    };
    int32_t days = 0;
    if (y >= 2000) {
        for (int32_t yy = 2000; yy < y; yy++)
            days += sc_is_leap(yy) ? 366 : 365;
    } else {
        for (int32_t yy = y; yy < 2000; yy++)
            days -= sc_is_leap(yy) ? 366 : 365;
    }
    int32_t mm = sc_clamp(m, 1, 12);
    days += mcum[mm - 1];
    if (mm > 2 && sc_is_leap(y)) days += 1;
    days += d - 1;
    return days;
}

/* Inverse of the above for a normalised absolute day. The
 * guard counters make the loops formally bounded; the ranges
 * reachable from clamped params plus any plausible free-run
 * are a few hundred years at most. */
static void sc_civil_from_days(int32_t days, int32_t *y,
                               int32_t *m, int32_t *d)
{
    static const int32_t dim[12] = {
        31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
    };
    int32_t yy = 2000;
    for (int guard = 0; guard < 1000; guard++) {
        int32_t yl = sc_is_leap(yy) ? 366 : 365;
        if (days >= yl) { days -= yl; yy++; }
        else if (days < 0) { yy--; days += sc_is_leap(yy) ? 366 : 365; }
        else break;
    }
    int32_t mm = 1;
    for (int guard = 0; guard < 12; guard++) {
        int32_t ml = dim[mm - 1] + ((mm == 2 && sc_is_leap(yy)) ? 1 : 0);
        if (days >= ml) { days -= ml; mm++; }
        else break;
    }
    *y = yy;
    *m = mm;
    *d = days + 1;
}

/* Split absolute minutes into whole days + minute of day,
 * floor semantics for negative (pre-2000) instants. */
static int64_t sc_day_split(int64_t abs_min, int32_t *min_of_day)
{
    int64_t q = abs_min / 1440;
    int64_t r = abs_min % 1440;
    if (r < 0) { q -= 1; r += 1440; }
    *min_of_day = (int32_t)r;
    return q;
}

/* Days past J2000.0 for an absolute UTC minute count, as
 * softfloat: whole days exactly from the integer, plus the
 * minute fraction. J2000.0 = 2000-01-01 12:00, hence -0.5 d. */
static f64 sc_dj2000(const solclock_state *s, int64_t utc_abs_min)
{
    int32_t rem = 0;
    int64_t day = sc_day_split(utc_abs_min, &rem);
    f64 d = f64_sub(sc_from_i32((int32_t)day), F64_HALF);
    return f64_add(d, f64_div(sc_from_i32(rem), s->c1440));
}

/* Declination from sin(delta): fixed-count Newton on
 * sinPi(2x) - y = 0 in cycles (derivative 2pi cosPi(2x)),
 * from the small-angle start x = y/2pi. |y| <= sin(eps) so
 * the derivative never gets small. Returns delta in cycles. */
static f64 sc_asin_cycles(const solclock_state *s, f64 y, f64 *resid)
{
    f64 x = f64_div(y, s->two_pi);
    for (int it = 0; it < SC_ASIN_ITERS; it++) {
        f64 twox = f64_mul(F64_TWO, x);
        f64 f = f64_sub(f64_sin_pi(twox), y);
        f64 fp = f64_mul(s->two_pi, f64_cos_pi(twox));
        x = f64_sub(x, f64_div(f, fp));
    }
    *resid = f64_sub(f64_sin_pi(f64_mul(F64_TWO, x)), y);
    return x;
}

/* The series of the header, evaluated at days past J2000.
 * All angles travel in cycles; sinPi(2c) is sin of c turns. */
static void sc_astro(const solclock_state *s, f64 d_j2000,
                     f64 *eot_min, f64 *decl_deg, f64 *resid,
                     f64 *mean_long, f64 *true_long, f64 *mean_anom)
{
    f64 T = f64_div(d_j2000, s->days_per_cy); /* Julian centuries */
    f64 lcyc = sc_frac(f64_div(f64_fma(s->el_ldot, T, s->sun_l0), s->c360));
    f64 mcyc = sc_frac(f64_div(f64_fma(s->mean_mdot, T, s->mean_m0), s->c360));
    f64 e = f64_fma(s->el_edot, T, s->el_e0);

    /* Equation of centre, radians: 2e sinM + 1.25 e^2 sin2M. */
    f64 sin_m = f64_sin_pi(f64_mul(F64_TWO, mcyc));
    f64 sin_2m = f64_sin_pi(f64_mul(f64_mul(F64_TWO, F64_TWO), mcyc));
    f64 crad = f64_fma(f64_mul(s->c125, f64_mul(e, e)), sin_2m,
                       f64_mul(f64_mul(F64_TWO, e), sin_m));
    f64 lamcyc = sc_frac(f64_add(lcyc, f64_div(crad, s->two_pi)));

    /* Reduction to the equator, radians:
     * q sin(2 lambda) - (q^2/2) sin(4 lambda). */
    f64 four = f64_mul(F64_TWO, F64_TWO);
    f64 sin_2l = f64_sin_pi(f64_mul(four, lamcyc));
    f64 sin_4l = f64_sin_pi(f64_mul(f64_mul(four, F64_TWO), lamcyc));
    f64 rrad = f64_sub(f64_mul(s->obl_q, sin_2l),
                       f64_mul(s->obl_q2half, sin_4l));

    /* EoT angle = -C + R, to minutes at 1440 per turn. */
    f64 eot_rad = f64_sub(rrad, crad);
    *eot_min = f64_mul(f64_div(eot_rad, s->two_pi), s->c1440);

    /* Declination: sin(delta) = sin(eps) sin(lambda). */
    f64 sin_decl = f64_mul(s->sin_eps, f64_sin_pi(f64_mul(F64_TWO, lamcyc)));
    *decl_deg = f64_mul(sc_asin_cycles(s, sin_decl, resid), s->c360);
    *mean_long = lcyc;
    *true_long = lamcyc;
    *mean_anom = mcyc;
}

/* Recompute every "now" quantity from base + elapsed integer
 * minutes. Called by apply (via resync) and by every step. */
static void sc_compute_now(solclock_state *s)
{
    int64_t local_abs = s->base_local_min + s->elapsed_min;
    int32_t min_day = 0;
    int64_t local_day = sc_day_split(local_abs, &min_day);
    s->cur_min_day = min_day;
    sc_civil_from_days((int32_t)local_day, &s->cur_year,
                       &s->cur_month, &s->cur_day);
    s->cur_doy = (int32_t)local_day -
                 sc_days_since_2000(s->cur_year, 1, 1) + 1;

    int64_t utc_abs = local_abs - (int64_t)s->set_utc_off;
    f64 dj = sc_dj2000(s, utc_abs);
    sc_astro(s, dj, &s->eot_min, &s->decl_deg, &s->decl_resid,
             &s->mean_long_cyc, &s->true_long_cyc, &s->mean_anom_cyc);

    /* Moon mean phase from the pinned new-moon epoch. utc days
     * past 2000-01-01 00:00 = dj + 0.5 by construction. */
    f64 utc_days = f64_add(dj, F64_HALF);
    f64 cyc = f64_div(f64_sub(utc_days, s->newmoon_d), s->synodic_d);
    s->moon_phase_cyc = sc_frac(cyc);
    s->moon_age_d = f64_mul(s->moon_phase_cyc, s->synodic_d);
    f64 two_ph = f64_mul(F64_TWO, s->moon_phase_cyc);
    s->moon_sun_x = f64_sin_pi(two_ph);
    s->moon_sun_z = f64_neg(f64_cos_pi(two_ph));
    s->moon_illum = f64_mul(f64_sub(F64_ONE, f64_cos_pi(two_ph)), F64_HALF);

    /* Hands. Mean hands from the integer minute of day;
     * apparent (sundial) hour hand from minute + EoT, at
     * 720 minutes per turn of an hour hand. Angle 0 is up,
     * clockwise positive: dx = sin, dy(up) = cos. */
    f64 mod = sc_from_i32(min_day);
    f64 c60 = f64_mul(sc_from_i32(6), sc_from_i32(10));    /* 60 */
    f64 c720 = f64_mul(c60, sc_from_i32(12));              /* 720 */
    f64 min_cyc = sc_frac(f64_div(mod, c60));
    f64 hr_cyc = sc_frac(f64_div(mod, c720));
    f64 app_cyc = sc_frac(f64_div(f64_add(mod, s->eot_min), c720));
    f64 a2 = f64_mul(F64_TWO, min_cyc);
    s->min_dx = f64_sin_pi(a2);
    s->min_dy = f64_cos_pi(a2);
    a2 = f64_mul(F64_TWO, hr_cyc);
    s->hr_dx = f64_sin_pi(a2);
    s->hr_dy = f64_cos_pi(a2);
    a2 = f64_mul(F64_TWO, app_cyc);
    s->app_dx = f64_sin_pi(a2);
    s->app_dy = f64_cos_pi(a2);

    /* Integer readouts, rounded by the library converter. */
    s->eot_min_i = sc_nearest_i32(s->eot_min);
    s->decl_tenths_i = sc_nearest_i32(f64_mul(s->decl_deg, sc_from_i32(10)));
    s->moon_age_tenths_i =
        sc_nearest_i32(f64_mul(s->moon_age_d, sc_from_i32(10)));
}

/* Analemma for one calendar year: EoT and declination at
 * local mean noon every 10 days from Jan 1. 36 bounded
 * evaluations; runs only on param change / year change. */
static void sc_compute_analemma(solclock_state *s, int32_t year)
{
    int32_t jan1 = sc_days_since_2000(year, 1, 1);
    for (int k = 0; k < SC_ANA_N; k++) {
        int64_t utc_abs = ((int64_t)jan1 + (int64_t)(k * 10)) * 1440 +
                          720 - (int64_t)s->set_utc_off;
        f64 dj = sc_dj2000(s, utc_abs);
        f64 eot, decl, resid, ml, tl, ma;
        sc_astro(s, dj, &eot, &decl, &resid, &ml, &tl, &ma);
        s->ana_eot[k] = eot;
        s->ana_decl[k] = decl;
    }
    s->ana_year = year;
}

/* A setting landed: re-derive the base instant from the
 * integers, restart free-run from it, refresh everything. */
static void sc_resync(solclock_state *s)
{
    int32_t days = sc_days_since_2000(s->set_year, s->set_month,
                                      s->set_day);
    s->base_local_min = (int64_t)days * 1440 +
                        (int64_t)s->set_hour * 60 + s->set_minute;
    s->elapsed_min = 0;
    sc_compute_now(s);
    sc_compute_analemma(s, s->cur_year);
}

/* apply(): SET_PARAM only (TOY_CAP_PARAMS). Values are
 * clamped to the declared ranges here, per the seam; unknown
 * types and out-of-range ids are ignored. */
static void solclock_apply(void *st_, const toy_event_t *ev)
{
    solclock_state *s = (solclock_state *)st_;
    if (ev->type != TOY_EV_SET_PARAM) return;
    switch (ev->id) {
    case SC_PARAM_YEAR:   s->set_year = sc_clamp(ev->v, SC_YEAR_MIN, SC_YEAR_MAX); break;
    case SC_PARAM_MONTH:  s->set_month = sc_clamp(ev->v, 1, 12); break;
    case SC_PARAM_DAY:    s->set_day = sc_clamp(ev->v, 1, 31); break;
    case SC_PARAM_HOUR:   s->set_hour = sc_clamp(ev->v, 0, 23); break;
    case SC_PARAM_MINUTE: s->set_minute = sc_clamp(ev->v, 0, 59); break;
    case SC_PARAM_UTCOFF: s->set_utc_off = sc_clamp(ev->v, -720, 840); break;
    default: return; /* not ours: no resync */
    }
    sc_resync(s);
}

static void solclock_reset(void *st_)
{
    solclock_state *s = (solclock_state *)st_;
    toy_memset(s, 0, sizeof *s);
    /* Standish & Williams approximate elements, JPL SSD
     * Table 1 (EM Bary row), the orrery's cited source:
     * mean longitude, longitude of perihelion, eccentricity
     * at J2000.0 and their rates per Julian century. */
    s->el_l0 = f64_dec("100.46457166");
    s->el_ldot = f64_dec("35999.37244981");
    s->el_varpi0 = f64_dec("102.93768193");
    s->el_varpidot = f64_dec("0.32327364");
    s->el_e0 = f64_dec("0.01671123");
    s->el_edot = f64_dec("-0.00004392");
    s->two_pi = f64_dec("6.28318530717958647692528676655900576839433879875");
    s->c360 = f64_dec("360");
    s->c1440 = f64_dec("1440");
    s->days_per_cy = f64_dec("36525");
    s->c125 = f64_dec("1.25");
    s->synodic_d = f64_dec("29.530588853");
    /* Geocentric forms: the Sun's mean longitude is Earth's
     * heliocentric longitude seen from the other side (+180),
     * and the mean anomaly is shared, M = L - varpi. */
    s->sun_l0 = f64_add(s->el_l0, f64_dec("180"));
    s->mean_m0 = f64_sub(s->el_l0, s->el_varpi0);
    s->mean_mdot = f64_sub(s->el_ldot, s->el_varpidot);
    /* Obliquity constants from eps itself, in softfloat:
     * sin(eps), and q = tan^2(eps/2) for the reduction series. */
    f64 eps_cyc = f64_div(f64_dec("23.4392911"), s->c360);
    s->sin_eps = f64_sin_pi(f64_mul(F64_TWO, eps_cyc));
    f64 tan_half = f64_div(f64_sin_pi(eps_cyc), f64_cos_pi(eps_cyc));
    s->obl_q = f64_mul(tan_half, tan_half);
    s->obl_q2half = f64_mul(f64_mul(s->obl_q, s->obl_q), F64_HALF);
    /* Pinned new-moon epoch 2000-01-06 18:14 UTC, as days
     * past 2000-01-01 00:00 UTC from its integer minutes:
     * 5 d + (18*60+14) min = 8294 min. */
    s->newmoon_d = f64_div(sc_from_i32(5 * 1440 + 18 * 60 + 14), s->c1440);
    /* Declared defaults through the step-0 sequence, exactly
     * as a runner applies user settings (the julia pattern):
     * reset() means the descriptor defaults, nothing else.
     * Each event resyncs; the last one leaves the final,
     * fully-set state. */
    for (uint32_t i = 0; i < toy_cart_solclock.param_count; i++) {
        toy_event_t ev;
        if (toy_param_event(&toy_cart_solclock, i,
                            toy_cart_solclock.params[i].i_def, 0, &ev) == 0)
            solclock_apply(s, &ev);
    }
}

static void solclock_step(void *st_)
{
    solclock_state *s = (solclock_state *)st_;
    s->steps++;
    s->elapsed_min += 1; /* one step = 60 sim-seconds, exactly */
    sc_compute_now(s);
    if (s->cur_year != s->ana_year)
        sc_compute_analemma(s, s->cur_year);
}

/* --- render: display conversion only, never feeds state --- */
static double sc_d64(f64 a)
{
    union { f64 u; double d; } v;
    v.u = a;
    return v.d;
}

/* Native display sqrt by fixed Newton iterations (the
 * house render pattern; no libm for freestanding hosts). */
static double sc_dsqrt(double v)
{
    if (v <= 0.0) return 0.0;
    double g = v > 1.0 ? v : 1.0;
    for (int i = 0; i < 24; i++) g = 0.5 * (g + v / g);
    return g;
}

/* A clock hand from a state unit vector (dx right, dy up):
 * a short counterweight tail and a tapered-feel shaft built
 * from parallel lines, width in pixels either side. */
static void sc_hand(toy_fb_t *fb, int cx, int cy, double dx, double dy_up,
                    int len, int tail, int half_w, uint16_t col)
{
    double sx = dx, sy = -dy_up;          /* screen coords */
    double px = -sy, py = sx;             /* perpendicular */
    int ex = cx + (int)(sx * (double)len);
    int ey = cy + (int)(sy * (double)len);
    int bx = cx - (int)(sx * (double)tail);
    int by = cy - (int)(sy * (double)tail);
    for (int t = -half_w; t <= half_w; t++) {
        int ox = (int)(px * (double)t + (t < 0 ? -0.5 : 0.5));
        int oy = (int)(py * (double)t + (t < 0 ? -0.5 : 0.5));
        if (t == 0) { ox = 0; oy = 0; }
        toy_fb_line(fb, bx + ox, by + oy, ex + ox, ey + oy, col);
    }
}

/* Append unsigned decimal of v (at least width digits,
 * zero-padded) at buf[k]; return new k. Integer only. */
static int sc_put_u(char *buf, int k, uint32_t v, int width)
{
    char tmp[10];
    int t = 0;
    if (v == 0) tmp[t++] = '0';
    while (v > 0 && t < 10) { tmp[t++] = (char)('0' + (v % 10u)); v /= 10u; }
    while (t < width && t < 10) tmp[t++] = '0';
    while (t > 0) buf[k++] = tmp[--t];
    return k;
}

/* Append a signed tenths value as e.g. -15.9 (plus shown
 * when force_plus), from the integer readout in state. */
static int sc_put_tenths(char *buf, int k, int32_t tv, int force_plus)
{
    uint32_t a;
    if (tv < 0) { buf[k++] = '-'; a = (uint32_t)(-tv); }
    else { if (force_plus) buf[k++] = '+'; a = (uint32_t)tv; }
    k = sc_put_u(buf, k, a / 10u, 0);
    buf[k++] = '.';
    buf[k++] = (char)('0' + (a % 10u));
    return k;
}

static void sc_center_text(toy_fb_t *fb, int cx, int y, int sc,
                           uint16_t col, const char *s, int nchars)
{
    toy_fb_text(fb, cx - (nchars * 4 * sc - sc) / 2, y, sc, col, s);
}

/* Moon shading for one (|x|, |y|) = (ax, ay) orbit: pixels
 * (x, +/-ay) for x = ax and (when ax > 0) x = -ax, with nx
 * the matching +/- na. d and its two tests are the exact
 * per-pixel expressions; d does not depend on the y sign. */
static void sc_moon_orbit(toy_fb_t *fb, int mx, int my, int ax, int ay,
                          double na, double sunx, double base,
                          uint16_t lit, uint16_t limb)
{
    for (int i = 0; i < (ax > 0 ? 2 : 1); i++) {
        int x = i ? -ax : ax;
        double d = (i ? -na : na) * sunx + base;
        if (!(d > 0.0)) continue;
        uint16_t c = d > 0.25 ? lit : limb;
        toy_fb_px(fb, mx + x, my + ay, c);
        if (ay > 0) toy_fb_px(fb, mx + x, my - ay, c);
    }
}

static void solclock_render(const void *st_, toy_fb_t *fb)
{
    const solclock_state *s = (const solclock_state *)st_;
    /* 12 tick directions, angle from 12 o'clock clockwise:
     * display literals (sin/cos of k*30 degrees, known exact
     * forms rounded for pixels). */
    static const double tick_sin[12] = {
        0.0, 0.5, 0.8660254, 1.0, 0.8660254, 0.5,
        0.0, -0.5, -0.8660254, -1.0, -0.8660254, -0.5
    };
    static const double tick_cos[12] = {
        1.0, 0.8660254, 0.5, 0.0, -0.5, -0.8660254,
        -1.0, -0.8660254, -0.5, 0.0, 0.5, 0.8660254
    };
    int cx = toy_cx(fb), cy = toy_cy(fb), R = toy_r(fb);
    int sc = R >= 150 ? 2 : 1;
    uint16_t ink = toy_rgb565(190, 210, 235);
    uint16_t gold = toy_rgb565(255, 200, 70);
    toy_fb_clear(fb, toy_rgb565(4, 6, 16));
    toy_fb_ring(fb, cx, cy, R - 2, toy_rgb565(28, 34, 54));
    sc_center_text(fb, cx, cy - R + 7 * sc, sc,
                   toy_rgb565(150, 170, 200), "SOLCLOCK", 8);

    /* --- main dial, upper centre --- */
    int fx = cx, fy = cy - (R * 17) / 100, rf = (R * 60) / 100;
    toy_fb_ring(fb, fx, fy, rf, toy_rgb565(70, 85, 120));
    toy_fb_ring(fb, fx, fy, rf - 1, toy_rgb565(40, 50, 80));
    for (int k = 0; k < 12; k++) {
        int big = (k % 3 == 0);
        int r1 = rf - (big ? rf / 8 + 2 : rf / 14 + 1);
        int x0 = fx + (int)(tick_sin[k] * (double)r1);
        int y0 = fy - (int)(tick_cos[k] * (double)r1);
        int x1 = fx + (int)(tick_sin[k] * (double)(rf - 2));
        int y1 = fy - (int)(tick_cos[k] * (double)(rf - 2));
        uint16_t tc = big ? ink : toy_rgb565(110, 130, 165);
        toy_fb_line(fb, x0, y0, x1, y1, tc);
        if (big) {
            /* thicken the quarter ticks by one parallel line */
            toy_fb_line(fb, x0 + (tick_cos[k] > 0.4 || tick_cos[k] < -0.4 ? 1 : 0),
                        y0 + (tick_cos[k] > 0.4 || tick_cos[k] < -0.4 ? 0 : 1),
                        x1 + (tick_cos[k] > 0.4 || tick_cos[k] < -0.4 ? 1 : 0),
                        y1 + (tick_cos[k] > 0.4 || tick_cos[k] < -0.4 ? 0 : 1), tc);
        }
    }
    /* Apparent solar hand first (thin, gold, sun disc on the
     * rim at its angle), then the mean-time hands in white so
     * clock time reads first at a glance. */
    double apx = sc_d64(s->app_dx), apy = sc_d64(s->app_dy);
    sc_hand(fb, fx, fy, apx, apy, (rf * 72) / 100, rf / 8, sc >= 2 ? 1 : 0, gold);
    int sx = fx + (int)(apx * (double)((rf * 88) / 100));
    int sy = fy - (int)(apy * (double)((rf * 88) / 100));
    toy_fb_disc(fb, sx, sy, rf / 16 + 1, gold);
    toy_fb_disc(fb, sx, sy, rf / 32 + 1, toy_rgb565(255, 244, 200));
    sc_hand(fb, fx, fy, sc_d64(s->hr_dx), sc_d64(s->hr_dy),
            (rf * 52) / 100, rf / 7, sc, ink);
    sc_hand(fb, fx, fy, sc_d64(s->min_dx), sc_d64(s->min_dy),
            (rf * 80) / 100, rf / 7, sc >= 2 ? 1 : 0, ink);
    toy_fb_disc(fb, fx, fy, rf / 24 + 1, ink);

    /* --- moon subdial, lower left --- */
    int mx = cx - (R * 44) / 100, my = cy + (R * 48) / 100;
    int rm = (R * 17) / 100;
    toy_fb_disc(fb, mx, my, rm, toy_rgb565(16, 20, 38));
    toy_fb_ring(fb, mx, my, rm, toy_rgb565(70, 85, 120));
    {
        /* Phase shading from first principles: the visible
         * hemisphere's unit normal n = (nx, ny, nz); the sun
         * direction in the sky plane, from state, is
         * (sin phi, 0, -cos phi) for phase phi (0 new: sun
         * beyond the Moon, dark; quarter: sun to the right,
         * the waxing side in the northern sky; full: sun
         * behind the viewer, lit). A point is lit when
         * n . s > 0. Display geometry only.
         *
         * Cost shape (toy5-render, pixel-identical): rr and
         * nz depend only on the unordered pair {|x|, |y|} -
         * fl(nx*nx) is sign-blind and the sum commutes in
         * IEEE - and fl((-k)/rm) == -fl(k/rm), so each
         * 24-Newton-iteration sc_dsqrt (a soft-double
         * division per iteration on the target) is computed
         * once per unordered pair and shared by its up to 8
         * symmetric pixels (379 pairs for 2,821 disc pixels
         * at 360). Every double fed to sc_dsqrt and to the
         * d > 0 / d > 0.25 tests is bit-for-bit the value
         * the per-pixel form computed; the pixels are
         * disjoint, so visit order cannot matter. rm <= 0
         * (a sub-6px glass) drew nothing in the old form
         * (0/0 = NaN, NaN > 0 false) and draws nothing here. */
        double sunx = sc_d64(s->moon_sun_x), sunz = sc_d64(s->moon_sun_z);
        uint16_t lit = toy_rgb565(225, 230, 240);
        uint16_t limb = toy_rgb565(150, 160, 185);
        if (rm > 0) {
            for (int b = 0; b <= rm; b++) {
                double nb = (double)b / (double)rm;
                double nb2 = nb * nb;
                if (nb2 > 1.0) continue; /* rr >= nb2: whole row out */
                for (int a = 0; a <= b; a++) {
                    double na = (double)a / (double)rm;
                    double rr = na * na + nb2;
                    if (rr > 1.0) continue;
                    double base = sc_dsqrt(1.0 - rr) * sunz;
                    sc_moon_orbit(fb, mx, my, a, b, na, sunx, base, lit, limb);
                    if (a != b)
                        sc_moon_orbit(fb, mx, my, b, a, nb, sunx, base,
                                      lit, limb);
                }
            }
        }
    }
    {
        /* moon age readout under its disc: "26.0D" */
        char buf[8];
        int k = sc_put_tenths(buf, 0, s->moon_age_tenths_i, 0);
        buf[k++] = 'D';
        buf[k] = '\0';
        sc_center_text(fb, mx, my + rm + 3 * sc, sc,
                       toy_rgb565(150, 170, 200), buf, k);
    }

    /* --- analemma, lower right: the year's (EoT, decl) at
     * mean noon, +EoT to the right, north declination up.
     * Axes through (0,0); the current Sun is the gold disc. */
    int ax = cx + (R * 44) / 100, ay = cy + (R * 48) / 100;
    double xsc = (double)((R * 19) / 100) / 17.0;   /* px per EoT min */
    double ysc = (double)((R * 23) / 100) / 23.44;  /* px per degree */
    uint16_t ana = toy_rgb565(90, 185, 205);
    toy_fb_line(fb, ax - (int)(17.0 * xsc), ay, ax + (int)(17.0 * xsc), ay,
                toy_rgb565(35, 45, 70));
    toy_fb_line(fb, ax, ay - (int)(23.44 * ysc), ax, ay + (int)(23.44 * ysc),
                toy_rgb565(35, 45, 70));
    for (int k = 0; k < SC_ANA_N; k++) {
        int k2 = (k + 1) % SC_ANA_N; /* closes 350d -> next Jan 1 */
        int x0 = ax + (int)(sc_d64(s->ana_eot[k]) * xsc);
        int y0 = ay - (int)(sc_d64(s->ana_decl[k]) * ysc);
        int x1 = ax + (int)(sc_d64(s->ana_eot[k2]) * xsc);
        int y1 = ay - (int)(sc_d64(s->ana_decl[k2]) * ysc);
        toy_fb_line(fb, x0, y0, x1, y1, ana);
    }
    toy_fb_disc(fb, ax + (int)(sc_d64(s->eot_min) * xsc),
                ay - (int)(sc_d64(s->decl_deg) * ysc),
                (R * 2) / 100 + 1, gold);
    {
        /* declination readout under the plot: "D-15.9" */
        char buf[10];
        int k = 0;
        buf[k++] = 'D';
        k = sc_put_tenths(buf, k, s->decl_tenths_i, 0);
        buf[k] = '\0';
        sc_center_text(fb, ax, ay + (int)(23.44 * ysc) + 3 * sc, sc,
                       toy_rgb565(150, 170, 200), buf, k);
    }

    /* --- date and EoT, bottom centre, from integers --- */
    {
        char buf[12];
        int k = 0;
        k = sc_put_u(buf, k, (uint32_t)s->cur_year, 4);
        buf[k++] = '-';
        k = sc_put_u(buf, k, (uint32_t)s->cur_month, 2);
        buf[k++] = '-';
        k = sc_put_u(buf, k, (uint32_t)s->cur_day, 2);
        buf[k] = '\0';
        sc_center_text(fb, cx, cy + (R * 62) / 100, sc, ink, buf, k);
    }
    {
        char buf[12];
        int k = 0;
        buf[k++] = 'E'; buf[k++] = 'O'; buf[k++] = 'T'; buf[k++] = ' ';
        int32_t v = s->eot_min_i;
        if (v < 0) { buf[k++] = '-'; v = -v; } else buf[k++] = '+';
        k = sc_put_u(buf, k, (uint32_t)v, 0);
        buf[k++] = 'M';
        buf[k] = '\0';
        sc_center_text(fb, cx, cy + (R * 62) / 100 + 7 * sc, sc, gold, buf, k);
    }
}

static const toy_param_t solclock_params[] = {
    /* The runner contract: six live I32s, applied in this
     * index order. The defaults pin the proof's opening
     * instant; an untouched session runs exactly this. */
    { "year", "Year", TOY_PARAM_I32, 0,
      SC_YEAR_MIN, SC_YEAR_MAX, 2026, 0, 0, 0, 0 },
    { "month", "Month", TOY_PARAM_I32, 0,
      1, 12, 11, 0, 0, 0, 0 },
    { "day", "Day", TOY_PARAM_I32, 0,
      1, 31, 6, 0, 0, 0, 0 },
    { "hour", "Hour", TOY_PARAM_I32, 0,
      0, 23, 10, 0, 0, 0, 0 },
    { "minute", "Minute", TOY_PARAM_I32, 0,
      0, 59, 0, 0, 0, 0, 0 },
    { "utc_offset_minutes", "UTC offset (min)", TOY_PARAM_I32, 0,
      -720, 840, 0, 0, 0, 0, 0 },
};

/* Proof script (one sim-day in two halves, reports at 360s):
 * step 0 pins 2026-11-06 10:00 UTC+0 through the step-0
 * settings path (near the EoT maximum, ~+16 min, so the gold
 * sundial hand sits visibly ahead of the mean hands, and the
 * Moon is a waning crescent of mean age ~26 d). 720 steps
 * free-run the hands to 22:00 the same day. At step 721 a
 * five-event jump re-sets the cart to 2027-02-11 12:00 - the
 * EoT minimum (~-14 min) in a new year, so the analemma
 * recompute boundary and the sundial hand falling BEHIND the
 * mean hands both land inside the hashed stream - and 720
 * more steps run past local midnight into 2027-02-12,
 * exercising the integer date rollover. */
static const toy_proof_input_t solclock_proof[] = {
    {   0, { TOY_EV_SET_PARAM, SC_PARAM_YEAR,   0, 0, 2026, 0, 0 } },
    {   0, { TOY_EV_SET_PARAM, SC_PARAM_MONTH,  0, 0, 11,   0, 0 } },
    {   0, { TOY_EV_SET_PARAM, SC_PARAM_DAY,    0, 0, 6,    0, 0 } },
    {   0, { TOY_EV_SET_PARAM, SC_PARAM_HOUR,   0, 0, 10,   0, 0 } },
    {   0, { TOY_EV_SET_PARAM, SC_PARAM_MINUTE, 0, 0, 0,    0, 0 } },
    {   0, { TOY_EV_SET_PARAM, SC_PARAM_UTCOFF, 0, 0, 0,    0, 0 } },
    { 721, { TOY_EV_SET_PARAM, SC_PARAM_YEAR,   0, 0, 2027, 0, 0 } },
    { 721, { TOY_EV_SET_PARAM, SC_PARAM_MONTH,  0, 0, 2,    0, 0 } },
    { 721, { TOY_EV_SET_PARAM, SC_PARAM_DAY,    0, 0, 11,   0, 0 } },
    { 721, { TOY_EV_SET_PARAM, SC_PARAM_HOUR,   0, 0, 12,   0, 0 } },
    { 721, { TOY_EV_SET_PARAM, SC_PARAM_MINUTE, 0, 0, 0,    0, 0 } },
};

/* Pacing: one step is one sim-minute; demo 1 step/frame is
 * the 1800x fast-forward argued in the header. Proof 1440
 * steps is two 720-step half-days as scripted above - each
 * step is one series evaluation (ten sinPi/cosPi-class calls
 * plus a 6-iteration asin), so the whole proof is far under
 * the routine board gate; measured figures are in the parcel
 * report, no board run is claimed here. */
const toy_cart_t toy_cart_solclock = {
    "solclock", "Solar Clock",
    "Sundial hand vs clock hands: equation of time, analemma, moon.",
    sizeof(solclock_state), 1, 1440, 360,
    solclock_reset, solclock_step, solclock_render,
    /* explicit WAVE-T2 tail: apply, params, param_count,
     * proof_inputs, proof_input_count, input_caps,
     * state_version, feed, sig_spec (signal-deaf) */
    solclock_apply, solclock_params, 6, solclock_proof, 11,
    TOY_CAP_PARAMS, 0, 0, { 0 }
};
