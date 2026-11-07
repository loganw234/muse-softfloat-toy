/* clock_civil.h - pure civil-date <-> Unix-epoch conversion for
 * the PCF85063-backed runner clock (toy5-shell).
 *
 * Host-compilable, integer-only, no allocation: the same seam
 * shape as the sensor drivers (board/imu_qmi8658.h etc.), unit
 * tested by tools/product_input_test.c.
 *
 * Calendar discipline matches the core: proleptic Gregorian
 * with the exact leap rule core/cart_solclock.c uses
 * (sc_is_leap: y%4==0 && y%100!=0, or y%400==0), so 2000 is a
 * leap year, 2100 is not. Where solclock counts days from
 * 2000-01-01 with a bounded year loop (its params are clamped
 * to a few centuries), this header uses the equivalent O(1)
 * days-from-civil / civil-from-days era arithmetic (Howard
 * Hinnant's public-domain algorithm) anchored at the Unix
 * epoch, because toy_time_t.utc_seconds is epoch seconds and
 * the runner must convert both ways every frame. The two
 * agree on every date either can name; the host test pins the
 * solclock-era anchors (2000-01-01 = day 10957) directly.
 *
 * RTC convention (board/clock_rtc.c, busfix round): the
 * PCF85063 registers hold UTC civil time. tz is applied
 * exactly once, at display (toy_clock_local_seconds =
 * utc + tz in core/toy_clock.h) and when a CLOCK SET edit
 * converts the owner's local hh:mm to UTC for set() (the
 * commit combiner below). This replaced the original
 * registers-hold-LOCAL convention after two shell boots
 * 1h36m apart reported UTC moving BACKWARD 5h23m53s: the
 * registers had been set to host UTC by toy_sensor_test's
 * SET (which writes fields verbatim), then read as local,
 * and a tz misread between the boots (see the RAM-byte
 * note below) re-interpreted the same registers under a
 * different offset. With UTC in the registers, a wrong tz
 * can only mislabel the displayed local time - it can
 * never move UTC. toy_sensor_test's SET therefore takes
 * UTC, matching what was actually typed on the live board.
 * The chip years are BCD 00..99 reported as 2000..2099
 * (rtc_pcf85063.h), which bounds what set() can store;
 * conversions here are general.
 *
 * Weekday convention: 0 = Sunday .. 6 = Saturday. The
 * PCF85063 weekday register is a free-running 0..6 counter
 * with a user-defined meaning (datasheet Table 5), so the
 * runner owns the convention; 1970-01-01 was a Thursday (4).
 */
#ifndef TOY_CLOCK_CIVIL_H
#define TOY_CLOCK_CIVIL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int32_t year;   /* full year, e.g. 2026 */
    int32_t month;  /* 1..12 */
    int32_t day;    /* 1..31 */
    int32_t hour;   /* 0..23 */
    int32_t minute; /* 0..59 */
    int32_t second; /* 0..59 (leap seconds do not exist here) */
} toy_civil_t;

static inline bool toy_civil_is_leap(int32_t y)
{
    return (y % 4 == 0 && y % 100 != 0) || y % 400 == 0;
}

static inline int32_t toy_civil_days_in_month(int32_t y, int32_t m)
{
    static const int8_t dim[12] = {
        31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
    };
    if (m < 1 || m > 12)
        return 0;
    return dim[m - 1] + ((m == 2 && toy_civil_is_leap(y)) ? 1 : 0);
}

/* True iff every field is a real civil instant (month lengths
 * and leap years judged; this is the cross-check the RTC
 * driver's per-field range gate deliberately does not do). */
static inline bool toy_civil_valid(const toy_civil_t *c)
{
    if (c->month < 1 || c->month > 12)
        return false;
    if (c->day < 1 || c->day > toy_civil_days_in_month(c->year, c->month))
        return false;
    return c->hour >= 0 && c->hour <= 23 && c->minute >= 0 &&
           c->minute <= 59 && c->second >= 0 && c->second <= 59;
}

/* Days since 1970-01-01 for a Gregorian date (Hinnant's
 * days_from_civil). Floor-correct for pre-epoch years too.
 * Pre: (y,m,d) is a real date; arithmetic normalises nothing. */
static inline int64_t toy_days_from_civil(int32_t y, int32_t m, int32_t d)
{
    int64_t yy = (int64_t)y - (m <= 2 ? 1 : 0);
    int64_t era = (yy >= 0 ? yy : yy - 399) / 400;
    int64_t yoe = yy - era * 400;                              /* 0..399 */
    int64_t mp = (int64_t)(m > 2 ? m - 3 : m + 9);             /* Mar=0 */
    int64_t doy = (153 * mp + 2) / 5 + (int64_t)d - 1;         /* 0..365 */
    int64_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;       /* 0..146096 */
    return era * 146097 + doe - 719468; /* 719468 = days 0000-03-01..epoch */
}

/* Inverse of toy_days_from_civil (Hinnant's civil_from_days). */
static inline void toy_civil_from_days(int64_t z, int32_t *y,
                                       int32_t *m, int32_t *d)
{
    z += 719468;
    int64_t era = (z >= 0 ? z : z - 146096) / 146097;
    int64_t doe = z - era * 146097;                                      /* 0..146096 */
    int64_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365; /* 0..399 */
    int64_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);               /* 0..365 */
    int64_t mp = (5 * doy + 2) / 153;                                    /* Mar=0 */
    int64_t dd = doy - (153 * mp + 2) / 5 + 1;                           /* 1..31 */
    int64_t mm = mp < 10 ? mp + 3 : mp - 9;                              /* 1..12 */
    int64_t yy = yoe + era * 400 + (mm <= 2 ? 1 : 0);
    *y = (int32_t)yy;
    *m = (int32_t)mm;
    *d = (int32_t)dd;
}

/* Floor split of epoch seconds into whole days + second of
 * day (0..86399). C division truncates toward zero, so
 * pre-epoch instants need the correction (solclock's
 * sc_day_split applies the same floor semantics). */
static inline int64_t toy_epoch_day_split(int64_t s, int32_t *sec_of_day)
{
    int64_t q = s / 86400;
    int64_t r = s % 86400;
    if (r < 0) {
        q -= 1;
        r += 86400;
    }
    *sec_of_day = (int32_t)r;
    return q;
}

/* Civil fields -> epoch seconds (the fields are read as UTC;
 * callers apply tz outside, exactly like toy_clock.h's
 * toy_clock_local_seconds). Pre: toy_civil_valid(c). */
static inline int64_t toy_civil_to_epoch(const toy_civil_t *c)
{
    return toy_days_from_civil(c->year, c->month, c->day) * 86400 +
           (int64_t)c->hour * 3600 + (int64_t)c->minute * 60 +
           (int64_t)c->second;
}

static inline void toy_epoch_to_civil(int64_t s, toy_civil_t *c)
{
    int32_t sod = 0;
    int64_t days = toy_epoch_day_split(s, &sod);
    toy_civil_from_days(days, &c->year, &c->month, &c->day);
    c->hour = sod / 3600;
    c->minute = (sod % 3600) / 60;
    c->second = sod % 60;
}

/* 0 = Sunday .. 6 = Saturday (epoch day 0 was Thursday = 4). */
static inline int32_t toy_civil_weekday(int32_t y, int32_t m, int32_t d)
{
    int64_t w = (toy_days_from_civil(y, m, d) + 4) % 7;
    if (w < 0)
        w += 7;
    return (int32_t)w;
}

/* CLOCK_COMMIT combiner (toy_shell.h contract): the shell
 * supplies edited local hh:mm + tz; the runner supplies its
 * own date. base_local_s is the runner's current LOCAL epoch
 * seconds (its calendar date and seconds-of-minute are kept -
 * the emu tour's runner does exactly this in shell_do_actions,
 * emu/main_emu.c); the result is the UTC epoch seconds for
 * that same local calendar date at hh:mm:(kept seconds) under
 * the NEW tz, ready for toy_clock_set(). Keeping the local
 * date (not the UTC date) is the point: editing the offset
 * must not move the day the owner sees on the glass. */
static inline int64_t toy_clock_commit_utc(int64_t base_local_s,
                                           int32_t hh, int32_t mm,
                                           int32_t tz_offset_min)
{
    int32_t sod = 0;
    int64_t days = toy_epoch_day_split(base_local_s, &sod);
    int64_t local_s = days * 86400 + (int64_t)hh * 3600 +
                      (int64_t)mm * 60 + (int64_t)(sod % 60);
    return local_s - (int64_t)tz_offset_min * 60;
}

/* --- RTC register image (UTC convention, busfix round) --------
 *
 * The exact conversions board/clock_rtc.c set()/now() use
 * between the toy_clock epoch and the PCF85063 register
 * fields, factored purely so the host test can pin
 * set(utc,tz) -> registers -> now() == utc without a chip.
 * Registers hold UTC, so neither direction takes tz at all:
 * that absence is the fix. (Under the old local convention
 * each direction applied tz once and consistently, so the
 * pair round-tripped too - the live failure needed tz to
 * CHANGE between boots, which is what the RAM-byte defect
 * below supplied. UTC registers make UTC immune to that.)
 */
static inline void toy_rtc_civil_from_utc(int64_t utc_seconds,
                                          toy_civil_t *reg_out)
{
    toy_epoch_to_civil(utc_seconds, reg_out);
}

static inline int64_t toy_rtc_utc_from_civil(const toy_civil_t *reg)
{
    return toy_civil_to_epoch(reg);
}

/* --- tz offset <-> PCF85063 RAM byte ---------------------------
 *
 * The PCF85063A has one free RAM byte (register 0x03,
 * RAM_byte; datasheet Rev 7 register map: "at 03h is a free
 * RAM byte"), battery-backed with the time registers. One
 * byte cannot hold an arbitrary minute offset, so it stores
 * the shell's own editing domain: multiples of 15 minutes in
 * [-720, +840] (toy_shell.c clamps clock_tz_offset_min to
 * exactly that range in 15-minute steps) = 105 values.
 * Offsets outside the domain (an odd-minute tz set over
 * some future host path) keep working in RAM for the
 * session but do not persist; the encode returns -1 and
 * clock_rtc leaves the byte alone.
 *
 * Validity sentinel (busfix round): the first encoding was
 * a bare index, 1+(tz+720)/15 in 1..105 with 0 = never
 * stored. After a power loss the live board booted with
 * tz=0 tz_src=rtc-ram although no set had ever stored it -
 * byte 49, indistinguishable from a stored UTC+0. A true
 * magic+complement needs two bytes and this chip has
 * exactly one free byte (the only neighbour candidate,
 * the 0x02 Offset register, detunes the oscillator if
 * written with user data), so the sentinel is the
 * strongest one-byte equivalent: valid bytes are exactly
 * TOY_TZ_RAM_MAGIC | index, index 0..104 in the low 7
 * bits, i.e. 0x80..0xE8. That set excludes every plausible
 * unstored content at once: the reset value 0x00, erased
 * 0xFF, the whole legacy bare-index range 1..105
 * (including the observed impostor 49), and BCD/ASCII-
 * looking time-register values. One byte with 105 values
 * necessarily has 105 valid codes, so this cannot reject
 * arbitrary corruption - it rejects the values an
 * unstored byte actually takes, and clock_rtc falls back
 * to TOY_TZ_DEFAULT_MIN on any other byte. Note the first
 * boot after this change: a legacy-encoded byte is (by
 * design) rejected and tz boots as the -420 default until
 * the next CLOCK SET commit stores the new format.
 */
#define TOY_TZ_RAM_MIN (-720)
#define TOY_TZ_RAM_MAX 840
#define TOY_TZ_RAM_STEP 15
#define TOY_TZ_DEFAULT_MIN (-420) /* owner's UTC-7, until set */
#define TOY_TZ_RAM_MAGIC 0x80u    /* bit7: "a tz was stored here" */
#define TOY_TZ_RAM_COUNT 105u     /* (840-(-720))/15 + 1 */

static inline int toy_tz_ram_encode(int32_t tz_offset_min)
{
    if (tz_offset_min < TOY_TZ_RAM_MIN || tz_offset_min > TOY_TZ_RAM_MAX)
        return -1;
    if (((tz_offset_min - TOY_TZ_RAM_MIN) % TOY_TZ_RAM_STEP) != 0)
        return -1;
    return (int)(TOY_TZ_RAM_MAGIC +
                 (unsigned)(tz_offset_min - TOY_TZ_RAM_MIN) /
                     TOY_TZ_RAM_STEP);
}

/* Stored byte -> tz. Returns false unless the byte carries
 * the magic bit AND an in-domain index (see sentinel note):
 * the caller falls back to TOY_TZ_DEFAULT_MIN, never invents
 * an offset from noise. */
static inline bool toy_tz_ram_decode(uint8_t byte, int32_t *tz_out)
{
    if ((byte & TOY_TZ_RAM_MAGIC) == 0u)
        return false;
    unsigned idx = (unsigned)(byte & (uint8_t)~TOY_TZ_RAM_MAGIC);
    if (idx >= TOY_TZ_RAM_COUNT)
        return false;
    *tz_out = TOY_TZ_RAM_MIN + (int32_t)idx * TOY_TZ_RAM_STEP;
    return true;
}

#endif
