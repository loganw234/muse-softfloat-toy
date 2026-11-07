/* clock_rtc.c - product implementation of toy_clock.h over
 * the PCF85063A RTC (toy5-shell). Linked into toy_product
 * ONLY (per-target source pick in board/CMakeLists.txt);
 * toy_board keeps the always-unset clock_board.c stub, whose
 * posture this file replaces for the real kit: civil time is
 * real when the chip vouches for it, unset otherwise, and
 * unset still never blocks anything (core/toy_clock.h).
 *
 * Register convention: the chip holds UTC civil time
 * (busfix round; full story in board/clock_civil.h). now()
 * reads the fields and converts straight to epoch via
 * toy_rtc_utc_from_civil; set() converts epoch -> fields
 * via toy_rtc_civil_from_utc, computes the weekday
 * (0=Sunday) of the UTC date, and bursts the registers
 * (which clears OS: writing the time is the act that
 * re-establishes trust). tz is NOT applied to the
 * registers in either direction - it lives in the RAM
 * byte + cache and is applied once at display by
 * toy_clock_local_seconds (core/toy_clock.h), so a tz
 * misread can mislabel local time but can never move UTC.
 *
 * Validity = transfer ok AND fields parsed as a real date
 * (driver ranges + month-length/leap cross-check here) AND
 * OS flag clear. Any I2C error returns valid=false with the
 * cached tz still reported; nothing here can wedge - every
 * transfer is the driver's timeout-bounded one.
 *
 * tz persistence: the PCF85063A's free RAM byte (register
 * 0x03, battery-backed with the time registers) holds the
 * offset in clock_civil.h's encoding (multiples of 15 min
 * in [-720,840] - the shell clock-set screen's whole domain;
 * magic-bit sentinel so reset/erased/legacy bytes are not
 * a stored tz). It is read once at bind and cached;
 * set() updates cache + byte. An off-domain tz (odd minutes)
 * works for the session but does not persist - the byte is
 * left untouched and the next boot falls back to the stored
 * or default offset. Default when nothing valid is stored:
 * TOY_TZ_DEFAULT_MIN (-420, owner's UTC-7). So unlike the
 * RAM-only alternative, power loss does NOT reset tz for
 * any tz the product UI can produce.
 *
 * mono_ms is the Pico boot clock via time_us_64 (same source
 * main_product's pacing already used); setting civil time
 * never disturbs it. clear() sets the OS flag: the fields
 * survive as an untrusted date source for the next commit,
 * but now() reports unset until the next set().
 */
#define TOY_CLOCK_ALLOW 1
#include "toy_clock.h"

#include "pico/time.h"

#include "clock_civil.h"
#include "clock_rtc.h"
#include "rtc_pcf85063.h"

static toy_i2c_t *rtc_bus; /* NULL until a successful bind */
static int32_t rtc_tz_min = TOY_TZ_DEFAULT_MIN;
static int rtc_tz_src = TOY_CLOCK_TZ_DEFAULT;

int toy_clock_rtc_bind(toy_i2c_t *i2c)
{
    if (!i2c)
        return -1;
    int rc = pcf85063_init(i2c);
    if (rc)
        return rc;
    rtc_bus = i2c;
    uint8_t byte = 0;
    int32_t tz = 0;
    if (pcf85063_read_ram(rtc_bus, &byte) == 0 &&
        toy_tz_ram_decode(byte, &tz)) {
        rtc_tz_min = tz;
        rtc_tz_src = TOY_CLOCK_TZ_RAM_BYTE;
    } else {
        rtc_tz_min = TOY_TZ_DEFAULT_MIN;
        rtc_tz_src = TOY_CLOCK_TZ_DEFAULT;
    }
    return 0;
}

int toy_clock_rtc_tz_source(void)
{
    return rtc_tz_src;
}

/* Bus-recovery re-probe (main_product.c): the bus was just
 * reset; prove the chip answers again. Re-runs
 * pcf85063_init (running, 24 h, CAP_SEL preserved; time/OS
 * untouched) and one time read as the probe transfer.
 * Deliberately does NOT reload the tz byte: the cached tz
 * (possibly a live session offset) stays in force across a
 * bus reset, exactly as the time registers do. */
int toy_clock_rtc_reprobe(void)
{
    if (!rtc_bus)
        return -1;
    int rc = pcf85063_init(rtc_bus);
    if (rc)
        return rc;
    pcf85063_time_t t;
    return pcf85063_read_time(rtc_bus, &t);
}

uint64_t toy_clock_mono_ms(void)
{
    return time_us_64() / 1000ull;
}

/* Read + fully validate the chip fields as a UTC civil
 * instant (registers hold UTC, see header). Judges
 * everything except the OS flag (callers decide what OS
 * means for them); *os_out may be NULL. */
static bool rtc_read_utc(toy_civil_t *out, bool *os_out)
{
    if (!rtc_bus)
        return false;
    pcf85063_time_t t;
    if (pcf85063_read_time(rtc_bus, &t) != 0 || !t.valid)
        return false;
    out->year = t.year;
    out->month = t.month;
    out->day = t.day;
    out->hour = t.hour;
    out->minute = t.minute;
    out->second = t.second;
    if (os_out)
        *os_out = t.os_flag;
    return toy_civil_valid(out);
}

toy_time_t toy_clock_now(void)
{
    toy_time_t t;
    t.utc_seconds = 0;
    t.tz_offset_min = rtc_tz_min;
    t.valid = false;
    toy_civil_t utc;
    bool os = true;
    if (!rtc_read_utc(&utc, &os) || os)
        return t;
    /* Registers hold UTC: no tz arithmetic here. */
    t.utc_seconds = toy_rtc_utc_from_civil(&utc);
    t.valid = true;
    return t;
}

bool toy_clock_rtc_local_epoch_raw(int64_t *local_s_out)
{
    toy_civil_t utc;
    if (!local_s_out || !rtc_read_utc(&utc, NULL))
        return false;
    /* Contract unchanged (LOCAL epoch out, for the commit
     * path's date source): register UTC + cached tz. */
    *local_s_out = toy_rtc_utc_from_civil(&utc) +
                   (int64_t)rtc_tz_min * 60;
    return true;
}

bool toy_clock_set(int64_t utc_seconds, int32_t tz_offset_min)
{
    if (!rtc_bus)
        return false;
    /* Registers hold UTC: fields are the UTC civil image;
     * tz_offset_min only updates the cached/persisted
     * display offset below, never the registers. */
    toy_civil_t utc;
    toy_rtc_civil_from_utc(utc_seconds, &utc);
    /* The chip stores BCD years 2000..2099 only; refuse
     * outside rather than wrap a century into the glass. */
    if (utc.year < 2000 || utc.year > 2099 ||
        !toy_civil_valid(&utc))
        return false;
    pcf85063_time_t t;
    t.year = (uint16_t)utc.year;
    t.month = (uint8_t)utc.month;
    t.day = (uint8_t)utc.day;
    t.weekday = (uint8_t)toy_civil_weekday(utc.year, utc.month,
                                           utc.day);
    t.hour = (uint8_t)utc.hour;
    t.minute = (uint8_t)utc.minute;
    t.second = (uint8_t)utc.second;
    t.os_flag = false;
    t.valid = true;
    if (pcf85063_set_time(rtc_bus, &t) != 0)
        return false;
    rtc_tz_min = tz_offset_min;
    int enc = toy_tz_ram_encode(tz_offset_min);
    if (enc >= 0 && pcf85063_write_ram(rtc_bus, (uint8_t)enc) == 0)
        rtc_tz_src = TOY_CLOCK_TZ_RAM_BYTE;
    else
        rtc_tz_src = TOY_CLOCK_TZ_SESSION; /* live, unpersisted */
    return true;
}

void toy_clock_clear(void)
{
    if (!rtc_bus)
        return;
    /* Mark untrusted, keep the fields (see header). A failed
     * transfer leaves the clock as it was - clear is a UI
     * state, never worth wedging over; now() re-reads the
     * flag either way. */
    (void)pcf85063_set_os(rtc_bus);
}
