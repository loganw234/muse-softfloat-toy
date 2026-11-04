/* clock_board.c - board implementation of toy_clock.h: STUB.
 *
 * The primary device is the RP2350-Touch-LCD-1.85C, which has
 * an RTC (docs/WAVE-T2.md), but the kit is not in hand yet.
 * Until it is, civil time is permanently UNSET on board: the
 * first-class state that disables schedule features and never
 * blocks simulation (core/toy_clock.h). Monotonic time is real
 * (Pico SDK boot clock) because pacing needs it regardless.
 * When the kit lands this file gains the RTC read and the
 * touch/USB manual-set path; the sim core does not change,
 * because nothing here is visible to cartridges.
 */
#define TOY_CLOCK_ALLOW 1
#include "toy_clock.h"
#include "pico/time.h"

uint64_t toy_clock_mono_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

toy_time_t toy_clock_now(void)
{
    toy_time_t t;
    t.utc_seconds = 0;
    t.tz_offset_min = 0;
    t.valid = false; /* stub: no RTC read until the RP2350 kit */
    return t;
}

bool toy_clock_set(int64_t utc_seconds, int32_t tz_offset_min)
{
    (void)utc_seconds;
    (void)tz_offset_min;
    return false; /* stub: nowhere to keep civil time yet */
}

void toy_clock_clear(void)
{
    /* Already unset. */
}
