/* clock_emu.c - host-emulator implementation of toy_clock.h.
 *
 * Manual-set plumbing (host side): the clock starts UNSET
 * unless the environment sets it - TOY_CLOCK_UTC_SECONDS
 * (int64, seconds since the Unix epoch) with optional
 * TOY_CLOCK_TZ_OFFSET_MIN (minutes, default 0). toy_emu's
 * `clock` mode prints the state; `bank` reads wall labels from
 * here. Once set, civil time advances with the monotonic
 * clock from the set anchor, so a long emu run sees time pass;
 * setting never affects stepping (nothing here is visible to
 * core/ - see the placement rule in toy_clock.h).
 */
#define TOY_CLOCK_ALLOW 1
#include "toy_clock.h"

#include <stdlib.h>

#if defined(_WIN32)
#include <windows.h>
uint64_t toy_clock_mono_ms(void)
{
    return (uint64_t)GetTickCount64();
}
#else
#include <time.h>
uint64_t toy_clock_mono_ms(void)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return 0;
    return (uint64_t)ts.tv_sec * 1000ull + (uint64_t)(ts.tv_nsec / 1000000L);
}
#endif

static int      clock_init_done;
static int      clock_valid;
static int64_t  clock_base_utc_s;
static uint64_t clock_base_mono_ms;
static int32_t  clock_tz_min;

static void clock_init_from_env(void)
{
    if (clock_init_done) return;
    clock_init_done = 1;
    const char *utc = getenv("TOY_CLOCK_UTC_SECONDS");
    if (!utc || !utc[0]) return; /* stays unset: first-class */
    char *end = NULL;
    long long v = strtoll(utc, &end, 10);
    if (end == utc) return;      /* unparseable: stay unset */
    int32_t tz = 0;
    const char *tzs = getenv("TOY_CLOCK_TZ_OFFSET_MIN");
    if (tzs && tzs[0]) tz = (int32_t)strtol(tzs, NULL, 10);
    clock_base_utc_s = (int64_t)v;
    clock_base_mono_ms = toy_clock_mono_ms();
    clock_tz_min = tz;
    clock_valid = 1;
}

toy_time_t toy_clock_now(void)
{
    clock_init_from_env();
    toy_time_t t;
    t.utc_seconds = 0;
    t.tz_offset_min = clock_tz_min;
    t.valid = clock_valid != 0;
    if (clock_valid) {
        uint64_t elapsed_ms = toy_clock_mono_ms() - clock_base_mono_ms;
        t.utc_seconds = clock_base_utc_s + (int64_t)(elapsed_ms / 1000ull);
    }
    return t;
}

bool toy_clock_set(int64_t utc_seconds, int32_t tz_offset_min)
{
    clock_init_done = 1; /* explicit set wins over the env */
    clock_base_utc_s = utc_seconds;
    clock_base_mono_ms = toy_clock_mono_ms();
    clock_tz_min = tz_offset_min;
    clock_valid = 1;
    return true;
}

void toy_clock_clear(void)
{
    clock_init_done = 1;
    clock_valid = 0;
}
