/* clock_rtc.h - product binding for the PCF85063 runner
 * clock (board/clock_rtc.c). toy_product only; toy_board
 * keeps the clock_board.c stub. See clock_rtc.c. */
#ifndef TOY_CLOCK_RTC_H
#define TOY_CLOCK_RTC_H

#include <stdbool.h>
#include <stdint.h>

#include "rtc_pcf85063.h" /* toy_i2c_t */

/* Bind the shared I2C bus and run pcf85063_init (running,
 * 24 h, CAP_SEL preserved, time and OS flag untouched).
 * Returns the driver's code (0 ok). Until a successful bind
 * every toy_clock_now() is the first-class unset snapshot
 * and toy_clock_set() refuses - a dead RTC degrades the
 * clock face, never the toy. */
int toy_clock_rtc_bind(toy_i2c_t *i2c);

/* Current tz offset in force (minutes) and where it came
 * from, for the boot serial line: TOY_CLOCK_TZ_RAM_BYTE (the
 * PCF85063 RAM byte held a stored offset) or
 * TOY_CLOCK_TZ_DEFAULT (nothing stored/readable: compiled
 * default). After a toy_clock_set() with an off-domain tz
 * the source reports DEFAULT-shaped provenance honestly as
 * TOY_CLOCK_TZ_SESSION: live now, gone at power loss. */
enum {
    TOY_CLOCK_TZ_DEFAULT = 0,
    TOY_CLOCK_TZ_RAM_BYTE = 1,
    TOY_CLOCK_TZ_SESSION = 2
};
int toy_clock_rtc_tz_source(void);

/* The RTC's raw LOCAL epoch seconds ignoring the OS flag
 * (fields must still parse as a real date), or false. The
 * CLOCK_COMMIT path uses this as its date source when
 * toy_clock_now() is invalid solely because OS is set: the
 * calendar fields can still be the owner's real date even
 * when the chip will not vouch for the time of day. */
bool toy_clock_rtc_local_epoch_raw(int64_t *local_s_out);

#endif
