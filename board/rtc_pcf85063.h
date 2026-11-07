/* rtc_pcf85063.h - NXP PCF85063A RTC driver (toy5-sensors).
 *
 * Board: Waveshare RP2350-Touch-AMOLED-1.75, schematic U4
 * PCF85063ATL + Y1 32.768 kHz crystal, RTC_INT -> GPIO27
 * (unused by this driver), on the shared GPIO6/7 I2C1 bus.
 * Fixed 7-bit address 1010001b = 0x51 (PCF85063A datasheet
 * Rev 7, section 9.5.1; no address pins).
 *
 * Time/date registers 0x04..0x0A are BCD (datasheet Table 5):
 *   0x04 Seconds  bit7 OS, bits6:0 BCD 00-59
 *   0x05 Minutes  bits6:0 BCD 00-59
 *   0x06 Hours    bits5:0 BCD 00-23 (24 h mode)
 *   0x07 Days     bits5:0 BCD 01-31
 *   0x08 Weekdays bits2:0 0-6 (free-running, user-defined)
 *   0x09 Months   bits4:0 BCD 01-12
 *   0x0A Years    BCD 00-99 (this driver reports 2000+yy)
 * The OS (oscillator stop) flag means "clock integrity is
 * not guaranteed" - set after power-up until cleared, and
 * whenever the oscillator stopped (datasheet 8.3.1.1). The
 * product posture (core/toy_clock.h) is that unset civil
 * time is first-class: this driver therefore surfaces the
 * OS flag as validity, never invents a time, and init does
 * NOT set the clock. Time is entered manually later (no
 * NTP); pcf85063_set_time() exists for that path and for
 * the test firmware's serial SET command.
 *
 * Init posture: ensure running (STOP=0), normal mode
 * (EXT_TEST=0), 24 h mode (12_24=0). CAP_SEL (7 pF vs
 * 12.5 pF internal load) depends on the fitted crystal's
 * CL, which the schematic does not state (it shows 22 pF
 * external caps C22/C23); init preserves the existing
 * CAP_SEL bit and reports it via pcf85063_read_control1()
 * rather than guessing. Init never issues the software
 * reset (Control_1 = 0x58), which would destroy a valid
 * battery-backed time.
 *
 * Host-compilable like the sibling drivers: BCD + register
 * pack/unpack inline, I2C side in rtc_pcf85063.c. Returns:
 * 0 ok, -1 bad arg/range, -2 I2C transfer failed.
 */
#ifndef TOY_RTC_PCF85063_H
#define TOY_RTC_PCF85063_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Opaque Pico SDK bus handle (struct i2c_inst in the SDK). */
typedef struct i2c_inst toy_i2c_t;

#define PCF85063_I2C_ADDR 0x51u

#define PCF85063_REG_CONTROL1 0x00u
#define PCF85063_REG_SECONDS  0x04u /* first of 7 time regs */
#define PCF85063_TIME_REG_COUNT 7u

/* Control_1 bits (datasheet Table 6). */
#define PCF85063_CTRL1_EXT_TEST (1u << 7)
#define PCF85063_CTRL1_STOP     (1u << 5)
#define PCF85063_CTRL1_SR       (1u << 4)
#define PCF85063_CTRL1_CIE      (1u << 2)
#define PCF85063_CTRL1_12_24    (1u << 1) /* 1 = 12 h mode */
#define PCF85063_CTRL1_CAP_SEL  (1u << 0) /* 1 = 12.5 pF */
#define PCF85063_SOFTWARE_RESET 0x58u     /* full-register value */

#define PCF85063_SECONDS_OS (1u << 7)

typedef struct {
    uint16_t year;    /* 2000..2099 as decoded; raw BCD kept via encode */
    uint8_t month;    /* 1..12 */
    uint8_t day;      /* 1..31 */
    uint8_t weekday;  /* 0..6, free-running user convention */
    uint8_t hour;     /* 0..23 (24 h mode) */
    uint8_t minute;   /* 0..59 */
    uint8_t second;   /* 0..59 */
    bool os_flag;     /* oscillator-stop: time not trustworthy */
    bool valid;       /* BCD + ranges decoded cleanly (OS not judged here) */
} pcf85063_time_t;

/* --- pure helpers (host-tested) ----------------------------------- */

/* BCD byte -> 0..99, or -1 if either nibble exceeds 9. */
static inline int pcf85063_bcd_decode(uint8_t v)
{
    unsigned hi = (unsigned)(v >> 4);
    unsigned lo = (unsigned)(v & 0x0Fu);
    if (hi > 9u || lo > 9u)
        return -1;
    return (int)(hi * 10u + lo);
}

/* 0..99 -> BCD byte. Caller guarantees the range. */
static inline uint8_t pcf85063_bin_to_bcd(unsigned v)
{
    return (uint8_t)(((v / 10u) << 4) | (v % 10u));
}

/* Unpack the 7 time registers (starting at Seconds). Masks
 * strip the OS/unused bits per the register table before
 * BCD decode; any bad nibble or out-of-range field leaves
 * valid=false with the raw-ish fields zeroed except os_flag. */
static inline void pcf85063_parse_time_regs(
    const uint8_t regs[PCF85063_TIME_REG_COUNT], pcf85063_time_t *out)
{
    out->year = 0;
    out->month = 0;
    out->day = 0;
    out->weekday = 0;
    out->hour = 0;
    out->minute = 0;
    out->second = 0;
    out->os_flag = ((regs[0] & PCF85063_SECONDS_OS) != 0u);
    out->valid = false;
    int sec = pcf85063_bcd_decode((uint8_t)(regs[0] & 0x7Fu));
    int min = pcf85063_bcd_decode((uint8_t)(regs[1] & 0x7Fu));
    int hr = pcf85063_bcd_decode((uint8_t)(regs[2] & 0x3Fu));
    int day = pcf85063_bcd_decode((uint8_t)(regs[3] & 0x3Fu));
    int wday = (int)(regs[4] & 0x07u); /* binary, not BCD */
    int mon = pcf85063_bcd_decode((uint8_t)(regs[5] & 0x1Fu));
    int yr = pcf85063_bcd_decode(regs[6]);
    if (sec < 0 || sec > 59 || min < 0 || min > 59 ||
        hr < 0 || hr > 23 || day < 1 || day > 31 ||
        mon < 1 || mon > 12 || yr < 0)
        return;
    out->second = (uint8_t)sec;
    out->minute = (uint8_t)min;
    out->hour = (uint8_t)hr;
    out->day = (uint8_t)day;
    out->weekday = (uint8_t)wday;
    out->month = (uint8_t)mon;
    out->year = (uint16_t)(2000 + yr);
    out->valid = true;
}

/* Pack into the 7 time registers. Seconds is written with
 * OS cleared (writing the time is the act that re-establishes
 * trust; the flag then re-sets only on a real stop).
 * Range checks only - month-length/leap cross-checks are
 * the chip's rollover business, documented as such.
 * Returns 0 ok, -1 on any out-of-range field. */
static inline int pcf85063_encode_time_regs(
    const pcf85063_time_t *t, uint8_t regs[PCF85063_TIME_REG_COUNT])
{
    if (t->second > 59u || t->minute > 59u || t->hour > 23u ||
        t->day < 1u || t->day > 31u || t->weekday > 6u ||
        t->month < 1u || t->month > 12u ||
        t->year < 2000u || t->year > 2099u)
        return -1;
    regs[0] = pcf85063_bin_to_bcd(t->second); /* OS = 0 */
    regs[1] = pcf85063_bin_to_bcd(t->minute);
    regs[2] = pcf85063_bin_to_bcd(t->hour);
    regs[3] = pcf85063_bin_to_bcd(t->day);
    regs[4] = (uint8_t)(t->weekday & 0x07u);
    regs[5] = pcf85063_bin_to_bcd(t->month);
    regs[6] = pcf85063_bin_to_bcd((unsigned)(t->year - 2000u));
    return 0;
}

/* --- I2C side (rtc_pcf85063.c, Pico SDK) -------------------------- */

int pcf85063_read_control1(toy_i2c_t *i2c, uint8_t *out);

/* Running + 24 h + normal mode, CAP_SEL preserved. Does not
 * touch the time registers or the OS flag. */
int pcf85063_init(toy_i2c_t *i2c);

int pcf85063_read_time(toy_i2c_t *i2c, pcf85063_time_t *out);

/* STOP, write all 7 registers in one burst (address pointer
 * auto-increments), restart. Clears OS via the encode. */
int pcf85063_set_time(toy_i2c_t *i2c, const pcf85063_time_t *t);

/* Clear only the OS flag, preserving the seconds value.
 * Useful after the caller has otherwise validated the time;
 * if the oscillator is truly stopped the flag will not
 * stay cleared (datasheet 8.3.1.1), which is the point. */
int pcf85063_clear_os(toy_i2c_t *i2c);

#endif
