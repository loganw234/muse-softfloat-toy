/* sensor_drivers_test.c - host test for the pure halves of
 * the toy5-sensors board drivers (board/imu_qmi8658.h,
 * board/touch_cst9217.h, board/rtc_pcf85063.h): QMI8658
 * burst parse + count->milli conversion, CST9217 report
 * parsing from crafted byte arrays, PCF85063 BCD +
 * time-register pack/unpack. The I2C halves need the Pico
 * SDK and are proven by the RP2350 build (+ on-board run).
 *
 * Deliberately NOT part of tools/run.sh (mirroring the other
 * standalone test files under tools). From the repo root:
 *   gcc -O2 -std=c11 -Wall -Wextra -Werror -Iboard \
 *       tools/sensor_drivers_test.c -o build/sensor_drivers_test
 *   ./build/sensor_drivers_test
 * Exit status 0 iff every check passes.
 */
#include <stdio.h>
#include <string.h>
#include "imu_qmi8658.h"
#include "touch_cst9217.h"
#include "rtc_pcf85063.h"

static int failures;
static int checks;

#define CHECK(cond) do { \
    checks++; \
    if (!(cond)) { \
        failures++; \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

/* --- QMI8658 conversions ------------------------------------------ */

static void test_qmi_convert(void)
{
    /* accel: 16384 LSB/g */
    CHECK(qmi8658_accel_to_millig(0) == 0);
    CHECK(qmi8658_accel_to_millig(16384) == 1000);
    CHECK(qmi8658_accel_to_millig(-16384) == -1000);
    CHECK(qmi8658_accel_to_millig(32767) == 2000);   /* 1999.94 -> 2000 */
    CHECK(qmi8658_accel_to_millig(-32768) == -2000); /* exact */
    CHECK(qmi8658_accel_to_millig(1) == 0);          /* 0.061 mg */
    CHECK(qmi8658_accel_to_millig(-1) == 0);
    CHECK(qmi8658_accel_to_millig(8) == 0);          /* 0.488 mg */
    CHECK(qmi8658_accel_to_millig(9) == 1);          /* 0.549 mg */
    CHECK(qmi8658_accel_to_millig(-9) == -1);
    CHECK(qmi8658_accel_to_millig(8192) == 500);     /* exact half-g */

    /* gyro: 128 LSB/dps; exact .5 ties at counts = 8 mod 16 */
    CHECK(qmi8658_gyro_to_millidps(0) == 0);
    CHECK(qmi8658_gyro_to_millidps(128) == 1000);
    CHECK(qmi8658_gyro_to_millidps(-128) == -1000);
    CHECK(qmi8658_gyro_to_millidps(8) == 63);    /* 62.5, tie away from 0 */
    CHECK(qmi8658_gyro_to_millidps(-8) == -63);  /* -62.5, tie away from 0 */
    CHECK(qmi8658_gyro_to_millidps(24) == 188);  /* 187.5 -> 188 */
    CHECK(qmi8658_gyro_to_millidps(1) == 8);     /* 7.8125 -> 8 */
    CHECK(qmi8658_gyro_to_millidps(32767) == 255992); /* just under 256 dps */

    /* temperature: 256 LSB/degC */
    CHECK(qmi8658_temp_to_millidegc(0) == 0);
    CHECK(qmi8658_temp_to_millidegc(256) == 1000);
    CHECK(qmi8658_temp_to_millidegc(-256) == -1000);
    CHECK(qmi8658_temp_to_millidegc(6400) == 25000); /* 25.000 C */
}

static void test_qmi_burst(void)
{
    /* TEMP=0x1900 (6400), AX=1, AY=-1, AZ=16384 (1 g),
     * GX=-32768, GY=32767, GZ=0; little-endian L,H pairs. */
    static const uint8_t buf[QMI8658_BURST_LEN] = {
        0x00, 0x19, 0x01, 0x00, 0xFF, 0xFF, 0x00, 0x40,
        0x00, 0x80, 0xFF, 0x7F, 0x00, 0x00
    };
    qmi8658_raw_t r;
    qmi8658_milli_t m;
    qmi8658_parse_burst(buf, &r);
    CHECK(r.temp_raw == 6400);
    CHECK(r.ax == 1 && r.ay == -1 && r.az == 16384);
    CHECK(r.gx == -32768 && r.gy == 32767 && r.gz == 0);
    qmi8658_raw_to_milli(&r, &m);
    CHECK(m.az_mg == 1000);
    CHECK(m.temp_mdegc == 25000);
    CHECK(m.gx_mdps == -256000);
}

/* --- CST9217 report parsing --------------------------------------- */

static void put_point(uint8_t *buf, size_t base, unsigned id,
                      unsigned event, unsigned x, unsigned y)
{
    buf[base + 0] = (uint8_t)((id << 4) | (event & 0x0Fu));
    buf[base + 1] = (uint8_t)(x >> 4);
    buf[base + 2] = (uint8_t)(y >> 4);
    buf[base + 3] = (uint8_t)(((x & 0x0Fu) << 4) | (y & 0x0Fu));
}

static void test_cst_parse(void)
{
    uint8_t buf[CST9217_REPORT_LEN];
    cst9217_report_t rep;

    /* no-touch: all-zero buffer is "no new report" */
    memset(buf, 0, sizeof buf);
    cst9217_parse_report(buf, sizeof buf, &rep);
    CHECK(rep.empty && rep.num_points == 0);

    /* idle marker in byte 0 */
    memset(buf, 0, sizeof buf);
    buf[0] = CST9217_ACK;
    cst9217_parse_report(buf, sizeof buf, &rep);
    CHECK(rep.empty && rep.num_points == 0);

    /* single touch, panel centre-ish */
    memset(buf, 0, sizeof buf);
    put_point(buf, 0, 0, CST9217_EVENT_CONTACT, 233, 120);
    buf[5] = 1;
    buf[6] = CST9217_ACK;
    cst9217_parse_report(buf, sizeof buf, &rep);
    CHECK(!rep.empty && rep.valid && rep.num_points == 1);
    CHECK(rep.points[0].track_id == 0 && rep.points[0].pressed);
    CHECK(rep.points[0].x == 233 && rep.points[0].y == 120);
    CHECK(cst9217_report_in_panel(&rep));

    /* coordinate extremes 0 and 465 */
    memset(buf, 0, sizeof buf);
    put_point(buf, 0, 0, CST9217_EVENT_CONTACT, 0, 0);
    buf[5] = 1;
    buf[6] = CST9217_ACK;
    cst9217_parse_report(buf, sizeof buf, &rep);
    CHECK(rep.valid && rep.points[0].x == 0 && rep.points[0].y == 0);
    memset(buf, 0, sizeof buf);
    put_point(buf, 0, 0, CST9217_EVENT_CONTACT, 465, 465);
    buf[5] = 1;
    buf[6] = CST9217_ACK;
    cst9217_parse_report(buf, sizeof buf, &rep);
    CHECK(rep.valid && rep.points[0].x == 465 && rep.points[0].y == 465);
    CHECK(cst9217_report_in_panel(&rep));

    /* just outside the panel: parses, flagged out-of-panel */
    memset(buf, 0, sizeof buf);
    put_point(buf, 0, 0, CST9217_EVENT_CONTACT, 466, 10);
    buf[5] = 1;
    buf[6] = CST9217_ACK;
    cst9217_parse_report(buf, sizeof buf, &rep);
    CHECK(rep.valid && rep.points[0].x == 466);
    CHECK(!cst9217_report_in_panel(&rep));

    /* two-finger report: second point at byte 7 */
    memset(buf, 0, sizeof buf);
    put_point(buf, 0, 0, CST9217_EVENT_CONTACT, 10, 20);
    put_point(buf, 7, 1, CST9217_EVENT_CONTACT, 400, 300);
    buf[5] = 2;
    buf[6] = CST9217_ACK;
    cst9217_parse_report(buf, sizeof buf, &rep);
    CHECK(rep.valid && rep.num_points == 2);
    CHECK(rep.points[0].track_id == 0 && rep.points[0].x == 10 &&
          rep.points[0].y == 20);
    CHECK(rep.points[1].track_id == 1 && rep.points[1].x == 400 &&
          rep.points[1].y == 300);

    /* bad validity echo: torn report rejected */
    buf[6] = 0x00;
    cst9217_parse_report(buf, sizeof buf, &rep);
    CHECK(!rep.empty && !rep.valid && rep.num_points == 0);
    buf[6] = CST9217_ACK;

    /* count 0 and count 3 rejected */
    buf[5] = 0;
    cst9217_parse_report(buf, sizeof buf, &rep);
    CHECK(!rep.valid && rep.num_points == 0);
    buf[5] = 3;
    cst9217_parse_report(buf, sizeof buf, &rep);
    CHECK(!rep.valid && rep.num_points == 0);
    buf[5] = 2;

    /* cover/gesture guard: byte4 high nibble + bit7 set */
    buf[4] = 0x80;
    cst9217_parse_report(buf, sizeof buf, &rep);
    CHECK(!rep.valid && rep.num_points == 0);
    buf[4] = 0x00;

    /* first-slot rule: slot0 not a contact, slot1 is -> all dropped */
    memset(buf, 0, sizeof buf);
    put_point(buf, 0, 0, 0x03 /* not contact */, 50, 50);
    put_point(buf, 7, 1, CST9217_EVENT_CONTACT, 60, 60);
    buf[5] = 2;
    buf[6] = CST9217_ACK;
    cst9217_parse_report(buf, sizeof buf, &rep);
    CHECK(!rep.valid && rep.num_points == 0);

    /* truncated buffer cannot hold the claimed second point */
    memset(buf, 0, sizeof buf);
    put_point(buf, 0, 0, CST9217_EVENT_CONTACT, 5, 5);
    buf[5] = 2;
    buf[6] = CST9217_ACK;
    cst9217_parse_report(buf, 10, &rep); /* needs 11 for 2 points */
    CHECK(!rep.valid && rep.num_points == 0);

    /* too-short buffer is empty, never a crash or a phantom point */
    cst9217_parse_report(buf, 3, &rep);
    CHECK(rep.empty && rep.num_points == 0);
}

/* --- PCF85063 BCD + time registers -------------------------------- */

static void test_rtc_bcd(void)
{
    CHECK(pcf85063_bcd_decode(0x00) == 0);
    CHECK(pcf85063_bcd_decode(0x09) == 9);
    CHECK(pcf85063_bcd_decode(0x10) == 10);
    CHECK(pcf85063_bcd_decode(0x59) == 59);
    CHECK(pcf85063_bcd_decode(0x99) == 99);
    CHECK(pcf85063_bcd_decode(0x0A) == -1); /* bad low nibble */
    CHECK(pcf85063_bcd_decode(0xA0) == -1); /* bad high nibble */
    CHECK(pcf85063_bcd_decode(0xFF) == -1);
    for (unsigned v = 0; v < 100u; v++)
        CHECK(pcf85063_bcd_decode(pcf85063_bin_to_bcd(v)) == (int)v);
}

static void test_rtc_regs(void)
{
    /* 2026-11-07 (wday 5) 23:59:30, OS set on seconds. */
    static const uint8_t regs[PCF85063_TIME_REG_COUNT] = {
        0x80u | 0x30u, 0x59, 0x23, 0x07, 0x05, 0x11, 0x26
    };
    pcf85063_time_t t;
    pcf85063_parse_time_regs(regs, &t);
    CHECK(t.valid && t.os_flag);
    CHECK(t.year == 2026 && t.month == 11 && t.day == 7);
    CHECK(t.weekday == 5 && t.hour == 23 && t.minute == 59 && t.second == 30);

    /* unused high bits are masked, not decoded */
    uint8_t dirty[PCF85063_TIME_REG_COUNT];
    memcpy(dirty, regs, sizeof dirty);
    dirty[0] = 0x30;          /* OS clear this time */
    dirty[1] |= 0x80u;        /* minutes bit7 unused */
    dirty[2] |= 0xC0u;        /* hours bits7:6 unused */
    dirty[5] |= 0xE0u;        /* months bits7:5 unused */
    pcf85063_parse_time_regs(dirty, &t);
    CHECK(t.valid && !t.os_flag && t.minute == 59 && t.hour == 23 &&
          t.month == 11);

    /* out-of-range / bad-BCD fields invalidate, never clamp */
    memcpy(dirty, regs, sizeof dirty);
    dirty[5] = 0x13; /* month 13 */
    pcf85063_parse_time_regs(dirty, &t);
    CHECK(!t.valid);
    memcpy(dirty, regs, sizeof dirty);
    dirty[0] = 0x60; /* second 60 */
    pcf85063_parse_time_regs(dirty, &t);
    CHECK(!t.valid);
    memcpy(dirty, regs, sizeof dirty);
    dirty[3] = 0x00; /* day 0 */
    pcf85063_parse_time_regs(dirty, &t);
    CHECK(!t.valid);
    memcpy(dirty, regs, sizeof dirty);
    dirty[6] = 0x2A; /* bad BCD year */
    pcf85063_parse_time_regs(dirty, &t);
    CHECK(!t.valid && t.os_flag); /* OS still surfaced */

    /* encode -> parse round-trip; encode clears OS */
    memset(&t, 0, sizeof t);
    t.year = 2000; t.month = 1; t.day = 1; t.weekday = 6;
    t.hour = 0; t.minute = 0; t.second = 0;
    uint8_t out[PCF85063_TIME_REG_COUNT];
    CHECK(pcf85063_encode_time_regs(&t, out) == 0);
    CHECK((out[0] & PCF85063_SECONDS_OS) == 0u);
    pcf85063_time_t back;
    pcf85063_parse_time_regs(out, &back);
    CHECK(back.valid && back.year == 2000 && back.weekday == 6);

    t.year = 2099; t.month = 12; t.day = 31; t.hour = 23;
    t.minute = 59; t.second = 59;
    CHECK(pcf85063_encode_time_regs(&t, out) == 0);
    pcf85063_parse_time_regs(out, &back);
    CHECK(back.valid && back.year == 2099 && back.second == 59);

    /* encode range gate */
    t.hour = 24;
    CHECK(pcf85063_encode_time_regs(&t, out) == -1);
    t.hour = 23; t.month = 13;
    CHECK(pcf85063_encode_time_regs(&t, out) == -1);
    t.month = 12; t.year = 1999;
    CHECK(pcf85063_encode_time_regs(&t, out) == -1);
    t.year = 2100;
    CHECK(pcf85063_encode_time_regs(&t, out) == -1);
    t.year = 2099; t.weekday = 7;
    CHECK(pcf85063_encode_time_regs(&t, out) == -1);
}

int main(void)
{
    test_qmi_convert();
    test_qmi_burst();
    test_cst_parse();
    test_rtc_bcd();
    test_rtc_regs();
    if (failures == 0)
        printf("sensor_drivers_test: PASS (%d checks)\n", checks);
    else
        printf("sensor_drivers_test: FAIL (%d/%d checks failed)\n",
               failures, checks);
    return failures ? 1 : 0;
}
