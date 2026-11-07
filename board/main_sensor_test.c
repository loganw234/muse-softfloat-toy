/* main_sensor_test.c - standalone I2C sensor bring-up firmware
 * (toy5-sensors parcel). NOT product firmware: no cartridges,
 * no display. It proves the QMI8658C IMU, CST9217 touch and
 * PCF85063A RTC drivers on the Waveshare RP2350-Touch-
 * AMOLED-1.75 over USB CDC while the owner taps/tilts.
 *
 * Bus: I2C1, SDA GPIO6, SCL GPIO7, 400 kHz (shared by all
 * five scanned devices; see board/SENSORS.md for the roster).
 * Pins: touch INT GPIO22 (active-low; level-polled every
 * loop pass and used to gate report reads - no IRQ) / RST
 * GPIO23, IMU INT1/2 GPIO28/29 (unused, poll-based), RTC
 * INT GPIO27 (unused). GPIO25 is
 * never touched (UART1 RX on this board); the status LED
 * follows TOY_BOARD_LED_PIN exactly as main_board.c.
 *
 * Phases (SENSORS.md documents expected output + owner
 * actions): 0 bus scan, 1 IMU id+init, 2 RTC init+read,
 * 3 touch reset+id+verified normal-mode entry, 4 AXP2101
 * read-only ID + codec presence note, 5 bounded 60 s live
 * loop (RTC 1 Hz, IMU ~10 Hz, INT-gated touch reads with
 * DOWN/MOVE/UP as they change, optional serial SET
 * command for the RTC), 6 summary then idle.
 *
 * SET command (type it into the CDC terminal during phase
 * 5):  SET 2026-11-07 12:34:56 5   (last field weekday 0-6)
 * Fields are UTC: the product clock's register convention
 * is UTC in the chip, tz applied at display only
 * (board/clock_civil.h), and SET writes the registers
 * verbatim. It is the only write path to the RTC here.
 * Nothing writes the AXP2101 or ES8311 at all.
 */
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico/unique_id.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "imu_qmi8658.h"
#include "touch_cst9217.h"
#include "rtc_pcf85063.h"

#ifndef TOY_GIT_HEAD
#define TOY_GIT_HEAD "unknown"
#endif
#ifndef TOY_BOARD_LED_PIN
#define TOY_BOARD_LED_PIN PICO_DEFAULT_LED_PIN
#endif
#if TOY_BOARD_LED_PIN >= 0
#define LED_INIT() do { gpio_init(TOY_BOARD_LED_PIN); \
    gpio_set_dir(TOY_BOARD_LED_PIN, GPIO_OUT); } while (0)
#define LED_TOGGLE() gpio_xor_mask(1u << TOY_BOARD_LED_PIN)
#else
#define LED_INIT() ((void)0)
#define LED_TOGGLE() ((void)0)
#endif

#define PIN_I2C_SDA 6u
#define PIN_I2C_SCL 7u
#define PIN_TOUCH_INT 22u
#define PIN_TOUCH_RST 23u
/* Documented, deliberately unused in this poll-based parcel:
 * IMU INT1 GPIO28, INT2 GPIO29; RTC INT GPIO27. */

#define AXP2101_I2C_ADDR 0x34u
#define AXP2101_REG_CHIP_ID 0x03u /* read-only ID, expect 0x4A */

#define LIVE_MS 60000u

static toy_i2c_t *bus(void) { return (toy_i2c_t *)i2c1; }

static bool addr_reserved(uint8_t addr)
{
    return (addr & 0x78u) == 0u || (addr & 0x78u) == 0x78u;
}

static void phase_scan(void)
{
    printf("SENSOR-TEST PHASE 0 bus-scan i2c1 sda=%u scl=%u hz=400000\n",
           PIN_I2C_SDA, PIN_I2C_SCL);
    printf("SCAN found");
    int found = 0;
    for (uint8_t addr = 0; addr < 128u; addr++) {
        if (addr_reserved(addr))
            continue;
        uint8_t b = 0;
        int n = i2c_read_timeout_us(i2c1, addr, &b, 1, false, 5000);
        if (n >= 0) {
            printf(" 0x%02x", addr);
            found++;
        }
    }
    printf(" (%d devices, expect 0x18 0x34 0x51 0x5a 0x6b)\n", found);
}

static int phase_imu(void)
{
    printf("SENSOR-TEST PHASE 1 imu QMI8658C addr=0x%02x\n",
           QMI8658_I2C_ADDR);
    uint8_t who = 0, rev = 0;
    int rc = qmi8658_probe(bus(), &who, &rev);
    printf("IMU who=0x%02x rev=0x%02x probe=%s (expect who=0x%02x rev=0x%02x)\n",
           who, rev, rc == 0 ? "OK" : "FAIL",
           QMI8658_WHO_AM_I_EXPECT, QMI8658_REVISION_EXPECT);
    if (rc)
        return rc;
    rc = qmi8658_init(bus());
    printf("IMU init=%s odr_hz=%u accel_lsb_per_g=%d gyro_lsb_per_dps=%d\n",
           rc == 0 ? "OK" : "FAIL", QMI8658_ODR_HZ,
           QMI8658_ACCEL_LSB_PER_G, QMI8658_GYRO_LSB_PER_DPS);
    return rc;
}

static void print_rtc(const char *tag, const pcf85063_time_t *t)
{
    printf("%s %04u-%02u-%02u %02u:%02u:%02u wday=%u os=%d valid=%d%s\n",
           tag, t->year, t->month, t->day, t->hour, t->minute,
           t->second, t->weekday, t->os_flag ? 1 : 0,
           t->valid ? 1 : 0,
           t->os_flag ? " (os=1: time not trusted until set)" : "");
}

static int phase_rtc(pcf85063_time_t *first)
{
    printf("SENSOR-TEST PHASE 2 rtc PCF85063A addr=0x%02x\n",
           PCF85063_I2C_ADDR);
    uint8_t ctrl = 0;
    int rc = pcf85063_read_control1(bus(), &ctrl);
    printf("RTC ctrl1_before=0x%02x read=%s\n", ctrl,
           rc == 0 ? "OK" : "FAIL");
    if (rc)
        return rc;
    rc = pcf85063_init(bus());
    uint8_t after = 0;
    (void)pcf85063_read_control1(bus(), &after);
    printf("RTC init=%s ctrl1_after=0x%02x cap_sel=%u (preserved)\n",
           rc == 0 ? "OK" : "FAIL", after,
           (unsigned)(after & PCF85063_CTRL1_CAP_SEL));
    if (rc)
        return rc;
    rc = pcf85063_read_time(bus(), first);
    if (rc == 0)
        print_rtc("RTC", first);
    else
        printf("RTC read=FAIL rc=%d\n", rc);
    return rc;
}

static int phase_touch(cst9217_config_t *cfg)
{
    printf("SENSOR-TEST PHASE 3 touch CST9217 addr=0x%02x rst=%u int=%u\n",
           CST9217_I2C_ADDR, PIN_TOUCH_RST, PIN_TOUCH_INT);
    int rc = cst9217_reset(cfg);
    printf("TOUCH reset=%s\n", rc == 0 ? "OK" : "FAIL");
    if (rc)
        return rc;
    cst9217_info_t info;
    memset(&info, 0, sizeof info);
    rc = cst9217_read_info(cfg, &info);
    printf("TOUCH chip=0x%04x project=0x%04x res=%ux%u fw=0x%08lx "
           "csum=0x%08lx check=0x%08lx info=%s (expect chip=0x%04x res=466x466)\n",
           info.chip_type, info.project_id, info.resolution_x,
           info.resolution_y, (unsigned long)info.fw_version,
           (unsigned long)info.checksum, (unsigned long)info.check_code,
           rc == 0 ? "OK" : "FAIL", CST9217_CHIP_ID_EXPECT);
    int rc2 = cst9217_enter_normal(cfg);
    printf("TOUCH normal=%s rc=%d int_level=%d (rc %d=echo-mismatch "
           "%d=nack; int idle=1, asserted=0)\n",
           rc2 == 0 ? "OK" : "FAIL", rc2,
           cst9217_int_level(cfg) ? 1 : 0,
           CST9217_ERR_MODE, CST9217_ERR_NACK);
    return rc ? rc : rc2;
}

static void phase_pmic_codec(void)
{
    printf("SENSOR-TEST PHASE 4 pmic+codec (read-only; nothing written)\n");
    uint8_t reg = AXP2101_REG_CHIP_ID;
    uint8_t id = 0;
    int n = i2c_write_timeout_us(i2c1, AXP2101_I2C_ADDR, &reg, 1,
                                 true, 10000);
    int m = (n == 1) ? i2c_read_timeout_us(i2c1, AXP2101_I2C_ADDR,
                                           &id, 1, false, 10000) : -1;
    if (m == 1)
        printf("AXP2101 id_reg03=0x%02x (expect 0x4a) read-only\n", id);
    else
        printf("AXP2101 id read FAILED (scan still proves presence)\n");
    printf("ES8311 present at 0x18 by scan only; codec registers untouched\n");
}

/* --- touch event derivation --------------------------------------- */

typedef struct {
    bool active;
    uint16_t x, y;      /* last reported position */
    uint16_t px, py;    /* last printed position (MOVE gating) */
} touch_track_t;

static void print_raw(const char *why, const uint8_t raw[CST9217_REPORT_LEN])
{
    printf("TOUCH RAW %s:", why);
    for (size_t i = 0; i < 11u; i++)
        printf(" %02x", raw[i]);
    printf("\n");
}

/* --- serial SET command ------------------------------------------- */

static int parse_uint_at(const char *s, size_t len, unsigned *out)
{
    if (len == 0)
        return -1;
    unsigned v = 0;
    for (size_t i = 0; i < len; i++) {
        if (s[i] < '0' || s[i] > '9')
            return -1;
        v = v * 10u + (unsigned)(s[i] - '0');
    }
    *out = v;
    return 0;
}

/* Fixed-shape line: "SET 2026-11-07 12:34:56 5" (21+ chars). */
static int parse_set_line(const char *line, pcf85063_time_t *t)
{
    if (strncmp(line, "SET ", 4) != 0)
        return -1;
    const char *s = line + 4;
    unsigned y, mo, d, h, mi, se, w;
    if (strlen(s) < 21u)
        return -1;
    if (s[4] != '-' || s[7] != '-' || s[10] != ' ' || s[13] != ':' ||
        s[16] != ':' || s[19] != ' ')
        return -1;
    if (parse_uint_at(s + 0, 4, &y) || parse_uint_at(s + 5, 2, &mo) ||
        parse_uint_at(s + 8, 2, &d) || parse_uint_at(s + 11, 2, &h) ||
        parse_uint_at(s + 14, 2, &mi) || parse_uint_at(s + 17, 2, &se) ||
        parse_uint_at(s + 20, 1, &w))
        return -1;
    memset(t, 0, sizeof *t);
    t->year = (uint16_t)y;
    t->month = (uint8_t)mo;
    t->day = (uint8_t)d;
    t->hour = (uint8_t)h;
    t->minute = (uint8_t)mi;
    t->second = (uint8_t)se;
    t->weekday = (uint8_t)w;
    uint8_t regs[PCF85063_TIME_REG_COUNT];
    return pcf85063_encode_time_regs(t, regs); /* range gate */
}

int main(void)
{
    stdio_init_all();
    LED_INIT();
    for (int i = 0; i < 150 && !stdio_usb_connected(); i++) {
        LED_TOGGLE();
        sleep_ms(100);
    }
    sleep_ms(200);
    char idstr[PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2 + 1];
    pico_get_unique_board_id_string(idstr, sizeof idstr);
    printf("SENSOR-TEST git %s serial %s clk_hz %lu\n",
           TOY_GIT_HEAD, idstr, (unsigned long)clock_get_hz(clk_sys));

    i2c_init(i2c1, 400u * 1000u);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);

    phase_scan();
    int imu_rc = phase_imu();
    pcf85063_time_t first;
    memset(&first, 0, sizeof first);
    int rtc_rc = phase_rtc(&first);
    cst9217_config_t touch_cfg = { bus(), PIN_TOUCH_RST, PIN_TOUCH_INT };
    int touch_rc = phase_touch(&touch_cfg);
    phase_pmic_codec();

    printf("SENSOR-TEST PHASE 5 live %us: tilt/rotate the board, "
           "tap and drag on the glass\n", LIVE_MS / 1000u);
    printf("SENSOR-TEST tip: send 'SET 2026-11-07 12:34:56 5' + Enter "
           "to set the RTC in UTC (weekday 0-6 last)\n");

    touch_track_t tracks[CST9217_MAX_POINTS];
    memset(tracks, 0, sizeof tracks);
    bool dumped_contact = false, dumped_release = false;
    bool dumped_no_report = false;
    unsigned imu_ok = 0, imu_err = 0, rtc_err = 0;
    unsigned t_down = 0, t_up = 0;
    unsigned touch_nack = 0, touch_no_report = 0, touch_int_falls = 0;
    bool touch_pending = false, prev_int_high = true;
    char line[48];
    size_t line_len = 0;

    uint32_t start = to_ms_since_boot(get_absolute_time());
    uint32_t next_imu = start, next_rtc = start, next_touch = start;
    uint32_t next_touch_probe = start; /* 1 Hz ungated fallback */
    uint32_t next_beat = start + 10000u;
    while (to_ms_since_boot(get_absolute_time()) - start < LIVE_MS) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        /* serial SET, non-blocking, one char per pass */
        int ch = getchar_timeout_us(0);
        if (ch >= 0) {
            if (ch == '\n' || ch == '\r') {
                if (line_len > 0) {
                    line[line_len] = '\0';
                    pcf85063_time_t t;
                    if (parse_set_line(line, &t) == 0) {
                        int rc = (rtc_rc == 0) ? pcf85063_set_time(bus(), &t) : -9;
                        printf("RTC SET %s -> %s\n", line,
                               rc == 0 ? "OK" : "FAIL");
                        if (rc == 0 && pcf85063_read_time(bus(), &t) == 0)
                            print_rtc("RTC now", &t);
                    } else {
                        printf("RTC SET rejected (want: SET YYYY-MM-DD HH:MM:SS W)\n");
                    }
                    line_len = 0;
                }
            } else if (line_len < sizeof line - 1u) {
                line[line_len++] = (char)ch;
            }
        }

        if (imu_rc == 0 && now >= next_imu) {
            next_imu = now + 100u; /* ~10 Hz */
            qmi8658_milli_t m;
            qmi8658_raw_t r;
            if (qmi8658_read_milli(bus(), &m, &r) == 0) {
                imu_ok++;
                int32_t tdeg = m.temp_mdegc / 1000;
                int32_t tfrac = m.temp_mdegc % 1000;
                if (tfrac < 0) tfrac = -tfrac;
                printf("IMU t=%lums a_raw=(%d,%d,%d) a_mg=(%ld,%ld,%ld) "
                       "g_mdps=(%ld,%ld,%ld) temp=%ld.%03ldC\n",
                       (unsigned long)(now - start), r.ax, r.ay, r.az,
                       (long)m.ax_mg, (long)m.ay_mg, (long)m.az_mg,
                       (long)m.gx_mdps, (long)m.gy_mdps, (long)m.gz_mdps,
                       (long)tdeg, (long)tfrac);
            } else {
                imu_err++;
            }
        }

        if (rtc_rc == 0 && now >= next_rtc) {
            next_rtc = now + 1000u; /* 1 Hz */
            pcf85063_time_t t;
            if (pcf85063_read_time(bus(), &t) == 0)
                print_rtc("RTC", &t);
            else
                rtc_err++;
        }

        /* INT is active-low, falling edge = report ready
         * (vendor flow). Sample every pass (GPIO only, no
         * I2C) and latch the assertion for the gated read. */
        if (touch_rc == 0) {
            bool lvl_high = cst9217_int_level(&touch_cfg);
            if (cst9217_int_fell(prev_int_high, lvl_high))
                touch_int_falls++;
            if (cst9217_int_is_asserted(lvl_high))
                touch_pending = true;
            prev_int_high = lvl_high;
        }

        bool any_active = tracks[0].active || tracks[1].active;
        if (touch_rc == 0 && now >= next_touch &&
            cst9217_should_read(touch_pending, any_active,
                                now >= next_touch_probe)) {
            next_touch = now + 20u; /* ~50 Hz max read cadence */
            next_touch_probe = now + 1000u;
            touch_pending = false;
            cst9217_report_t rep;
            uint8_t raw[CST9217_REPORT_LEN];
            int trc = cst9217_read_report(&touch_cfg, &rep, raw);
            if (trc == CST9217_NO_REPORT) {
                /* All-FF window: nothing latched. Counted
                 * apart from NACKs; never a release frame. */
                touch_no_report++;
                if (!dumped_no_report) {
                    print_raw("first-no-report-ff", raw);
                    dumped_no_report = true;
                }
            } else if (trc != CST9217_OK) {
                touch_nack++;
            } else if (!rep.empty) {
                bool now_active[CST9217_MAX_POINTS] = { false, false };
                uint16_t nx[CST9217_MAX_POINTS] = { 0, 0 };
                uint16_t ny[CST9217_MAX_POINTS] = { 0, 0 };
                if (rep.valid) {
                    for (unsigned i = 0; i < rep.num_points; i++) {
                        unsigned id = rep.points[i].track_id;
                        if (id < CST9217_MAX_POINTS) {
                            now_active[id] = true;
                            nx[id] = rep.points[i].x;
                            ny[id] = rep.points[i].y;
                        }
                    }
                    if (!dumped_contact) {
                        print_raw("first-contact", raw);
                        dumped_contact = true;
                    }
                } else if (!dumped_release) {
                    /* Non-empty frame with no accepted contact:
                     * believed to be the lift/release frame. */
                    print_raw("first-no-contact", raw);
                    dumped_release = true;
                }
                for (unsigned id = 0; id < CST9217_MAX_POINTS; id++) {
                    touch_track_t *tr = &tracks[id];
                    if (!tr->active && now_active[id]) {
                        tr->active = true;
                        tr->x = nx[id]; tr->y = ny[id];
                        tr->px = nx[id]; tr->py = ny[id];
                        t_down++;
                        printf("TOUCH DOWN id=%u x=%u y=%u int=%d\n",
                               id, tr->x, tr->y,
                               cst9217_int_level(&touch_cfg) ? 1 : 0);
                    } else if (tr->active && now_active[id]) {
                        tr->x = nx[id]; tr->y = ny[id];
                        unsigned dx = tr->x > tr->px ? tr->x - tr->px : tr->px - tr->x;
                        unsigned dy = tr->y > tr->py ? tr->y - tr->py : tr->py - tr->y;
                        if (dx + dy >= 2u) {
                            tr->px = tr->x; tr->py = tr->y;
                            printf("TOUCH MOVE id=%u x=%u y=%u\n",
                                   id, tr->x, tr->y);
                        }
                    } else if (tr->active && !now_active[id]) {
                        tr->active = false;
                        t_up++;
                        printf("TOUCH UP id=%u x=%u y=%u int=%d\n",
                               id, tr->x, tr->y,
                               cst9217_int_level(&touch_cfg) ? 1 : 0);
                    }
                }
            }
        }

        if (now >= next_beat) {
            next_beat = now + 10000u;
            unsigned elapsed = (now - start) / 1000u;
            printf("SENSOR-TEST live %us/%us (keep tapping/tilting)\n",
                   elapsed, LIVE_MS / 1000u);
            LED_TOGGLE();
        }
        sleep_ms(2);
    }

    printf("SENSOR-TEST PHASE 6 done imu_ok=%u imu_err=%u rtc_err=%u "
           "touch_down=%u touch_up=%u touch_nack=%u "
           "touch_no_report_ff=%u touch_int_falls=%u\n",
           imu_ok, imu_err, rtc_err, t_down, t_up, touch_nack,
           touch_no_report, touch_int_falls);
    printf("SENSOR-TEST DONE (idle; reset to run again)\n");
    for (;;) {
        LED_TOGGLE();
        sleep_ms(500);
    }
}
