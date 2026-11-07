/* touch_cst9217.h - Hynitron CST9217 capacitive touch driver
 * (toy5-sensors). Poll-based; no IRQ infrastructure yet.
 *
 * Board: Waveshare RP2350-Touch-AMOLED-1.75. The controller
 * rides the display FPC (schematic J3: TP_SDA/TP_SCL onto the
 * shared GPIO6/7 I2C1 bus, TP_INT -> GPIO22, TP_RESET ->
 * GPIO23). Fixed 7-bit I2C address 0x5A. Panel is 466x466,
 * so valid coordinates are 0..465 on each axis.
 *
 * Protocol sources: no public CST9217 datasheet PDF was
 * reachable from the Waveshare resources page (it links
 * QMI8658C/PCF85063A/ES8311/AXP2101/CO5300 only), so the
 * register facts below come from the Hynitron-family
 * reference flow as preserved in two independent open
 * drivers consulted for hardware facts only (SensorLib
 * TouchDrvCST92xx, shipped in Waveshare BSPs, and the Rust
 * cst92xx crate which mirrors it and cites CST9217 datasheet
 * section 10.5 for reset timing). All code here is original.
 * Every constant is cited inline; board/SENSORS.md carries
 * the full table. Anything the first on-board run
 * contradicts gets fixed against the raw-report dump the
 * test firmware prints - that is what bring-up is for.
 *
 * Registers are 16-bit, sent big-endian; data payloads are
 * little-endian. Command registers:
 *   0xD000  touch report (read; write back 0xAB to ack)
 *   0xD101  enter debug/info (command) mode for attributes
 *   0xD106  disable low-power scan (else reports starve
 *           ~10 s after boot - fatal to a 60 s poll test)
 *   0xD109  normal scanning mode
 *   0xD1F8  resolution: u16le res_x, u16le res_y
 *   0xD1FC  firmware checkcode: u32le
 *   0xD204  u16le project_id, u16le chip_type (0x9217 here)
 *   0xD208  u32le fw_version, u32le checksum
 *
 * Touch report layout (15-byte read at 0xD000, up to 2
 * points; see cst9217_parse_report for the decode rules):
 *   [0]     point0: track_id<<4 | event (0x06 = contact)
 *   [1]     point0 x[11:4]
 *   [2]     point0 y[11:4]
 *   [3]     point0 x[3:0]<<4 | y[3:0]
 *   [4]     gesture/cover byte (see parser)
 *   [5]     point count in low 7 bits
 *   [6]     validity echo, must be 0xAB
 *   [7..10] point1, same 4-byte packing as [0..3]
 *   [11..14] unused tail of the fixed-size read
 * An all-zero buffer means "no new report" and must NOT be
 * acked. Any other buffer is acked by writing the 3 bytes
 * {0xD0, 0x00, 0xAB} before decoding.
 *
 * Press semantics: the reference decode accepts only event
 * nibble 0x06 as an active contact; there is no separate
 * up-code in the accepted set, so releases are observed as
 * a track_id disappearing. DOWN/MOVE/UP derivation from
 * successive reports lives in the test firmware (and later
 * in the live runner), not in this driver.
 *
 * Host-compilable like imu_qmi8658.h: pure parser inline,
 * I2C/GPIO side in touch_cst9217.c. Returns: 0 ok, -1 bad
 * arg, -2 I2C transfer failed, -3 unexpected identity.
 */
#ifndef TOY_TOUCH_CST9217_H
#define TOY_TOUCH_CST9217_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Opaque Pico SDK bus handle (struct i2c_inst in the SDK). */
typedef struct i2c_inst toy_i2c_t;

#define CST9217_I2C_ADDR 0x5Au
#define CST9217_CHIP_ID_EXPECT 0x9217u
#define CST9220_CHIP_ID_ALT 0x9220u /* sibling part, same decode */

#define CST9217_REG_READ        0xD000u
#define CST9217_REG_DEBUG_MODE  0xD101u
#define CST9217_REG_DIS_LP_SCAN 0xD106u
#define CST9217_REG_NORMAL_MODE 0xD109u
#define CST9217_REG_RESOLUTION  0xD1F8u
#define CST9217_REG_CHECK_CODE  0xD1FCu
#define CST9217_REG_CHIP_TYPE   0xD204u
#define CST9217_REG_FW_VERSION  0xD208u

#define CST9217_ACK 0xABu
#define CST9217_EVENT_CONTACT 0x06u
#define CST9217_MAX_POINTS 2u
#define CST9217_REPORT_LEN 15u /* 2 points * 5 + 5 */
#define CST9217_COORD_MAX 465u /* 466x466 panel: 0..465 */

typedef struct {
    toy_i2c_t *i2c;
    unsigned rst_gpio; /* GPIO23 on this board */
    unsigned int_gpio; /* GPIO22 on this board */
} cst9217_config_t;

typedef struct {
    uint32_t check_code;
    uint16_t resolution_x;
    uint16_t resolution_y;
    uint16_t project_id;
    uint16_t chip_type; /* expect 0x9217 */
    uint32_t fw_version;
    uint32_t checksum;
} cst9217_info_t;

typedef struct {
    uint8_t track_id;
    uint8_t event; /* raw low nibble; 0x06 = contact */
    uint16_t x;    /* 0..465 panel space (12-bit raw field) */
    uint16_t y;
    bool pressed;  /* event == CST9217_EVENT_CONTACT */
} cst9217_point_t;

typedef struct {
    bool empty;    /* no new report (all-zero / idle marker) */
    bool valid;    /* passed the 0xAB + count checks */
    uint8_t num_points; /* accepted points, 0..2 */
    cst9217_point_t points[CST9217_MAX_POINTS];
} cst9217_report_t;

/* --- pure parser (host-tested) ------------------------------------ */

/* Decode one report buffer. len must be >= 11 to hold two
 * points; shorter buffers decode point0 only if the count
 * says 1, else invalid. Rules, in order:
 *  - all-zero, or buf[0] in {0x00, 0xAB}: empty.
 *  - buf[6] != 0xAB: invalid (torn/stale report).
 *  - cover/gesture guard: (buf[4] & 0xF0) != 0 with bit7 set
 *    means a cover-screen gesture frame, not points: invalid.
 *  - count = buf[5] & 0x7F must be 1..2.
 *  - per point: id = b>>4, event = b&0x0F; only event 0x06
 *    with id < 2 is an accepted contact; coordinates unpack
 *    as x = b1<<4 | b3>>4, y = b2<<4 | (b3&0x0F).
 *  - if slot 0 yields no accepted contact the whole report
 *    is discarded (reference-driver rule: a report whose
 *    first slot is not a live contact is not a touch frame).
 */
static inline void cst9217_parse_report(const uint8_t *buf, size_t len,
                                        cst9217_report_t *out)
{
    out->empty = false;
    out->valid = false;
    out->num_points = 0;
    for (size_t i = 0; i < CST9217_MAX_POINTS; i++) {
        out->points[i].track_id = 0;
        out->points[i].event = 0;
        out->points[i].x = 0;
        out->points[i].y = 0;
        out->points[i].pressed = false;
    }
    if (!buf || len < 7u) {
        out->empty = true;
        return;
    }
    bool any = false;
    for (size_t i = 0; i < len; i++)
        if (buf[i] != 0u)
            any = true;
    if (!any || buf[0] == CST9217_ACK || buf[0] == 0x00u) {
        out->empty = true;
        return;
    }
    if (buf[6] != CST9217_ACK)
        return; /* invalid */
    if ((buf[4] & 0xF0u) != 0u && (buf[4] >> 7) == 0x01u)
        return; /* cover/gesture frame, not points */
    unsigned count = (unsigned)(buf[5] & 0x7Fu);
    if (count == 0u || count > CST9217_MAX_POINTS)
        return; /* invalid */
    out->valid = true;
    bool slot0_accepted = false;
    for (unsigned i = 0; i < count; i++) {
        size_t base = (size_t)i * 5u + (i == 0u ? 0u : 2u);
        if (base + 4u > len) {
            out->valid = false;
            out->num_points = 0;
            return;
        }
        const uint8_t *p = buf + base;
        unsigned id = (unsigned)(p[0] >> 4);
        unsigned ev = (unsigned)(p[0] & 0x0Fu);
        if (ev == CST9217_EVENT_CONTACT && id < CST9217_MAX_POINTS) {
            cst9217_point_t *pt = &out->points[out->num_points];
            pt->track_id = (uint8_t)id;
            pt->event = (uint8_t)ev;
            pt->x = (uint16_t)(((uint16_t)p[1] << 4) | (uint16_t)(p[3] >> 4));
            pt->y = (uint16_t)(((uint16_t)p[2] << 4) | (uint16_t)(p[3] & 0x0Fu));
            pt->pressed = true;
            out->num_points++;
            if (i == 0u)
                slot0_accepted = true;
        }
    }
    if (!slot0_accepted) {
        /* First-slot rule (see header): by slot, not by
         * acceptance order - a lone slot-1 contact with a
         * dead slot 0 is discarded whole. */
        out->valid = false;
        out->num_points = 0;
    }
}

/* True if every accepted point sits inside the 466 panel. */
static inline bool cst9217_report_in_panel(const cst9217_report_t *rep)
{
    for (unsigned i = 0; i < rep->num_points; i++) {
        if (rep->points[i].x > CST9217_COORD_MAX ||
            rep->points[i].y > CST9217_COORD_MAX)
            return false;
    }
    return true;
}

/* --- I2C/GPIO side (touch_cst9217.c, Pico SDK) -------------------- */

/* Configure INT as input (no pull change beyond SDK input
 * default; board wiring owns the level) and RST as output,
 * then pulse RST low 10 ms / settle 100 ms (datasheet
 * TRST ~0.1 ms, TRON ~100 ms via the cst92xx citation). */
int cst9217_reset(const cst9217_config_t *cfg);

/* Enter info mode and read identity attributes. Returns -3
 * if chip_type is neither 0x9217 nor 0x9220 (info still
 * filled for printing). Leaves the part in info mode;
 * call cst9217_enter_normal() before polling reports. */
int cst9217_read_info(const cst9217_config_t *cfg, cst9217_info_t *info);

/* Normal scanning + disable low-power scan. Best-effort
 * direct command writes; see SENSORS.md for the caveat. */
int cst9217_enter_normal(const cst9217_config_t *cfg);

/* Read + ack (when non-empty) + decode one report. When
 * raw_out is non-NULL it receives the untouched bytes for
 * bring-up dumps. */
int cst9217_read_report(const cst9217_config_t *cfg,
                        cst9217_report_t *out,
                        uint8_t raw_out[CST9217_REPORT_LEN]);

/* Current INT pin level (true = high). Polled diagnostics
 * only; no IRQ is registered by this driver. */
bool cst9217_int_level(const cst9217_config_t *cfg);

#endif
