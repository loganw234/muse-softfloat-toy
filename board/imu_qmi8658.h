/* imu_qmi8658.h - QMI8658C 6-axis IMU driver (toy5-sensors).
 *
 * Board: Waveshare RP2350-Touch-AMOLED-1.75. Shared I2C1
 * (SDA GPIO6 / SCL GPIO7, 400 kHz). Schematic U3 is annotated
 * "0X6B": the QMI8658C 7-bit address is 0x6A with SA0 low /
 * unconnected and 0x6B with SA0 pulled up (QMI8658C Datasheet
 * Rev B, I2C interface section; Waveshare schematic
 * RP2350-Touch-AMOLED-1.75.pdf). INT1/INT2 land on GPIO28/29;
 * this driver is poll-based and does not use them yet.
 *
 * Fixed configuration (see board/SENSORS.md for the full
 * reasoning and citations):
 *   accel +/-2 g   at 16384 LSB/g   (CTRL2 aFS=000)
 *   gyro  +/-256 dps at 128 LSB/dps (CTRL3 gFS=100)
 *   ODR 125 Hz Normal, both         (aODR/gODR=0110)
 * The signal seam (core/toy_signal.h) and the four IMU carts
 * (gpend/seismo/attitude/heartbeat, all IMU6 at 100 Hz) were
 * pinned against exactly 16384 LSB/g and 128 LSB/(deg/s), so
 * these ranges make live raw counts drop-in compatible with
 * the synthetic proof streams. There is no 100 Hz Normal ODR
 * on this part (Normal table: ... 250, 125, 62.5 ...; the
 * 128 Hz entry is Low Power, accel-only), so 125 Hz is the
 * nearest documented rate at/above 100 Hz; the actual rate is
 * exported as QMI8658_ODR_HZ and any future live runner must
 * deal with 125 -> 100 explicitly, never silently.
 *
 * LSB weights at this configuration:
 *   accel 1 LSB = 1000/16384 mg  ~= 0.0610 mg
 *   gyro  1 LSB = 1000/128 mdps  = 7.8125 mdps
 * Datasheet noise density is 200 ug/sqrt(Hz) accel /
 * 15 mdps/sqrt(Hz) gyro (Rev B electro-mechanical tables);
 * SENSORS.md works the RMS numbers against heartbeat's
 * modelled signal. Conversions below are integer-only with
 * round-to-nearest, ties away from zero (same convention as
 * toy_px_to_milli in core/toy_input.h). No native float.
 *
 * This header is host-compilable: the pure helpers (burst
 * parse, count->milli) are static inline and unit-tested by
 * tools/sensor_drivers_test.c. The I2C functions are declared
 * here against an opaque bus handle and defined in
 * imu_qmi8658.c (Pico SDK). Return convention: 0 = ok,
 * negative = error (-1 bad arg, -2 I2C transfer failed,
 * -3 identity/config mismatch).
 */
#ifndef TOY_IMU_QMI8658_H
#define TOY_IMU_QMI8658_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Opaque Pico SDK bus handle (struct i2c_inst in the SDK). */
typedef struct i2c_inst toy_i2c_t;

#define QMI8658_I2C_ADDR        0x6Bu /* SA0 pulled up (schematic) */
#define QMI8658_WHO_AM_I_EXPECT 0x05u /* WHO_AM_I (0x00), Rev B Table 21 */
#define QMI8658_REVISION_EXPECT 0x7Cu /* REVISION_ID (0x01), Rev B; reported, not fatal */

/* Register addresses (QMI8658C Datasheet Rev B, UI register map). */
#define QMI8658_REG_WHO_AM_I   0x00u
#define QMI8658_REG_REVISION   0x01u
#define QMI8658_REG_CTRL1      0x02u
#define QMI8658_REG_CTRL2      0x03u /* accel FS + ODR */
#define QMI8658_REG_CTRL3      0x04u /* gyro FS + ODR */
#define QMI8658_REG_CTRL5      0x06u /* LPF enables/modes */
#define QMI8658_REG_CTRL7      0x08u /* sensor enables */
#define QMI8658_REG_STATUSINT  0x2Du
#define QMI8658_REG_STATUS0    0x2Eu /* bit0 aDA, bit1 gDA */
#define QMI8658_REG_TEMP_L     0x33u
#define QMI8658_REG_AX_L       0x35u
#define QMI8658_REG_RESET      0x60u /* write 0xB0: soft reset */
#define QMI8658_RESET_VALUE    0xB0u
#define QMI8658_REG_RESET_DONE 0x4Du /* reads 0x80 after successful reset */
#define QMI8658_RESET_DONE_VAL 0x80u

/* Configuration values written by qmi8658_init (see SENSORS.md).
 * CTRL1: ADDR_AI=1 (auto-increment for burst reads), BE=0
 * (little-endian data; power-on default BE=1 must be cleared),
 * oscillator enabled. CTRL2: aFS=000 (+/-2 g), aODR=0110
 * (125 Hz). CTRL3: gFS=100 (+/-256 dps), gODR=0110 (125 Hz).
 * CTRL5: both on-chip LPFs disabled - carts filter in their
 * own integer code and seismo wants the raw spectrum.
 * CTRL7: aEN|gEN, AttitudeEngine off, syncSmpl off. */
#define QMI8658_CTRL1_VALUE 0x40u
#define QMI8658_CTRL2_VALUE 0x06u
#define QMI8658_CTRL3_VALUE 0x46u
#define QMI8658_CTRL5_VALUE 0x00u
#define QMI8658_CTRL7_VALUE 0x03u

#define QMI8658_ODR_HZ          125u   /* actual configured rate */
#define QMI8658_ACCEL_LSB_PER_G 16384  /* +/-2 g sensitivity */
#define QMI8658_GYRO_LSB_PER_DPS 128   /* +/-256 dps sensitivity */
#define QMI8658_TEMP_LSB_PER_DEGC 256  /* temperature sensitivity */

/* One burst sample: signed raw counts, IMU6 order ax..gz to
 * match TOY_SIG_DEVICE_IMU6 (core/toy_signal.h), plus the
 * temperature counts that precede AX_L in the register map. */
typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp_raw; /* 256 LSB/degC, two's complement */
} qmi8658_raw_t;

/* Same sample in milli-units (mg, mdps, milli-degC). */
typedef struct {
    int32_t ax_mg, ay_mg, az_mg;
    int32_t gx_mdps, gy_mdps, gz_mdps;
    int32_t temp_mdegc;
} qmi8658_milli_t;

/* --- pure helpers (host-tested) ---------------------------------- */

/* Round-to-nearest, ties away from zero: counts*1000/scale. */
static inline int32_t qmi8658_scale_milli(int32_t counts, int32_t lsb_per_unit)
{
    int64_t num = (int64_t)counts * 1000;
    int64_t den = (int64_t)lsb_per_unit;
    int64_t q = num / den;
    int64_t r = num % den;
    int64_t ar = r < 0 ? -r : r;
    if (2 * ar >= den)
        q += (num < 0 ? -1 : 1);
    return (int32_t)q;
}

static inline int32_t qmi8658_accel_to_millig(int16_t counts)
{
    return qmi8658_scale_milli(counts, QMI8658_ACCEL_LSB_PER_G);
}

static inline int32_t qmi8658_gyro_to_millidps(int16_t counts)
{
    return qmi8658_scale_milli(counts, QMI8658_GYRO_LSB_PER_DPS);
}

static inline int32_t qmi8658_temp_to_millidegc(int16_t counts)
{
    return qmi8658_scale_milli(counts, QMI8658_TEMP_LSB_PER_DEGC);
}

/* Little-endian int16 with fully defined sign handling. */
static inline int16_t qmi8658_i16le(const uint8_t *p)
{
    uint16_t u = (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
    int32_t s = (u <= 0x7FFFu) ? (int32_t)u : (int32_t)u - 0x10000;
    return (int16_t)s;
}

/* Parse a 14-byte burst read starting at TEMP_L (0x33):
 * TEMP, AX, AY, AZ, GX, GY, GZ, each L then H. Requires
 * CTRL1 ADDR_AI=1 and BE=0 (qmi8658_init sets both). */
#define QMI8658_BURST_LEN 14u
static inline void qmi8658_parse_burst(const uint8_t buf[QMI8658_BURST_LEN],
                                       qmi8658_raw_t *out)
{
    out->temp_raw = qmi8658_i16le(buf + 0);
    out->ax = qmi8658_i16le(buf + 2);
    out->ay = qmi8658_i16le(buf + 4);
    out->az = qmi8658_i16le(buf + 6);
    out->gx = qmi8658_i16le(buf + 8);
    out->gy = qmi8658_i16le(buf + 10);
    out->gz = qmi8658_i16le(buf + 12);
}

static inline void qmi8658_raw_to_milli(const qmi8658_raw_t *raw,
                                        qmi8658_milli_t *out)
{
    out->ax_mg = qmi8658_accel_to_millig(raw->ax);
    out->ay_mg = qmi8658_accel_to_millig(raw->ay);
    out->az_mg = qmi8658_accel_to_millig(raw->az);
    out->gx_mdps = qmi8658_gyro_to_millidps(raw->gx);
    out->gy_mdps = qmi8658_gyro_to_millidps(raw->gy);
    out->gz_mdps = qmi8658_gyro_to_millidps(raw->gz);
    out->temp_mdegc = qmi8658_temp_to_millidegc(raw->temp_raw);
}

/* --- I2C side (imu_qmi8658.c, Pico SDK) --------------------------- */

/* Read WHO_AM_I / REVISION_ID. Either out may be NULL. */
int qmi8658_probe(toy_i2c_t *i2c, uint8_t *who_out, uint8_t *rev_out);

/* Soft reset, then apply the fixed configuration above and
 * verify CTRL2/CTRL3/CTRL7 by readback. Blocks ~100 ms for
 * the reset (datasheet system turn-on is far longer from
 * cold power; by firmware time the part has long powered). */
int qmi8658_init(toy_i2c_t *i2c);

/* STATUS0 new-data bits: bit0 accel, bit1 gyro. *ready gets
 * (aDA && gDA). */
int qmi8658_data_ready(toy_i2c_t *i2c, bool *ready_out);

int qmi8658_read_raw(toy_i2c_t *i2c, qmi8658_raw_t *out);
int qmi8658_read_milli(toy_i2c_t *i2c, qmi8658_milli_t *out,
                       qmi8658_raw_t *raw_out_or_null);

#endif
