/* imu_qmi8658.c - QMI8658C I2C side. See imu_qmi8658.h.
 *
 * Every transfer is timeout-bounded so a missing/dead part
 * degrades to an error line, never a hung firmware. Integer
 * math only; no native float anywhere in this file.
 */
#include "imu_qmi8658.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"

#define QMI_TIMEOUT_US 10000u /* per transfer; bus is 400 kHz */

static int qmi_write_reg(i2c_inst_t *i2c, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    int n = i2c_write_timeout_us(i2c, QMI8658_I2C_ADDR, buf,
                                 sizeof buf, false, QMI_TIMEOUT_US);
    return (n == (int)sizeof buf) ? 0 : -2;
}

static int qmi_read_regs(i2c_inst_t *i2c, uint8_t reg, uint8_t *dst, size_t len)
{
    int n = i2c_write_timeout_us(i2c, QMI8658_I2C_ADDR, &reg, 1,
                                 true, QMI_TIMEOUT_US); /* no stop: repeated start */
    if (n != 1)
        return -2;
    n = i2c_read_timeout_us(i2c, QMI8658_I2C_ADDR, dst, len,
                            false, QMI_TIMEOUT_US);
    return (n == (int)len) ? 0 : -2;
}

static int qmi_read_reg(i2c_inst_t *i2c, uint8_t reg, uint8_t *out)
{
    return qmi_read_regs(i2c, reg, out, 1);
}

int qmi8658_probe(toy_i2c_t *bus, uint8_t *who_out, uint8_t *rev_out)
{
    if (!bus)
        return -1;
    i2c_inst_t *i2c = (i2c_inst_t *)bus;
    uint8_t who = 0, rev = 0;
    int rc = qmi_read_reg(i2c, QMI8658_REG_WHO_AM_I, &who);
    if (rc)
        return rc;
    rc = qmi_read_reg(i2c, QMI8658_REG_REVISION, &rev);
    if (rc)
        return rc;
    if (who_out)
        *who_out = who;
    if (rev_out)
        *rev_out = rev;
    return (who == QMI8658_WHO_AM_I_EXPECT) ? 0 : -3;
}

int qmi8658_init(toy_i2c_t *bus)
{
    if (!bus)
        return -1;
    i2c_inst_t *i2c = (i2c_inst_t *)bus;

    /* Soft reset (Rev B 5.9: write 0xB0 to 0x60; register
     * 0x4D then reads 0x80). Poll briefly, then a fixed
     * settle; accel turn-on is 3 ms + 3/ODR, gyro 60 ms +
     * 3/ODR from power-on default, so 100 ms covers both. */
    int rc = qmi_write_reg(i2c, QMI8658_REG_RESET, QMI8658_RESET_VALUE);
    if (rc)
        return rc;
    for (int i = 0; i < 50; i++) {
        uint8_t done = 0;
        if (qmi_read_reg(i2c, QMI8658_REG_RESET_DONE, &done) == 0 &&
            done == QMI8658_RESET_DONE_VAL)
            break;
        sleep_ms(2);
    }
    sleep_ms(100);

    rc = qmi_write_reg(i2c, QMI8658_REG_CTRL1, QMI8658_CTRL1_VALUE);
    if (rc) return rc;
    rc = qmi_write_reg(i2c, QMI8658_REG_CTRL2, QMI8658_CTRL2_VALUE);
    if (rc) return rc;
    rc = qmi_write_reg(i2c, QMI8658_REG_CTRL3, QMI8658_CTRL3_VALUE);
    if (rc) return rc;
    rc = qmi_write_reg(i2c, QMI8658_REG_CTRL5, QMI8658_CTRL5_VALUE);
    if (rc) return rc;
    rc = qmi_write_reg(i2c, QMI8658_REG_CTRL7, QMI8658_CTRL7_VALUE);
    if (rc) return rc;

    /* Readback: config that did not stick is a failed init,
     * not a surprise later. CTRL1/CTRL5 included. */
    static const struct { uint8_t reg, want; } expect[] = {
        { QMI8658_REG_CTRL1, QMI8658_CTRL1_VALUE },
        { QMI8658_REG_CTRL2, QMI8658_CTRL2_VALUE },
        { QMI8658_REG_CTRL3, QMI8658_CTRL3_VALUE },
        { QMI8658_REG_CTRL5, QMI8658_CTRL5_VALUE },
        { QMI8658_REG_CTRL7, QMI8658_CTRL7_VALUE },
    };
    for (size_t i = 0; i < sizeof expect / sizeof expect[0]; i++) {
        uint8_t got = 0;
        rc = qmi_read_reg(i2c, expect[i].reg, &got);
        if (rc)
            return rc;
        if (got != expect[i].want)
            return -3;
    }
    return 0;
}

int qmi8658_data_ready(toy_i2c_t *bus, bool *ready_out)
{
    if (!bus || !ready_out)
        return -1;
    uint8_t st = 0;
    int rc = qmi_read_reg((i2c_inst_t *)bus, QMI8658_REG_STATUS0, &st);
    if (rc)
        return rc;
    *ready_out = ((st & 0x03u) == 0x03u);
    return 0;
}

int qmi8658_read_raw(toy_i2c_t *bus, qmi8658_raw_t *out)
{
    if (!bus || !out)
        return -1;
    uint8_t buf[QMI8658_BURST_LEN];
    int rc = qmi_read_regs((i2c_inst_t *)bus, QMI8658_REG_TEMP_L,
                           buf, sizeof buf);
    if (rc)
        return rc;
    qmi8658_parse_burst(buf, out);
    return 0;
}

int qmi8658_read_milli(toy_i2c_t *bus, qmi8658_milli_t *out,
                       qmi8658_raw_t *raw_out_or_null)
{
    if (!out)
        return -1;
    qmi8658_raw_t raw;
    int rc = qmi8658_read_raw(bus, &raw);
    if (rc)
        return rc;
    qmi8658_raw_to_milli(&raw, out);
    if (raw_out_or_null)
        *raw_out_or_null = raw;
    return 0;
}
