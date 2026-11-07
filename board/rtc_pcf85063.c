/* rtc_pcf85063.c - PCF85063A I2C side. See header. */
#include "rtc_pcf85063.h"
#include "hardware/i2c.h"

#define RTC_TIMEOUT_US 10000u

static int rtc_write(i2c_inst_t *i2c, const uint8_t *src, size_t len)
{
    int n = i2c_write_timeout_us(i2c, PCF85063_I2C_ADDR, src, len,
                                 false, RTC_TIMEOUT_US);
    return (n == (int)len) ? 0 : -2;
}

static int rtc_read_at(i2c_inst_t *i2c, uint8_t reg, uint8_t *dst, size_t len)
{
    int n = i2c_write_timeout_us(i2c, PCF85063_I2C_ADDR, &reg, 1,
                                 true, RTC_TIMEOUT_US);
    if (n != 1)
        return -2;
    n = i2c_read_timeout_us(i2c, PCF85063_I2C_ADDR, dst, len,
                            false, RTC_TIMEOUT_US);
    return (n == (int)len) ? 0 : -2;
}

int pcf85063_read_control1(toy_i2c_t *bus, uint8_t *out)
{
    if (!bus || !out)
        return -1;
    return rtc_read_at((i2c_inst_t *)bus, PCF85063_REG_CONTROL1, out, 1);
}

int pcf85063_init(toy_i2c_t *bus)
{
    if (!bus)
        return -1;
    i2c_inst_t *i2c = (i2c_inst_t *)bus;
    uint8_t ctrl = 0;
    int rc = rtc_read_at(i2c, PCF85063_REG_CONTROL1, &ctrl, 1);
    if (rc)
        return rc;
    uint8_t want = (uint8_t)(ctrl & PCF85063_CTRL1_CAP_SEL);
    /* EXT_TEST=0, STOP=0 (run), SR=0, CIE=0, 12_24=0 (24 h). */
    if (want != ctrl) {
        uint8_t buf[2] = { PCF85063_REG_CONTROL1, want };
        rc = rtc_write(i2c, buf, sizeof buf);
        if (rc)
            return rc;
    }
    return 0;
}

int pcf85063_read_time(toy_i2c_t *bus, pcf85063_time_t *out)
{
    if (!bus || !out)
        return -1;
    uint8_t regs[PCF85063_TIME_REG_COUNT];
    int rc = rtc_read_at((i2c_inst_t *)bus, PCF85063_REG_SECONDS,
                         regs, sizeof regs);
    if (rc)
        return rc;
    pcf85063_parse_time_regs(regs, out);
    return 0;
}

int pcf85063_set_time(toy_i2c_t *bus, const pcf85063_time_t *t)
{
    if (!bus || !t)
        return -1;
    i2c_inst_t *i2c = (i2c_inst_t *)bus;
    uint8_t regs[PCF85063_TIME_REG_COUNT];
    if (pcf85063_encode_time_regs(t, regs) != 0)
        return -1;
    uint8_t ctrl = 0;
    int rc = rtc_read_at(i2c, PCF85063_REG_CONTROL1, &ctrl, 1);
    if (rc)
        return rc;
    uint8_t caps = (uint8_t)(ctrl & PCF85063_CTRL1_CAP_SEL);
    uint8_t stop[2] = { PCF85063_REG_CONTROL1,
                        (uint8_t)(caps | PCF85063_CTRL1_STOP) };
    rc = rtc_write(i2c, stop, sizeof stop);
    if (rc)
        return rc;
    uint8_t buf[1 + PCF85063_TIME_REG_COUNT];
    buf[0] = PCF85063_REG_SECONDS;
    for (size_t i = 0; i < PCF85063_TIME_REG_COUNT; i++)
        buf[1 + i] = regs[i];
    rc = rtc_write(i2c, buf, sizeof buf);
    if (rc)
        return rc;
    uint8_t run[2] = { PCF85063_REG_CONTROL1, caps };
    return rtc_write(i2c, run, sizeof run);
}

int pcf85063_clear_os(toy_i2c_t *bus)
{
    if (!bus)
        return -1;
    i2c_inst_t *i2c = (i2c_inst_t *)bus;
    uint8_t sec = 0;
    int rc = rtc_read_at(i2c, PCF85063_REG_SECONDS, &sec, 1);
    if (rc)
        return rc;
    uint8_t buf[2] = { PCF85063_REG_SECONDS,
                       (uint8_t)(sec & (uint8_t)~PCF85063_SECONDS_OS) };
    return rtc_write(i2c, buf, sizeof buf);
}
