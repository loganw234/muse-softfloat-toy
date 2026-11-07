/* touch_cst9217.c - CST9217 I2C/GPIO side. See header. */
#include "touch_cst9217.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"

#define CST_TIMEOUT_US 10000u

static int cst_write(const cst9217_config_t *cfg, const uint8_t *src, size_t len)
{
    int n = i2c_write_timeout_us((i2c_inst_t *)cfg->i2c, CST9217_I2C_ADDR,
                                 src, len, false, CST_TIMEOUT_US);
    return (n == (int)len) ? 0 : -2;
}

static int cst_write_cmd(const cst9217_config_t *cfg, uint16_t reg)
{
    uint8_t b[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFFu) };
    return cst_write(cfg, b, sizeof b);
}

static int cst_read_at(const cst9217_config_t *cfg, uint16_t reg,
                       uint8_t *dst, size_t len)
{
    uint8_t b[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFFu) };
    i2c_inst_t *i2c = (i2c_inst_t *)cfg->i2c;
    int n = i2c_write_timeout_us(i2c, CST9217_I2C_ADDR, b, sizeof b,
                                 true, CST_TIMEOUT_US);
    if (n != (int)sizeof b)
        return -2;
    n = i2c_read_timeout_us(i2c, CST9217_I2C_ADDR, dst, len,
                            false, CST_TIMEOUT_US);
    return (n == (int)len) ? 0 : -2;
}

static uint16_t cst_u16le(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t cst_u32le(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

int cst9217_reset(const cst9217_config_t *cfg)
{
    if (!cfg || !cfg->i2c)
        return -1;
    gpio_init(cfg->int_gpio);
    gpio_set_dir(cfg->int_gpio, GPIO_IN);
    gpio_init(cfg->rst_gpio);
    gpio_set_dir(cfg->rst_gpio, GPIO_OUT);
    gpio_put(cfg->rst_gpio, 0);
    sleep_ms(10);
    gpio_put(cfg->rst_gpio, 1);
    sleep_ms(100);
    return 0;
}

int cst9217_read_info(const cst9217_config_t *cfg, cst9217_info_t *info)
{
    if (!cfg || !cfg->i2c || !info)
        return -1;
    uint8_t buf[8];
    int rc = cst_write_cmd(cfg, CST9217_REG_DEBUG_MODE);
    if (rc)
        return rc;
    sleep_ms(10);
    rc = cst_read_at(cfg, CST9217_REG_CHECK_CODE, buf, 4);
    if (rc)
        return rc;
    info->check_code = cst_u32le(buf);
    rc = cst_read_at(cfg, CST9217_REG_RESOLUTION, buf, 4);
    if (rc)
        return rc;
    info->resolution_x = cst_u16le(buf + 0);
    info->resolution_y = cst_u16le(buf + 2);
    rc = cst_read_at(cfg, CST9217_REG_CHIP_TYPE, buf, 4);
    if (rc)
        return rc;
    info->project_id = cst_u16le(buf + 0);
    info->chip_type = cst_u16le(buf + 2);
    rc = cst_read_at(cfg, CST9217_REG_FW_VERSION, buf, 8);
    if (rc)
        return rc;
    info->fw_version = cst_u32le(buf + 0);
    info->checksum = cst_u32le(buf + 4);
    if (info->chip_type != CST9217_CHIP_ID_EXPECT &&
        info->chip_type != CST9220_CHIP_ID_ALT)
        return -3;
    return 0;
}

int cst9217_enter_normal(const cst9217_config_t *cfg)
{
    if (!cfg || !cfg->i2c)
        return -1;
    int rc = cst_write_cmd(cfg, CST9217_REG_NORMAL_MODE);
    if (rc)
        return rc;
    sleep_ms(10);
    rc = cst_write_cmd(cfg, CST9217_REG_DIS_LP_SCAN);
    if (rc)
        return rc;
    sleep_ms(10);
    return 0;
}

int cst9217_read_report(const cst9217_config_t *cfg,
                        cst9217_report_t *out,
                        uint8_t raw_out[CST9217_REPORT_LEN])
{
    if (!cfg || !cfg->i2c || !out)
        return -1;
    uint8_t buf[CST9217_REPORT_LEN];
    int rc = cst_read_at(cfg, CST9217_REG_READ, buf, sizeof buf);
    if (rc)
        return rc;
    if (raw_out) {
        for (size_t i = 0; i < sizeof buf; i++)
            raw_out[i] = buf[i];
    }
    bool any = false;
    for (size_t i = 0; i < sizeof buf; i++)
        if (buf[i] != 0u)
            any = true;
    if (any) {
        uint8_t ack[3] = { 0xD0u, 0x00u, CST9217_ACK };
        rc = cst_write(cfg, ack, sizeof ack);
        if (rc)
            return rc;
    }
    cst9217_parse_report(buf, sizeof buf, out);
    return 0;
}

bool cst9217_int_level(const cst9217_config_t *cfg)
{
    if (!cfg)
        return false;
    return gpio_get(cfg->int_gpio);
}
