#include "TMP117.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tmp117, LOG_LEVEL_INF);

/* Write a 16-bit value */
static int tmp117_write16(const struct device *i2c, uint8_t addr, uint8_t reg, uint16_t val)
{
    uint8_t buf[3] = { reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    return i2c_write(i2c, buf, sizeof(buf), addr);
}

/* Read a 16-bit value */
static int tmp117_read16(const struct device *i2c, uint8_t addr, uint8_t reg, uint16_t *val)
{
    uint8_t buf[2];
    int ret = i2c_write_read(i2c, addr, &reg, 1, buf, 2);
    if (ret) return ret;
    *val = ((uint16_t)buf[0] << 8) | buf[1];
    return 0;
}

/* Initialize TMP117: verify ID and set config */
int tmp117_init(const struct device *i2c, uint8_t addr)
{
    uint16_t id;
    int ret = tmp117_read16(i2c, addr, TMP117_REG_ID, &id);
    if (ret) return ret;

    if (id != 0x0117) {
        LOG_ERR("TMP117 at 0x%02X not found (ID=0x%04X)", addr, id);
        return -ENODEV;
    }
    LOG_INF("TMP117 detected at 0x%02X", addr);

    /* Example config: continuous conversion, average=8, alert=DRDY */
    ret = tmp117_write16(i2c, addr, TMP117_REG_CONFIG, 0x023C);
    if (ret) {
        LOG_ERR("Config write failed (0x%02X)", addr);
        return ret;
    }
    return 0;
}

/* Read temperature in °C*100 (integer) */
int tmp117_read_c(const struct device *i2c, uint8_t addr, int *c_x100)
{
    uint16_t raw;
    int ret = tmp117_read16(i2c, addr, TMP117_REG_TEMP, &raw);
    if (ret) return ret;

    int16_t sraw = (int16_t)raw;
    /* TMP117 LSB = 7.8125 m°C = 0.0078125 °C */
    int32_t temp_cx100 = (sraw * 78125) / 1000;  /* scaled to °C*100 */
    *c_x100 = (int)temp_cx100;
    return 0;
}
