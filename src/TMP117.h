#ifndef TMP117_H_
#define TMP117_H_

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <stdint.h>

/* TMP117 register map */
#define TMP117_REG_TEMP    0x00
#define TMP117_REG_CONFIG  0x01
#define TMP117_REG_ID      0x0F

/* TMP117 addresses */
#define TMP117_ADDR_48     0x48
#define TMP117_ADDR_49     0x49

int tmp117_init(const struct device *i2c_dev, uint8_t addr);
int tmp117_read_c(const struct device *i2c_dev, uint8_t addr, int *c_x100);

#endif /* TMP117_H_ */
