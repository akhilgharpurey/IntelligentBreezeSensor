#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/services/nus.h>
#include <zephyr/sys/printk.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

/* ===== I2C / TMP117 ===== */
#define I2C_NODE              DT_NODELABEL(i2c0)   /* SDA=P0.26, SCL=P0.27 per overlay */

#define TMP117_ADDR_A         0x48                 /* ADD0=GND */
#define TMP117_ADDR_B         0x49                 /* ADD0=VDD */
#define TMP117_REG_TEMP       0x00                 /* signed 16-bit; 1 LSB = 1/128 °C */
#define TMP117_REG_DEVID      0x0F                 /* expect 0x0117 */

static const struct device *i2c_dev;

/* ---- I2C helpers ---- */
static int i2c_wr_rd_u16(uint8_t addr, uint8_t reg, uint16_t *out)
{
    uint8_t b[2];
    int ret = i2c_write_read(i2c_dev, addr, &reg, 1, b, 2);
    if (ret) return ret;
    *out = ((uint16_t)b[0] << 8) | b[1];
    return 0;
}

static int tmp117_read_devid(uint8_t addr, uint16_t *id_out)
{
    return i2c_wr_rd_u16(addr, TMP117_REG_DEVID, id_out);
}

static int tmp117_read_raw(uint8_t addr, int16_t *raw_out)
{
    uint16_t u16;
    int ret = i2c_wr_rd_u16(addr, TMP117_REG_TEMP, &u16);
    if (ret) return ret;
    *raw_out = (int16_t)u16; /* two's complement */
    return 0;
}

/* Convert raw -> °C*100 (integer). LSB = 1/128 °C => °C*100 ≈ raw*25/32 */
static int tmp117_c_x100_from_raw(int16_t raw)
{
    int64_t v = ((int64_t)raw * 25);
    /* rounding (optional): add +16/-16 before /32 */
    if (v >= 0) v += 16; else v -= 16;
    return (int)(v / 32);
}

static int tmp117_read_c_f_x100(uint8_t addr, int *c_x100, int *f_x100)
{
    int16_t raw;
    int ret = tmp117_read_raw(addr, &raw);
    if (ret) return ret;
    int cx100 = tmp117_c_x100_from_raw(raw);
    int fx100 = (cx100 * 9) / 5 + 3200;  /* F = C*9/5 + 32 (scaled by 100) */
    *c_x100 = cx100;
    *f_x100 = fx100;
    return 0;
}

/* Simple “soft” scan for debug */
static void i2c_scan_bus(void)
{
    printk("I2C scan start\n");
    for (uint8_t addr = 0x03; addr < 0x78; addr++) {
        uint8_t reg = 0x00, byte;
        if (i2c_write_read(i2c_dev, addr, &reg, 1, &byte, 1) == 0) {
            printk(" - found device at 0x%02X\n", addr);
        }
    }
    printk("I2C scan done\n");
}

/* ===== BLE (Zephyr NUS) ===== */
static struct bt_conn *current_conn;
static struct bt_le_ext_adv *adv;

static void nus_rx_cb(struct bt_conn *conn, const void *data, uint16_t len, void *ctx)
{
    ARG_UNUSED(conn); ARG_UNUSED(ctx);
    printk("NUS RX: %.*s\n", len, (const uint8_t *)data);
}

static struct bt_nus_cb nus_cb = { .received = nus_rx_cb };

static void on_connected(struct bt_conn *conn, uint8_t err)
{
    if (err) { printk("BLE connect failed (err %u)\n", err); return; }
    current_conn = bt_conn_ref(conn);
    printk("BLE connected\n");
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("BLE disconnected (reason %u)\n", reason);
    if (current_conn) { bt_conn_unref(current_conn); current_conn = NULL; }
    (void)bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
}

BT_CONN_CB_DEFINE(conn_cbs) = {
    .connected = on_connected,
    .disconnected = on_disconnected,
};

static int start_advertising(void)
{
    const struct bt_le_adv_param params =
        BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_USE_NAME | BT_LE_ADV_OPT_CONNECTABLE,
                             BT_GAP_ADV_FAST_INT_MIN_2,
                             BT_GAP_ADV_FAST_INT_MAX_2,
                             NULL);
    int err = bt_le_ext_adv_create(&params, NULL, &adv);
    if (err) return err;
    return bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
}

static void bt_ready(int err)
{
    if (err) { printk("bt_ready err=%d\n", err); return; }
    err = bt_nus_cb_register(&nus_cb, NULL);
    if (err) { printk("bt_nus_cb_register err=%d\n", err); return; }
    err = start_advertising();
    if (err) { printk("adv start err=%d\n", err); }
}

/* ===== App ===== */
int main(void)
{
    printk("TMP117 x2 + BLE NUS demo (I2C0 on P0.26/P0.27)\n");

    /* I2C bind + config */
    i2c_dev = DEVICE_DT_GET(I2C_NODE);
    if (!device_is_ready(i2c_dev)) {
        printk("I2C device not ready\n");
        return 0;
    }
    (void)i2c_configure(i2c_dev, I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_STANDARD));
    k_sleep(K_MSEC(10)); /* power-up settle */

    /* Optional scan for visibility */
    i2c_scan_bus();

    /* Detect TMP117 at 0x48 / 0x49 by DEVID */
    bool haveA = false, haveB = false;
    uint16_t id;
    if (tmp117_read_devid(TMP117_ADDR_A, &id) == 0) {
        printk("0x48 DEVID=0x%04X %s\n", id, (id == 0x0117) ? "(TMP117 OK)" : "(unexpected)");
        haveA = (id == 0x0117);
    } else {
        printk("0x48 DEVID read failed\n");
    }
    if (tmp117_read_devid(TMP117_ADDR_B, &id) == 0) {
        printk("0x49 DEVID=0x%04X %s\n", id, (id == 0x0117) ? "(TMP117 OK)" : "(unexpected)");
        haveB = (id == 0x0117);
    } else {
        printk("0x49 DEVID read failed\n");
    }
    if (!haveA && !haveB) {
        printk("No TMP117 detected at 0x48/0x49. Check wiring/pull-ups/ADD0 straps.\n");
    }

    /* Bring up BLE */
    int err = bt_enable(bt_ready);
    printk("bt_enable -> %d\n", err);

    while (1) {
        int eA = 0, eB = 0;
        int cA=0, fA=0, cB=0, fB=0;
        bool okA=false, okB=false;

        if (haveA) { eA = tmp117_read_c_f_x100(TMP117_ADDR_A, &cA, &fA); okA = (eA==0); }
        if (haveB) { eB = tmp117_read_c_f_x100(TMP117_ADDR_B, &cB, &fB); okB = (eB==0); }

        char line[96];
        if (okA && okB) {
            int avg_c = (cA + cB) / 2;
            int avg_f = (fA + fB) / 2;
            snprintf(line, sizeof line,
                "0x48=%d.%02dC %d.%02dF  0x49=%d.%02dC %d.%02dF  AVG=%d.%02dC %d.%02dF\n",
                cA/100,  (cA<0?-cA%100:cA%100),  fA/100,  fA%100,
                cB/100,  (cB<0?-cB%100:cB%100),  fB/100,  fB%100,
                avg_c/100, avg_c%100,            avg_f/100, avg_f%100);
        } else if (okA) {
            snprintf(line, sizeof line,
                "0x48=%d.%02dC %d.%02dF  0x49=N/A\n",
                cA/100, (cA<0?-cA%100:cA%100),  fA/100, fA%100);
        } else if (okB) {
            snprintf(line, sizeof line,
                "0x48=N/A  0x49=%d.%02dC %d.%02dF\n",
                cB/100, (cB<0?-cB%100:cB%100),  fB/100, fB%100);
        } else {
            snprintf(line, sizeof line, "TMP117 read errors: 0x48=%d 0x49=%d\n", eA, eB);
        }

        /* Console */
        printk("%s", line);

        /* BLE NUS */
        if (current_conn) {
            int se = bt_nus_send(current_conn, line, strlen(line));
            if (se) printk("NUS send err %d\n", se);
        } else {
            /* If none detected at boot, occasionally rescan/probe while idle */
            if (!haveA && !haveB) {
                i2c_scan_bus();
                if (tmp117_read_devid(TMP117_ADDR_A, &id) == 0 && id == 0x0117) haveA = true;
                if (tmp117_read_devid(TMP117_ADDR_B, &id) == 0 && id == 0x0117) haveB = true;
            }
        }

        k_sleep(K_MSEC(500));
    }
}
