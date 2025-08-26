#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/services/nus.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include <stdio.h>
#include "TMP117.h"

/* I2C binding */
#define I2C_NODE DT_NODELABEL(i2c0)
static const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);

/* BLE */
static struct bt_conn *current_conn;
static struct bt_le_ext_adv *adv;

static void nus_rx_cb(struct bt_conn *conn, const void *data, uint16_t len, void *ctx)
{
    ARG_UNUSED(conn); ARG_UNUSED(ctx);
    printk("NUS RX: %.*s\n", len, (const char *)data);
}
static struct bt_nus_cb nus_cb = { .received = nus_rx_cb };

static void on_connected(struct bt_conn *conn, uint8_t err)
{
    if (err) { printk("BLE connect failed (%u)\n", err); return; }
    current_conn = bt_conn_ref(conn);
    printk("BLE connected\n");
}
static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("BLE disconnected (%u)\n", reason);
    if (current_conn) { bt_conn_unref(current_conn); current_conn = NULL; }
    bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
}
BT_CONN_CB_DEFINE(conn_cbs) = { .connected=on_connected, .disconnected=on_disconnected };

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
    bt_nus_cb_register(&nus_cb, NULL);
    start_advertising();
}

/* ===== Application ===== */
int main(void)
{
    printk("TMP117 dual-sensor BLE Alert demo\n");

    if (!device_is_ready(i2c_dev)) {
        printk("I2C device not ready\n");
        return 0;
    }

    /* Init BLE */
    bt_enable(bt_ready);

    /* Init both TMP117s */
    tmp117_init(i2c_dev, TMP117_ADDR_48);
    tmp117_init(i2c_dev, TMP117_ADDR_49);

    while (1) {
        int tA=0, tB=0, avg=0;
        int eA = tmp117_read_c(i2c_dev, TMP117_ADDR_48, &tA);
        int eB = tmp117_read_c(i2c_dev, TMP117_ADDR_49, &tB);

        if (eA==0 && eB==0) {
            avg = (tA + tB) / 2;
            char msg[64];
            snprintf(msg, sizeof msg,
                     "Avg=%d.%02d C %s\n",
                     avg/100, avg%100,
                     (avg > 2600) ? "PWM_ON" : "PWM_OFF");

            printk("%s", msg);
            if (current_conn) {
                bt_nus_send(current_conn, msg, strlen(msg));
            }
        } else {
            printk("Sensor read error: %d %d\n", eA, eB);
        }
        k_sleep(K_SECONDS(1));
    }
}
