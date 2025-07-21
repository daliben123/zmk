
/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/settings/settings.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/matrix.h>
#include <zmk/kscan.h>
#include <zmk/display.h>
#include <drivers/ext_power.h>

#include <zmk/hid.h>
#include <dt-bindings/zmk/mouse.h>
#include <zmk/hid_indicators.h>
#include <zmk/indicator_capslock.h>
#ifdef CONFIG_ZMK_MOUSE
#include <zmk/mouse.h>
#endif /* CONFIG_ZMK_MOUSE */
static const struct device *get_a320_device(void) {
    const struct device *dev = DEVICE_DT_GET_ANY(avago_a320);

    if (dev == NULL) {
        printk("\nError: no device found.\n");
        return NULL;
    }
    if (!device_is_ready(dev)) {
        printk("\nError: Device \"%s\" is not ready; "
               "check the driver initialization logs for errors.\n",
               dev->name);
        return NULL;
    }
    printk("Found device \"%s\", getting sensor data\n", dev->name);
    return dev;
}
int main(void) {
    LOG_INF("Welcome to ZMK!\n");

    if (zmk_kscan_init(DEVICE_DT_GET(ZMK_MATRIX_NODE_ID)) != 0) {
        return -ENOTSUP;
    }
    const struct device *dev = get_a320_device();
    if (dev == NULL) {
        return;
    }
#ifdef CONFIG_ZMK_DISPLAY
    zmk_display_init();
#endif /* CONFIG_ZMK_DISPLAY */
    struct sensor_value xy_pos;
    while (1) {
        sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &xy_pos);
        char target_9900[] = "bb9900";
        char target_q10[] = "bbq10";
        char target_q20[] = "bbq20";
        char target_q30[] = "bbq30";
        char target_9981[] = "bbp9981";
        char target_9983[] = "bbp9983";
        char target_bbcase[] = "bbcase";
        if (strcmp(CONFIG_ZMK_KEYBOARD_NAME, target_9900) == 0 ||
            strcmp(CONFIG_ZMK_KEYBOARD_NAME, target_bbcase) == 0) {
            int8_t x = xy_pos.val2;
            int8_t y = xy_pos.val1;
            int8_t scroll_x = 0;
            int8_t scroll_y = 0;
            if (zmk_hid_indicators_get_current_profile() == 2 ||
                zmk_hid_indicators_get_current_profile() == 3 ||
                zmk_hid_indicators_get_current_profile() == 7 ||
                zmk_hid_indicators_get_current_profile() == 4) {
                if (abs(y) >= 128) {
                    scroll_x = -x / 24;
                    scroll_y = -y / 24;
                } else if (abs(y) >= 64 && abs(y) < 128) {
                    scroll_x = -x / 16;
                    scroll_y = -y / 16;
                } else if (abs(y) >= 32 && abs(y) < 64) {
                    scroll_x = -x / 12;
                    scroll_y = -y / 12;
                } else if (abs(y) >= 21 && abs(y) < 32) {
                    scroll_x = -x / 8;
                    scroll_y = -y / 8;
                } else if (abs(y) >= 3 && abs(y) < 20) {
                    scroll_x = -(x > 0) ? 1 : (x < 0) ? -1 : 0;
                    scroll_y = -((y > 0) ? 1 : (y < 0) ? -1 : 0);
                } else if (abs(y) >= 0 && abs(y) < 2) {
                    scroll_x = -(x > 0) ? 1 : (x < 0) ? -1 : 0;
                    ;
                    scroll_y = 0;
                }
                int Scroll_INTERVAL = CONFIG_TRACKPAD_SCROLL_INTERVAL;
                k_sleep(K_MSEC(Scroll_INTERVAL));
                x = 0;
                y = 0;
            } else {
                x = ((x < 127) ? x : (x - 256)) * 1.5 * CONFIG_TRACKPAD_SPEEDMULTIPLIER_HORIZONTAL /
                    100;
                y = ((y < 127) ? y : (y - 256)) * 1.5 * CONFIG_TRACKPAD_SPEEDMULTIPLIER_VERTICAL /
                    100;
            }
            zmk_hid_mouse_movement_set(0, 0);
            zmk_hid_mouse_movement_update(x, y);
            zmk_hid_mouse_scroll_set(0, 0);
            zmk_hid_mouse_scroll_update(scroll_x, scroll_y);
            zmk_endpoints_send_mouse_report();
        } else if (strcmp(CONFIG_ZMK_KEYBOARD_NAME, target_q10) == 0 ||
                   strcmp(CONFIG_ZMK_KEYBOARD_NAME, target_q30) == 0 ||
                   strcmp(CONFIG_ZMK_KEYBOARD_NAME, target_9981) == 0 ||
                   strcmp(CONFIG_ZMK_KEYBOARD_NAME, target_9983) == 0) {
            int8_t x = xy_pos.val1;
            int8_t y = -1 * xy_pos.val2;
            // LOG_DBG("x value : %d , y value : %d\r\n", val->val1, val->val2);
            int8_t scroll_x = 0;
            int8_t scroll_y = 0;
            if (zmk_hid_indicators_get_current_profile() == 2 ||
                zmk_hid_indicators_get_current_profile() == 3 ||
                zmk_hid_indicators_get_current_profile() == 7 ||
                zmk_hid_indicators_get_current_profile() == 4) {
                if (abs(y) >= 128) {
                    scroll_x = -x / 24;
                    scroll_y = -y / 24;
                } else if (abs(y) >= 64 && abs(y) < 128) {
                    scroll_x = -x / 16;
                    scroll_y = -y / 16;
                } else if (abs(y) >= 32 && abs(y) < 64) {
                    scroll_x = -x / 12;
                    scroll_y = -y / 12;
                } else if (abs(y) >= 21 && abs(y) < 32) {
                    scroll_x = -x / 8;
                    scroll_y = -y / 8;
                } else if (abs(y) >= 3 && abs(y) < 20) {
                    scroll_x = -(x > 0) ? 1 : (x < 0) ? -1 : 0;
                    scroll_y = -((y > 0) ? 1 : (y < 0) ? -1 : 0);
                } else if (abs(y) >= 0 && abs(y) < 2) {
                    scroll_x = -(x > 0) ? 1 : (x < 0) ? -1 : 0;
                    ;
                    scroll_y = 0;
                }
                int Scroll_INTERVAL = CONFIG_TRACKPAD_SCROLL_INTERVAL;
                k_sleep(K_MSEC(Scroll_INTERVAL));
                x = 0;
                y = 0;
            } else {
                x = ((x < 127) ? x : (x - 256)) * 1.5 * CONFIG_TRACKPAD_SPEEDMULTIPLIER_HORIZONTAL /
                    100;
                y = ((y < 127) ? y : (y - 256)) * 1.5 * CONFIG_TRACKPAD_SPEEDMULTIPLIER_VERTICAL /
                    100;
            }
            zmk_hid_mouse_movement_set(0, 0);
            zmk_hid_mouse_movement_update(x, y);
            zmk_hid_mouse_scroll_set(0, 0);
            zmk_hid_mouse_scroll_update(scroll_x, scroll_y);
            zmk_endpoints_send_mouse_report();
        } else if (strcmp(CONFIG_ZMK_KEYBOARD_NAME, target_q20) == 0) {
            int8_t x = xy_pos.val2;
            int8_t y = -1 * xy_pos.val1;
            // LOG_DBG("x value : %d , y value : %d\r\n", val->val1, val->val2);
            int8_t scroll_x = 0;
            int8_t scroll_y = 0;
            if (zmk_hid_indicators_get_current_profile() == 2 ||
                zmk_hid_indicators_get_current_profile() == 3 ||
                zmk_hid_indicators_get_current_profile() == 7 ||
                zmk_hid_indicators_get_current_profile() == 4) {
                if (abs(y) >= 128) {
                    scroll_x = -x / 24;
                    scroll_y = -y / 24;
                } else if (abs(y) >= 64 && abs(y) < 128) {
                    scroll_x = -x / 16;
                    scroll_y = -y / 16;
                } else if (abs(y) >= 32 && abs(y) < 64) {
                    scroll_x = -x / 12;
                    scroll_y = -y / 12;
                } else if (abs(y) >= 21 && abs(y) < 32) {
                    scroll_x = -x / 8;
                    scroll_y = -y / 8;
                } else if (abs(y) >= 3 && abs(y) < 20) {
                    scroll_x = -(x > 0) ? 1 : (x < 0) ? -1 : 0;
                    scroll_y = -((y > 0) ? 1 : (y < 0) ? -1 : 0);
                } else if (abs(y) >= 0 && abs(y) < 2) {
                    scroll_x = -(x > 0) ? 1 : (x < 0) ? -1 : 0;
                    ;
                    scroll_y = 0;
                }
                int Scroll_INTERVAL = CONFIG_TRACKPAD_SCROLL_INTERVAL;
                k_sleep(K_MSEC(Scroll_INTERVAL));
                x = 0;
                y = 0;
            } else {
                x = ((x < 127) ? x : (x - 256)) * 1.5 * CONFIG_TRACKPAD_SPEEDMULTIPLIER_HORIZONTAL /
                    100;
                y = ((y < 127) ? y : (y - 256)) * 1.5 * CONFIG_TRACKPAD_SPEEDMULTIPLIER_VERTICAL /
                    100;
            }
            zmk_hid_mouse_movement_set(0, 0);
            zmk_hid_mouse_movement_update(x, y);
            zmk_hid_mouse_scroll_set(0, 0);
            zmk_hid_mouse_scroll_update(scroll_x, scroll_y);
            zmk_endpoints_send_mouse_report();
        }
        int polling_ms = (1.0 / (float)CONFIG_INPUT_A320_POLLINGRATE) * 1000.0;
        k_sleep(K_MSEC(polling_ms));
    }
}
