/*
 * Copyright (c) 2020 The ZMK Contributors
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
#endif

/* ================ 关键修改部分 ================ */
// 1. 简化键盘型号判断逻辑
static inline bool is_keyboard_type(const char *target) {
    return strcmp(CONFIG_ZMK_KEYBOARD_NAME, target) == 0;
}

// 2. 抽离轴方向调整函数
static void adjust_axis_direction(int8_t *x, int8_t *y) {
    if (is_keyboard_type("bb9900") || is_keyboard_type("bbcase")) {
        // 默认方向无需调整
    } else if (is_keyboard_type("bbq10") || is_keyboard_type("bbq30") || 
               is_keyboard_type("bbp9981") || is_keyboard_type("bbp9983")) {
        *y = -*y; // Y轴反转
    } else if (is_keyboard_type("bbq20")) {
        *x = -*x; // X轴反转
    }
}

// 3. 滚动模式处理函数
static void handle_scroll(int8_t x, int8_t y, int8_t *scroll_x, int8_t *scroll_y) {
    if (abs(y) >= 128) {
        *scroll_x = -x / 24;
        *scroll_y = -y / 24;
    } else if (abs(y) >= 64) {
        *scroll_x = -x / 16;
        *scroll_y = -y / 16;
    } else if (abs(y) >= 32) {
        *scroll_x = -x / 12;
        *scroll_y = -y / 12;
    } else if (abs(y) >= 21) {
        *scroll_x = -x / 8;
        *scroll_y = -y / 8;
    } else if (abs(y) >= 3) {
        *scroll_x = (x > 0) ? -1 : (x < 0) ? 1 : 0;
        *scroll_y = (y > 0) ? -1 : (y < 0) ? 1 : 0;
    } else {
        *scroll_x = (x > 0) ? -1 : (x < 0) ? 1 : 0;
        *scroll_y = 0;
    }
}

// 4. 更新HID报告函数
static void update_mouse_report(int8_t x, int8_t y, int8_t scroll_x, int8_t scroll_y) {
    zmk_hid_mouse_movement_set(0, 0);
    zmk_hid_mouse_movement_update(x, y);
    zmk_hid_mouse_scroll_set(0, 0);
    zmk_hid_mouse_scroll_update(scroll_x, scroll_y);
    zmk_endpoints_send_mouse_report();
}

/* ================ 设备初始化 ================ */
static const struct device *get_a320_device(void) {
    const struct device *dev = DEVICE_DT_GET_ANY(avago_a320);
    if (dev == NULL) {
        LOG_ERR("A320 device not found in DT");
        return NULL;
    }
    if (!device_is_ready(dev)) {
        LOG_ERR("Device \"%s\" not ready", dev->name);
        return NULL;
    }
    LOG_INF("Using A320 device: %s", dev->name);
    return dev;
}

int main(void) {
    LOG_INF("ZMK initialized");

    if (zmk_kscan_init(DEVICE_DT_GET(ZMK_MATRIX_NODE_ID)) != 0) {
        LOG_ERR("Kscan init failed");
        return -ENODEV;
    }

    const struct device *a320_dev = get_a320_device();
    if (a320_dev == NULL) {
        return -ENODEV; // 设备未就绪时终止程序
    }

#ifdef CONFIG_ZMK_DISPLAY
    zmk_display_init();
#endif

    while (1) {
        /* ================ 数据获取逻辑优化 ================ */
        // 1. 主动获取传感器数据
        if (sensor_sample_fetch(a320_dev) != 0) {
            LOG_WRN("Sensor fetch failed");
            k_sleep(K_MSEC(10));
            continue;
        }

        // 2. 分别读取X/Y位移通道（修正通道类型）
        struct sensor_value x_val, y_val;
        sensor_channel_get(a320_dev, SENSOR_CHAN_POS_DX, &x_val); // X位移[1](@ref)
        sensor_channel_get(a320_dev, SENSOR_CHAN_POS_DY, &y_val); // Y位移[1](@ref)

        // 3. 转换为有符号位移量
        int8_t x = x_val.val1;
        int8_t y = y_val.val1;
        if (x >= 128) x -= 256; // 处理补码
        if (y >= 128) y -= 256;

        // 4. 根据键盘型号调整方向
        adjust_axis_direction(&x, &y);

        // 5. 判断是否为滚动模式
        int8_t scroll_x = 0, scroll_y = 0;
        const uint8_t profile = zmk_hid_indicators_get_current_profile();
        const bool is_scroll_mode = (profile == 2 || profile == 3 || profile == 4 || profile == 7);

        if (is_scroll_mode) {
            handle_scroll(x, y, &scroll_x, &scroll_y);
            k_sleep(K_MSEC(CONFIG_TRACKPAD_SCROLL_INTERVAL));
        } else {
            // 移动模式处理
            x = x * 1.5 * CONFIG_TRACKPAD_SPEEDMULTIPLIER_HORIZONTAL / 100;
            y = y * 1.5 * CONFIG_TRACKPAD_SPEEDMULTIPLIER_VERTICAL / 100;
        }

        // 6. 更新HID报告
        update_mouse_report(x, y, scroll_x, scroll_y);

        /* ================ 按配置的轮询率延时 ================ */
        const uint32_t polling_ms = 1000 / CONFIG_INPUT_A320_POLLINGRATE;
        k_sleep(K_MSEC(polling_ms));
    }
    return 0;
}
