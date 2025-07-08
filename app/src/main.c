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

/* 获取A320传感器设备 */
static const struct device *get_a320_device(void) {
    const struct device *dev = DEVICE_DT_GET_ANY(avago_a320);

    if (dev == NULL) {
        LOG_ERR("未找到A320设备");
        return NULL;
    }
    if (!device_is_ready(dev)) {
        LOG_ERR("设备 \"%s\" 未就绪，请检查驱动初始化日志", dev->name);
        return NULL;
    }
    LOG_INF("找到A320设备: \"%s\"", dev->name);
    return dev;
}

/* 处理鼠标移动和滚动的通用函数 */
static void process_mouse_movement(const struct device *dev) {
    struct sensor_value dx, dy;
    int16_t x = 0, y = 0;
    int8_t scroll_x = 0, scroll_y = 0;
    
    /* 获取位移数据 */
    if (sensor_sample_fetch(dev) < 0) {
        LOG_ERR("获取传感器数据失败");
        return;
    }
    
    /* 读取X/Y位移通道 */
    if (sensor_channel_get(dev, SENSOR_CHAN_POS_DX, &dx) < 0 ||
        sensor_channel_get(dev, SENSOR_CHAN_POS_DY, &dy) < 0) {
        LOG_ERR("读取位移通道失败");
        return;
    }
    
    /* 转换为微米级位移 */
    x = sensor_value_to_micro(&dx);
    y = sensor_value_to_micro(&dy);
    
    /* 根据HID配置文件处理移动或滚动 */
    if (zmk_hid_indicators_get_current_profile() == 2 ||
        zmk_hid_indicators_get_current_profile() == 3 ||
        zmk_hid_indicators_get_current_profile() == 7 ||
        zmk_hid_indicators_get_current_profile() == 4) {
        /* 滚动模式 */
        if (abs(y) >= 128000) {  // 128 mm
            scroll_x = -x / 24000;  // 缩放因子
            scroll_y = -y / 24000;
        } else if (abs(y) >= 64000) {  // 64 mm
            scroll_x = -x / 16000;
            scroll_y = -y / 16000;
        } else if (abs(y) >= 32000) {  // 32 mm
            scroll_x = -x / 12000;
            scroll_y = -y / 12000;
        } else if (abs(y) >= 21000) {  // 21 mm
            scroll_x = -x / 8000;
            scroll_y = -y / 8000;
        } else if (abs(y) >= 3000) {  // 3 mm
            scroll_x = (x > 0) ? -1 : (x < 0) ? 1 : 0;
            scroll_y = (y > 0) ? -1 : (y < 0) ? 1 : 0;
        } else if (abs(y) < 2000) {  // < 2 mm
            scroll_x = (x > 0) ? -1 : (x < 0) ? 1 : 0;
        }
        k_sleep(K_MSEC(CONFIG_TRACKPAD_SCROLL_INTERVAL));
    } else {
        /* 指针移动模式 */
        x = x * CONFIG_TRACKPAD_SPEEDMULTIPLIER_HORIZONTAL / 100;
        y = y * CONFIG_TRACKPAD_SPEEDMULTIPLIER_VERTICAL / 100;
    }
    
    /* 发送HID报告 */
    zmk_hid_mouse_movement_set(0, 0);
    zmk_hid_mouse_movement_update(x, y);
    zmk_hid_mouse_scroll_set(0, 0);
    zmk_hid_mouse_scroll_update(scroll_x, scroll_y);
    zmk_endpoints_send_mouse_report();
}

int main(void) {
    LOG_INF("欢迎使用ZMK!");
    
    /* 初始化键盘矩阵 */
    if (zmk_kscan_init(DEVICE_DT_GET(ZMK_MATRIX_NODE_ID)) != 0) {
        LOG_ERR("键盘矩阵初始化失败");
        return -ENOTSUP;
    }
    
    /* 获取传感器设备 */
    const struct device *a320_dev = get_a320_device();
    if (a320_dev == NULL) {
        return -ENODEV;
    }
    
#ifdef CONFIG_ZMK_DISPLAY
    zmk_display_init();
#endif /* CONFIG_ZMK_DISPLAY */
    
    /* 主循环 */
    while (1) {
        process_mouse_movement(a320_dev);
        
        /* 根据配置的轮询率休眠 */
        uint32_t polling_ms = (1000 / CONFIG_INPUT_A320_POLLINGRATE);
        k_sleep(K_MSEC(polling_ms));
    }
}
