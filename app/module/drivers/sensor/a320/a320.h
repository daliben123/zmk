#pragma once  // 确保头文件保护在第一行

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

/* 寄存器定义 */
#define A320_PRODUCT_ID_REG  0x00
#define A320_MOTION_REG      0x02
#define A320_DELTA_X_REG     0x03
#define A320_DELTA_Y_REG     0x04

/* 状态位掩码 */
#define BIT_MOTION_MOT (1 << 0)  // 修正为bit0
#define BIT_MOTION_OVF (1 << 4)

/* 数据与配置结构体（关键修复） */
#ifdef DT_DRV_COMPAT  // 防止与驱动文件冲突
#undef DT_DRV_COMPAT
#endif

struct a320_data {
    struct k_mutex mutex;
    int16_t x_position;
    int16_t y_position;
    uint8_t last_status;  // 新增调试字段
};

struct a320_config {
    struct i2c_dt_spec bus;
    struct gpio_dt_spec nrst_gpio;
    struct gpio_dt_spec motion_gpio;
    struct gpio_dt_spec shutdown_gpio;
};
