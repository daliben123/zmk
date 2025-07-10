#pragma once
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

struct a320_data {
    struct k_mutex mutex;       // 互斥锁保护多线程访问
    int16_t x_position;         // 缓存X轴数据
    int16_t y_position;         // 缓存Y轴数据
};

struct a320_config {
    struct i2c_dt_spec bus;     // I2C总线配置
#if DT_INST_NODE_HAS_PROP(0, nrst_gpios)
    struct gpio_dt_spec nrst_gpio; // 复位引脚
#endif
#if DT_INST_NODE_HAS_PROP(0, motion_gpios)
    struct gpio_dt_spec motion_gpio; // 运动检测引脚
#endif
#if DT_INST_NODE_HAS_PROP(0, shutdown_gpios)
    struct gpio_dt_spec shutdown_gpio; // 关机引脚
#endif
};

// 寄存器定义
#define Motion 0x02
#define Delta_X 0x03
#define Delta_Y 0x04

// 状态位掩码
#define BIT_MOTION_MOT (1 << 7)  // 运动检测标志
#define BIT_MOTION_OVF (1 << 4)  // 数据溢出标志
