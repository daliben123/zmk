#pragma once
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

/* 传感器关键寄存器定义（严格遵循数据手册） */
#define A320_PRODUCT_ID_REG  0x00  // 产品ID寄存器[7](@ref)
#define A320_MOTION_REG      0x02  // 运动状态寄存器
#define A320_DELTA_X_REG     0x03  // X轴位移寄存器
#define A320_DELTA_Y_REG     0x04  // Y轴位移寄存器

/* 状态位掩码（关键修复） */
#define BIT_MOTION_MOT (1 << 0)  // 运动检测标志（修正为bit0）
#define BIT_MOTION_OVF (1 << 4)  // 数据溢出标志（保持bit4）

/* 预期产品ID值 */
#define A320_EXPECTED_ID     0x58  // 根据传感器手册确定

struct a320_data {
    struct k_mutex mutex;       // 互斥锁保护多线程访问
    int16_t x_position;         // 缓存X轴数据（-32768 ~ +32767）
    int16_t y_position;         // 缓存Y轴数据
    uint8_t last_status;        // 最近一次状态寄存器值（调试用）
};

struct a320_config {
    struct i2c_dt_spec bus;     // I²C总线配置
    struct gpio_dt_spec nrst_gpio;     // 复位引脚（低电平有效）[1](@ref)
    struct gpio_dt_spec motion_gpio;   // 运动检测中断引脚（可选）
    struct gpio_dt_spec shutdown_gpio; // 电源控制引脚（高电平有效）
};

// 设备树访问宏（防止硬编码）
#define DT_DRV_COMPAT avago_a320#pragma once
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
    struct i2c_dt_spec bus;  // I²C总线配置
    struct gpio_dt_spec nrst_gpio;  // 复位引脚[6](@ref)
    struct gpio_dt_spec motion_gpio;  // 运动检测引脚
    struct gpio_dt_spec shutdown_gpio;  // 电源控制引脚
};

// 寄存器定义
#define Motion 0x02
#define Delta_X 0x03
#define Delta_Y 0x04

// 状态位掩码
#define BIT_MOTION_MOT (1 << 7)  // 运动检测标志
#define BIT_MOTION_OVF (1 << 4)  // 数据溢出标志
