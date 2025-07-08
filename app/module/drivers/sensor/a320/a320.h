/* a320.h - 传感器寄存器定义和配置结构 */
#pragma once
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

/* 运动检测状态位 */
#define BIT_MOTION_MOT (1 << 7)
#define BIT_MOTION_OVF (1 << 4)

/* 传感器寄存器定义 */
enum a320_registers {
    MOTION_REG      = 0x02,
    DELTA_X_REG     = 0x03,
    DELTA_Y_REG     = 0x04,
    CONFIG_REG      = 0x11,
    RESOLUTION_REG  = 0x62,
    SOFT_RESET_REG  = 0x3A
};

/* 传感器数据结构 */
struct a320_data {
    int16_t delta_x;    // X轴位移（有符号）
    int16_t delta_y;    // Y轴位移（有符号）
    uint8_t status;     // 运动状态寄存器缓存
    struct k_mutex data_mutex; // 数据访问互斥锁
};

/* 设备配置结构 */
struct a320_config {
    struct i2c_dt_spec bus;      // I2C总线配置
    uint8_t resolution;          // 分辨率设置（CPI）
    
    /* 可选GPIO引脚 */
#if DT_INST_NODE_HAS_PROP(0, nrst_gpios)
    struct gpio_dt_spec nrst_gpio; // 硬件复位引脚
#endif
#if DT_INST_NODE_HAS_PROP(0, motion_gpios)
    struct gpio_dt_spec motion_gpio; // 运动中断引脚
#endif
};
