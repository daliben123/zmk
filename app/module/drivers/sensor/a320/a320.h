#pragma once
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>

/* 寄存器定义 */
#define A320_REG_PRODUCT_ID      0x00
#define A320_REG_REVISION_ID     0x01
#define A320_REG_MOTION          0x02
#define A320_REG_DELTA_X         0x03
#define A320_REG_DELTA_Y         0x04
#define A320_REG_SQUAL           0x05
#define A320_REG_SELF_TEST       0x10
#define A320_REG_SOFT_RESET      0x3a

/* 运动检测标志位 */
#define BIT_MOTION_MOT           (1 << 7)
#define BIT_MOTION_OVF           (1 << 4)

/* 设备配置结构体 */
struct a320_config {
    struct i2c_dt_spec bus;         // I²C总线配置
    struct k_mutex polling_mutex;   // 多线程互斥锁
    
#if DT_INST_NODE_HAS_PROP(0, nrst_gpios)
    struct gpio_dt_spec nrst_gpio;  // 复位引脚
#endif
#if DT_INST_NODE_HAS_PROP(0, motion_gpios)
    struct gpio_dt_spec motion_gpio; // 运动中断引脚
#endif
};

/* 设备数据存储结构体 */
struct a320_data {
    int16_t delta_x;                // X轴位移
    int16_t delta_y;                // Y轴位移
    uint8_t motion_status;          // 运动状态
};
