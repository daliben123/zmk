#pragma once
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

/* 寄存器定义（与数据手册完全一致） */
#define A320_PRODUCT_ID_REG   0x00  // 产品ID寄存器
#define A320_MOTION_REG       0x02  // 运动状态寄存器
#define A320_DELTA_X_REG      0x03  // X轴位移寄存器（双字节）
#define A320_DELTA_Y_REG      0x05  // Y轴位移寄存器（双字节）
#define A320_CONFIG_REG       0x0A  // 配置寄存器

/* 状态位掩码 */
#define BIT_MOTION_MOT        (1 << 0)  // 运动检测标志（bit0）
#define BIT_MOTION_OVF        (1 << 4)  // 数据溢出标志（bit4）

/* 设备树兼容性标识符（声明供外部使用） */
#define A320_DT_COMPAT_STR "avago,a320"

/* 结构体前向声明 */
struct a320_data;
struct a320_config;

/* 不再声明a320_hardware_reset函数 */
