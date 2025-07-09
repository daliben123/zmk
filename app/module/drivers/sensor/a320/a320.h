/* drivers/sensor/a320/a320.h */
#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

/* 设备树兼容性定义（与驱动匹配） */
#define DT_DRV_COMPAT avago_a320

/* ================ 寄存器地址与状态位定义 ================ */
/* 寄存器地址（对应驱动中A320_REG_*引用） */
#define A320_REG_STATUS     0x02    /* 运动状态寄存器（含溢出标志） */
#define A320_REG_DELTA_X    0x03    /* X轴原生位移计数寄存器 */
#define A320_REG_DELTA_Y    0x04    /* Y轴原生位移计数寄存器 */
#define A320_REG_CONFIG     0x11    /* 配置寄存器（采样率等） */
#define A320_REG_ID         0x00    /* 设备ID寄存器（产品ID） */

/* 状态位定义（对应驱动中A320_STATUS_*） */
#define A320_STATUS_OVF     BIT(4)  /* 数据溢出标志位（bit4） */
#define A320_STATUS_MOT     BIT(7)  /* 运动检测标志位（bit7） */

/* 设备ID（与驱动中A320_DEVICE_ID匹配，需根据硬件手册确认） */
#define A320_DEVICE_ID      0x32    /* 示例：A320的产品ID值 */

/* ================ 传感器通道定义 ================ */
/* 与驱动中channel_get支持的通道匹配 */
#define A320_CHAN_DX        SENSOR_CHAN_POS_DX  /* X轴位移通道 */
#define A320_CHAN_DY        SENSOR_CHAN_POS_DY  /* Y轴位移通道 */

/* ================ 数据结构声明 ================ */
/* 驱动运行时数据（RAM区） */
struct a320_data {
    struct k_sem data_sem;          /* 数据就绪信号量 */
    struct k_mutex i2c_mutex;       /* I2C总线互斥锁 */
    int8_t delta_x;                 /* X轴原生位移计数（-128~127） */
    int8_t delta_y;                 /* Y轴原生位移计数（-128~127） */
    struct gpio_callback int_cb;    /* 中断回调结构 */
};

/* 驱动配置数据（ROM区，来自设备树） */
struct a320_config {
    struct i2c_dt_spec bus;         /* I2C总线规格（设备树定义） */
    struct gpio_dt_spec reset_gpio; /* 复位引脚（设备树可选） */
    struct gpio_dt_spec int_gpio;   /* 中断引脚（设备树可选） */
};

/* ================ 驱动函数原型（内部使用，无需外部暴露） ================ */
/* 注：静态函数无需在头文件声明，此处仅为驱动内部逻辑关联 */
#ifdef __cplusplus
extern "C" {
#endif

/* 硬件复位函数（驱动内部调用） */
static inline void a320_hardware_reset(const struct device *dev);

/* 寄存器访问函数（驱动内部调用） */
static inline int a320_reg_access(const struct device *dev, 
                                 uint8_t reg, uint8_t *val, bool write);

#ifdef __cplusplus
}
#endif
