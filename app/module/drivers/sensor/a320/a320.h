/* drivers/sensor/a320/a320.h */
#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

/* 设备树兼容性标识（与C文件首行匹配） */
#define DT_DRV_COMPAT avago_a320

/* ================ 寄存器地址（C文件中实际访问的地址） ================ */
#define A320_REG_STATUS     0x02    /* 运动状态寄存器（C文件中用于检测溢出） */
#define A320_REG_DELTA_X    0x03    /* X轴位移寄存器（C文件中a320_sample_fetch读取） */
#define A320_REG_DELTA_Y    0x04    /* Y轴位移寄存器（C文件中a320_sample_fetch读取） */
#define A320_REG_CONFIG     0x11    /* 配置寄存器（C文件中用于设置采样率） */
#define A320_REG_ID         0x00    /* 设备ID寄存器（C文件中初始化验证） */

/* ================ 寄存器位定义（C文件中实际使用的标志） ================ */
/* 状态寄存器（A320_REG_STATUS）位定义，与C文件中判断逻辑匹配 */
#define A320_STATUS_OVF     BIT(4)  /* 溢出标志（对应C文件中"status & A320_STATUS_OVF"） */
#define A320_STATUS_MOT     BIT(7)  /* 运动标志（C文件中断处理的触发条件） */

/* 设备ID常量（C文件中用于验证设备合法性） */
#define A320_DEVICE_ID      0x32    /* 需与硬件实际ID一致，C文件中"dev_id != A320_DEVICE_ID" */

/* ================ 数据结构（与C文件定义完全一致） ================ */
/* 驱动运行时数据（C文件中dev->data指向的结构） */
struct a320_data {
    struct k_sem data_sem;          /* 数据同步信号量（C文件中断函数中k_sem_give） */
    struct k_mutex i2c_mutex;       /* I2C互斥锁（C文件中a320_reg_access使用） */
    int8_t delta_x;                 /* X轴原生位移（C文件中sample_fetch更新，channel_get返回） */
    int8_t delta_y;                 /* Y轴原生位移（同上） */
    struct gpio_callback int_cb;    /* 中断回调结构（C文件中a320_init_interrupt初始化） */
};

/* 驱动配置数据（C文件中dev->config指向的结构） */
struct a320_config {
    struct i2c_dt_spec bus;         /* I2C总线规格（C文件中i2c_reg_read/write_byte_dt使用） */
    struct gpio_dt_spec reset_gpio; /* 复位GPIO（C文件中a320_hardware_reset操作） */
    struct gpio_dt_spec int_gpio;   /* 中断GPIO（C文件中a320_init_interrupt配置） */
};

/* ================ 函数原型（与C文件中函数定义一一对应） ================ */
#ifdef __cplusplus
extern "C" {
#endif

/* 寄存器访问函数（C文件中static函数，仅声明为内部使用） */
static int a320_reg_access(const struct device *dev, 
                          uint8_t reg, uint8_t *val, bool write);

/* 数据采集与通道获取（C文件中传感器API实现） */
static int a320_sample_fetch(const struct device *dev, enum sensor_channel chan);
static int a320_channel_get(const struct device *dev, enum sensor_channel chan,
                           struct sensor_value *val);

/* 中断处理函数（C文件中回调函数，修正签名匹配） */
static void a320_motion_handler(const struct device *dev,
                                struct gpio_callback *cb,
                                uint32_t pins);

/* 硬件控制函数（C文件中初始化和复位逻辑） */
static void a320_hardware_reset(const struct device *dev);
static int a320_init_interrupt(const struct device *dev);
static int a320_attr_set(const struct device *dev, enum sensor_channel chan,
                        enum sensor_attribute attr, const struct sensor_value *val);

/* 驱动初始化函数（C文件中设备初始化入口） */
static int a320_init(const struct device *dev);

#ifdef __cplusplus
}
#endif
