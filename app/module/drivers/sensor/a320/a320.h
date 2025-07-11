#ifndef A320_H_
#define A320_H_

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

/* 寄存器定义（与数据手册完全一致） */
#define A320_PRODUCT_ID_REG   0x00  // 产品ID寄存器
#define A320_MOTION_REG       0x02  // 运动状态寄存器
#define A320_DELTA_X_REG      0x03  // X轴位移寄存器（双字节）
#define A320_DELTA_Y_REG      0x05  // Y轴位移寄存器（双字节）[2](@ref)
#define A320_CONFIG_REG       0x0A  // 配置寄存器

/* 状态位掩码 */
#define BIT_MOTION_MOT        (1 << 0)  // 运动检测标志（bit0）
#define BIT_MOTION_OVF        (1 << 4)  // 数据溢出标志（bit4）[1](@ref)

/* 设备树兼容性标识符（声明供外部使用） */
#define A320_DT_COMPAT_STR "avago,a320"

/* ---------- 关键数据结构声明（定义在.c文件中） ---------- */
struct a320_data;  // 前向声明私有数据
struct a320_config {
    struct i2c_dt_spec bus;        // I²C总线配置（必需）
    struct gpio_dt_spec nrst_gpio; // 复位引脚（可选）
};

/* ---------- 传感器通道定义 ---------- */
/* 扩展标准传感器通道以支持位移数据 */
#define SENSOR_CHAN_POS_DX   (SENSOR_CHAN_PRIV_START + 0)  // X轴位移通道
#define SENSOR_CHAN_POS_DY   (SENSOR_CHAN_PRIV_START + 1)  // Y轴位移通道

/* ---------- 驱动API原型 ---------- */
#ifdef __cplusplus
extern "C" {
#endif

/* 硬件复位函数（供初始化流程调用） */
int a320_hardware_reset(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* A320_H_ */
