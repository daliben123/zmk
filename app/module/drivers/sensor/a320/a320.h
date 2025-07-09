// drivers/zephyr/sensor/a320/a320.h
#pragma once
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>

/* 寄存器定义 */
#define A320_REG_MOTION        0x02
#define A320_REG_DELTA_X       0x03
#define A320_REG_DELTA_Y       0x04
#define A320_REG_CONFIG        0x11
#define A320_BIT_MOTION_MOT    BIT(7)
#define A320_BIT_MOTION_OVF    BIT(4)

/* 设备数据结构 */
struct a320_data {
    int16_t delta_x; 
    int16_t delta_y;
    struct k_sem data_sem;
    struct gpio_callback motion_cb;
};

/* 设备配置结构 */
struct a320_config {
    struct i2c_dt_spec bus;
    struct gpio_dt_spec nrst_gpio;
    struct gpio_dt_spec motion_gpio;
    struct k_mutex i2c_mutex;
};

/* 驱动API实现 */
int a320_trigger_set(const struct device *dev, 
                     const struct sensor_trigger *trig,
                     sensor_trigger_handler_t handler);
