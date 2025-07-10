#pragma once
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

struct a320_data {
    struct k_mutex mutex;
    int16_t x_position;
    int16_t y_position;
};

struct a320_config {
    struct i2c_dt_spec bus;
#if DT_INST_NODE_HAS_PROP(0, nrst_gpios)
    struct gpio_dt_spec nrst_gpio;
#endif
#if DT_INST_NODE_HAS_PROP(0, motion_gpios)
    struct gpio_dt_spec motion_gpio;
#endif
#if DT_INST_NODE_HAS_PROP(0, shutdown_gpios)
    struct gpio_dt_spec shutdown_gpio;
#endif
};

// A320 Register Defines
#define Motion 0x02
#define Delta_X 0x03
#define Delta_Y 0x04

/* Detection */
#define BIT_MOTION_MOT (1 << 7)
#define BIT_MOTION_OVF (1 << 4)
