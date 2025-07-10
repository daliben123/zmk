#define DT_DRV_COMPAT avago_a320

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/logging/log.h>

#include "a320.h"

LOG_MODULE_REGISTER(A320, CONFIG_SENSOR_LOG_LEVEL);

static int a320_read_reg(const struct device *dev, uint8_t reg_addr, uint8_t *value) {
    const struct a320_config *cfg = dev->config;
    int ret = i2c_reg_read_byte_dt(&cfg->bus, reg_addr, value);
    if (ret == 0) {
        return 0;
    }
    LOG_ERR("Failed to read register 0x%x: %d", reg_addr, ret);
    return ret;
}

static int a320_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct a320_data *data = dev->data;
    const struct a320_config *cfg = dev->config;
    uint8_t buf[3]; // 用于读取Motion, Delta_X, Delta_Y三个寄存器（Motion一个字节，Delta_X和Delta_Y各一个字节）

    if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_POS_DX) {
        return -ENOTSUP;
    }

    k_mutex_lock(&data->mutex, K_FOREVER);

    // 使用突发读取：从Motion寄存器开始，连续读取3个字节（Motion, Delta_X, Delta_Y）
    int ret = i2c_burst_read_dt(&cfg->bus, Motion, buf, sizeof(buf));
    if (ret != 0) {
        LOG_ERR("Burst read failed: %d", ret);
        k_mutex_unlock(&data->mutex);
        return ret;
    }

    // 检查运动标志和溢出标志
    uint8_t ifmotion = buf[0];
    if ((ifmotion & BIT_MOTION_MOT) && !(ifmotion & BIT_MOTION_OVF)) {
        // 读取到的Delta_X和Delta_Y是8位有符号数，转换为16位有符号数
        data->x_position = (int8_t)buf[1];
        data->y_position = (int8_t)buf[2];
    } else {
        data->x_position = 0;
        data->y_position = 0;
    }

    k_mutex_unlock(&data->mutex);
    return 0;
}

static int a320_channel_get(const struct device *dev, enum sensor_channel chan,
                            struct sensor_value *val) {
    struct a320_data *data = dev->data;
    if (chan == SENSOR_CHAN_POS_DX) {
        val->val1 = data->x_position;
        val->val2 = 0;
        return 0;
    } else if (chan == SENSOR_CHAN_POS_DY) {
        val->val1 = data->y_position;
        val->val2 = 0;
        return 0;
    }
    return -ENOTSUP;
}

static const struct sensor_driver_api a320_driver_api = {
    .sample_fetch = a320_sample_fetch,
    .channel_get = a320_channel_get,
};

static int a320_hardware_reset(const struct device *dev) {
    const struct a320_config *cfg = dev->config;
    int ret = 0;

#if DT_INST_NODE_HAS_PROP(0, shutdown_gpios)
    // 确保关机引脚可用
    if (gpio_is_ready_dt(&cfg->shutdown_gpio)) {
        // 关闭传感器电源（低电平有效）
        ret = gpio_pin_set_dt(&cfg->shutdown_gpio, 1); // 先拉高，确保可以控制
        k_msleep(1);
        ret = gpio_pin_set_dt(&cfg->shutdown_gpio, 0); // 拉低，关闭电源
        if (ret < 0) {
            LOG_ERR("Failed to set shutdown pin to low");
            return ret;
        }
        k_msleep(10); // 等待完全放电
    }
#endif

#if DT_INST_NODE_HAS_PROP(0, nrst_gpios)
    // 确保复位引脚可用
    if (gpio_is_ready_dt(&cfg->nrst_gpio)) {
        // 在电源关闭期间，将复位引脚拉低
        ret = gpio_pin_set_dt(&cfg->nrst_gpio, 0);
        if (ret < 0) {
            LOG_ERR("Failed to set reset pin to low");
            return ret;
        }
    }
#endif

#if DT_INST_NODE_HAS_PROP(0, shutdown_gpios)
    // 重新打开电源
    if (gpio_is_ready_dt(&cfg->shutdown_gpio)) {
        ret = gpio_pin_set_dt(&cfg->shutdown_gpio, 1); // 拉高，打开电源
        if (ret < 0) {
            LOG_ERR("Failed to set shutdown pin to high");
            return ret;
        }
        k_msleep(1); // 电源稳定时间
    }
#endif

#if DT_INST_NODE_HAS_PROP(0, nrst_gpios)
    if (gpio_is_ready_dt(&cfg->nrst_gpio)) {
        // 保持复位引脚低电平一段时间（复位脉冲）
        k_msleep(10); // 复位脉冲宽度10ms
        // 释放复位引脚（拉高）
        ret = gpio_pin_set_dt(&cfg->nrst_gpio, 1);
        if (ret < 0) {
            LOG_ERR("Failed to set reset pin to high");
            return ret;
        }
    }
#endif

    // 等待传感器启动
    k_msleep(50);
    return 0;
}

static int a320_init(const struct device *dev) {
    const struct a320_config *cfg = dev->config;
    struct a320_data *data = dev->data;

    if (!device_is_ready(cfg->bus.bus)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    k_mutex_init(&data->mutex);

#if DT_INST_NODE_HAS_PROP(0, nrst_gpios)
    if (cfg->nrst_gpio.port) {
        if (!gpio_is_ready_dt(&cfg->nrst_gpio)) {
            LOG_ERR("Reset GPIO device not ready");
            return -ENODEV;
        }
        int ret = gpio_pin_configure_dt(&cfg->nrst_gpio, GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure reset GPIO");
            return ret;
        }
    }
#endif

#if DT_INST_NODE_HAS_PROP(0, shutdown_gpios)
    if (cfg->shutdown_gpio.port) {
        if (!gpio_is_ready_dt(&cfg->shutdown_gpio)) {
            LOG_ERR("Shutdown GPIO device not ready");
            return -ENODEV;
        }
        int ret = gpio_pin_configure_dt(&cfg->shutdown_gpio, GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure shutdown GPIO");
            return ret;
        }
        // 初始状态：打开电源（高电平）
        ret = gpio_pin_set_dt(&cfg->shutdown_gpio, 1);
        if (ret < 0) {
            LOG_ERR("Failed to set shutdown pin to high");
            return ret;
        }
    }
#endif

    // 执行硬件复位
    if (a320_hardware_reset(dev) != 0) {
        LOG_ERR("Hardware reset failed");
        return -EIO;
    }

    LOG_DBG("A320 initialized");
    return 0;
}

#define A320_DEFINE(inst)                                                                          \
    static struct a320_data a320_data_;                                                      \
    static const struct a320_config a320_cfg_ = {                                            \
        .bus = I2C_DT_SPEC_INST_GET(inst),                                                         \
        DT_INST_GPIO_CFG_GET(inst, nrst_gpios),                                                    \
        DT_INST_GPIO_CFG_GET(inst, motion_gpios),                                                  \
        DT_INST_GPIO_CFG_GET(inst, shutdown_gpios),                                                \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(inst, a320_init, NULL, &a320_data_, &a320_cfg_,              \
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &a320_driver_api);

DT_INST_FOREACH_STATUS_OKAY(A320_DEFINE)
