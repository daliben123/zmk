#define DT_DRV_COMPAT avago_a320

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include "a320.h"

LOG_MODULE_REGISTER(A320, CONFIG_SENSOR_LOG_LEVEL);

struct a320_data {
    int16_t delta_x;
    int16_t delta_y;
    uint8_t status;
};

struct a320_config {
    struct i2c_dt_spec bus;
    uint8_t resolution; // 从设备树获取
};

static int a320_read_reg(const struct device *dev, uint8_t reg_addr) {
    const struct a320_config *cfg = dev->config;
    uint8_t val;
    int ret = i2c_reg_read_byte_dt(&cfg->bus, reg_addr, &val);
    return (ret == 0) ? val : ret; // 错误时返回负的错误码
}

static int a320_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct a320_data *data = dev->data;
    int motion_reg = a320_read_reg(dev, Motion);
    
    if (motion_reg < 0) {
        LOG_ERR("Failed to read status");
        return motion_reg;
    }
    data->status = (uint8_t)motion_reg;

    if ((data->status & BIT_MOTION_MOT) && !(data->status & BIT_MOTION_OVF)) {
        data->delta_x = (int16_t)a320_read_reg(dev, Delta_X);
        data->delta_y = (int16_t)a320_read_reg(dev, Delta_Y);
    } else {
        data->delta_x = 0;
        data->delta_y = 0;
    }
    return 0;
}

static int a320_channel_get(const struct device *dev, enum sensor_channel chan,
                           struct sensor_value *val) {
    struct a320_data *data = dev->data;
    
    switch (chan) {
    case SENSOR_CHAN_POS_DX:
        sensor_value_set(val, data->delta_x);
        break;
    case SENSOR_CHAN_POS_DY:
        sensor_value_set(val, data->delta_y);
        break;
    default:
        return -ENOTSUP;
    }
    return 0;
}

static const struct sensor_driver_api a320_driver_api = {
    .sample_fetch = a320_sample_fetch,
    .channel_get = a320_channel_get,
};

static int a320_init(const struct device *dev) {
    const struct a320_config *cfg = dev->config;
    if (!i2c_is_ready_dt(&cfg->bus)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }
    // 配置传感器
    i2c_reg_write_byte_dt(&cfg->bus, Config_Reg, cfg->resolution);
    LOG_INF("A320 Initialized (Resolution: %d CPI)", cfg->resolution);
    return 0;
}

#define A320_DEFINE(inst)                                                      \
    static struct a320_data a320_data_##inst;                                   \
    static const struct a320_config a320_cfg_##inst = {                         \
        .bus = I2C_DT_SPEC_INST_GET(inst),                                     \
        .resolution = DT_INST_PROP(inst, resolution), /* 从设备树获取 */        \
    };                                                                          \
    DEVICE_DT_INST_DEFINE(inst, a320_init, NULL, &a320_data_##inst,             \
                          &a320_cfg_##inst, POST_KERNEL,                        \
                          CONFIG_SENSOR_INIT_PRIORITY, &a320_driver_api);

DT_INST_FOREACH_STATUS_OKAY(A320_DEFINE)
