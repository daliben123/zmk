#define DT_DRV_COMPAT avago_a320
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include "a320.h"

LOG_MODULE_REGISTER(A320, CONFIG_SENSOR_LOG_LEVEL);

/* 寄存器读取函数 */
static int a320_read_reg(const struct device *dev, uint8_t reg_addr, uint8_t *value) {
    const struct a320_config *cfg = dev->config;
    return i2c_reg_read_byte_dt(&cfg->bus, reg_addr, value);
}

/* 数据采集函数 */
static int a320_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct a320_data *data = dev->data;
    const struct a320_config *cfg = dev->config;
    uint8_t motion_val = 0;

    /* 加锁保护多线程访问 */
    k_mutex_lock(&cfg->polling_mutex, K_FOREVER);

    /* 读取运动状态寄存器 */
    if (a320_read_reg(dev, A320_REG_MOTION, &motion_val) < 0) {
        k_mutex_unlock(&cfg->polling_mutex);
        return -EIO;
    }

    /* 检测有效运动且无溢出 */
    if ((motion_val & BIT_MOTION_MOT) && !(motion_val & BIT_MOTION_OVF)) {
        uint8_t delta_x_low, delta_y_low;
        a320_read_reg(dev, A320_REG_DELTA_X, &delta_x_low);
        a320_read_reg(dev, A320_REG_DELTA_Y, &delta_y_low);
        data->delta_x = (int16_t)delta_x_low;
        data->delta_y = (int16_t)delta_y_low;
    } else {
        data->delta_x = 0;
        data->delta_y = 0;
    }
    data->motion_status = motion_val;
    k_mutex_unlock(&cfg->polling_mutex);
    return 0;
}

/* 数据通道获取函数 */
static int a320_channel_get(const struct device *dev, enum sensor_channel chan,
                            struct sensor_value *val) {
    struct a320_data *data = dev->data;

    switch (chan) {
    case SENSOR_CHAN_POS_DX:
        val->val1 = data->delta_x;
        break;
    case SENSOR_CHAN_POS_DY:
        val->val1 = data->delta_y;
        break;
    case SENSOR_CHAN_MOTION:
        val->val1 = !!(data->motion_status & BIT_MOTION_MOT);
        break;
    default:
        return -ENOTSUP;
    }
    return 0;
}

/* 驱动API结构体 */
static const struct sensor_driver_api a320_driver_api = {
    .sample_fetch = a320_sample_fetch,
    .channel_get = a320_channel_get,
};

/* 设备初始化 */
static int a320_init(const struct device *dev) {
    const struct a320_config *cfg = dev->config;
    struct a320_data *data = dev->data;

    /* 检查I²C总线 */
    if (!device_is_ready(cfg->bus.bus)) {
        LOG_ERR("I²C总线未就绪");
        return -ENODEV;
    }

    /* 初始化互斥锁 */
    k_mutex_init(&cfg->polling_mutex);

    /* 初始化GPIO（若设备树配置） */
#if DT_INST_NODE_HAS_PROP(0, nrst_gpios)
    if (gpio_is_ready_dt(&cfg->nrst_gpio)) {
        gpio_pin_configure_dt(&cfg->nrst_gpio, GPIO_OUTPUT_ACTIVE);
        k_msleep(10); // 复位脉冲
        gpio_pin_set_dt(&cfg->nrst_gpio, 0);
    }
#endif

    LOG_INF("A320传感器初始化完成");
    return 0;
}

/* 设备树实例化 */
#define A320_DEFINE(inst)                                                      \
    static struct a320_data a320_data_##inst;                                  \
    static const struct a320_config a320_cfg_##inst = {                        \
        .bus = I2C_DT_SPEC_INST_GET(inst),                                     \
        DT_INST_GPIO_SPEC_IF_ENABLED(inst, nrst_gpios),                        \
        DT_INST_GPIO_SPEC_IF_ENABLED(inst, motion_gpios),                      \
    };                                                                         \
    DEVICE_DT_INST_DEFINE(inst, a320_init, NULL,                               \
                          &a320_data_##inst, &a320_cfg_##inst,                 \
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,            \
                          &a320_driver_api);

DT_INST_FOREACH_STATUS_OKAY(A320_DEFINE)
