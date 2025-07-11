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

/* 寄存器读取函数（带重试机制） */
static int a320_read_reg(const struct device *dev, uint8_t reg_addr, uint8_t *value) {
    const struct a320_config *cfg = dev->config;
    int retry = 3, ret;
    while (retry--) {
        ret = i2c_reg_read_byte_dt(&cfg->bus, reg_addr, value);
        if (ret == 0) break;
        k_msleep(1); // 延迟后重试
    }
    if (ret != 0) {
        LOG_ERR("读取寄存器0x%x失败: %d", reg_addr, ret);
        return -EIO;
    }
    return 0;
}

/* 硬件复位序列（关键优化点） */
static int a320_hardware_reset(const struct device *dev) {
    const struct a320_config *cfg = dev->config;
    struct a320_data *data = dev->data;
    int ret = 0;

    k_mutex_lock(&data->mutex, K_FOREVER); // 多线程保护

    /* 1. 断电（若支持） */
    if (cfg->shutdown_gpio.port) {
        gpio_pin_set_dt(&cfg->shutdown_gpio, 0); // 低电平断电
        k_busy_wait(300); // 300μs电容放电
    }

    /* 2. 复位脉冲（10ms低电平） */
    if (cfg->nrst_gpio.port) {
        gpio_pin_set_dt(&cfg->nrst_gpio, 0); // 拉低复位
        k_msleep(10); // 精确满足最小值（手册≥4μs）
        gpio_pin_set_dt(&cfg->nrst_gpio, 1); // 释放复位
    }

    /* 3. 重新上电 */
    if (cfg->shutdown_gpio.port) {
        gpio_pin_set_dt(&cfg->shutdown_gpio, 1);
    }

    /* 4. 轮询设备ID确认复位成功 */
    uint8_t id;
    int retry = 5;
    do {
        k_msleep(5); // 等待传感器稳定
        ret = a320_read_reg(dev, A320_PRODUCT_ID_REG, &id);
    } while (ret != 0 && id != A320_EXPECTED_ID && --retry > 0);

    k_mutex_unlock(&data->mutex);
    return (retry > 0) ? 0 : -EIO; // 返回状态
}

/* 批量数据采集（与您提供代码完全一致） */
static int a320_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct a320_data *data = dev->data;
    const struct a320_config *cfg = dev->config;
    uint8_t buf[3]; // Motion + Delta_X + Delta_Y

    // 加锁保护多线程访问
    k_mutex_lock(&data->mutex, K_FOREVER);
    
    // 突发读取3个寄存器（减少I2C事务）
    int ret = i2c_burst_read_dt(&cfg->bus, Motion, buf, sizeof(buf));
    if (ret != 0) {
        LOG_ERR("批量读取失败: %d", ret);
        k_mutex_unlock(&data->mutex);
        return ret;
    }

    // 解析运动状态和溢出标志
    uint8_t motion_status = buf[0];
    if ((motion_status & BIT_MOTION_MOT) && !(motion_status & BIT_MOTION_OVF)) {
        // 转换8位有符号数为16位（直接缓存原始数据）
        data->x_position = (int8_t)buf[1];
        data->y_position = (int8_t)buf[2];
    } else {
        data->x_position = 0;
        data->y_position = 0;
    }

    k_mutex_unlock(&data->mutex);
    return 0;
}

/* 通道数据获取（与您提供代码完全一致） */
static int a320_channel_get(const struct device *dev, enum sensor_channel chan,
                            struct sensor_value *val) {
    struct a320_data *data = dev->data;

    switch (chan) {
    case SENSOR_CHAN_POS_DX:
        val->val1 = data->x_position;
        val->val2 = 0;
        return 0;
    case SENSOR_CHAN_POS_DY:
        val->val1 = data->y_position;
        val->val2 = 0;
        return 0;
    default:
        return -ENOTSUP;
    }
}

/* 驱动API结构 */
static const struct sensor_driver_api a320_driver_api = {
    .sample_fetch = a320_sample_fetch,
    .channel_get = a320_channel_get,
};

/* 设备初始化（重构GPIO检查） */
static int a320_init(const struct device *dev) {
    const struct a320_config *cfg = dev->config;
    struct a320_data *data = dev->data;
    k_mutex_init(&data->mutex);

    /* 1. I²C总线检查 */
    if (!device_is_ready(cfg->bus.bus)) {
        LOG_ERR("I²C控制器未就绪");
        return -ENODEV;
    }

    /* 2. 动态配置GPIO（替代预处理宏） */
    if (cfg->nrst_gpio.port) {
        if (!gpio_is_ready_dt(&cfg->nrst_gpio)) {
            LOG_ERR("复位GPIO设备未就绪");
            return -ENODEV;
        }
        gpio_pin_configure_dt(&cfg->nrst_gpio, GPIO_OUTPUT_INACTIVE);
    }

    if (cfg->shutdown_gpio.port) {
        if (!gpio_is_ready_dt(&cfg->shutdown_gpio)) {
            LOG_ERR("电源控制GPIO未就绪");
            return -ENODEV;
        }
        gpio_pin_configure_dt(&cfg->shutdown_gpio, GPIO_OUTPUT_INACTIVE);
    }

    /* 3. 执行复位并验证 */
    if (a320_hardware_reset(dev) != 0) {
        LOG_ERR("传感器初始化失败");
        return -EIO;
    }

    LOG_INF("A320传感器就绪");
    return 0;
}

/* 设备实例定义（关键修复） */
#define A320_DEFINE(inst)                                                  \
    static struct a320_data a320_data_##inst;                              \
    static const struct a320_config a320_cfg_##inst = {                    \
        .bus = I2C_DT_SPEC_INST_GET(inst),                                 \
        .nrst_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, nrst_gpios, {0}),     \
        .motion_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, motion_gpios, {0}),  \
        .shutdown_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, shutdown_gpios, {0}) \
    };                                                                     \
    DEVICE_DT_INST_DEFINE(inst, a320_init, NULL,                           \
                          &a320_data_##inst, &a320_cfg_##inst,             \
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,        \
                          &a320_driver_api);

DT_INST_FOREACH_STATUS_OKAY(A320_DEFINE)
