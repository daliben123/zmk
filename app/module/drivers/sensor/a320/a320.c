#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include "a320.h"

/* 定义设备树兼容标识符 */
#define DT_DRV_COMPAT avago_a320

LOG_MODULE_REGISTER(a320, CONFIG_SENSOR_LOG_LEVEL);

/* 寄存器地址常量 */
#define A320_PRODUCT_ID_REG   0x00
#define A320_MOTION_REG       0x02
#define A320_DELTA_X_REG      0x03
#define A320_DELTA_Y_REG      0x04
#define A320_EXPECTED_ID      0x58

/* 状态位掩码 */
#define BIT_MOTION_MOT        (1 << 0)  // 运动检测标志
#define BIT_MOTION_OVF        (1 << 4)  // 数据溢出标志

/* 驱动私有数据结构 */
struct a320_data {
    struct k_mutex mutex;
    int16_t x_position;      // X轴位移缓存
    int16_t y_position;      // Y轴位移缓存
    uint8_t last_status;      // 最近状态寄存器值
    struct gpio_callback motion_cb;  // 运动中断回调
};

/* 设备配置结构 */
struct a320_config {
    struct i2c_dt_spec bus;           // I²C总线规格
    struct gpio_dt_spec nrst_gpio;    // 复位引脚
    struct gpio_dt_spec motion_gpio;  // 运动中断引脚
    struct gpio_dt_spec shutdown_gpio;// 电源控制引脚
};

/* 硬件复位函数 */
static int a320_hardware_reset(const struct device *dev) {
    const struct a320_config *cfg = dev->config;
    int ret;

    /* 拉低复位引脚至少10ms */
    gpio_pin_set_dt(&cfg->nrst_gpio, 1);
    k_msleep(15);
    gpio_pin_set_dt(&cfg->nrst_gpio, 0);
    k_msleep(5);  // 等待传感器稳定

    /* 验证传感器ID */
    uint8_t id;
    ret = i2c_reg_read_byte_dt(&cfg->bus, A320_PRODUCT_ID_REG, &id);
    if (ret != 0) {
        LOG_ERR("I²C读取失败: %d", ret);
        return ret;
    }

    if (id != A320_EXPECTED_ID) {
        LOG_ERR("无效传感器ID: 0x%02X (预期: 0x%02X)", id, A320_EXPECTED_ID);
        return -ENODEV;
    }

    return 0;
}

/* 运动中断回调 */
static void motion_interrupt_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    struct a320_data *data = CONTAINER_OF(cb, struct a320_data, motion_cb);
    k_mutex_lock(&data->mutex, K_FOREVER);
    
    /* 读取运动状态寄存器 */
    i2c_reg_read_byte_dt(&data->bus, A320_MOTION_REG, &data->last_status);
    
    /* 仅当检测到有效运动时更新数据 */
    if (data->last_status & BIT_MOTION_MOT) {
        uint8_t buf[2];
        i2c_burst_read_dt(&data->bus, A320_DELTA_X_REG, buf, 2);
        data->x_position = (buf[0] << 8) | buf[1];
        
        i2c_burst_read_dt(&data->bus, A320_DELTA_Y_REG, buf, 2);
        data->y_position = (buf[0] << 8) | buf[1];
    }
    k_mutex_unlock(&data->mutex);
}

/* 初始化中断引脚 */
static int init_motion_interrupt(const struct device *dev) {
    const struct a320_config *cfg = dev->config;
    struct a320_data *data = dev->data;
    int ret;

    if (!gpio_is_ready_dt(&cfg->motion_gpio)) {
        LOG_ERR("GPIO设备未就绪");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&cfg->motion_gpio, GPIO_INPUT);
    if (ret != 0) {
        LOG_ERR("GPIO配置失败: %d", ret);
        return ret;
    }

    gpio_init_callback(&data->motion_cb, motion_interrupt_handler, BIT(cfg->motion_gpio.pin));
    ret = gpio_add_callback(cfg->motion_gpio.port, &data->motion_cb);
    if (ret != 0) {
        LOG_ERR("回调注册失败: %d", ret);
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&cfg->motion_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    return ret;
}

/* 传感器采样实现 */
static int a320_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    // 中断驱动模式下无需主动采样
    return 0;
}

/* 数据获取接口 */
static int a320_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val) {
    struct a320_data *data = dev->data;
    
    k_mutex_lock(&data->mutex, K_FOREVER);
    switch (chan) {
    case SENSOR_CHAN_POS_DX:
        val->val1 = data->x_position;
        val->val2 = 0;
        break;
    case SENSOR_CHAN_POS_DY:
        val->val1 = data->y_position;
        val->val2 = 0;
        break;
    default:
        k_mutex_unlock(&data->mutex);
        return -ENOTSUP;
    }
    k_mutex_unlock(&data->mutex);
    return 0;
}

/* 驱动初始化函数 */
static int a320_init(const struct device *dev) {
    struct a320_data *data = dev->data;
    const struct a320_config *cfg = dev->config;
    int ret;

    k_mutex_init(&data->mutex);

    /* 验证I²C总线 */
    if (!device_is_ready(cfg->bus.bus)) {
        LOG_ERR("I²C设备未就绪");
        return -ENODEV;
    }

    /* 配置GPIO引脚 */
    if (gpio_is_ready_dt(&cfg->nrst_gpio)) {
        gpio_pin_configure_dt(&cfg->nrst_gpio, GPIO_OUTPUT_INACTIVE);
    }

    /* 执行硬件复位 */
    ret = a320_hardware_reset(dev);
    if (ret != 0) {
        return ret;
    }

    /* 初始化运动中断 */
    if (cfg->motion_gpio.port) {
        ret = init_motion_interrupt(dev);
        if (ret != 0) {
            LOG_WRN("中断初始化失败(%d), 使用轮询模式", ret);
        }
    }

    LOG_INF("A320传感器初始化完成");
    return 0;
}

/* 传感器驱动API实现 */
static const struct sensor_driver_api a320_driver_api = {
    .sample_fetch = a320_sample_fetch,
    .channel_get = a320_channel_get,
};

/* 设备实例化宏 */
#define A320_DEFINE(inst) \
    static struct a320_data a320_data_##inst; \
    static const struct a320_config a320_config_##inst = { \
        .bus = I2C_DT_SPEC_INST_GET(inst), \
        .nrst_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, nrst_gpios, {0}), \
        .motion_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, motion_gpios, {0}), \
        .shutdown_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, shutdown_gpios, {0}) \
    }; \
    SENSOR_DEVICE_DT_INST_DEFINE(inst, \
                  a320_init, \
                  NULL, \
                  &a320_data_##inst, \
                  &a320_config_##inst, \
                  POST_KERNEL, \
                  CONFIG_SENSOR_INIT_PRIORITY, \
                  &a320_driver_api);

/* 生成设备实例 */
DT_INST_FOREACH_STATUS_OKAY(A320_DEFINE)
