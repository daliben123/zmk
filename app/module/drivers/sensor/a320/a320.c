#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include "a320.h"  // 头文件仅声明结构体，无具体定义

LOG_MODULE_REGISTER(a320, CONFIG_SENSOR_LOG_LEVEL);

/* 设备树兼容标识符（唯一在.c文件中定义） */
#define DT_DRV_COMPAT avago_a320

/* 寄存器定义（与头文件一致） */
#define A320_PRODUCT_ID_REG  0x00
#define A320_MOTION_REG      0x02
#define A320_DELTA_X_REG     0x03
#define A320_DELTA_Y_REG     0x04
#define A320_EXPECTED_ID     0x58

/* 状态掩码 */
#define BIT_MOTION_MOT  (1 << 0)  // 运动检测标志[2](@ref)
#define BIT_MOTION_OVF  (1 << 4)  // 数据溢出标志

/* 私有数据结构（仅在.c中定义） */
struct a320_data {
    struct k_mutex mutex;
    int16_t x_position;       // X轴位移（-32768~32767）
    int16_t y_position;       // Y轴位移
    uint8_t last_status;       // 最新状态寄存器值（调试用）
    struct k_work fetch_work; // 异步采样工作队列
};

/* 设备配置结构 */
struct a320_config {
    struct i2c_dt_spec bus;           // I²C总线
    struct gpio_dt_spec nrst_gpio;    // 复位引脚（低电平有效）
};

/* 硬件复位序列 */
static int a320_hardware_reset(const struct device *dev) {
    const struct a320_config *cfg = dev->config;
    int ret;

    /* 复位脉冲（低电平15ms） */
    gpio_pin_set_dt(&cfg->nrst_gpio, 1);
    k_msleep(15);
    gpio_pin_set_dt(&cfg->nrst_gpio, 0);
    k_msleep(5);

    /* 验证产品ID */
    uint8_t id;
    ret = i2c_reg_read_byte_dt(&cfg->bus, A320_PRODUCT_ID_REG, &id);
    if (ret != 0) {
        LOG_ERR("I²C读取失败: %d", ret);
        return ret;
    }

    if (id != A320_EXPECTED_ID) {
        LOG_ERR("无效ID: 0x%02X (预期: 0x%02X)", id, A320_EXPECTED_ID);
        return -ENODEV;
    }

    return 0;
}

/* 数据采样工作函数 */
static void fetch_work_handler(struct k_work *work) {
    struct a320_data *data = CONTAINER_OF(work, struct a320_data, fetch_work);
    const struct device *dev = data->dev;
    const struct a320_config *cfg = dev->config;

    k_mutex_lock(&data->mutex, K_FOREVER);
    
    /* 读取运动状态 */
    i2c_reg_read_byte_dt(&cfg->bus, A320_MOTION_REG, &data->last_status);
    
    /* 仅当检测到运动时更新数据 */
    if (data->last_status & BIT_MOTION_MOT) {
        uint8_t buf[2];
        i2c_burst_read_dt(&cfg->bus, A320_DELTA_X_REG, buf, 2);
        data->x_position = (buf[0] << 8) | buf[1];
        
        i2c_burst_read_dt(&cfg->bus, A320_DELTA_Y_REG, buf, 2);
        data->y_position = (buf[0] << 8) | buf[1];
    }
    k_mutex_unlock(&data->mutex);
}

/* 传感器采样接口 */
static int a320_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct a320_data *data = dev->data;
    
    /* 提交异步采样任务（避免阻塞） */
    k_work_submit(&data->fetch_work);
    return 0;
}

/* 数据通道获取接口 */
static int a320_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val) {
    struct a320_data *data = dev->data;
    int16_t value = 0;

    k_mutex_lock(&data->mutex, K_FOREVER);
    switch (chan) {
    case SENSOR_CHAN_POS_DX:  // X轴位移通道
        value = data->x_position;
        break;
    case SENSOR_CHAN_POS_DY:  // Y轴位移通道
        value = data->y_position;
        break;
    default:
        k_mutex_unlock(&data->mutex);
        return -ENOTSUP;
    }
    k_mutex_unlock(&data->mutex);

    /* 转换为Zephyr标准数据格式 */
    val->val1 = value / 1000;      // 整数部分（毫米）
    val->val2 = (value % 1000) * 1000; // 小数部分（微米）
    return 0;
}

/* 驱动初始化 */
static int a320_init(const struct device *dev) {
    struct a320_data *data = dev->data;
    const struct a320_config *cfg = dev->config;
    int ret;

    data->dev = dev;
    k_mutex_init(&data->mutex);
    k_work_init(&data->fetch_work, fetch_work_handler);

    /* 验证I²C总线 */
    if (!device_is_ready(cfg->bus.bus)) {
        LOG_ERR("I²C设备未就绪");
        return -ENODEV;
    }

    /* 初始化复位引脚 */
    if (cfg->nrst_gpio.port) {
        gpio_pin_configure_dt(&cfg->nrst_gpio, GPIO_OUTPUT_INACTIVE);
    }

    /* 执行硬件复位 */
    ret = a320_hardware_reset(dev);
    if (ret != 0) {
        return ret;
    }

    LOG_INF("A320传感器初始化完成");
    return 0;
}

/* 驱动API实现 */
static const struct sensor_driver_api a320_driver_api = {
    .sample_fetch = a320_sample_fetch,
    .channel_get = a320_channel_get,
};

/* 设备实例化宏 */
#define A320_DEFINE(inst) \
    static struct a320_data a320_data_##inst; \
    static const struct a320_config a320_config_##inst = { \
        .bus = I2C_DT_SPEC_INST_GET(inst), \
        .nrst_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, nrst_gpios, {0}) \
    }; \
    SENSOR_DEVICE_DT_INST_DEFINE( \
        inst, \
        a320_init, \
        NULL, \
        &a320_data_##inst, \
        &a320_config_##inst, \
        POST_KERNEL, \
        CONFIG_SENSOR_INIT_PRIORITY, \
        &a320_driver_api \
    );

/* 生成设备实例 */
DT_INST_FOREACH_STATUS_OKAY(A320_DEFINE)
