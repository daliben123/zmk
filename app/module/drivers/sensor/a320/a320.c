/* a320.c - 传感器驱动实现 */
#define DT_DRV_COMPAT avago_a320

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include "a320.h"

LOG_MODULE_REGISTER(A320, CONFIG_SENSOR_LOG_LEVEL);

/* 寄存器读取函数 */
static int a320_read_reg(const struct device *dev, uint8_t reg_addr) {
    const struct a320_config *cfg = dev->config;
    uint8_t val;
    int ret = i2c_reg_read_byte_dt(&cfg->bus, reg_addr, &val);
    return (ret == 0) ? val : ret; // 返回寄存器值或错误码
}

/* 寄存器写入函数 */
static int a320_write_reg(const struct device *dev, uint8_t reg_addr, uint8_t value) {
    const struct a320_config *cfg = dev->config;
    return i2c_reg_write_byte_dt(&cfg->bus, reg_addr, value);
}

/* 传感器复位函数 */
static void a320_hw_reset(const struct device *dev) {
    const struct a320_config *cfg = dev->config;
    
#if DT_INST_NODE_HAS_PROP(0, nrst_gpios)
    if (cfg->nrst_gpio.port) {
        gpio_pin_set_dt(&cfg->nrst_gpio, 1);
        k_msleep(5);
        gpio_pin_set_dt(&cfg->nrst_gpio, 0);
        k_msleep(50); // 复位后等待稳定
    } else 
#endif
    {
        // 软件复位
        a320_write_reg(dev, SOFT_RESET_REG, 0x5A);
        k_msleep(50);
    }
}

/* 数据采集函数 */
static int a320_sample_fetch(const struct device *dev, 
                            enum sensor_channel chan) {
    struct a320_data *data = dev->data;
    
    // 加锁保证数据一致性
    k_mutex_lock(&data->data_mutex, K_FOREVER);
    
    // 读取运动状态寄存器
    int motion_reg = a320_read_reg(dev, MOTION_REG);
    if (motion_reg < 0) {
        k_mutex_unlock(&data->data_mutex);
        LOG_ERR("Motion reg read error: %d", motion_reg);
        return motion_reg;
    }
    
    data->status = (uint8_t)motion_reg;
    
    // 检查有效运动
    if ((data->status & BIT_MOTION_MOT) && 
        !(data->status & BIT_MOTION_OVF)) {
        // 读取位移数据
        data->delta_x = (int16_t)a320_read_reg(dev, DELTA_X_REG);
        data->delta_y = (int16_t)a320_read_reg(dev, DELTA_Y_REG);
    } else {
        data->delta_x = 0;
        data->delta_y = 0;
    }
    
    k_mutex_unlock(&data->data_mutex);
    return 0;
}

/* 数据获取函数 */
static int a320_channel_get(const struct device *dev,
                          enum sensor_channel chan,
                          struct sensor_value *val) {
    struct a320_data *data = dev->data;
    int ret = 0;
    
    k_mutex_lock(&data->data_mutex, K_FOREVER);
    
    switch (chan) {
    case SENSOR_CHAN_POS_DX:
        sensor_value_set(val, data->delta_x);
        break;
    case SENSOR_CHAN_POS_DY:
        sensor_value_set(val, data->delta_y);
        break;
    default:
        ret = -ENOTSUP; // 不支持的通道
    }
    
    k_mutex_unlock(&data->data_mutex);
    return ret;
}

/* 驱动API */
static const struct sensor_driver_api a320_driver_api = {
    .sample_fetch = a320_sample_fetch,
    .channel_get = a320_channel_get,
};

/* 设备初始化 */
static int a320_init(const struct device *dev) {
    const struct a320_config *cfg = dev->config;
    struct a320_data *data = dev->data;
    
    // 初始化互斥锁
    k_mutex_init(&data->data_mutex);
    
    // 检查I2C总线
    if (!i2c_is_ready_dt(&cfg->bus)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }
    
    // 初始化GPIO
#if DT_INST_NODE_HAS_PROP(0, nrst_gpios)
    if (cfg->nrst_gpio.port) {
        if (!gpio_is_ready_dt(&cfg->nrst_gpio)) {
            LOG_ERR("Reset GPIO not ready");
            return -EIO;
        }
        gpio_pin_configure_dt(&cfg->nrst_gpio, GPIO_OUTPUT_INACTIVE);
    }
#endif

#if DT_INST_NODE_HAS_PROP(0, motion_gpios)
    if (cfg->motion_gpio.port) {
        if (!gpio_is_ready_dt(&cfg->motion_gpio)) {
            LOG_ERR("Motion GPIO not ready");
            return -EIO;
        }
        gpio_pin_configure_dt(&cfg->motion_gpio, GPIO_INPUT);
    }
#endif

    // 硬件复位
    a320_hw_reset(dev);
    
    // 配置分辨率
    int ret = a320_write_reg(dev, RESOLUTION_REG, cfg->resolution);
    if (ret < 0) {
        LOG_ERR("Resolution config failed: %d", ret);
        return ret;
    }
    
    LOG_INF("A320 initialized (Resolution: %d CPI)", cfg->resolution);
    return 0;
}

/* 设备实例化宏 */
#define A320_DEFINE(inst)                                                     \
    static struct a320_data a320_data_##inst;                                 \
    static const struct a320_config a320_cfg_##inst = {                       \
        .bus = I2C_DT_SPEC_INST_GET(inst),                                    \
        .resolution = DT_INST_PROP(inst, resolution),                         \
        DT_INST_GPIO_SPEC_IF_EXISTS(inst, nrst_gpios),                        \
        DT_INST_GPIO_SPEC_IF_EXISTS(inst, motion_gpios)                       \
    };                                                                         \
    DEVICE_DT_INST_DEFINE(inst, a320_init, NULL,                               \
                          &a320_data_##inst, &a320_cfg_##inst,                 \
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,            \
                          &a320_driver_api);

/* 创建所有设备树实例 */
DT_INST_FOREACH_STATUS_OKAY(A320_DEFINE)
