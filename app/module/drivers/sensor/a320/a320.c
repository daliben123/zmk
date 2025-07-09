#define DT_DRV_COMPAT avago_a320

#include "a320.h"
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>
LOG_MODULE_REGISTER(A320, CONFIG_SENSOR_LOG_LEVEL);

/* ================ 关键数据结构 ================ */
struct a320_data {
    struct k_sem data_sem;          // 数据同步信号量
    struct k_mutex i2c_mutex;       // I2C总线互斥锁（移至RAM区）
    int8_t delta_x;                 // X轴原生位移计数
    int8_t delta_y;                 // Y轴原生位移计数
    struct gpio_callback int_cb;    // 中断回调结构
};

struct a320_config {
    struct i2c_dt_spec bus;         // I2C总线规格
    struct gpio_dt_spec reset_gpio; // 复位GPIO（设备树定义）
    struct gpio_dt_spec int_gpio;   // 中断GPIO（设备树定义）
};

/* ================ 中断处理函数（修正签名） ================ */
static void a320_motion_handler(const struct device *dev,
                                struct gpio_callback *cb,
                                uint32_t pins) {
    struct a320_data *data = dev->data;
    k_sem_give(&data->data_sem); // 触发数据采集[7](@ref)
}

/* ================ 寄存器安全访问 ================ */
static int a320_reg_access(const struct device *dev, 
                          uint8_t reg, uint8_t *val, bool write) {
    const struct a320_config *cfg = dev->config;
    struct a320_data *data = dev->data;
    
    k_mutex_lock(&data->i2c_mutex, K_FOREVER); // 使用数据区互斥锁
    
    int ret = write ? i2c_reg_write_byte_dt(&cfg->bus, reg, *val)
                   : i2c_reg_read_byte_dt(&cfg->bus, reg, val);
    
    k_mutex_unlock(&data->i2c_mutex);
    if (ret) {
        LOG_ERR("I2C access error: %d", ret);
    }
    return ret;
}

/* ================ 数据采集（增强错误处理） ================ */
static int a320_sample_fetch(const struct device *dev, 
                            enum sensor_channel chan) {
    struct a320_data *data = dev->data;
    uint8_t status;
    int ret;

    // 等待运动中断触发（超时100ms）
    if (k_sem_take(&data->data_sem, K_MSEC(100))) {
        LOG_WRN("Data wait timeout");
        return -ETIMEDOUT;
    }

    // 读取运动状态寄存器
    ret = a320_reg_access(dev, A320_REG_STATUS, &status, false);
    if (ret) return ret;

    // 检测数据溢出
    if (status & A320_STATUS_OVF) {
        LOG_ERR("Sensor data overflow");
        return -EOVERFLOW;
    }

    // 读取原始位移数据
    ret = a320_reg_access(dev, A320_REG_DELTA_X, (uint8_t*)&data->delta_x, false);
    ret |= a320_reg_access(dev, A320_REG_DELTA_Y, (uint8_t*)&data->delta_y, false);
    
    return ret ? -EIO : 0;
}

/* ================ 通道数据获取 ================ */
static int a320_channel_get(const struct device *dev,
                           enum sensor_channel chan,
                           struct sensor_value *val) {
    struct a320_data *data = dev->data;
    switch (chan) {
    case SENSOR_CHAN_POS_DX:
        val->val1 = data->delta_x;  // 原生X位移计数
        val->val2 = 0;
        break;
    case SENSOR_CHAN_POS_DY:
        val->val1 = data->delta_y;  // 原生Y位移计数
        val->val2 = 0;
        break;
    default:
        return -ENOTSUP;
    }
    return 0;
}

/* ================ 动态配置采样率（修复计算逻辑） ================ */
static int a320_attr_set(const struct device *dev,
                        enum sensor_channel chan,
                        enum sensor_attribute attr,
                        const struct sensor_value *val) {
    if (chan != SENSOR_CHAN_ALL) return -ENOTSUP;
    
    if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
        // 直接获取Hz整数值（忽略val2分数部分）
        uint32_t freq_hz = val->val1; 
        
        // 验证范围（100-2000Hz）
        if (freq_hz < 100 || freq_hz > 2000) {
            LOG_ERR("Invalid sample rate: %dHz", freq_hz);
            return -EINVAL;
        }
        
        // 计算寄存器值（freq_hz/100 - 1）
        uint8_t reg_val = (freq_hz / 100) - 1; 
        return a320_reg_access(dev, A320_REG_CONFIG, &reg_val, true);
    }
    return -ENOTSUP;
}

/* ================ 硬件复位函数 - 严格100ms时序 ================ */
static void a320_hardware_reset(const struct device *dev) {
    const struct a320_config *cfg = dev->config;
    if (!cfg->reset_gpio.port) {
        LOG_WRN("No reset GPIO defined");
        return;
    }

    // 1. 配置为输出且初始高电平
    gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_INACTIVE);
    
    // 2. 拉低维持100ms±5ms
    gpio_pin_set_dt(&cfg->reset_gpio, 0);
    k_busy_wait(100000); // 精确100ms延时[1](@ref)
    
    // 3. 释放复位（拉高）
    gpio_pin_set_dt(&cfg->reset_gpio, 1);
    
    // 4. 等待芯片稳定（规格书要求）
    k_msleep(50);
}

/* ================ 中断配置 - 下降沿触发（增强GPIO配置） ================ */
static int a320_init_interrupt(const struct device *dev) {
    const struct a320_config *cfg = dev->config;
    struct a320_data *data = dev->data;
    int ret;

    if (!cfg->int_gpio.port) {
        LOG_ERR("No interrupt GPIO defined");
        return -ENODEV;
    }

    // 应用设备树中的GPIO标志（如上拉电阻）
    ret = gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT);
    if (ret) {
        LOG_ERR("GPIO config failed: %d", ret);
        return ret;
    }

    // 设置下降沿触发
    ret = gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_EDGE_FALLING);
    if (ret) return ret;

    // 注册中断回调
    gpio_init_callback(&data->int_cb, a320_motion_handler, BIT(cfg->int_gpio.pin));
    return gpio_add_callback(cfg->int_gpio.port, &data->int_cb);
}

/* ================ 设备初始化流程（增加设备验证） ================ */
static int a320_init(const struct device *dev) {
    const struct a320_config *cfg = dev->config;
    struct a320_data *data = dev->data;
    uint8_t dev_id;
    int ret;

    // 验证I2C总线状态
    if (!device_is_ready(cfg->bus.bus)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    // 初始化同步对象
    k_sem_init(&data->data_sem, 0, 1);
    k_mutex_init(&data->i2c_mutex); // 初始化数据区互斥锁

    // 执行硬件复位
    a320_hardware_reset(dev);
    
    // 验证设备ID
    ret = a320_reg_access(dev, A320_REG_ID, &dev_id, false);
    if (ret || dev_id != A320_DEVICE_ID) {
        LOG_ERR("Device ID mismatch: 0x%02x (expected 0x%02x)", 
                dev_id, A320_DEVICE_ID);
        return -ENODEV;
    }

    // 配置中断
    ret = a320_init_interrupt(dev);
    if (ret) {
        LOG_WRN("Interrupt init failed: %d (fallback to polling)", ret);
        // 可在此添加轮询模式后备方案
    }

    // 设置默认采样率（500Hz）
    uint8_t init_val = 0x05;
    return a320_reg_access(dev, A320_REG_CONFIG, &init_val, true);
}

/* ================ 驱动API结构 ================ */
static const struct sensor_driver_api a320_driver_api = {
    .sample_fetch = a320_sample_fetch,
    .channel_get = a320_channel_get,
    .attr_set = a320_attr_set,
};

/* ================ 设备实例化宏（安全GPIO获取） ================ */
#define A320_DEFINE(inst)                                                     \
    static struct a320_data a320_data_##inst;                                 \
    static const struct a320_config a320_config_##inst = {                    \
        .bus = I2C_DT_SPEC_INST_GET(inst),                                   \
        .reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),      \
        .int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}),         \
    };                                                                        \
    DEVICE_DT_INST_DEFINE(inst, a320_init, NULL,                              \
                          &a320_data_##inst, &a320_config_##inst,             \
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,           \
                          &a320_driver_api);

DT_INST_FOREACH_STATUS_OKAY(A320_DEFINE)
