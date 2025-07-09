#define DT_DRV_COMPAT avago_a320

#include "a320.h"
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(A320, CONFIG_SENSOR_LOG_LEVEL);

/* 中断处理函数 */
static void a320_motion_handler(const struct device *dev) {
    struct a320_data *data = dev->data;
    k_sem_give(&data->data_sem); // 触发数据采集
}

/* 寄存器安全访问 */
static int a320_reg_access(const struct device *dev, 
                          uint8_t reg, uint8_t *val, bool write) {
    const struct a320_config *cfg = dev->config;
    k_mutex_lock(&cfg->i2c_mutex, K_FOREVER);
    
    int ret = write ? i2c_reg_write_byte_dt(&cfg->bus, reg, *val)
                   : i2c_reg_read_byte_dt(&cfg->bus, reg, val);
    
    k_mutex_unlock(&cfg->i2c_mutex);
    return ret;
}

/* 数据采集（中断驱动） */
static int a320_sample_fetch(const struct device *dev, 
                            enum sensor_channel chan) {
    struct a320_data *data = dev->data;
    uint8_t status;
    int ret;

    // 等待运动中断触发
    if (k_sem_take(&data->data_sem, K_MSEC(100))) {
        return -ETIMEDOUT;
    }

    // 读取运动状态
    ret = a320_reg_access(dev, A320_REG_MOTION, &status, false);
    if (ret) return ret;

    // 错误检测
    if (status & A320_BIT_MOTION_OVF) {
        LOG_WRN("Data overflow");
        return -EOVERFLOW;
    }

    // 直接存储原始位移计数（无单位转换）
    ret = a320_reg_access(dev, A320_REG_DELTA_X, (uint8_t*)&data->delta_x, false);
    ret |= a320_reg_access(dev, A320_REG_DELTA_Y, (uint8_t*)&data->delta_y, false);
    return ret;
}

/* 返回原始位移计数（无单位转换） */
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

/* 动态配置采样率（100-2000Hz） */
static int a320_attr_set(const struct device *dev,
                        enum sensor_channel chan,
                        enum sensor_attribute attr,
                        const struct sensor_value *val) {
    if (chan != SENSOR_CHAN_ALL) return -ENOTSUP;
    
    if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
        uint16_t rate = sensor_value_to_milli(val);
        uint8_t reg_val = CLAMP(rate / 100, 0x01, 0x14); // 100Hz-2000Hz
        return a320_reg_access(dev, A320_REG_CONFIG, &reg_val, true);
    }
    return -ENOTSUP;
}

/* ================ 关键修改部分 ================ */
/* 硬件复位函数 - 严格100ms时序 */
static void a320_hardware_reset(const struct device *dev) {
    const struct a320_config *cfg = dev->config;
    if (!cfg->reset_gpio.port) return;

    // 1. 配置复位引脚为输出模式（初始高电平）
    gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_INACTIVE); // [9](@ref)
    
    // 2. 拉低复位引脚并维持100ms
    gpio_pin_set_dt(&cfg->reset_gpio, 0);  // Active low
    k_msleep(100);  // 严格满足100ms低电平要求
    
    // 3. 释放复位（拉高）
    gpio_pin_set_dt(&cfg->reset_gpio, 1);
    
    // 4. 等待芯片稳定
    k_msleep(50);  // 复位后稳定时间
}

/* 中断配置 - 下降沿触发 */
static int a320_init_interrupt(const struct device *dev) {
    const struct a320_config *cfg = dev->config;
    struct a320_data *data = dev->data;
    int ret;

    if (!cfg->int_gpio.port) return -ENODEV;

    // 1. 配置中断引脚为输入模式
    ret = gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT); // [9](@ref)
    if (ret) return ret;

    // 2. 设置下降沿触发（从高到低跳变）
    ret = gpio_pin_interrupt_configure_dt(&cfg->int_gpio, 
                                        GPIO_INT_EDGE_FALLING); // [6,9](@ref)
    if (ret) return ret;

    // 3. 注册回调函数
    gpio_init_callback(&data->int_cb, a320_motion_handler, 
                      BIT(cfg->int_gpio.pin));
    
    // 4. 添加回调至GPIO设备
    return gpio_add_callback(cfg->int_gpio.port, &data->int_cb);
}
/* ============================================== */

/* 设备初始化流程 */
static int a320_init(const struct device *dev) {
    const struct a320_config *cfg = dev->config;
    struct a320_data *data = dev->data;

    // 初始化同步对象
    k_sem_init(&data->data_sem, 0, 1);
    k_mutex_init(&cfg->i2c_mutex);

    // 硬件复位（使用修改后的函数）
    a320_hardware_reset(dev);
    
    // 中断初始化（使用修改后的函数）
    int ret = a320_init_interrupt(dev);
    if (ret) return ret;

    // 初始配置（默认500Hz采样率）
    uint8_t init_val = 0x05;
    return a320_reg_access(dev, A320_REG_CONFIG, &init_val, true);
}

/* 驱动API结构 */
static const struct sensor_driver_api a320_driver_api = {
    .sample_fetch = a320_sample_fetch,
    .channel_get = a320_channel_get,
    .attr_set = a320_attr_set,
};

/* 设备实例化宏 */
#define A320_DEFINE(inst)                                                     \
    static struct a320_data a320_data_##inst;                                 \
    static const struct a320_config a320_config_##inst = {                   \
        .bus = I2C_DT_SPEC_INST_GET(inst),                                   \
        .reset_gpio = GPIO_DT_SPEC_INST_GET(inst, reset_gpios), /* 改名 */  \
        .int_gpio = GPIO_DT_SPEC_INST_GET(inst, int_gpios),    /* 改名 */  \
    };                                                                       \
    DEVICE_DT_INST_DEFINE(inst, a320_init, NULL,                             \
                          &a320_data_##inst, &a320_config_##inst,            \
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,           \
                          &a320_driver_api);

DT_INST_FOREACH_STATUS_OKAY(A320_DEFINE)
