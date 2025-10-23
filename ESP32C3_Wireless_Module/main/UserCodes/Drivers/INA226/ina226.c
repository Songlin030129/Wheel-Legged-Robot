#include "ina226.h"
#include <math.h>
#include "esp_log.h"

static const char *TAG = "INA226";

ina226_t ina226;
/**
 * @brief 读取 INA226 寄存器
 */
esp_err_t ina226_read_reg(ina226_t *dev, uint8_t reg, uint16_t *out)
{
    uint8_t buf[2] = {0};
    uint8_t reg_addr = reg;
    esp_err_t err = bsp_i2c_master_write_read(dev->i2c_dev_handle, &reg_addr, 1, buf, sizeof(buf), 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK)
        return err;
    *out = ((uint16_t)buf[0] << 8) | buf[1];
    return ESP_OK;
}

/**
 * @brief 写入 INA226 寄存器
 */
esp_err_t ina226_write_reg(ina226_t *dev, uint8_t reg, uint16_t val)
{
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = (val >> 8) & 0xFF;
    buf[2] = val & 0xFF;
    return bsp_i2c_master_write(dev->i2c_dev_handle, buf, sizeof(buf), 1000 / portTICK_PERIOD_MS);
}

/**
 * @brief 通过配置寄存器的 RST 位执行软复位
 */
esp_err_t ina226_reset(ina226_t *dev)
{
    uint16_t cfg = INA226_CONFIG_RST_BIT;
    return ina226_write_reg(dev, INA226_REG_CONFIG, cfg);
}

/**
 * @brief 根据配置结构体生成 CONFIG 寄存器值
 *
 * @param cfg 配置参数
 * @return 需要写入 CONFIG 寄存器的 16 位值
 */
static uint16_t make_config(const ina226_config_t *cfg)
{
    uint16_t v = 0;
    v |= ((uint16_t)(cfg->avg & 0x7) << 9);
    v |= ((uint16_t)(cfg->vbus_ct & 0x7) << 6);
    v |= ((uint16_t)(cfg->vsh_ct & 0x7) << 3);
    v |= ((uint16_t)(cfg->mode & 0x7) << 0);
    return v;
}

/**
 * @brief 初始化 INA226（配置 I2C、写入校准值与工作模式）
 */
esp_err_t ina226_init(ina226_t *dev, i2c_port_t port, uint8_t addr, uint16_t speed, float r_shunt_ohm, float max_expected_current_a,
                      const ina226_config_t *cfg)
{
    if (!dev || !cfg)
        return ESP_ERR_INVALID_ARG;
    if (addr < INA226_ADDR_MIN || addr > INA226_ADDR_MAX)
        return ESP_ERR_INVALID_ARG;
    if (r_shunt_ohm <= 0 || max_expected_current_a <= 0)
        return ESP_ERR_INVALID_ARG;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = speed,
    };

    bsp_i2c_add_device(port, &dev_cfg, &(dev->i2c_dev_handle));

    dev->r_shunt = r_shunt_ohm;

    dev->current_lsb = max_expected_current_a / 32768.0f;
    float cal_f = 0.00512f / (dev->current_lsb * dev->r_shunt);
    if (cal_f > 0xFFFF)
        return ESP_ERR_INVALID_ARG;
    dev->cal_reg = (uint16_t)lroundf(cal_f);
    dev->power_lsb = 25.0f * dev->current_lsb;

    ESP_ERROR_CHECK_WITHOUT_ABORT(ina226_reset(dev));
    vTaskDelay(pdMS_TO_TICKS(2));

    uint16_t config_val = make_config(cfg);
    esp_err_t err = ina226_write_reg(dev, INA226_REG_CONFIG, config_val);
    if (err != ESP_OK)
        return err;

    err = ina226_write_reg(dev, INA226_REG_CALIB, dev->cal_reg);
    if (err != ESP_OK)
        return err;

    ESP_LOGI(TAG, "init ok: addr=0x%02X, Rsh=%.6f, I_LSB=%.9fA/bit, CAL=0x%04X, P_LSB=%.9fW/bit", addr, dev->r_shunt, dev->current_lsb, dev->cal_reg,
             dev->power_lsb);
    return ESP_OK;
}

/**
 * @brief 读取分流电阻两端电压（单位：V）
 */
esp_err_t ina226_read_shunt_voltage(ina226_t *dev, float *v_shunt)
{
    uint16_t raw;
    esp_err_t err = ina226_read_reg(dev, INA226_REG_SHUNT_V, &raw);
    if (err != ESP_OK)
        return err;
    int16_t sraw = (int16_t)raw;
    *v_shunt = (float)sraw * INA226_SHUNT_LSB_V;
    return ESP_OK;
}

/**
 * @brief 读取总线电压（单位：V）
 */
esp_err_t ina226_read_bus_voltage(ina226_t *dev, float *v_bus)
{
    uint16_t raw;
    esp_err_t err = ina226_read_reg(dev, INA226_REG_BUS_V, &raw);
    if (err != ESP_OK)
        return err;
    *v_bus = (float)raw * INA226_BUS_LSB_V;
    return ESP_OK;
}

/**
 * @brief 读取电流（单位：A）
 */
esp_err_t ina226_read_current(ina226_t *dev, float *current_a)
{
    uint16_t raw;
    esp_err_t err = ina226_read_reg(dev, INA226_REG_CURRENT, &raw);
    if (err != ESP_OK)
        return err;
    int16_t s = (int16_t)raw;
    *current_a = (float)s * dev->current_lsb;
    return ESP_OK;
}

/**
 * @brief 读取功率（单位：W）
 */
esp_err_t ina226_read_power(ina226_t *dev, float *power_w)
{
    uint16_t raw;
    esp_err_t err = ina226_read_reg(dev, INA226_REG_POWER, &raw);
    if (err != ESP_OK)
        return err;
    *power_w = (float)raw * dev->power_lsb;
    return ESP_OK;
}

/**
 * @brief 配置 Mask/Enable 寄存器
 */
esp_err_t ina226_set_mask_enable(ina226_t *dev, uint16_t mask)
{
    return ina226_write_reg(dev, INA226_REG_MASK_ENABLE, mask);
}

/**
 * @brief 设置告警阈值寄存器
 */
esp_err_t ina226_set_alert_limit(ina226_t *dev, uint16_t limit)
{
    return ina226_write_reg(dev, INA226_REG_ALERT_LIMIT, limit);
}
