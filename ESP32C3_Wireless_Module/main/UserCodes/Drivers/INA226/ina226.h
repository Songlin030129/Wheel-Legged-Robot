#ifndef __INA226_H__
#define __INA226_H__
#include <stdint.h>
#include "bsp_i2c.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// ========= 地址与寄存器（基于数据手册） =========
// A1/A0 引脚组合得到 7bit I2C 地址：0x40 ~ 0x4F
// 例如 A1=A0=GND -> 0b1000000 = 0x40
#define INA226_ADDR_MIN 0x40
#define INA226_ADDR_MAX 0x4F

// 寄存器地址
#define INA226_REG_CONFIG 0x00
#define INA226_REG_SHUNT_V 0x01
#define INA226_REG_BUS_V 0x02
#define INA226_REG_POWER 0x03
#define INA226_REG_CURRENT 0x04
#define INA226_REG_CALIB 0x05
#define INA226_REG_MASK_ENABLE 0x06
#define INA226_REG_ALERT_LIMIT 0x07

// 量化步长
#define INA226_SHUNT_LSB_V (2.5e-6f)  // 2.5 µV/bit
#define INA226_BUS_LSB_V (1.25e-3f)   // 1.25 mV/bit

// CONFIG 寄存器位（见手册）
#define INA226_CONFIG_RST_BIT (1U << 15)

// AVG (14:12)
typedef enum {
    INA226_AVG_1 = 0,
    INA226_AVG_4 = 1,
    INA226_AVG_16 = 2,
    INA226_AVG_64 = 3,
    INA226_AVG_128 = 4,
    INA226_AVG_256 = 5,
    INA226_AVG_512 = 6,
    INA226_AVG_1024 = 7,
} ina226_avg_t;

// VBUSCT (11:9) / VSHCT (8:6) 转换时间（从快到慢，参考典型 140us~8.244ms）
typedef enum {
    INA226_CT_140US = 0,
    INA226_CT_204US = 1,
    INA226_CT_332US = 2,
    INA226_CT_588US = 3,
    INA226_CT_1P1MS = 4,
    INA226_CT_2P116MS = 5,
    INA226_CT_4P156MS = 6,
    INA226_CT_8P244MS = 7,
} ina226_ct_t;

// MODE (5:3)
typedef enum {
    INA226_MODE_POWER_DOWN = 0,
    INA226_MODE_SHUNT_TRIG = 1,
    INA226_MODE_BUS_TRIG = 2,
    INA226_MODE_SHUNT_BUS_TRIG = 3,
    INA226_MODE_ADC_OFF_100 = 4,  // 预留
    INA226_MODE_SHUNT_CONT = 5,
    INA226_MODE_BUS_CONT = 6,
    INA226_MODE_SHUNT_BUS_CONT = 7,
} ina226_mode_t;

typedef struct {
    ina226_avg_t avg;     // 平均次数
    ina226_ct_t vbus_ct;  // 总线电压转换时间
    ina226_ct_t vsh_ct;   // 分流电压转换时间
    ina226_mode_t mode;   // 工作模式
} ina226_config_t;

#define INA226_CONFIG_DEFAULT() \
    ((ina226_config_t){.avg = INA226_AVG_4, .vbus_ct = INA226_CT_588US, .vsh_ct = INA226_CT_588US, .mode = INA226_MODE_SHUNT_BUS_CONT})

typedef struct {
    i2c_master_dev_handle_t i2c_dev_handle;
    float r_shunt;      // 分流电阻（Ω）
    float current_lsb;  // A/bit
    float power_lsb;    // W/bit (= 25 * current_lsb)
    uint16_t cal_reg;   // 写入的校准寄存器值
} ina226_t;

// ========== API ==========

// 初始化（计算并写入校准，配置 CONFIG）
/**
 * @brief 初始化 INA226（计算并写入校准值，配置 CONFIG）。
 * @param dev INA226 设备句柄（输出: 初始化后的内部变量将被填充）
 * @param port I2C 控制器端口号
 * @param addr 7 位 INA226 器件地址
 * @param speed 设备通信速率
 * @param r_shunt_ohm 分流电阻阻值（Ω）
 * @param max_expected_current_a 预期最大电流（A），用于确定 Current_LSB
 * @param cfg 配置参数（平均次数/采样时间/工作模式）
 * @return ESP_OK 成功；参数非法或 I2C 失败时返回错误码
 * @note 公式: Current_LSB = Imax/2^15; CAL = 0.00512/(Current_LSB*Rshunt); Power_LSB = 25*Current_LSB
 */
esp_err_t ina226_init(ina226_t *dev, i2c_port_t port, uint8_t addr, uint16_t speed, float r_shunt_ohm, float max_expected_current_a,
                      const ina226_config_t *cfg);

// 复位（软件复位位）
/**
 * @brief 软件复位（通过配置寄存器 RST 位）。
 * @param dev INA226 设备句柄
 * @return ESP_OK 成功；否则返回错误码
 */
esp_err_t ina226_reset(ina226_t *dev);

// 读取原始寄存器
/**
 * @brief 读取 16 位寄存器。
 * @param dev INA226 设备句柄
 * @param reg 寄存器地址
 * @param out 输出参数，返回读取值
 * @return ESP_OK 成功；否则返回错误码
 */
esp_err_t ina226_read_reg(ina226_t *dev, uint8_t reg, uint16_t *out);
/**
 * @brief 写入 16 位寄存器。
 * @param dev INA226 设备句柄
 * @param reg 寄存器地址
 * @param val 待写入的数值
 * @return ESP_OK 成功；否则返回错误码
 */
esp_err_t ina226_write_reg(ina226_t *dev, uint8_t reg, uint16_t val);

// 读取物理量
/**
 * @brief 读取分流电阻两端电压（V）。
 * @param dev INA226 设备句柄
 * @param v_shunt 输出参数，电压（V）
 * @return ESP_OK 成功；否则返回错误码
 */
esp_err_t ina226_read_shunt_voltage(ina226_t *dev, float *v_shunt);
/**
 * @brief 读取总线电压（V）。
 * @param dev INA226 设备句柄
 * @param v_bus 输出参数，电压（V）
 * @return ESP_OK 成功；否则返回错误码
 */
esp_err_t ina226_read_bus_voltage(ina226_t *dev, float *v_bus);
/**
 * @brief 读取电流（A）。
 * @param dev INA226 设备句柄
 * @param current_a 输出参数，电流（A）
 * @return ESP_OK 成功；否则返回错误码
 */
esp_err_t ina226_read_current(ina226_t *dev, float *current_a);
/**
 * @brief 读取功率（W）。
 * @param dev INA226 设备句柄
 * @param power_w 输出参数，功率（W）
 * @return ESP_OK 成功；否则返回错误码
 */
esp_err_t ina226_read_power(ina226_t *dev, float *power_w);

// 告警相关（可选）
/**
 * @brief 设置 Mask/Enable 寄存器（告警/中断相关）。
 * @param dev INA226 设备句柄
 * @param mask 掩码/使能位字段
 * @return ESP_OK 成功；否则返回错误码
 */
esp_err_t ina226_set_mask_enable(ina226_t *dev, uint16_t mask);
/**
 * @brief 设置告警阈值寄存器。
 * @param dev INA226 设备句柄
 * @param limit 告警门限寄存器值（含义由 Mask/Enable 配置决定）
 * @return ESP_OK 成功；否则返回错误码
 */
esp_err_t ina226_set_alert_limit(ina226_t *dev, uint16_t limit);

#ifdef __cplusplus
}
#endif

extern ina226_t ina226;
#endif
