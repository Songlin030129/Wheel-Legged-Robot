#ifndef __BSP_PWM_H__
#define __BSP_PWM_H__

#include "common_inc.h"
#include "driver/ledc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 LEDC 定时器
 *
 * @param mode       LEDC 速度模式（ESP32-C3 仅支持低速）
 * @param timer      LEDC 定时器编号
 * @param resolution 占空比分辨率
 * @param freq_hz    目标频率（Hz）
 * @return ESP_OK 表示初始化成功；否则返回底层错误码
 */
esp_err_t bsp_pwm_timer_init(ledc_mode_t mode, ledc_timer_t timer, ledc_timer_bit_t resolution, uint32_t freq_hz);

/**
 * @brief 初始化 LEDC 通道
 *
 * @param mode    LEDC 速度模式
 * @param channel LEDC 通道编号
 * @param timer   绑定的 LEDC 定时器
 * @param gpio    输出 GPIO
 * @param duty    初始占空比（计数值）
 * @return ESP_OK 表示初始化成功；否则返回底层错误码
 */
esp_err_t bsp_pwm_channel_init(ledc_mode_t mode, ledc_channel_t channel, ledc_timer_t timer, int gpio, uint32_t duty);

/**
 * @brief 设置 LEDC 输出频率
 *
 * @param mode    LEDC 速度模式
 * @param timer   LEDC 定时器编号
 * @param freq_hz 新的频率（Hz）
 * @return ESP_OK 表示设置成功；否则返回底层错误码
 */
esp_err_t bsp_pwm_set_freq(ledc_mode_t mode, ledc_timer_t timer, uint32_t freq_hz);

/**
 * @brief 设置 LEDC 占空比（计数值，并立即更新硬件）
 *
 * @param mode    LEDC 速度模式
 * @param channel LEDC 通道编号
 * @param duty    占空比对应的计数值
 * @return ESP_OK 表示设置成功；否则返回底层错误码
 */
esp_err_t bsp_pwm_set_duty(ledc_mode_t mode, ledc_channel_t channel, uint32_t duty);

#ifdef __cplusplus
}
#endif

#endif  // __BSP_PWM_H__
