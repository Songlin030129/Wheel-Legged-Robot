#ifndef __BUZ_H__
#define __BUZ_H__

#include <stdint.h>
#include "UserCodes/common_inc.h"
#include "bsp_pwm.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 蜂鸣器控制句柄
 */
typedef struct {
    uint8_t pin;                  /**< GPIO 引脚编号 */
    ledc_mode_t speed_mode;       /**< LEDC 速度模式（ESP32-C3 仅低速） */
    ledc_timer_t timer;           /**< LEDC 定时器 */
    ledc_channel_t channel;       /**< LEDC 通道 */
    ledc_timer_bit_t resolution;  /**< 占空比分辨率 */
    uint32_t freq_hz;             /**< 当前频率（Hz） */
    uint8_t duty_percent;         /**< 当前占空比（0~100%） */
    uint8_t started;              /**< 是否已经开始输出 */
} BUZ_t;

/**
 * @brief 初始化蜂鸣器 PWM
 *
 * @param buz           蜂鸣器句柄
 * @param gpio          PWM 输出引脚
 * @param timer         LEDC 定时器编号
 * @param channel       LEDC 通道编号
 * @param freq_hz       初始频率（Hz）
 * @param duty_percent  初始占空比（0~100%）
 * @param resolution    LEDC 占空比分辨率
 * @return ESP_OK 初始化成功；否则返回底层错误码
 */
esp_err_t BUZ_init(BUZ_t *buz,
                   uint8_t gpio,
                   ledc_timer_t timer,
                   ledc_channel_t channel,
                   uint32_t freq_hz,
                   uint8_t duty_percent,
                   ledc_timer_bit_t resolution);

/**
 * @brief 设置蜂鸣器频率
 *
 * @param buz     蜂鸣器句柄
 * @param freq_hz 目标频率（Hz）
 * @return ESP_OK 设置成功；否则返回底层错误码
 */
esp_err_t BUZ_set_freq(BUZ_t *buz, uint32_t freq_hz);

/**
 * @brief 设置蜂鸣器占空比
 *
 * @param buz           蜂鸣器句柄
 * @param duty_percent  目标占空比（0~100%）
 * @return ESP_OK 设置成功；否则返回底层错误码
 */
esp_err_t BUZ_set_duty(BUZ_t *buz, uint8_t duty_percent);

/**
 * @brief 开始输出蜂鸣器信号
 *
 * @param buz 蜂鸣器句柄
 * @return ESP_OK 启动成功；否则返回底层错误码
 */
esp_err_t BUZ_start(BUZ_t *buz);

/**
 * @brief 停止蜂鸣器输出
 *
 * @param buz 蜂鸣器句柄
 * @return ESP_OK 停止成功；否则返回底层错误码
 */
esp_err_t BUZ_stop(BUZ_t *buz);

/**
 * @brief 简易蜂鸣控制：以给定参数鸣叫指定毫秒
 *
 * @param buz           蜂鸣器句柄
 * @param freq_hz       蜂鸣频率（Hz）
 * @param duty_percent  占空比（0~100%）
 * @param duration_ms   持续时间（毫秒）
 * @return ESP_OK 成功；否则返回底层错误码
 */
esp_err_t BUZ_beep_ms(BUZ_t *buz, uint32_t freq_hz, uint8_t duty_percent, uint32_t duration_ms);

#ifdef __cplusplus
}
#endif

#endif
