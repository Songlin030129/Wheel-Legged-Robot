#include "BUZ.h"
#include "esp_log.h"

static const char *TAG_BUZ = "BUZ";

/**
 * @brief 计算指定分辨率下的最大计数值
 *
 * @param res LEDC 占空比分辨率
 * @return uint32_t 最大计数值
 */
static inline uint32_t buz_max_duty_ticks(ledc_timer_bit_t res)
{
    return (1U << res) - 1U;
}

/**
 * @brief 根据当前占空比刷新 PWM 输出
 *
 * @param buz 蜂鸣器句柄
 * @return ESP_OK 成功；否则返回底层错误码
 */
static esp_err_t buz_apply_duty_(BUZ_t *buz)
{
    uint32_t max_ticks = buz_max_duty_ticks(buz->resolution);
    uint32_t duty = (uint32_t)buz->duty_percent * max_ticks / 100U;
    return bsp_pwm_set_duty(buz->speed_mode, buz->channel, duty);
}

/**
 * @brief 初始化蜂鸣器 PWM 输出
 */
esp_err_t BUZ_init(BUZ_t *buz, uint8_t gpio, ledc_timer_t timer, ledc_channel_t channel, uint32_t freq_hz, uint8_t duty_percent,
                   ledc_timer_bit_t resolution)
{
    if (!buz)
        return ESP_ERR_INVALID_ARG;
    if (duty_percent > 100)
        return ESP_ERR_INVALID_ARG;
    if (freq_hz == 0)
        return ESP_ERR_INVALID_ARG;

    buz->pin = gpio;
    buz->speed_mode = LEDC_LOW_SPEED_MODE;
    buz->timer = timer;
    buz->channel = channel;
    buz->resolution = resolution;
    buz->freq_hz = freq_hz;
    buz->duty_percent = duty_percent;
    buz->started = 0;

    esp_err_t err = bsp_pwm_channel_init(buz->speed_mode, buz->channel, buz->timer, buz->pin, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_BUZ, "channel_init failed: %d", (int)err);
        return err;
    }

    return ESP_OK;
}

/**
 * @brief 设置蜂鸣器频率
 */
esp_err_t BUZ_set_freq(BUZ_t *buz, uint32_t freq_hz)
{
    if (!buz || freq_hz == 0)
        return ESP_ERR_INVALID_ARG;
    esp_err_t err = bsp_pwm_set_freq(buz->speed_mode, buz->timer, freq_hz);
    if (err == ESP_OK)
        buz->freq_hz = freq_hz;
    return err;
}

/**
 * @brief 设置蜂鸣器占空比
 */
esp_err_t BUZ_set_duty(BUZ_t *buz, uint8_t duty_percent)
{
    if (!buz || duty_percent > 100)
        return ESP_ERR_INVALID_ARG;
    buz->duty_percent = duty_percent;
    return buz_apply_duty_(buz);
}

/**
 * @brief 启动蜂鸣器输出
 */
esp_err_t BUZ_start(BUZ_t *buz)
{
    if (!buz)
        return ESP_ERR_INVALID_ARG;
    esp_err_t err = buz_apply_duty_(buz);
    if (err == ESP_OK)
        buz->started = 1;
    return err;
}

/**
 * @brief 停止蜂鸣器输出
 */
esp_err_t BUZ_stop(BUZ_t *buz)
{
    if (!buz)
        return ESP_ERR_INVALID_ARG;
    esp_err_t err = bsp_pwm_set_duty(buz->speed_mode, buz->channel, 0);
    if (err == ESP_OK)
        buz->started = 0;
    return err;
}

/**
 * @brief 以阻塞方式让蜂鸣器鸣叫指定时长
 */
esp_err_t BUZ_beep_ms(BUZ_t *buz, uint32_t freq_hz, uint8_t duty_percent, uint32_t duration_ms)
{
    if (!buz)
        return ESP_ERR_INVALID_ARG;
    esp_err_t err = BUZ_set_freq(buz, freq_hz);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_BUZ, "set_freq failed: %d", (int)err);
        return err;
    }

    err = BUZ_set_duty(buz, duty_percent);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_BUZ, "set_duty failed: %d", (int)err);
        return err;
    }

    err = BUZ_start(buz);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_BUZ, "start failed: %d", (int)err);
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    return BUZ_stop(buz);
}
