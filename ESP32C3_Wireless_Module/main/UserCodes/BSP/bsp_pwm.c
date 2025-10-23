#include "bsp_pwm.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "bsp_pwm";
static bool s_timer_initialized[LEDC_TIMER_MAX] = {0};

esp_err_t bsp_pwm_timer_init(ledc_mode_t mode, ledc_timer_t timer, ledc_timer_bit_t resolution, uint32_t freq_hz)
{
    if (timer < 0 || timer >= LEDC_TIMER_MAX)
        return ESP_ERR_INVALID_ARG;
    ledc_timer_config_t cfg = {
        .speed_mode = mode,
        .duty_resolution = resolution,
        .timer_num = timer,
        .freq_hz = freq_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&cfg);
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "LEDC timer %d already configured", timer);
        err = ESP_OK;
    }
    ESP_RETURN_ON_ERROR(err, TAG, "ledc_timer_config failed");
    s_timer_initialized[timer] = true;
    return ESP_OK;
}

esp_err_t bsp_pwm_channel_init(ledc_mode_t mode, ledc_channel_t channel, ledc_timer_t timer, int gpio, uint32_t duty)
{
    if (channel < 0 || channel >= LEDC_CHANNEL_MAX)
        return ESP_ERR_INVALID_ARG;
    if (timer < 0 || timer >= LEDC_TIMER_MAX)
        return ESP_ERR_INVALID_ARG;
    if (!s_timer_initialized[timer])
        return ESP_ERR_INVALID_STATE;

    ledc_channel_config_t cfg = {
        .gpio_num = gpio,
        .speed_mode = mode,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = timer,
        .duty = duty,
        .hpoint = 0,
    };
    esp_err_t err = ledc_channel_config(&cfg);
    ESP_RETURN_ON_ERROR(err, TAG, "ledc_channel_config failed");
    return ESP_OK;
}

esp_err_t bsp_pwm_set_freq(ledc_mode_t mode, ledc_timer_t timer, uint32_t freq_hz)
{
    if (timer < 0 || timer >= LEDC_TIMER_MAX)
        return ESP_ERR_INVALID_ARG;
    if (!s_timer_initialized[timer])
        return ESP_ERR_INVALID_STATE;
    esp_err_t err = ledc_set_freq(mode, timer, freq_hz);
    if (err == ESP_ERR_INVALID_ARG) {
        ESP_LOGE(TAG, "invalid freq %lu Hz for timer %d", (unsigned long)freq_hz, (int)timer);
    }
    ESP_RETURN_ON_ERROR(err, TAG, "ledc_set_freq failed");
    return ESP_OK;
}

esp_err_t bsp_pwm_set_duty(ledc_mode_t mode, ledc_channel_t channel, uint32_t duty)
{
    if (channel < 0 || channel >= LEDC_CHANNEL_MAX)
        return ESP_ERR_INVALID_ARG;
    esp_err_t err = ledc_set_duty(mode, channel, duty);
    ESP_RETURN_ON_ERROR(err, TAG, "ledc_set_duty failed");
    err = ledc_update_duty(mode, channel);
    ESP_RETURN_ON_ERROR(err, TAG, "ledc_update_duty failed");
    return ESP_OK;
}
