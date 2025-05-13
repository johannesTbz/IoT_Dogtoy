#include "servo.h"
#include "driver/ledc.h"
#include "esp_log.h"

#define SERVO_GPIO 4
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_FREQ_HZ 50
#define LEDC_RESOLUTION LEDC_TIMER_16_BIT

static const char *TAG = "Servo";

static uint32_t pulse_to_duty(int pulse_width) {
    return (pulse_width * (1 << 16)) / 20000;
}

void set_servo_position(int pulse_width) {
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, pulse_to_duty(pulse_width));
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

void servo_init(void) {
    ESP_LOGI(TAG, "Initierar Servo...");
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_RESOLUTION,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ledc_channel_config_t channel_conf = {
        .gpio_num = SERVO_GPIO,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    
    ledc_timer_config(&timer_conf);

    ledc_channel_config(&channel_conf);

}

