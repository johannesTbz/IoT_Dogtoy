static uint32_t pulse_to_duty(int pulse_width) {
    return (pulse_width * (1 << 16)) / 20000;
}

void set_servo_position(int pulse_width) {
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, pulse_to_duty(pulse_width));
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

void servo_init(void) {
    ledc_timer_config_t timer_conf = { ... }
    ledc_channel_config_t channel_conf = { ... }
    ledc_timer_config(&timer_conf);
    ledc_channel_config(&channel_conf);
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

