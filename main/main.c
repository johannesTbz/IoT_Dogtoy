#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include <stdlib.h>

#define I2C_MASTER_SCL_IO 22       // SCL
#define I2C_MASTER_SDA_IO 21       // SDA
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000  // 100 kHz

#define MPU9250_ADDR 0x68          // I2C-adress för MPU-9250
#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B

#define SERVO_GPIO 4
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_FREQ_HZ 50
#define LEDC_RESOLUTION LEDC_TIMER_16_BIT

#define SERVO_STOP 1500            // Neutral position
#define SERVO_FORWARD 2000         // Aktiverad motor
#define MOTION_THRESHOLD 5000      // Skillnad från baseline som utlöser motor
#define MOTOR_RUN_DURATION_MS 2000 //Hur länge motorn ska vara på
#define BASELINE_DELAY_MS 3000     // Fördröjning efter att motorn stoppats innan baseline uppdateras

static const char *TAG = "BallControl";

// Omvandlar pulsbredd i µs till duty cycle (16-bit)
static uint32_t pulse_to_duty(int pulse_width) {
    return (pulse_width * (1 << 16)) / 20000;
}

// Skriver ett byte till ett register hos MPU9250
esp_err_t mpu9250_write_byte(uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Läser angivna antal byte från ett register hos MPU9250
esp_err_t mpu9250_read_bytes(uint8_t reg, uint8_t *data, size_t length) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, length, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Initierar MPU9250 (väcker den)
void mpu9250_init() {
    ESP_LOGI(TAG, "Initierar MPU9250...");
    uint8_t who_am_i = 0;
    mpu9250_read_bytes(WHO_AM_I_REG, &who_am_i, 1);
    ESP_LOGI(TAG, "MPU9250 WHO_AM_I: 0x%X", who_am_i);

    ESP_ERROR_CHECK(mpu9250_write_byte(PWR_MGMT_1, 0x00)); // Väcker sensorn ur sleep-läge
}

// Läser X-axelns acceleration (två byte, kombineras till ett 16-bitarsvärde)
int16_t read_accel_x() {
    uint8_t data[2];
    mpu9250_read_bytes(ACCEL_XOUT_H, data, 2);
    return (int16_t)((data[0] << 8) | data[1]);
}

// Ställer in servots position genom att ange pulsbredd
void set_servo_position(int pulse_width) {
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, pulse_to_duty(pulse_width));
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

void app_main(void) {
    ESP_LOGI(TAG, "Initierar I2C...");
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    mpu9250_init();

    ESP_LOGI(TAG, "Initierar Servo...");
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_RESOLUTION,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t channel_conf = {
        .gpio_num = SERVO_GPIO,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel_conf);

    //baselinevärde
    int16_t baseline = read_accel_x();
    ESP_LOGI(TAG, "Initial baseline: %d", baseline);

    bool motor_active = false;
    TickType_t motor_start_time = 0;

    while (1) {
        int16_t current = read_accel_x();
        ESP_LOGI(TAG, "Accel X: %d", current);

        if (!motor_active) {
            // Om skillnaden från baseline överstiger tröskeln aktivera motorn
            if (abs(current - baseline) > MOTION_THRESHOLD) {
                ESP_LOGI(TAG, "Rörelse upptäckt! Aktiverar motor...");
                motor_active = true;
                motor_start_time = xTaskGetTickCount();
                set_servo_position(SERVO_FORWARD);
            }
        } else {
            // Om motorn är aktiv, kolla om ska stängas av efter en fast tid
            if ((xTaskGetTickCount() - motor_start_time) >= pdMS_TO_TICKS(MOTOR_RUN_DURATION_MS)) {
                ESP_LOGI(TAG, "Motoraktivitet avslutad. Stoppar motor...");
                set_servo_position(SERVO_STOP);
                motor_active = false;
                vTaskDelay(pdMS_TO_TICKS(BASELINE_DELAY_MS));
                baseline = current;
                ESP_LOGI(TAG, "Ny baseline: %d", baseline);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/ledc.h"
// #include "esp_log.h"

// #define SERVO_GPIO      4                   // GPIO för servot
// #define LEDC_TIMER      LEDC_TIMER_0
// #define LEDC_MODE       LEDC_LOW_SPEED_MODE
// #define LEDC_CHANNEL    LEDC_CHANNEL_0
// #define LEDC_FREQ_HZ    50                  // Standard 50 Hz för servon
// #define LEDC_RESOLUTION LEDC_TIMER_16_BIT

// // Pulsvärden i mikrosekunder (µs)
// // OBS! Dessa kan behöva kalibreras för din servomotor.
// #define SERVO_STOP      1500    // Neutral position (stopp)
// #define SERVO_FORWARD   2000    // Roterar i en riktning
// #define SERVO_REVERSE   1000    // Roterar i motsatt riktning

// static const char *TAG = "ServoTest";

// // Omvandlar en pulsbredd (µs) till en duty cycle för LEDC (16-bit)
// static uint32_t pulse_to_duty(int pulse_width) {
//     return (pulse_width * (1 << 16)) / 20000;
// }

// // Ställer in servots position genom att uppdatera PWM-signalen
// void set_servo_position(int pulse_width) {
//     ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, pulse_to_duty(pulse_width));
//     ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
// }

// void app_main(void) {
//     ESP_LOGI(TAG, "Initierar Servo...");

//     // Konfigurera LEDC-timer för PWM
//     ledc_timer_config_t timer_conf = {
//         .speed_mode = LEDC_MODE,
//         .duty_resolution = LEDC_RESOLUTION,
//         .timer_num = LEDC_TIMER,
//         .freq_hz = LEDC_FREQ_HZ,
//         .clk_cfg = LEDC_AUTO_CLK
//     };
//     ledc_timer_config(&timer_conf);

//     // Konfigurera LEDC-kanal som styr servot
//     ledc_channel_config_t channel_conf = {
//         .gpio_num = SERVO_GPIO,
//         .speed_mode = LEDC_MODE,
//         .channel = LEDC_CHANNEL,
//         .timer_sel = LEDC_TIMER,
//         .duty = 0,
//         .hpoint = 0
//     };
//     ledc_channel_config(&channel_conf);

//     // Testar servomotorns rotation genom att cykla mellan olika lägen
//     while (1) {
//         // Rotera i en riktning (t.ex. medurs)
//         ESP_LOGI(TAG, "Roterar framåt...");
//         set_servo_position(SERVO_FORWARD);
//         vTaskDelay(pdMS_TO_TICKS(5000));  // Låt motorn snurra i 5 sekunder

//         // Stoppa servot
//         ESP_LOGI(TAG, "Stoppar servo...");
//         set_servo_position(SERVO_STOP);
//         vTaskDelay(pdMS_TO_TICKS(2000));  // Stanna i 2 sekunder

//         // Rotera i motsatt riktning (t.ex. moturs)
//         ESP_LOGI(TAG, "Roterar bakåt...");
//         set_servo_position(SERVO_REVERSE);
//         vTaskDelay(pdMS_TO_TICKS(5000));  // Låt motorn snurra i 5 sekunder

//         // Stoppa servot igen
//         ESP_LOGI(TAG, "Stoppar servo...");
//         set_servo_position(SERVO_STOP);
//         vTaskDelay(pdMS_TO_TICKS(2000));  // Stanna i 2 sekunder
//     }
// }
