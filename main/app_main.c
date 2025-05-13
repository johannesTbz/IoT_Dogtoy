#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include <stdlib.h>

#include "servo.h"


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

//initiera servo
servo_init();



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

