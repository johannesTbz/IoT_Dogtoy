#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <stdlib.h>
#include "servo.h"
#include "mpu9250.h"

#define SERVO_STOP 1500            // Neutral position
#define SERVO_FORWARD 2000         // Aktiverad motor
#define MOTION_THRESHOLD 5000      // Skillnad från baseline som utlöser motor
#define MOTOR_RUN_DURATION_MS 2000 //Hur länge motorn ska vara på
#define BASELINE_DELAY_MS 3000     // Fördröjning efter att motorn stoppats innan baseline uppdateras

#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000


static const char *TAG = "BallControl";

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

    servo_init();

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

