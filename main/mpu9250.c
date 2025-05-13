// Skriver ett byte till ett register hos MPU9250
static esp_err_t mpu9250_write_byte(uint8_t reg, uint8_t data) {
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
static esp_err_t mpu9250_read_bytes(uint8_t reg, uint8_t *data, size_t length) {
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

// Initierar MPU9250
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
