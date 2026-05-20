/*
 * FreeRTOS ESP32 sensor monitor -- initial skeleton
 * sensor_task: reads BMP280 via I2C every 500 ms and logs temperature
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "bmp280.h"

#define I2C_PORT         I2C_NUM_0
#define I2C_SCL_GPIO     22
#define I2C_SDA_GPIO     21
#define I2C_FREQ_HZ      100000
#define SENSOR_PERIOD_MS 500

static const char *TAG = "FIRMWARE";
static bmp280_calib_t bmp_calib;

static void i2c_master_init(void)
{
    const i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_SDA_GPIO,
        .scl_io_num       = I2C_SCL_GPIO,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &cfg));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));
}

/* Task A: sample the sensor every 500 ms and log to serial */
static void sensor_task(void *pvParameters)
{
    while (1) {
        float temp;
        esp_err_t err = bmp280_read_temperature(I2C_PORT, BMP280_ADDR_PRIMARY,
                                                 &bmp_calib, &temp);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "temp=%.2f C  t=%lld us", temp, esp_timer_get_time());
        } else if (err == ESP_ERR_INVALID_RESPONSE) {
            ESP_LOGW(TAG, "BMP280 not ready yet");
        } else {
            ESP_LOGE(TAG, "I2C error: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    }
}

void app_main(void)
{
    i2c_master_init();
    ESP_ERROR_CHECK(bmp280_init(I2C_PORT, BMP280_ADDR_PRIMARY, &bmp_calib));

    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "sensor_task started");
}
