/*
 * FreeRTOS ESP32 sensor monitor
 * Tasks:
 *   sensor_task  -- reads BMP280, pushes readings to sensorQueue
 *   control_task -- consumes queue, drives LED based on threshold
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "bmp280.h"

#define I2C_PORT           I2C_NUM_0
#define I2C_SCL_GPIO       22
#define I2C_SDA_GPIO       21
#define I2C_FREQ_HZ        100000
#define ALERT_LED_GPIO     GPIO_NUM_2
#define ALERT_THRESHOLD_C  30.0f
#define SENSOR_PERIOD_MS   500

static const char *TAG = "FIRMWARE";
static bmp280_calib_t bmp_calib;
static QueueHandle_t  sensorQueue;

typedef struct {
    float   temperature;
    int64_t timestamp_us;
} sensor_reading_t;

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

static void sensor_task(void *pvParameters)
{
    sensor_reading_t reading;
    while (1) {
        float temp;
        if (bmp280_read_temperature(I2C_PORT, BMP280_ADDR_PRIMARY,
                                    &bmp_calib, &temp) == ESP_OK) {
            reading.temperature  = temp;
            reading.timestamp_us = esp_timer_get_time();
            if (xQueueSend(sensorQueue, &reading, pdMS_TO_TICKS(50)) != pdTRUE) {
                ESP_LOGW(TAG, "sensorQueue full -- reading dropped");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    }
}

/* control_task: blocks on sensorQueue, applies temperature threshold */
static void control_task(void *pvParameters)
{
    sensor_reading_t reading;
    while (1) {
        if (xQueueReceive(sensorQueue, &reading, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (reading.temperature > ALERT_THRESHOLD_C) {
                gpio_set_level(ALERT_LED_GPIO, 1);
                ESP_LOGI(TAG, "ALERT: %.2f C exceeds threshold", reading.temperature);
            } else {
                gpio_set_level(ALERT_LED_GPIO, 0);
            }
        }
    }
}

void app_main(void)
{
    i2c_master_init();
    ESP_ERROR_CHECK(bmp280_init(I2C_PORT, BMP280_ADDR_PRIMARY, &bmp_calib));

    gpio_set_direction(ALERT_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(ALERT_LED_GPIO, 0);

    sensorQueue = xQueueCreate(10, sizeof(sensor_reading_t));
    configASSERT(sensorQueue);

    xTaskCreate(sensor_task,  "sensor_task",  4096, NULL, 5, NULL);
    xTaskCreate(control_task, "control_task", 4096, NULL, 6, NULL);
    ESP_LOGI(TAG, "boot: 2 tasks, threshold=%.1f C", ALERT_THRESHOLD_C);
}
