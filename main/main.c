/*
 * FreeRTOS ESP32 sensor monitor
 * New in this commit: GPIO4 falling-edge ISR signals buttonSemaphore.
 * control_task polls it non-blocking after the queue receive.
 * Latency issue noted: button response is gated by sensor interval.
 * Will fix in next commit using a queue set.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "bmp280.h"

#define I2C_PORT           I2C_NUM_0
#define I2C_SCL_GPIO       22
#define I2C_SDA_GPIO       21
#define I2C_FREQ_HZ        100000
#define BUTTON_GPIO        GPIO_NUM_4
#define ALERT_LED_GPIO     GPIO_NUM_2
#define ALERT_THRESHOLD_C  30.0f
#define SENSOR_PERIOD_MS   500

static const char *TAG = "FIRMWARE";
static bmp280_calib_t    bmp_calib;
static QueueHandle_t     sensorQueue;
static SemaphoreHandle_t buttonSemaphore;

typedef struct {
    float   temperature;
    int64_t timestamp_us;
} sensor_reading_t;

/* ISR: minimal -- no logging, no blocking, no heap ops */
static void IRAM_ATTR button_isr_handler(void *arg)
{
    BaseType_t higher_prio_woken = pdFALSE;
    xSemaphoreGiveFromISR(buttonSemaphore, &higher_prio_woken);
    portYIELD_FROM_ISR_ARG(higher_prio_woken);
}

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
            xQueueSend(sensorQueue, &reading, pdMS_TO_TICKS(50));
        }
        vTaskDelay(pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    }
}

static void control_task(void *pvParameters)
{
    sensor_reading_t reading;
    while (1) {
        /* Block waiting for sensor data (up to 1 s) */
        if (xQueueReceive(sensorQueue, &reading, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (reading.temperature > ALERT_THRESHOLD_C) {
                gpio_set_level(ALERT_LED_GPIO, 1);
                ESP_LOGI(TAG, "ALERT: %.2f C", reading.temperature);
            } else {
                gpio_set_level(ALERT_LED_GPIO, 0);
            }
        }
        /* Non-blocking poll for button (latency = up to queue block time) */
        if (xSemaphoreTake(buttonSemaphore, 0) == pdTRUE) {
            gpio_set_level(ALERT_LED_GPIO, 0);
            ESP_LOGI(TAG, "button override: LED cleared");
        }
    }
}

void app_main(void)
{
    i2c_master_init();
    ESP_ERROR_CHECK(bmp280_init(I2C_PORT, BMP280_ADDR_PRIMARY, &bmp_calib));

    gpio_set_direction(ALERT_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(ALERT_LED_GPIO, 0);
    gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_GPIO, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(BUTTON_GPIO, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL);

    sensorQueue     = xQueueCreate(10, sizeof(sensor_reading_t));
    buttonSemaphore = xSemaphoreCreateBinary();
    configASSERT(sensorQueue);
    configASSERT(buttonSemaphore);

    xTaskCreate(sensor_task,  "sensor_task",  4096, NULL, 5, NULL);
    xTaskCreate(control_task, "control_task", 4096, NULL, 6, NULL);
    ESP_LOGI(TAG, "boot: ISR on GPIO%d, LED on GPIO%d", BUTTON_GPIO, ALERT_LED_GPIO);
}
