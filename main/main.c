/*
 * FreeRTOS Multi-Task Sensor Monitor  (ESP32 + BMP280)
 *
 * Tasks:
 *   sensor_task  (pri 5) -- reads BMP280 via I2C every 500 ms
 *                           updates latest_reading (mutex), pushes to sensorQueue
 *   control_task (pri 6) -- Queue Set: wakes on sensor data OR button press
 *                           drives ALERT_LED based on temperature threshold
 *   logger_task  (pri 4) -- reads latest_reading (mutex) every 1 s
 *                           writes "t=... temp=..." line over UART
 *
 * Synchronisation:
 *   sensorQueue      Queue (depth 10)   sensor_task -> control_task
 *   dataMutex        Mutex              protects latest_reading (sensor <-> logger)
 *   buttonSemaphore  Binary semaphore   ISR -> control_task
 *   controlQueueSet  Queue Set          fans sensorQueue + buttonSemaphore
 *
 * Race condition demo:
 *   Comment out the MUTEX GUARDS blocks in sensor_task and logger_task,
 *   rebuild, and run.  The logger will print mismatched timestamp/temperature
 *   pairs under load -- a classic TOCTOU race.  Re-enable to fix.
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"
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

static QueueHandle_t     sensorQueue;
static SemaphoreHandle_t dataMutex;
static SemaphoreHandle_t buttonSemaphore;
static QueueSetHandle_t  controlQueueSet;

typedef struct {
    float   temperature;
    int64_t timestamp_us;
} sensor_reading_t;

static sensor_reading_t  latest_reading;
static bmp280_calib_t    bmp_calib;

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
        esp_err_t err = bmp280_read_temperature(I2C_PORT, BMP280_ADDR_PRIMARY,
                                                 &bmp_calib, &temp);
        if (err == ESP_OK) {
            reading.temperature  = temp;
            reading.timestamp_us = esp_timer_get_time();

            /* ---- MUTEX GUARDS (comment out both blocks to reproduce race) ---- */
            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                latest_reading = reading;
                xSemaphoreGive(dataMutex);
            }

            if (xQueueSend(sensorQueue, &reading, pdMS_TO_TICKS(50)) != pdTRUE) {
                ESP_LOGW(TAG, "sensorQueue full -- reading dropped");
            }
        } else if (err == ESP_ERR_INVALID_RESPONSE) {
            ESP_LOGW(TAG, "BMP280 not ready");
        } else {
            ESP_LOGE(TAG, "I2C error: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    }
}

static void control_task(void *pvParameters)
{
    sensor_reading_t reading;
    while (1) {
        QueueSetMemberHandle_t active =
            xQueueSelectFromSet(controlQueueSet, portMAX_DELAY);

        if (active == sensorQueue) {
            xQueueReceive(sensorQueue, &reading, 0);
            if (reading.temperature > ALERT_THRESHOLD_C) {
                gpio_set_level(ALERT_LED_GPIO, 1);
                ESP_LOGI(TAG, "ALERT: %.2f C > threshold %.1f C",
                         reading.temperature, ALERT_THRESHOLD_C);
            } else {
                gpio_set_level(ALERT_LED_GPIO, 0);
            }
        } else if (active == buttonSemaphore) {
            xSemaphoreTake(buttonSemaphore, 0);
            gpio_set_level(ALERT_LED_GPIO, 0);
            ESP_LOGI(TAG, "button ISR: alert cleared by manual override");
        }
    }
}

/* ---- MUTEX GUARDS (comment out to reproduce race) ---- */
static void logger_task(void *pvParameters)
{
    char buf[64];
    while (1) {
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            int len = snprintf(buf, sizeof(buf), "t=%lld temp=%.2f\r\n",
                               latest_reading.timestamp_us,
                               latest_reading.temperature);
            xSemaphoreGive(dataMutex);
            uart_write_bytes(UART_NUM_0, buf, len);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
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
    dataMutex       = xSemaphoreCreateMutex();
    buttonSemaphore = xSemaphoreCreateBinary();
    controlQueueSet = xQueueCreateSet(10 + 1);
    configASSERT(sensorQueue);
    configASSERT(dataMutex);
    configASSERT(buttonSemaphore);
    configASSERT(controlQueueSet);

    xQueueAddToSet(sensorQueue,     controlQueueSet);
    xQueueAddToSet(buttonSemaphore, controlQueueSet);

    xTaskCreate(sensor_task,  "sensor_task",  4096, NULL, 5, NULL);
    xTaskCreate(control_task, "control_task", 4096, NULL, 6, NULL);
    xTaskCreate(logger_task,  "logger_task",  4096, NULL, 4, NULL);

    ESP_LOGI(TAG, "boot: sensor=%dms threshold=%.1fC queue_set=yes",
             SENSOR_PERIOD_MS, ALERT_THRESHOLD_C);
}
