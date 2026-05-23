/*
 * FreeRTOS ESP32 sensor monitor
 * Upgrade: replaced non-blocking semaphore poll with FreeRTOS Queue Set.
 * control_task now blocks on BOTH sensorQueue and buttonSemaphore
 * simultaneously -- zero-latency button response regardless of sensor rate.
 *
 *   sensor_task --[reading]--> sensorQueue    --+
 *                                               +--> Queue Set --> control_task
 *   button ISR  --[give]----> buttonSemaphore --+
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
static QueueSetHandle_t  controlQueueSet;

typedef struct {
    float   temperature;
    int64_t timestamp_us;
} sensor_reading_t;

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
            if (xQueueSend(sensorQueue, &reading, pdMS_TO_TICKS(50)) != pdTRUE) {
                ESP_LOGW(TAG, "sensorQueue full -- reading dropped");
            }
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
    controlQueueSet = xQueueCreateSet(10 + 1);
    configASSERT(sensorQueue);
    configASSERT(buttonSemaphore);
    configASSERT(controlQueueSet);

    xQueueAddToSet(sensorQueue,     controlQueueSet);
    xQueueAddToSet(buttonSemaphore, controlQueueSet);

    xTaskCreate(sensor_task,  "sensor_task",  4096, NULL, 5, NULL);
    xTaskCreate(control_task, "control_task", 4096, NULL, 6, NULL);
    ESP_LOGI(TAG, "boot: queue set active -- zero-latency button response");
}
