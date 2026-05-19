/**
 * @file bmp280.c
 * @brief Real BMP280 I2C driver implementation.
 *
 * Every I2C transaction in this file targets an actual BMP280 register.
 * The hardware may be simulated by Wokwi, but the frames on the wire
 * (address, register byte, data bytes) are identical to those you would
 * send to a physical sensor.
 *
 * Reference: Bosch BMP280 datasheet v1.26, sections 4.2, 8.2.
 */

#include "bmp280.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "BMP280";

/* I2C transaction timeout – 100 ms is generous for a 100 kHz bus */
#define I2C_TIMEOUT_MS  100

/* ── Low-level helpers ─────────────────────────────────────────────── */

/**
 * Write a single byte to a register.
 * Frame: START | ADDR+W | REG | VALUE | STOP
 */
static esp_err_t reg_write(i2c_port_t port, uint8_t addr,
                            uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return i2c_master_write_to_device(port, addr, buf, sizeof(buf),
                                      pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

/**
 * Read `len` bytes starting at `reg`.
 * Frame: START | ADDR+W | REG | RSTART | ADDR+R | DATA[0..n] | STOP
 */
static esp_err_t reg_read(i2c_port_t port, uint8_t addr,
                           uint8_t reg, uint8_t *out, size_t len)
{
    return i2c_master_write_read_device(port, addr, &reg, 1, out, len,
                                        pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

/* ── Public API ────────────────────────────────────────────────────── */

esp_err_t bmp280_init(i2c_port_t port, uint8_t addr, bmp280_calib_t *calib)
{
    esp_err_t ret;

    /* ── 1. Chip-ID check ───────────────────────────────────────────
     * Register 0xD0 must return 0x60 for a genuine BMP280.
     * A NACK here means the device is absent or address is wrong.
     */
    uint8_t chip_id = 0;
    ret = reg_read(port, addr, BMP280_REG_CHIP_ID, &chip_id, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "chip-ID read failed (NACK or timeout): %s",
                 esp_err_to_name(ret));
        return ret;
    }
    if (chip_id != BMP280_CHIP_ID) {
        ESP_LOGE(TAG, "unexpected chip ID 0x%02X (expected 0x%02X)",
                 chip_id, BMP280_CHIP_ID);
        return ESP_ERR_INVALID_RESPONSE;
    }
    ESP_LOGI(TAG, "chip ID 0x%02X OK", chip_id);

    /* ── 2. Read factory calibration coefficients (0x88–0x8D) ──────
     *
     * NVM layout (little-endian 16-bit words, §4.2.2):
     *   0x88–0x89  dig_T1  uint16
     *   0x8A–0x8B  dig_T2  int16
     *   0x8C–0x8D  dig_T3  int16
     */
    uint8_t raw_calib[6];
    ret = reg_read(port, addr, 0x88, raw_calib, sizeof(raw_calib));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "calibration read failed: %s", esp_err_to_name(ret));
        return ret;
    }

    calib->dig_T1 = (uint16_t)(raw_calib[1] << 8 | raw_calib[0]);
    calib->dig_T2 = (int16_t) (raw_calib[3] << 8 | raw_calib[2]);
    calib->dig_T3 = (int16_t) (raw_calib[5] << 8 | raw_calib[4]);

    ESP_LOGI(TAG, "calib: T1=%u T2=%d T3=%d",
             calib->dig_T1, calib->dig_T2, calib->dig_T3);

    /* ── 3. Configure: temp oversampling ×1, normal mode ───────────
     *
     * ctrl_meas (0xF4):
     *   bits [7:5]  osrs_t  = 001  (×1 temperature oversampling)
     *   bits [4:2]  osrs_p  = 000  (pressure skipped — not wired)
     *   bits [1:0]  mode    = 11   (normal / continuous mode)
     *
     * Combined: 0b00100011 = 0x23
     */
    ret = reg_write(port, addr, BMP280_REG_CTRL_MEAS, 0x23);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ctrl_meas write failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "BMP280 initialised, normal mode");
    return ESP_OK;
}

esp_err_t bmp280_read_temperature(i2c_port_t port, uint8_t addr,
                                   const bmp280_calib_t *calib,
                                   float *temp_c)
{
    /* ── Read raw 20-bit ADC value from 0xFA–0xFC ───────────────────
     *
     * Register map:
     *   0xFA  temp_msb  [7:0]
     *   0xFB  temp_lsb  [7:0]
     *   0xFC  temp_xlsb [7:4]  (bits [3:0] are always 0)
     *
     * adc_T = (msb << 12) | (lsb << 4) | (xlsb >> 4)  → 20-bit value
     */
    uint8_t raw[3];
    esp_err_t ret = reg_read(port, addr, BMP280_REG_TEMP_MSB, raw, sizeof(raw));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "temperature read I2C error: %s", esp_err_to_name(ret));
        return ret;
    }

    int32_t adc_T = ((int32_t)raw[0] << 12) |
                    ((int32_t)raw[1] <<  4) |
                    ((int32_t)raw[2] >>  4);

    /* The reset/skipped value 0x80000 means the sensor has not yet
     * completed a measurement.  Treat it as an invalid reading. */
    if (adc_T == 0x80000) {
        ESP_LOGW(TAG, "ADC value is 0x80000 — measurement not ready");
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* ── Bosch integer compensation formula (datasheet §8.2) ────────
     *
     * var1 and var2 are intermediate Q8.23 fixed-point values.
     * t_fine is the temperature-scaled value also used for pressure.
     * Final temperature = t_fine / 5120 → degrees Celsius × 100.
     *
     * The casts to int32_t and the right-shifts are all specified
     * verbatim in the Bosch datasheet and must not be "simplified".
     */
    int32_t var1, var2, t_fine;

    var1 = ((((adc_T >> 3) - ((int32_t)calib->dig_T1 << 1))) *
             ((int32_t)calib->dig_T2)) >> 11;

    var2 = (((((adc_T >> 4) - ((int32_t)calib->dig_T1)) *
               ((adc_T >> 4) - ((int32_t)calib->dig_T1))) >> 12) *
              ((int32_t)calib->dig_T3)) >> 14;

    t_fine = var1 + var2;

    /* t_fine / 5120 gives °C; integer division so multiply × 100
     * first for 0.01 °C resolution, then convert to float. */
    int32_t T_x100 = (t_fine * 5 + 128) >> 8;  /* °C × 100, rounded */
    *temp_c = (float)T_x100 / 100.0f;

    return ESP_OK;
}
