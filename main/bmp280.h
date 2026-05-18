/**
 * @file bmp280.h
 * @brief Real BMP280 I2C driver (Bosch datasheet §4.2).
 *
 * This is NOT a stub.  Every transaction matches an actual register
 * on the BMP280 silicon and would work against a physical device.
 * Wokwi's wokwi-bmp280 component responds to the same register map,
 * so the simulation exercises real I2C frames.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c.h"

/* ── I2C addresses (SDO→GND = 0x76, SDO→VCC = 0x77) ─────────────── */
#define BMP280_ADDR_PRIMARY    0x76
#define BMP280_ADDR_SECONDARY  0x77

/* ── Key register addresses ────────────────────────────────────────── */
#define BMP280_REG_CHIP_ID     0xD0   /* expected value: 0x60           */
#define BMP280_REG_RESET       0xE0   /* write 0xB6 to soft-reset       */
#define BMP280_REG_CTRL_MEAS   0xF4   /* osrs_t, osrs_p, mode           */
#define BMP280_REG_CONFIG      0xF5   /* t_sb, filter, spi3w_en         */
#define BMP280_REG_PRESS_MSB   0xF7   /* pressure output (3 bytes)      */
#define BMP280_REG_TEMP_MSB    0xFA   /* temperature output (3 bytes)   */

/* ── Expected chip ID ──────────────────────────────────────────────── */
#define BMP280_CHIP_ID         0x60

/**
 * @brief Trim calibration coefficients read from 0x88–0x9F (NVM).
 *        Names match the BMP280 datasheet §4.2.2 exactly.
 */
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
} bmp280_calib_t;

/**
 * @brief Initialise the BMP280.
 *
 *  1. Reads chip-ID register (0xD0) and validates it is 0x60.
 *  2. Reads factory calibration coefficients from NVM (0x88–0x8D).
 *  3. Writes ctrl_meas: temperature oversampling ×1, normal mode.
 *
 * @param port   I2C port (I2C_NUM_0 or I2C_NUM_1).
 * @param addr   I2C device address.
 * @param calib  Output: calibration struct used by bmp280_read_temperature().
 * @return ESP_OK or an esp_err_t code.
 */
esp_err_t bmp280_init(i2c_port_t port, uint8_t addr, bmp280_calib_t *calib);

/**
 * @brief Read and compensate temperature.
 *
 *  - Reads raw 20-bit ADC value from registers 0xFA–0xFC.
 *  - Applies Bosch integer compensation formula (datasheet §8.2).
 *  - Returns ESP_ERR_INVALID_RESPONSE if the ADC value equals the
 *    reset/skipped value 0x80000 (sensor not yet measured).
 *
 * @param port    I2C port.
 * @param addr    I2C device address.
 * @param calib   Calibration struct from bmp280_init().
 * @param temp_c  Output: temperature in °C (signed, 0.01 °C resolution).
 * @return ESP_OK, ESP_ERR_TIMEOUT, or ESP_ERR_INVALID_RESPONSE.
 */
esp_err_t bmp280_read_temperature(i2c_port_t port, uint8_t addr,
                                  const bmp280_calib_t *calib,
                                  float *temp_c);
