/**
 * @file adc.h
 * @brief Multi-channel ADC driver with running average and hysteresis
 * 
 * This module provides ADC functionality with support for multiple channels,
 * running average filtering, and hysteresis detection. All functions are
 * NULL-safe and use mutex protection for thread safety.
 */

#ifndef ADC_H
#define ADC_H

#include "freertos/FreeRTOS.h"
#include "esp_err.h"

/**
 * @brief Initialize the ADC subsystem
 * 
 * Initializes all configured ADC channels, creates the processing task,
 * and loads calibration data from NVS flash memory.
 * 
 * @return pdPASS if initialization successful, pdFAIL otherwise
 * @note This function is NULL-safe
 */
BaseType_t adc_init(void);

/**
 * @brief Deinitialize the ADC subsystem
 * 
 * Stops the ADC task and releases all resources.
 * 
 * @return pdPASS if deinitialization successful, pdFAIL otherwise
 * @note This function is NULL-safe
 */
BaseType_t adc_deinit(void);

/**
 * @brief Get normalized ADC value for a specific channel
 * 
 * Returns the processed ADC value after applying running average
 * and hysteresis filtering. The value is normalized based on the
 * channel's min/max configuration.
 * 
 * @param[in] channel Channel index (0 to ADC_MAX_CHANNELS-1)
 * @param[out] v Pointer to store the normalized value
 * @param[in] wait Maximum time to wait for the mutex (in ticks)
 * @return ESP_OK if successful
 *         ESP_ERR_INVALID_ARG if channel is invalid or v is NULL
 *         ESP_ERR_TIMEOUT if mutex timeout
 * @note This function is NULL-safe and thread-safe
 */
esp_err_t adc_get_normalized(uint8_t channel, uint32_t *v, TickType_t wait);

/**
 * @brief Get raw ADC value for a specific channel
 * 
 * Returns the raw ADC value without normalization.
 * 
 * @param[in] channel Channel index (0 to ADC_MAX_CHANNELS-1)
 * @param[out] v Pointer to store the raw value
 * @param[in] wait Maximum time to wait for the mutex (in ticks)
 * @return ESP_OK if successful
 *         ESP_ERR_INVALID_ARG if channel is invalid or v is NULL
 *         ESP_ERR_TIMEOUT if mutex timeout
 * @note This function is NULL-safe and thread-safe
 */
esp_err_t adc_get_raw(uint8_t channel, uint32_t *v, TickType_t wait);

/**
 * @brief Set calibration parameters for a channel
 * 
 * Sets the minimum and maximum values for ADC calibration and
 * stores them in NVS flash memory.
 * 
 * @param[in] channel Channel index (0 to ADC_MAX_CHANNELS-1)
 * @param[in] min Minimum ADC value (0-4095)
 * @param[in] max Maximum ADC value (0-4095)
 * @return ESP_OK if successful
 *         ESP_ERR_INVALID_ARG if channel or values are invalid
 * @note This function is NULL-safe and thread-safe
 */
esp_err_t adc_set_calibration(uint8_t channel, uint32_t min, uint32_t max);

/**
 * @brief Get calibration parameters for a channel
 * 
 * Retrieves the current minimum and maximum calibration values.
 * 
 * @param[in] channel Channel index (0 to ADC_MAX_CHANNELS-1)
 * @param[out] min Pointer to store minimum value
 * @param[out] max Pointer to store maximum value
 * @return ESP_OK if successful
 *         ESP_ERR_INVALID_ARG if channel is invalid or pointers are NULL
 * @note This function is NULL-safe and thread-safe
 */
esp_err_t adc_get_calibration(uint8_t channel, uint32_t *min, uint32_t *max);

/**
 * @brief Set hysteresis value for a channel
 * 
 * Sets the hysteresis threshold and stores it in NVS flash.
 * 
 * @param[in] channel Channel index (0 to ADC_MAX_CHANNELS-1)
 * @param[in] hysteresis Hysteresis value (1-1000)
 * @return ESP_OK if successful
 *         ESP_ERR_INVALID_ARG if channel or hysteresis is invalid
 * @note This function is NULL-safe and thread-safe
 */
esp_err_t adc_set_hysteresis(uint8_t channel, uint32_t hysteresis);

/**
 * @brief Get hysteresis value for a channel
 * 
 * Retrieves the current hysteresis threshold.
 * 
 * @param[in] channel Channel index (0 to ADC_MAX_CHANNELS-1)
 * @param[out] hysteresis Pointer to store hysteresis value
 * @return ESP_OK if successful
 *         ESP_ERR_INVALID_ARG if channel is invalid or pointer is NULL
 * @note This function is NULL-safe and thread-safe
 */
esp_err_t adc_get_hysteresis(uint8_t channel, uint32_t *hysteresis);

#endif /* ADC_H */