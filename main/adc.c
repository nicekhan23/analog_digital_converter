/**
 * @file adc.c
 * @brief Multi-channel A/D converter with running average & running hysteresis
 * 
 * This implementation provides thread-safe, NULL-safe ADC operations with
 * configurable channels, calibration, and filtering.
 */

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "esp_console.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include "argtable3/argtable3.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "hal/adc_types.h"
#include "util.h"
#include "adc.h"

#define LOG_LEVEL_LOCAL ESP_LOG_INFO

/* ADC Hardware Configuration */
#define ADC_UNIT                    ADC_UNIT_1
#define SAMPLE_FREQ_HZ              20000
#define READ_BUFFER_SIZE            1024
#define ADC_CONV_MODE               ADC_CONV_SINGLE_UNIT_1
#define ADC_ATTEN                   ADC_ATTEN_DB_12
#define ADC_BIT_WIDTH               SOC_ADC_DIGI_MAX_BITWIDTH
#define ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE1

/* Configuration from Kconfig */
#ifndef CONFIG_ADC_MAX_CHANNELS
#define ADC_MAX_CHANNELS 4
#else
#define ADC_MAX_CHANNELS CONFIG_ADC_MAX_CHANNELS
#endif

#ifndef CONFIG_ADC_RUNNING_AVG_SIZE
#define RUNNING_AVG_SIZE 10
#else
#define RUNNING_AVG_SIZE CONFIG_ADC_RUNNING_AVG_SIZE
#endif

#define ADC_MIN (0)
#define ADC_MAX (1 << 12)

/* NVS Keys */
#define NVS_NAMESPACE "adc_storage"
#define NVS_KEY_MIN_FMT "ch%d_min"
#define NVS_KEY_MAX_FMT "ch%d_max"
#define NVS_KEY_HYST_FMT "ch%d_hyst"

/**
 * @brief Running hysteresis structure for one channel
 */
typedef struct {
    uint32_t min;           /**< Minimum threshold */
    uint32_t max;           /**< Maximum threshold */
    uint32_t hysteresis;    /**< Hysteresis value */
} r_hyst_t;

/**
 * @brief Running average structure for one channel
 */
typedef struct {
    uint32_t queue[RUNNING_AVG_SIZE];  /**< Circular buffer for averaging */
    uint8_t ptr;                        /**< Current position in buffer */
} r_avg_t;

/**
 * @brief Per-channel ADC data structure
 */
typedef struct {
    uint32_t raw_value;         /**< Latest raw ADC value */
    uint32_t normalized_value;  /**< Processed value */
    r_hyst_t r_hyst;           /**< Hysteresis state */
    r_avg_t r_avg;             /**< Running average state */
    uint32_t min_cal;          /**< Calibration minimum */
    uint32_t max_cal;          /**< Calibration maximum */
} adc_channel_data_t;

/* Module static variables */
static const char* TAG = "ADC";
static TaskHandle_t task_handle = NULL;
static adc_continuous_handle_t handle = NULL;
static SemaphoreHandle_t adc_mutex = NULL;

/* Channel configuration - map to physical ADC channels */
static const adc_channel_t physical_channels[ADC_MAX_CHANNELS] = {
    ADC_CHANNEL_6,  /* GPIO34 */
    ADC_CHANNEL_7,  /* GPIO35 */
#if ADC_MAX_CHANNELS >= 3
    ADC_CHANNEL_4,  /* GPIO32 */
#endif
#if ADC_MAX_CHANNELS >= 4
    ADC_CHANNEL_5,  /* GPIO33 */
#endif
#if ADC_MAX_CHANNELS >= 5
    ADC_CHANNEL_0,  /* GPIO36 */
#endif
#if ADC_MAX_CHANNELS >= 6
    ADC_CHANNEL_3,  /* GPIO39 */
#endif
};

/* Per-channel data */
static adc_channel_data_t channel_data[ADC_MAX_CHANNELS];

/* Error statistics */
static struct {
    uint32_t conversions;
    uint32_t invalid_channel;
    uint32_t read_errors;
    uint32_t timeout;
} errors;

static uint8_t result[READ_BUFFER_SIZE];

/* Forward declarations */
static void register_cmd(void);
static esp_err_t save_channel_config(uint8_t channel);
static esp_err_t load_channel_config(uint8_t channel);

/**
 * @brief Check if channel index is valid
 * 
 * @param[in] channel Channel index to check
 * @return true if valid, false otherwise
 */
static inline bool chk_chn(uint8_t channel) {
    return (channel < ADC_MAX_CHANNELS);
}

/**
 * @brief ADC conversion done callback (ISR context)
 * 
 * @param[in] handle ADC handle
 * @param[in] edata Event data
 * @param[in] user_data User data pointer
 * @return true if higher priority task was woken
 */
static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, 
                                     const adc_continuous_evt_data_t *edata, 
                                     void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    vTaskNotifyGiveFromISR(task_handle, &mustYield);
    return (mustYield == pdTRUE);
}

/**
 * @brief Initialize ADC hardware
 * 
 * @param[in] channel Array of ADC channels
 * @param[in] channel_num Number of channels
 * @param[out] out_handle Pointer to store ADC handle
 */
static void continuous_adc_init(const adc_channel_t *channel, uint8_t channel_num, 
                               adc_continuous_handle_t *out_handle)
{
    if (!channel || !out_handle) {
        ESP_LOGE(TAG, "continuous_adc_init: NULL pointer");
        return;
    }

    adc_continuous_handle_t adc_handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = READ_BUFFER_SIZE * 4,
        .conv_frame_size = READ_BUFFER_SIZE,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = SAMPLE_FREQ_HZ,
        .conv_mode = ADC_CONV_MODE,
        .format = ADC_OUTPUT_TYPE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    
    for (int i = 0; i < channel_num; i++) {
        adc_pattern[i].atten = ADC_ATTEN;
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = ADC_UNIT;
        adc_pattern[i].bit_width = ADC_BIT_WIDTH;

        ESP_LOGI(TAG, "Channel[%d]: atten=%d, channel=%d, unit=%d", 
                 i, adc_pattern[i].atten, adc_pattern[i].channel, adc_pattern[i].unit);
    }
    
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig_cfg));

    *out_handle = adc_handle;
}

/**
 * @brief Apply running hysteresis to input value
 * 
 * @param[in] channel Channel index
 * @param[in] input Input ADC value
 * @return Filtered value
 */
static uint32_t running_hyst(uint8_t channel, uint32_t input)
{
    if (!chk_chn(channel)) {
        return input;
    }

    r_hyst_t *hyst = &channel_data[channel].r_hyst;
    
    ESP_LOGD(TAG, "Ch%d running_hyst, input:%"PRIu32, channel, input);
    
    if (input <= hyst->max && input >= hyst->min) {
        return hyst->min + (hyst->max - hyst->min) / 2;
    }

    if (input > hyst->max) {
        hyst->max = MIN(input + (hyst->hysteresis / 2), 
                       channel_data[channel].max_cal);
        hyst->min = (hyst->max > hyst->hysteresis) ? 
                    (hyst->max - hyst->hysteresis) : 0;
        return input;
    }

    hyst->min = MAX(input - (hyst->hysteresis / 2), 
                   channel_data[channel].min_cal);
    hyst->max = MIN(hyst->min + hyst->hysteresis, 
                   channel_data[channel].max_cal);
    return input;
}

/**
 * @brief Calculate running average
 * 
 * @param[in] channel Channel index
 * @param[in] input Input value
 * @return Averaged value
 */
static uint32_t running_average(uint8_t channel, uint32_t input)
{
    if (!chk_chn(channel)) {
        return input;
    }

    r_avg_t *avg = &channel_data[channel].r_avg;
    
    ESP_LOGD(TAG, "Ch%d running_average, input:%"PRIu32, channel, input);
    
    avg->queue[avg->ptr] = input;
    avg->ptr = (avg->ptr + 1) % RUNNING_AVG_SIZE;

    uint64_t sum = 0;
    for (int i = 0; i < RUNNING_AVG_SIZE; i++) {
        sum += avg->queue[i];
    }

    return (sum / RUNNING_AVG_SIZE);
}

/**
 * @brief ADC processing task
 * 
 * @param[in] p Task parameter (unused)
 */
static void task_adc(void *p)
{
    ESP_LOGD(TAG, "Enter task_adc");
    ESP_ERROR_CHECK(adc_continuous_start(handle));

    for(;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        esp_err_t ret;
        uint32_t ret_num;
    
        ret = adc_continuous_read(handle, result, READ_BUFFER_SIZE, &ret_num, 0);
        
        if (ret == ESP_OK) {
            errors.conversions++;
            
            /* Process all samples in the buffer */
            for (uint32_t i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
                
                /* Find which channel this sample belongs to */
                for (uint8_t ch = 0; ch < ADC_MAX_CHANNELS; ch++) {
                    if ((physical_channels[ch] & 0x7) == p->type1.channel) {
                        if (xSemaphoreTake(adc_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                            uint32_t raw = p->type1.data;
                            channel_data[ch].raw_value = raw;
                            channel_data[ch].normalized_value = 
                                running_average(ch, running_hyst(ch, raw));
                            xSemaphoreGive(adc_mutex);
                        }
                        break;
                    }
                }
            }
        } else if (ret == ESP_ERR_TIMEOUT) {
            errors.timeout++;
        } else {
            errors.read_errors++;
        }
    }
}

/**
 * @brief Save channel configuration to NVS
 * 
 * @param[in] channel Channel index
 * @return ESP_OK on success
 */
static esp_err_t save_channel_config(uint8_t channel)
{
    if (!chk_chn(channel)) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        return err;
    }

    char key[16];
    
    /* Save min value */
    snprintf(key, sizeof(key), NVS_KEY_MIN_FMT, channel);
    err = nvs_set_u32(nvs, key, channel_data[channel].min_cal);
    if (err != ESP_OK) goto cleanup;
    
    /* Save max value */
    snprintf(key, sizeof(key), NVS_KEY_MAX_FMT, channel);
    err = nvs_set_u32(nvs, key, channel_data[channel].max_cal);
    if (err != ESP_OK) goto cleanup;
    
    /* Save hysteresis */
    snprintf(key, sizeof(key), NVS_KEY_HYST_FMT, channel);
    err = nvs_set_u32(nvs, key, channel_data[channel].r_hyst.hysteresis);
    if (err != ESP_OK) goto cleanup;
    
    err = nvs_commit(nvs);

cleanup:
    nvs_close(nvs);
    return err;
}

/**
 * @brief Load channel configuration from NVS
 * 
 * @param[in] channel Channel index
 * @return ESP_OK on success
 */
static esp_err_t load_channel_config(uint8_t channel)
{
    if (!chk_chn(channel)) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        return err;
    }

    char key[16];
    uint32_t value;
    
    /* Load min value */
    snprintf(key, sizeof(key), NVS_KEY_MIN_FMT, channel);
    err = nvs_get_u32(nvs, key, &value);
    if (err == ESP_OK) {
        channel_data[channel].min_cal = value;
    }
    
    /* Load max value */
    snprintf(key, sizeof(key), NVS_KEY_MAX_FMT, channel);
    err = nvs_get_u32(nvs, key, &value);
    if (err == ESP_OK) {
        channel_data[channel].max_cal = value;
    }
    
    /* Load hysteresis */
    snprintf(key, sizeof(key), NVS_KEY_HYST_FMT, channel);
    err = nvs_get_u32(nvs, key, &value);
    if (err == ESP_OK) {
        channel_data[channel].r_hyst.hysteresis = value;
    }

    nvs_close(nvs);
    return ESP_OK;
}

/**
 * @brief Public API implementations
 */

BaseType_t adc_init(void)
{
    esp_log_level_set(TAG, LOG_LEVEL_LOCAL);
    ESP_LOGD(TAG, "Enter adc_init");

    /* Create mutex */
    adc_mutex = xSemaphoreCreateMutex();
    if (adc_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return pdFAIL;
    }

    /* Initialize hardware */
    continuous_adc_init(physical_channels, ADC_MAX_CHANNELS, &handle);

    /* Initialize statistics */
    bzero(&errors, sizeof(errors));

    /* Initialize channel data with defaults from Kconfig */
    const uint32_t default_mins[] = {
        CONFIG_ADC_CH0_MIN, CONFIG_ADC_CH1_MIN,
#if ADC_MAX_CHANNELS >= 3
        CONFIG_ADC_CH2_MIN,
#endif
#if ADC_MAX_CHANNELS >= 4
        CONFIG_ADC_CH3_MIN,
#endif
#if ADC_MAX_CHANNELS >= 5
        CONFIG_ADC_CH4_MIN,
#endif
#if ADC_MAX_CHANNELS >= 6
        CONFIG_ADC_CH5_MIN,
#endif
    };

    const uint32_t default_maxs[] = {
        CONFIG_ADC_CH0_MAX, CONFIG_ADC_CH1_MAX,
#if ADC_MAX_CHANNELS >= 3
        CONFIG_ADC_CH2_MAX,
#endif
#if ADC_MAX_CHANNELS >= 4
        CONFIG_ADC_CH3_MAX,
#endif
#if ADC_MAX_CHANNELS >= 5
        CONFIG_ADC_CH4_MAX,
#endif
#if ADC_MAX_CHANNELS >= 6
        CONFIG_ADC_CH5_MAX,
#endif
    };

    for (uint8_t ch = 0; ch < ADC_MAX_CHANNELS; ch++) {
        bzero(&channel_data[ch], sizeof(adc_channel_data_t));
        
        /* Set defaults */
        channel_data[ch].min_cal = default_mins[ch];
        channel_data[ch].max_cal = default_maxs[ch];
        channel_data[ch].r_hyst.min = default_mins[ch];
        channel_data[ch].r_hyst.max = default_mins[ch] + CONFIG_ADC_HYSTERESIS;
        channel_data[ch].r_hyst.hysteresis = CONFIG_ADC_HYSTERESIS;
        
        /* Try to load from NVS */
        load_channel_config(ch);
        
        ESP_LOGI(TAG, "Ch%d: min=%"PRIu32", max=%"PRIu32", hyst=%"PRIu32,
                 ch, channel_data[ch].min_cal, channel_data[ch].max_cal,
                 channel_data[ch].r_hyst.hysteresis);
    }

    /* Register callbacks */
    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));

    /* Register commands */
    register_cmd();

    /* Create task */
    BaseType_t res = xTaskCreate(task_adc, "adc", 4096, NULL, 
                                 uxTaskPriorityGet(NULL), &task_handle);
    
    return res;
}

BaseType_t adc_deinit(void)
{
    if (task_handle) {
        vTaskDelete(task_handle);
        task_handle = NULL;
    }
    
    if (handle) {
        adc_continuous_stop(handle);
        adc_continuous_deinit(handle);
        handle = NULL;
    }
    
    if (adc_mutex) {
        vSemaphoreDelete(adc_mutex);
        adc_mutex = NULL;
    }
    
    return pdPASS;
}

esp_err_t adc_get_normalized(uint8_t channel, uint32_t *v, TickType_t wait)
{
    if (!chk_chn(channel) || v == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(adc_mutex, wait) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    *v = channel_data[channel].normalized_value;
    xSemaphoreGive(adc_mutex);
    
    return ESP_OK;
}

esp_err_t adc_get_raw(uint8_t channel, uint32_t *v, TickType_t wait)
{
    if (!chk_chn(channel) || v == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(adc_mutex, wait) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    *v = channel_data[channel].raw_value;
    xSemaphoreGive(adc_mutex);
    
    return ESP_OK;
}

esp_err_t adc_set_calibration(uint8_t channel, uint32_t min, uint32_t max)
{
    if (!chk_chn(channel) || min >= max || max > ADC_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(adc_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    channel_data[channel].min_cal = min;
    channel_data[channel].max_cal = max;
    channel_data[channel].r_hyst.min = min;
    channel_data[channel].r_hyst.max = MIN(min + channel_data[channel].r_hyst.hysteresis, max);
    
    xSemaphoreGive(adc_mutex);

    return save_channel_config(channel);
}

esp_err_t adc_get_calibration(uint8_t channel, uint32_t *min, uint32_t *max)
{
    if (!chk_chn(channel) || min == NULL || max == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(adc_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    *min = channel_data[channel].min_cal;
    *max = channel_data[channel].max_cal;
    
    xSemaphoreGive(adc_mutex);
    
    return ESP_OK;
}

esp_err_t adc_set_hysteresis(uint8_t channel, uint32_t hysteresis)
{
    if (!chk_chn(channel) || hysteresis == 0 || hysteresis > 1000) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(adc_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    channel_data[channel].r_hyst.hysteresis = hysteresis;
    
    xSemaphoreGive(adc_mutex);

    return save_channel_config(channel);
}

esp_err_t adc_get_hysteresis(uint8_t channel, uint32_t *hysteresis)
{
    if (!chk_chn(channel) || hysteresis == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(adc_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    *hysteresis = channel_data[channel].r_hyst.hysteresis;
    
    xSemaphoreGive(adc_mutex);
    
    return ESP_OK;
}

/**
 * @brief Command line interface implementation
 */

/* Command arguments */
struct {
    struct arg_lit *help;
    struct arg_int *channel;
    struct arg_int *min;
    struct arg_int *max;
    struct arg_int *hyst;
    struct arg_lit *status;
    struct arg_lit *calibrate;
    struct arg_lit *errors_flag;
    struct arg_end *end;
} args;

/**
 * @brief Print command help
 */
static void print_help(void)
{
    printf("ADC Multi-Channel Control\n");
    arg_print_syntax(stdout, (void *) &args, "\n");
    arg_print_glossary(stdout, (void *)&args, "  %-25s %s\n");
}

/**
 * @brief Print channel status
 * 
 * @param[in] channel Channel index
 */
static void print_channel_status(uint8_t channel)
{
    if (!chk_chn(channel)) {
        printf("Invalid channel %d\n", channel);
        return;
    }

    uint32_t raw, norm, min, max, hyst;
    
    if (adc_get_raw(channel, &raw, pdMS_TO_TICKS(100)) != ESP_OK) {
        printf("Ch%d: Failed to read raw value\n", channel);
        return;
    }
    
    if (adc_get_normalized(channel, &norm, pdMS_TO_TICKS(100)) != ESP_OK) {
        printf("Ch%d: Failed to read normalized value\n", channel);
        return;
    }
    
    if (adc_get_calibration(channel, &min, &max) != ESP_OK) {
        printf("Ch%d: Failed to read calibration\n", channel);
        return;
    }
    
    if (adc_get_hysteresis(channel, &hyst) != ESP_OK) {
        printf("Ch%d: Failed to read hysteresis\n", channel);
        return;
    }

    printf("-- Channel %d --\n", channel);
    printf("  Raw: %"PRIu32"\n", raw);
    printf("  Normalized: %"PRIu32"\n", norm);
    printf("  Calibration: min=%"PRIu32", max=%"PRIu32"\n", min, max);
    printf("  Hysteresis: %"PRIu32"\n", hyst);
}

/**
 * @brief Print error statistics
 */
static void print_errors(void)
{
    printf("-- Error Statistics --\n");
    printf("  Conversions: %"PRIu32"\n", errors.conversions);
    printf("  Invalid channel: %"PRIu32"\n", errors.invalid_channel);
    printf("  Read errors: %"PRIu32"\n", errors.read_errors);
    printf("  Timeouts: %"PRIu32"\n", errors.timeout);
}

/**
 * @brief ADC command handler
 */
static int cmd_adc(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void *)&args);
    
    if (nerrors || args.help->count > 0) {
        print_help();
        return 0;
    }

    /* Handle status request */
    if (args.status->count > 0) {
        if (args.channel->count > 0) {
            print_channel_status(args.channel->ival[0]);
        } else {
            for (uint8_t ch = 0; ch < ADC_MAX_CHANNELS; ch++) {
                print_channel_status(ch);
            }
        }
        return 0;
    }

    /* Handle error statistics */
    if (args.errors_flag->count > 0) {
        print_errors();
        return 0;
    }

    /* Handle calibration */
    if (args.calibrate->count > 0) {
        if (args.channel->count == 0) {
            printf("Channel required for calibration\n");
            return 1;
        }
        
        uint8_t ch = args.channel->ival[0];
        
        if (args.min->count > 0 && args.max->count > 0) {
            esp_err_t err = adc_set_calibration(ch, args.min->ival[0], args.max->ival[0]);
            if (err == ESP_OK) {
                printf("Ch%d calibration set: min=%d, max=%d\n", 
                       ch, args.min->ival[0], args.max->ival[0]);
            } else {
                printf("Failed to set calibration: %s\n", esp_err_to_name(err));
                return 1;
            }
        }
        
        if (args.hyst->count > 0) {
            esp_err_t err = adc_set_hysteresis(ch, args.hyst->ival[0]);
            if (err == ESP_OK) {
                printf("Ch%d hysteresis set: %d\n", ch, args.hyst->ival[0]);
            } else {
                printf("Failed to set hysteresis: %s\n", esp_err_to_name(err));
                return 1;
            }
        }
        
        return 0;
    }

    print_help();
    return 0;
}

/**
 * @brief Register ADC commands
 */
static void register_cmd(void)
{
    args.help = arg_litn("h", "help", 0, 1, "Show help");
    args.channel = arg_int0("c", "channel", "<0-5>", "Channel number");
    args.min = arg_int0("m", "min", "<value>", "Minimum calibration value");
    args.max = arg_int0("M", "max", "<value>", "Maximum calibration value");
    args.hyst = arg_int0("y", "hyst", "<value>", "Hysteresis value");
    args.status = arg_litn("s", "status", 0, 1, "Show channel status");
    args.calibrate = arg_litn("C", "calibrate", 0, 1, "Set calibration");
    args.errors_flag = arg_litn("e", "errors", 0, 1, "Show error statistics");
    args.end = arg_end(8);
    
    esp_console_cmd_t cmd = {
        .argtable = &args,
        .command = "adc",
        .func = cmd_adc,
        .help = "Multi-channel ADC control\n"
                "Examples:\n"
                "  adc -s              Show all channels\n"
                "  adc -s -c 0         Show channel 0\n"
                "  adc -C -c 0 -m 100 -M 3900  Calibrate channel 0\n"
                "  adc -C -c 1 -y 50   Set hysteresis for channel 1\n"
                "  adc -e              Show error statistics\n"
    };

    esp_console_cmd_register(&cmd);
}