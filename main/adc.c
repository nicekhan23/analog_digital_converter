/*
    A/D converter with running average & running hysteresis
*/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "esp_console.h"
#include "esp_log_buffer.h"
#include "esp_log_level.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include "esp_log.h"
#include "argtable3/argtable3.h"

#include "hal/adc_types.h"
#include "util.h"

#include "adc.h"

#define LOG_LEVEL_LOCAL ESP_LOG_INFO

#define ADC_UNIT                    ADC_UNIT_1
#define SAMPLE_FREQ_HZ 				20000
#define ADC_CHANNELS 1
#define READ_BUFFER_SIZE            1024

#define ADC_CONV_MODE               ADC_CONV_SINGLE_UNIT_1
#define ADC_ATTEN                   ADC_ATTEN_DB_12
#define ADC_BIT_WIDTH               SOC_ADC_DIGI_MAX_BITWIDTH

#define ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE1


static adc_channel_t channel[ADC_CHANNELS] = {ADC_CHANNEL_6};

#define ADC_MIN (0)
#define ADC_MAX (1 << 12)
#define ADC_HYST 40
#define RUNNING_AVG_SIZE 10

typedef struct
{
    uint32_t min;
    uint32_t max;
    uint32_t hysteresis;
} r_hyst_t;

typedef struct
{
    uint32_t queue[RUNNING_AVG_SIZE];
    uint8_t ptr;
} r_avg_t;

static const char* TAG = "ADC";
static TaskHandle_t task_handle;
static adc_continuous_handle_t handle = NULL;

static struct {
    uint32_t conversions;
    uint32_t invalid_channel;
    uint32_t read_errors;
    uint32_t timeout;
} errors;

static uint32_t normalized_value = 0;
static r_hyst_t r_hyst;
static r_avg_t r_avg;

static uint8_t result[READ_BUFFER_SIZE];


static void register_cmd();

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    //Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(task_handle, &mustYield);

    return (mustYield == pdTRUE);
}

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = READ_BUFFER_SIZE*4, 
        .conv_frame_size = READ_BUFFER_SIZE,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

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

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%"PRIx8, i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%"PRIx8, i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%"PRIx8, i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}

static uint32_t running_hyst(uint32_t input)
{
	ESP_LOGD(TAG, "Enter running_hyst, input:%"PRIu32"\n", input);
    if (input <= r_hyst.max && input >= r_hyst.min) 
        return r_hyst.min+(r_hyst.max - r_hyst.min) / 2;
	

    if (input > r_hyst.max)
    {
        r_hyst.max = MIN(input + (r_hyst.hysteresis / 2), ADC_MAX);
        r_hyst.min = r_hyst.max - r_hyst.hysteresis;
	    return input;
    }

    r_hyst.min = MAX(input - (r_hyst.hysteresis / 2), ADC_MIN);
    r_hyst.max = r_hyst.min + r_hyst.hysteresis;
	return input;
}

static uint32_t running_average(uint32_t input)
{
	ESP_LOGD(TAG, "Enter running_average, input:%"PRIu32, input);
    r_avg.queue[r_avg.ptr] = input;
    r_avg.ptr = (r_avg.ptr + 1) % RUNNING_AVG_SIZE;

    uint64_t sum = 0;
    for (int i = 0; i < RUNNING_AVG_SIZE; i++) {
        sum += r_avg.queue[i];
	}

    return (sum / RUNNING_AVG_SIZE);
}

static void task_adc(void *p){
	ESP_LOGD(TAG, "Enter task_adc");
    ESP_ERROR_CHECK(adc_continuous_start(handle));


    for(;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        esp_err_t ret;
        uint32_t ret_num;
	
        ret = adc_continuous_read(handle, result, READ_BUFFER_SIZE, &ret_num, 0);
        if (ret == ESP_OK) {
//            ESP_LOGI("TASK", "ret is %x, ret_num is %"PRIu32" bytes", ret, ret_num);
			//ESP_LOG_BUFFER_HEX(TAG, result, READ_BUFFER_SIZE);
            adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[0];
			//ESP_LOGI(TAG, "ch:%d, d:%d", p->type1.channel, p->type1.data);

			uint32_t raw=p->type1.data;
			normalized_value=running_average(running_hyst(raw));
			printf("\t\traw:%"PRIu32", normalized value:%4"PRIu32"\r", raw, normalized_value);

            ++errors.conversions;        
            continue;        
        } 
        if (ret == ESP_ERR_TIMEOUT) {
            ++errors.timeout;
            continue;
        }
        ++errors.read_errors;
    } 
}


BaseType_t adc_init() {
    esp_log_level_set(TAG,LOG_LEVEL_LOCAL); 
	ESP_LOGD(TAG, "Enter adc_init");
    continuous_adc_init(channel, ADC_CHANNELS, &handle);

    bzero(&errors, sizeof(errors));
	bzero(r_avg.queue, sizeof(r_avg.queue));
	r_avg.ptr=0;

	r_hyst.min=0;
	r_hyst.max=ADC_HYST;
	r_hyst.hysteresis=ADC_HYST;

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
	register_cmd();
    BaseType_t res = xTaskCreate(task_adc, "adc", 4096, NULL, uxTaskPriorityGet(NULL), &task_handle);
    return res;
}


// adc [-h|--help]  
#define CMD_ADC "adc"

struct {
	struct arg_lit *help;
    struct arg_int *channel                 // press ctrl+space after i expected result is arg_int
	struct arg_end *end;
} args;

static void print_help() {
	arg_print_syntax(stdout, (void *) &args, "\n");
	arg_print_glossary(stdout, (void *)&args, "%-25s \n");
}

static void print_status_rhyst() {
	printf("-- Running hystersis --\n");
	printf("min:%"PRIu32", max:%"PRIu32", hyst:%"PRIu32"\n", r_hyst.min, r_hyst.max, r_hyst.hysteresis);
}

static void print_status_r_avg() {
	printf("-- Running average --\n");
	printf("size:%d, ptr:%d, values:", RUNNING_AVG_SIZE, r_avg.ptr);
	for (int i=0;i<RUNNING_AVG_SIZE;i++)
		printf("%d: %"PRIu32", ", i, r_avg.queue[i]);
	printf("\n");
}

static void print_current() {
	printf("normalized:%"PRIu32"\n", normalized_value);
}

static void print_status() {
	print_status_rhyst();
	print_status_r_avg();
	print_current();
}

static int cmd_adc(int argc, char **argv) {
	int errors=arg_parse(argc, argv, (void *)&args);
	if (errors||args.help->count>0) {
		print_help();
		return 0;
	}

	print_status();
	return 0;

}

static void register_cmd() {
	args.help = arg_litn("h", "help", 0, 1, "A/D converter help");
	args.channel = arg_int0("c", "channel", "<channel>", "ADC channel number (0-based)");
	args.end = arg_end(2);
	
	esp_console_cmd_t cmd = {
		.argtable = &args,
		.command = CMD_ADC,
		.func = cmd_adc,
		.help = "A/D commands"
	};

	esp_console_cmd_register(&cmd);
}