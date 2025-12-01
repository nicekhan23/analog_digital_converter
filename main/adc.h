#ifndef ADC_H
#define ADC_H

#include "freertos/FreeRTOS.h"

BaseType_t adc_init();

BaseType_t adc_deinit();

BaseType_t adc_get(uint32_t *v, TickType_t wait);

#endif /* ADC_H */