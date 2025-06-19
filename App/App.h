#ifndef __APP__H__
#define __APP__H__

#include "main.h"

typedef enum
{
    ADC_ADS1251 = 0,
    ADC_ADS1231 = 1,
    ADC_T = 2
} ADC_enum;

#define COUNT_REG_SPI_BUF (3)
typedef struct 
{
    uint16_t spi_buf[3];
    int16_t  data_i16;
    int32_t  data_i32;
} ADC_struct;

#define PROGRAM_ADC_MAX_FILTER_ORDER   (10)
typedef struct {
    float value;
    float value_last;
    float valueRaw;
    float buf[PROGRAM_ADC_MAX_FILTER_ORDER];
    uint8_t bufIdx;
    uint8_t filter_N;
    uint8_t order;
}ADC_filter_typedef;

#define COUNT_ADC (3)
typedef struct 
{
    ADC_filter_typedef adc_filter[COUNT_ADC];
    ADC_struct ADC_ADS1251;
    ADC_struct ADC_ADS1231;
    int16_t ADC_T_data_i16;
} App_struct;

//------------------------------ FUNCTION ------------------------------//
void app_main();
void app_init();
void app_update_reg();
void app_adc_data_filter(uint32_t ADC_Buf_raw, ADC_enum ADC);
void app_adc_filter_init();
//---------------------------- FUNCTION END ----------------------------//

#endif