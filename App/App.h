#ifndef __APP__H__
#define __APP__H__

#include "main.h"

#define PROGRAM_ADC_MAX_FILTER_ORDER         (24)

typedef enum
{
    ADC_ADS1251 = 0,
    ADC_CPU = 1,
    ADC_T = 2
} ADC_enum;

typedef struct 
{
    uint16_t state_led_rele;
    uint16_t control_led_rele; 
    uint16_t spi_buf_0[3];
    uint16_t ADC_CPU_data;
    uint16_t ADC_T_data;
    uint16_t ADC_ADS1251_data_u16;
    uint32_t ADC_ADS1251_data_u32;
} Mdb_data_AO_struct;

typedef struct {
    float value;
    float value_last;
    float valueRaw;
    float buf[PROGRAM_ADC_MAX_FILTER_ORDER];
    uint8_t bufIdx;
    uint8_t filter_N;
    uint8_t order;
}ADC_filter_typedef;

typedef struct 
{
    Mdb_data_AO_struct Mdb_data_AO;
    ADC_filter_typedef adc_filter[3];
} App_struct;

//------------------------------ FUNCTION ------------------------------//
void app_main();
void app_init();
void app_update_reg();
void app_adc_data_filter(uint32_t ADC_Buf_raw, ADC_enum ADC);
void app_adc_filter_init();
void app_control_led_rele(uint16_t control_led_rele);
//---------------------------- FUNCTION END ----------------------------//

#endif