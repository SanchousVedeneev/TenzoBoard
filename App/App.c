#include "App.h"
#include "BSP.h"
#include "ProtocolMbRtuSlaveCtrl.h"

App_struct App;

void app_main(void)
{
  bsp_init();
  app_init();

  while (1) // основной цикл
  {
  asm ("nop");
  }
}

void app_init()
{
  protocolMbRtuSlaveCtrl_init(1);
  app_adc_filter_init();
  BSP_SLEEP_ON;
  return;
}

void app_adc_filter_init()
{
// -------------------------- ADC_ADS1251 -------------------------- //
    App.adc_filter[ADC_ADS1251].value = 0.0f;
    App.adc_filter[ADC_ADS1251].value_last = 0.0f;
    App.adc_filter[ADC_ADS1251].valueRaw = 0.0f;
    for (uint8_t j = 0; j < PROGRAM_ADC_MAX_FILTER_ORDER; j++)
    {
      App.adc_filter[ADC_ADS1251].buf[j] = 0.0f;
    }
    App.adc_filter[ADC_ADS1251].bufIdx = 0;
    App.adc_filter[ADC_ADS1251].filter_N = 10;
    App.adc_filter[ADC_ADS1251].order = 5;
// ------------------------ ADC_ADS1251 END ------------------------ //

// -------------------------- ADC_ADS1231 -------------------------- //
    App.adc_filter[ADC_ADS1231].value = 0.0f;
    App.adc_filter[ADC_ADS1231].value_last = 0.0f;
    App.adc_filter[ADC_ADS1231].valueRaw = 0.0f;
    for (uint8_t j = 0; j < PROGRAM_ADC_MAX_FILTER_ORDER; j++)
    {
      App.adc_filter[ADC_ADS1231].buf[j] = 0.0f;
    }
    App.adc_filter[ADC_ADS1231].bufIdx = 0;
    App.adc_filter[ADC_ADS1231].filter_N = 10;
    App.adc_filter[ADC_ADS1231].order = 5;
// ------------------------ ADC_ADS1231 END ------------------------ //
}

void bsp_tim7_1ms_callback()
{
  app_update_reg();
  protocolMbRtuSlaveCtrl_update_tables();
}

void app_update_reg()
{
  App.Mdb_data_AI.spi_buf_ADS1251[0] = SPI_data_rx_ADS1251[0];
  App.Mdb_data_AI.spi_buf_ADS1251[1] = SPI_data_rx_ADS1251[1];
  App.Mdb_data_AI.spi_buf_ADS1251[2] = SPI_data_rx_ADS1251[2];

  App.Mdb_data_AI.ADC_T_data_i16 = App.adc_filter[ADC_T].value;

  App.Mdb_data_AI.ADC_ADS1251_data_i16 = App.adc_filter[ADC_ADS1251].value;
  App.Mdb_data_AI.ADC_ADS1251_data_i32 = (uint32_t)(App.adc_filter[ADC_ADS1251].value*10.0f);
}

void bsp_ADC_data_ready()
{
  uint16_t ADC_T_data = 0;
  
  ADC_T_data = (uint16_t)(HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1)/16);
  //@do
  // Дописать обработчик для значения температуры c АЦП

  BSP_SLEEP_OFF;
  HAL_Delay(1);
  app_adc_data_filter(bsp_get_data_spi_ads1251(), ADC_ADS1251);


  BSP_SLEEP_ON;
  BSP_LED_TOGGLE(BSP_LED_1);
}

#define ADC_ADS1251_MAX_VAL   (float)(8388607.0f)
#define ADC_ADS1251_REF_VOLT  (float)(4.0f)

#define ADC_ADS1231_MAX_VAL   (float)(8388607.0f)
#define ADC_ADS1231_REF_VOLT  (float)(4.0f)

void app_adc_data_filter(uint32_t ADC_Buf_raw, ADC_enum adc)
{
  float value = 0.0f;
  float valueLast = 0.0f;
  float kFilter = 0.0f;
  float data = 0.0f;
  float sum = 0.0f;
  if (adc == ADC_ADS1251)
  {
    data = ((float)ADC_Buf_raw / ADC_ADS1251_MAX_VAL * ADC_ADS1251_REF_VOLT * 1000.0f);
  }
  else if (adc == ADC_ADS1231)
  {
    data = ((float)ADC_Buf_raw / ADC_ADS1231_MAX_VAL * ADC_ADS1231_REF_VOLT * 1000.0f);
  }

  App.adc_filter[adc].buf[App.adc_filter[adc].bufIdx++] = data;

  if (App.adc_filter[adc].bufIdx == App.adc_filter[adc].order) 
  {
    App.adc_filter[adc].bufIdx = 0;
  }

  for(uint8_t idx = 0; idx < App.adc_filter[adc].order; idx++)
  {
    sum += App.adc_filter[adc].buf[idx];
  }

  App.adc_filter[adc].valueRaw = sum / App.adc_filter[adc].order;
  
  //--------------------//
  value = App.adc_filter[adc].valueRaw;
  valueLast = App.adc_filter[adc].value_last;
  kFilter = 2.0f / ((float)App.adc_filter[adc].filter_N + 1.0f);
  value = valueLast + kFilter*(value - valueLast);
  App.adc_filter[adc].value = value;
  App.adc_filter[adc].value_last = value;
}



