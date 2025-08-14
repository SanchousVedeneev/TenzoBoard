#include "App.h"
#include "BSP.h"
#include "ProtocolMbRtuSlaveCtrl.h"

App_struct App;

// @do
// Добавить сохранение во Flash

void app_main(void)
{
  bsp_init();
  app_init();
    while (1) // основной цикл
  {
    HAL_Delay(500);
  }
}

void app_init()
{
  protocolMbRtuSlaveCtrl_init(1);
  app_setupParam_init();
  app_adc_filter_init();
  BSP_PWR_TENZO_ON;
  return;
}

void app_setupParam_init()
{
  app_setupParam_setDefolt();
}

void app_setupParam_setDefolt()
{
  App.setupParam.ADC_ADS1251_order = 2;
  App.setupParam.ADC_ADS1251_filterN = 2;

  App.setupParam.ADC_ADS1231_order = 2;
  App.setupParam.ADC_ADS1231_filterN = 2;

  App.setupParam.ADC_T_order = 2;
  App.setupParam.ADC_T_filterN = 2;
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
    App.adc_filter[ADC_ADS1251].filter_N = App.setupParam.ADC_ADS1251_filterN;
    App.adc_filter[ADC_ADS1251].order = App.setupParam.ADC_ADS1251_order;
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
    App.adc_filter[ADC_ADS1231].filter_N = App.setupParam.ADC_ADS1231_filterN;
    App.adc_filter[ADC_ADS1231].order = App.setupParam.ADC_ADS1231_order;
// ------------------------ ADC_ADS1231 END ------------------------ //

// ----------------------------- ADC_T ----------------------------- //
    App.adc_filter[ADC_T].value = 0.0f;
    App.adc_filter[ADC_T].value_last = 0.0f;
    App.adc_filter[ADC_T].valueRaw = 0.0f;
    for (uint8_t j = 0; j < PROGRAM_ADC_MAX_FILTER_ORDER; j++)
    {
      App.adc_filter[ADC_T].buf[j] = 0.0f;
    }
    App.adc_filter[ADC_T].bufIdx = 0;
    App.adc_filter[ADC_T].filter_N = App.setupParam.ADC_T_filterN;
    App.adc_filter[ADC_T].order = App.setupParam.ADC_T_order;
// --------------------------- ADC_T END --------------------------- //

}

#define GET_ADC_VALUE_PERIOD (uint16_t)(0U)
void bsp_tim7_1000ms_callback()
{
  static uint16_t cnt_1ms = 0;

  if (cnt_1ms++ >= GET_ADC_VALUE_PERIOD)
  {
    BSP_PWR_TENZO_ON;
    BSP_LED_ON(BSP_LED_1);

    app_adc_data_filter(bsp_get_data_spi_ads1251(), ADC_ADS1251);
    app_adc_data_filter(bsp_get_data_spi_ads1231(), ADC_ADS1231);
    
    BSP_LED_OFF(BSP_LED_1);
    BSP_PWR_TENZO_OFF;

    app_update_reg();
    protocolMbRtuSlaveCtrl_update_tables();
    cnt_1ms = 0;
  }
  return;
}

#define K_X10      (float)(10.0f)
#define K_X1000    (float)(1000.0f)
#define K_X10000   (float)(10000.0f)
void app_update_reg()
{
  // --- ADC_ADS1251
  for (uint8_t i = 0; i < COUNT_REG_SPI_BUF; i++)
  {
    App.ADC_ADS1251.spi_buf[i] = SPI_data_rx_ADS1251[i];
  }
  App.ADC_ADS1251.data_i16 = (int16_t)(App.adc_filter[ADC_ADS1251].value * K_X1000);  // [В*1000]
  App.ADC_ADS1251.data_i32 = (int32_t)(App.adc_filter[ADC_ADS1251].value * K_X10000); // [В*10000]
  
  // --- ADC_ADS1231
  for (uint8_t i = 0; i < COUNT_REG_SPI_BUF; i++)
  {
    App.ADC_ADS1231.spi_buf[i] = SPI_data_rx_ADS1231[i];
  }
  App.ADC_ADS1231.data_i16 = (int16_t)(App.adc_filter[ADC_ADS1231].value * K_X1000);  // [В*1000]
  App.ADC_ADS1231.data_i32 = (int32_t)(App.adc_filter[ADC_ADS1231].value * K_X10000); // [В*10000]

  // -- ADC_T
  App.ADC_T_data_i16 = (int16_t)(App.adc_filter[ADC_T].value * K_X10);                // [C*10]
}

void bsp_ADC_data_ready()
{
  app_adc_data_filter(((HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1)/16)), ADC_T);
}

#define ADC_24BIT_FUL_SCALE   (float)(0x16777215UL)

#define ADC_ADS1251_MAX_VAL   (float)(8388607.0f)
#define ADC_ADS1251_REF_VOLT  (float)(4.096f)

#define ADC_ADS1231_MAX_VAL   (float)(4194303.0f)
#define ADC_ADS1231_REF_VOLT  (float)(2.5f)

void app_adc_data_filter(uint32_t ADC_Buf_raw, ADC_enum adc)
{
  float value = 0.0f;
  float valueLast = 0.0f;
  float kFilter = 0.0f;
  float data = 0.0f;
  float sum = 0.0f;

  if (adc == ADC_ADS1251)
  {
    // Положительное напряжение на входе АЦП
    if (ADC_Buf_raw <= (uint32_t)ADC_ADS1251_MAX_VAL)
    {
      data = (float)ADC_Buf_raw / ADC_ADS1251_MAX_VAL * ADC_ADS1251_REF_VOLT;
    }
    // Отрицательное напряжение на входе АЦП
    else
    {
      data = ((float)ADC_Buf_raw - ADC_24BIT_FUL_SCALE - 1.0f) / (ADC_ADS1251_MAX_VAL + 1.0f) * ADC_ADS1251_REF_VOLT;
    }
  }
  else if (adc == ADC_ADS1231)
  {
    // Положительное напряжение на входе АЦП
    if ((float)ADC_Buf_raw <= ADC_ADS1231_MAX_VAL)
    {
      data = (float)ADC_Buf_raw / ADC_ADS1231_MAX_VAL * ADC_ADS1231_REF_VOLT;
    }
    // Отрицательное напряжение на входе АЦП
    else
    {
      data = ((float)ADC_Buf_raw - (ADC_ADS1231_MAX_VAL*2) - 1.0f) / (ADC_ADS1231_MAX_VAL + 1.0f) * ADC_ADS1231_REF_VOLT;
    }
  }
  else if (adc == ADC_T)
  {
    //@do
    // Дописать обработчик для значения температуры c АЦП
    asm("NOP");
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

  App.adc_filter[adc].valueRaw = sum / (float)App.adc_filter[adc].order;

  value = App.adc_filter[adc].valueRaw;

  valueLast = App.adc_filter[adc].value_last;

  kFilter = 2.0f / ((float)App.adc_filter[adc].filter_N + 1.0f);

  value = valueLast + kFilter * (value - valueLast);

  App.adc_filter[adc].value = App.adc_filter[adc].value_last = value;
}
