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
  BSP_SLEEP_PWR_TENZO_ON;
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

#define GET_ADC_VALUE_PERIOD (uint16_t)(200)
void bsp_tim7_1ms_callback()
{
  static uint16_t cnt_1ms = 0;

  if (cnt_1ms++ > GET_ADC_VALUE_PERIOD)
  {
    BSP_SLEEP_PWR_TENZO_OFF;
    HAL_Delay(3);

    app_adc_data_filter(bsp_get_data_spi_ads1251(), ADC_ADS1251);
    app_adc_data_filter(bsp_get_data_spi_ads1231(), ADC_ADS1231);
    BSP_SLEEP_PWR_TENZO_ON;

    app_update_reg();
    protocolMbRtuSlaveCtrl_update_tables();

    BSP_LED_TOGGLE(BSP_LED_1);
    cnt_1ms = 0;
  }
  return;
}

void app_update_reg()
{
  // --- ADC_ADS1251
  for (uint8_t i = 0; i < COUNT_REG_SPI_BUF; i++)
  {
    App.ADC_ADS1251.spi_buf[i] = SPI_data_rx_ADS1251[i];
  }
  App.ADC_ADS1251.data_i16 = (int16_t)(App.adc_filter[ADC_ADS1251].value * 1000.0f);  // [В*1000]
  App.ADC_ADS1251.data_i32 = (int32_t)(App.adc_filter[ADC_ADS1251].value * 10000.0f); // [В*10000]
  
  // --- ADC_ADS1231
  for (uint8_t i = 0; i < COUNT_REG_SPI_BUF; i++)
  {
    App.ADC_ADS1231.spi_buf[i] = SPI_data_rx_ADS1231[i];
  }
  App.ADC_ADS1231.data_i16 = (int16_t)(App.adc_filter[ADC_ADS1231].value * 1000.0f);  // [В*1000]
  App.ADC_ADS1231.data_i32 = (int32_t)(App.adc_filter[ADC_ADS1231].value * 10000.0f); // [В*10000]

  // -- ADC_T
  App.ADC_T_data_i16 = (int16_t)App.adc_filter[ADC_T].value;
}

void bsp_ADC_data_ready()
{
  app_adc_data_filter(((HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1)/16)), ADC_T);
}

#define ADC_24BIT_FUL_SCALE   (float)(16777215.0f)

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
    // Положительное напряжение на входе АЦП
    if (ADC_Buf_raw < (uint32_t)ADC_ADS1251_MAX_VAL)
    {
      data = (float)ADC_Buf_raw / ADC_ADS1251_MAX_VAL * ADC_ADS1251_REF_VOLT;
    }
    // Отрицательное напряжение на входе АЦП
    else
    {
      data = (float)((ADC_Buf_raw - (uint32_t)(ADC_24BIT_FUL_SCALE))) / ADC_ADS1251_MAX_VAL * ADC_ADS1251_REF_VOLT;
    }
  }
  else if (adc == ADC_ADS1231)
  {
    // Положительное напряжение на входе АЦП
    if (ADC_Buf_raw < (uint32_t)ADC_ADS1251_MAX_VAL)
    {
      data = (float)ADC_Buf_raw / ADC_ADS1231_MAX_VAL * ADC_ADS1231_REF_VOLT;
    }
    // Отрицательное напряжение на входе АЦП
    else
    {
      data = (float)((ADC_Buf_raw - (uint32_t)(ADC_24BIT_FUL_SCALE))) / ADC_ADS1251_MAX_VAL * ADC_ADS1251_REF_VOLT;
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

  App.adc_filter[adc].valueRaw = sum / App.adc_filter[adc].order;
  
  //--------------------//
  value = App.adc_filter[adc].valueRaw;
  valueLast = App.adc_filter[adc].value_last;
  kFilter = 2.0f / ((float)App.adc_filter[adc].filter_N + 1.0f);
  value = valueLast + kFilter*(value - valueLast);
  App.adc_filter[adc].value = value;
  App.adc_filter[adc].value_last = value;
}



