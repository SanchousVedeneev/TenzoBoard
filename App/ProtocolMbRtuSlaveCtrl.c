
#include "ProtocolMbRtuSlaveCtrl.h"

#include "App.h"
#include "BSP.h"

uint8_t modbusBufRxTxRtu485[MODBUS_SS_BUF_CNT];

//--------------------  PROTOCOL ---------------------//
//---1000
#define MDB_TABLE_BSP_REG_NO (1)
enum mdb_table_bsp
{
  tab_bsp_spi_buf_ADS1251_0 = MDB_TABLE_BSP_REG_NO,
  tab_bsp_spi_buf_ADS1251_1,
  tab_bsp_spi_buf_ADS1251_2,
  tab_bsp_ADC_ADS1251_data_i16,
  tab_bsp_ADC_ADS1251_data_i32_1,
  tab_bsp_ADC_ADS1251_data_i32_2,

  tab_bsp_spi_buf_ADS1231_0,
  tab_bsp_spi_buf_ADS1231_1,
  tab_bsp_spi_buf_ADS1231_2,
  tab_bsp_ADC_ADS1231_data_i16,
  tab_bsp_ADC_ADS1231_data_i32_1,
  tab_bsp_ADC_ADS1231_data_i32_2,

  tab_bsp_ADC_T_data_i16
};
#define MDB_BSP_BUF_COUNT (tab_bsp_ADC_T_data_i16 - MDB_TABLE_BSP_REG_NO + 1)
uint16_t mdb_bsp_buf[MDB_BSP_BUF_COUNT];

ModbusSS_table_t mdb_table_bsp = {
    .buf = (uint8_t *)mdb_bsp_buf,
    .quantity = MDB_BSP_BUF_COUNT,
    .regNo = MDB_TABLE_BSP_REG_NO,
    .type = ModbusSS_Holding};
//--------------------  PROTOCOL END---------------------//

//--------------------  TABLES ARRAY ---------------------//
ModbusSS_table_t *modbusTables[] = {
    &mdb_table_bsp
};
//--------------------  TABLES ARRAY END---------------------//

//--------------------  MODBUS STRUCT ---------------------//
ModbusSS_t modbusSS_rtu_rs485 = {
    .cbHoldingUpdate = protocolMbRtuSlaveCtrl_callback_H_WRITE,
    .cbHoldingRequest = NULL, // protocolMbRtuSlaveCtrl_callback_H_REQ, //modbusHoldingReq,
    .rtuTcp = MODBUS_SS_RTU,
    .bufRxTx = modbusBufRxTxRtu485,
    .slaveId = 1,
    .tables = modbusTables,
    .tablesCount = 1
};
//--------------------  MODBUS STRUCT END---------------------//

//------------------------ EXTERN ------------------------
extern App_struct App;
//---------------------- EXTERN END-----------------------

//------------------------ REGULAR FCN ------------------------
void protocolMbRtuSlaveCtrl_init(uint8_t portNo)
{
  HAL_Delay(100);
  bsp_rs485_setPortToModbusRtu(portNo, modbusBufRxTxRtu485, MODBUS_SS_BUF_CNT);
  modbusSS_rtu_rs485.slaveId = bsp_get_adr_mdb();
}

__INLINE void protocolMbRtuSlaveCtrl_update_tables()
{
  ModbusSS_SetWord(&mdb_table_bsp, tab_bsp_spi_buf_ADS1251_0,        App.ADC_ADS1251.spi_buf[0]);
  ModbusSS_SetWord(&mdb_table_bsp, tab_bsp_spi_buf_ADS1251_1,        App.ADC_ADS1251.spi_buf[1]);
  ModbusSS_SetWord(&mdb_table_bsp, tab_bsp_spi_buf_ADS1251_2,        App.ADC_ADS1251.spi_buf[2]);
  ModbusSS_SetWord(&mdb_table_bsp, tab_bsp_ADC_ADS1251_data_i16,     (uint16_t)App.ADC_ADS1251.data_i16);
  ModbusSS_SetWord(&mdb_table_bsp, tab_bsp_ADC_ADS1251_data_i32_1,   (uint16_t)( (uint32_t)App.ADC_ADS1251.data_i32 & 0x0000FFFFUL));
  ModbusSS_SetWord(&mdb_table_bsp, tab_bsp_ADC_ADS1251_data_i32_2,   (uint16_t)(((uint32_t)App.ADC_ADS1251.data_i32 & 0xFFFF0000UL) >> 16));

  ModbusSS_SetWord(&mdb_table_bsp, tab_bsp_spi_buf_ADS1231_0,        App.ADC_ADS1231.spi_buf[0]);
  ModbusSS_SetWord(&mdb_table_bsp, tab_bsp_spi_buf_ADS1231_1,        App.ADC_ADS1231.spi_buf[1]);
  ModbusSS_SetWord(&mdb_table_bsp, tab_bsp_spi_buf_ADS1231_2,        App.ADC_ADS1231.spi_buf[2]);
  ModbusSS_SetWord(&mdb_table_bsp, tab_bsp_ADC_ADS1231_data_i16,     (uint16_t)App.ADC_ADS1231.data_i16);
  ModbusSS_SetWord(&mdb_table_bsp, tab_bsp_ADC_ADS1231_data_i32_1,   (uint16_t)( (uint32_t)App.ADC_ADS1231.data_i32 & 0x0000FFFFUL));
  ModbusSS_SetWord(&mdb_table_bsp, tab_bsp_ADC_ADS1231_data_i32_2,   (uint16_t)(((uint32_t)App.ADC_ADS1231.data_i32 & 0xFFFF0000UL) >> 16));

  ModbusSS_SetWord(&mdb_table_bsp, tab_bsp_ADC_T_data_i16,           (uint16_t)App.ADC_T_data_i16);
}
//------------------------ REGULAR FCN END------------------------

//------------------------------- MODBUS CALLBACKS -------------------------------------------//
__weak void protocolMbRtuSlaveCtrl_callback_H_WRITE(ModbusSS_table_t *table, uint16_t reg, uint16_t quantity)
{

  //uint16_t value = 0.0f;
  asm("NOP");

  // if (table == &mdb_table_bsp) // Диапазон BSP
  // {
  //   value = ModbusSS_GetWord(&mdb_table_bsp, reg);
  //   switch (reg)
  //   {
  //   case tab_bsp_control_led_rele:
  //     App.Mdb_data_AO.control_led_rele = value;
  //     app_control_led_rele(value);
  //     break;
  //   default:
  //     break;
  //   }
  // }
  
}

__weak void protocolMbRtuSlaveCtrl_callback_H_READ(ModbusSS_table_t *table, uint16_t reg, uint16_t quantity)
{

  asm("NOP");
  //             if (table == &modbusTableHolding1)
  // {
  //   for (int r = reg; r < reg + quantity; r++)
  //   {
  //     asm("NOP");
  //     switch (r)
  //     {
  //     case MBP_AI1_X:
  //       ModbusSS_SetWord(table, r, bsp_ai_read_cache(BSP_AI1));
  //       asm("NOP");
  //       break;
  //     case MBP_AI2_Y:
  //       ModbusSS_SetWord(table, r, bsp_ai_read_cache(BSP_AI2));
  //       asm("NOP");
  //       break;
  //     case MBP_DI_STATE:
  //       ModbusSS_SetWord(table, r, bsp_di_get_cache_pack16());
  //       asm("NOP");
  //       break;
  //     default:
  //       break;
  //     }
  //   }
  // }
}
//------------------------------- MODBUS CALLBACKS END-------------------------------------------//

//------------------------------- HW CALLBACK -------------------------------------------//
void bsp_rs485_callback_rxBlockReady(uint8_t portNo)
{

  int32_t blockSizeByte = 0;
  if ((blockSizeByte = ModbusSS_ParseRxData(&modbusSS_rtu_rs485)) == 0)
  {
    // bug with reset modbus!!!
    asm("NOP");
  }
  else if (blockSizeByte != -1)
  {
    asm("NOP");
    bsp_rs485_sendBlock(portNo, modbusSS_rtu_rs485.bufRxTx, blockSizeByte);
    asm("NOP");
  }
}
//------------------------------- HW CALLBACK END-------------------------------------------//




