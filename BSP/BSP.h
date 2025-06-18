#ifndef __BSP__H__
#define __BSP__H__

#include "main.h"
#include "tim.h"
#include "spi.h"
#include "usart.h"
#include "adc.h"

//------------------------------ DEFINE ------------------------------//
#define BSP_LED_1                 DO_LED_1_GPIO_Port, DO_LED_1_Pin
#define BSP_LED_2                 DO_LED_2_GPIO_Port, DO_LED_2_Pin
#define BSP_LED_3                 DO_LED_3_GPIO_Port, DO_LED_3_Pin
#define BSP_LED_4                 DO_LED_4_GPIO_Port, DO_LED_4_Pin

#define BSP_LED_ON(LED)			  HAL_GPIO_WritePin(LED, GPIO_PIN_SET)
#define BSP_LED_OFF(LED)	      HAL_GPIO_WritePin(LED, GPIO_PIN_RESET)
#define BSP_LED_TOGGLE(LED)		  HAL_GPIO_TogglePin(LED)

#define BSP_REL_1                 DO_REL_1_GPIO_Port, DO_REL_1_Pin
#define BSP_REL_2                 DO_REL_2_GPIO_Port, DO_REL_2_Pin
#define BSP_REL_3                 DO_REL_3_GPIO_Port, DO_REL_3_Pin
#define BSP_REL_4                 DO_REL_4_GPIO_Port, DO_REL_4_Pin

#define BSP_REL_ON(REL)			  HAL_GPIO_WritePin(REL, GPIO_PIN_SET)
#define BSP_REL_OFF(REL)		  HAL_GPIO_WritePin(REL, GPIO_PIN_RESET)
#define BSP_REL_TOGGLE(REL)		  HAL_GPIO_TogglePin(REL)

#define BSP_GET_BIT(REG, BIT)     (REG & (1 << BIT))
#define BSP_SET_BIT(REG, BIT)     (REG |= (1 << BIT))
#define BSP_RESET_BIT(REG, BIT)   (REG &= ~(1 << BIT))

#define BSP_GET_DI(PORT_PIN)      (HAL_GPIO_ReadPin(PORT_PIN))

//---------------------------- DEFINE END ----------------------------//

//------------------------------- ENUM -------------------------------//
typedef enum
{
    LED_1 = 0,
    LED_2,
    LED_3,
    LED_4,
    REL_1,
    REL_2,
    REL_3,
    REL_4
}LED_REL_ENUM;
//----------------------------- ENUM END -----------------------------//

// ----------------------------- RS-485 ----------------------------- //
void bsp_rs485_setPortToModbusRtu(uint8_t portNo, uint8_t *bufRxTX, uint16_t bufSizeByte);

void bsp_rs485_sendBlock(uint8_t portNo, uint8_t *buf, uint8_t bufSizeByte);
void bsp_rs485_sendTestBlock(uint8_t portNo);

void bsp_rs485_callback_rxBlockReady(uint8_t portNo);
void bsp_rs485_callback_rxTimeout(uint8_t portNo);

#define BSP_RS485_1_IRQ_HANDLER_RTOF 			USART1_IRQHandler
#define BSP_RS485_1_IRQ_HANDLER_DMA_RX 			DMA1_Channel1_IRQHandler
#define BSP_RS485_1_IRQ_HANDLER_DMA_TX 			DMA1_Channel2_IRQHandler
// --------------------------- RS-485 END----------------------------- //

// ----------------------------- TIM ----------------------------- //
void bsp_tim7_1ms_start();
void bsp_tim7_1ms_callback();

void bsp_tim6_10ms_start();
// --------------------------- TIM END --------------------------- //

// ----------------------------- SPI ----------------------------- //
uint32_t bsp_get_data_spi();
extern uint8_t SPI_DATA_RX[8];
// --------------------------- SPI END --------------------------- //

// ----------------------------- ADC ----------------------------- //
void bsp_ADC_data_ready();
// --------------------------- ADC END --------------------------- //

void bsp_init();

#endif