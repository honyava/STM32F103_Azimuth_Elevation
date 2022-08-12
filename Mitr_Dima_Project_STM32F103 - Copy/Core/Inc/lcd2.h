#ifndef LCD_H
#define LCD_H
//-------------------------------------------------------------------
#include "stm32f1xx_hal.h"
#include <cstdlib>
//-------------------------------------------------------------------
#endif /* LCD_H */

#define RESET_ACTIVE() HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET)
#define RESET_IDLE() HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET)
#define CS_ACTIVE() HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_RESET)
#define CS_IDLE() HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_SET)
#define DC_COMMAND() HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET)
#define DC_DATA() HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET)



void LCD_ini(uint16_t w_size, uint16_t h_size);

