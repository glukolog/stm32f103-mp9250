#ifndef _U8G_ARM_H
#define _U8G_ARM_H

// Pins Definition (SPI 3wire)
// LCD RS - CS   - STM32 SPI1 NSS  PA4
// LCD RW - MOSI - STM32 SPI1 MOSI PA7
// LCD E  - CLK  - STM32 SPI1 CLK  PA5
// LCD speed 1Mhz (2Mhz)

// Init:
// u8g2_Setup_st7920_s_128x64_2(&lcd, U8G2_R0, u8x8_byte_3wire_hw_spi, u8g2_gpio_and_delay_stm32);
// RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE); //Ticks SPI1.


//#include "stm32f10x.h"
//#include "stm32f10x_it.h"
//#include "stm32f10x_spi.h"
#include <stdio.h>
#include <stm32f10x.h>
#include "u8g2.h"
//#include "u8g2-master\csrc\u8x8.h"
//#include "stm32f4xx_conf.h"

#define CS_ON()        GPIO_SetBits(GPIOA, GPIO_Pin_4)   //Hardware CS
#define CS_OFF()       GPIO_ResetBits(GPIOA, GPIO_Pin_4)

//*************************************************************************
extern void delay_ms(uint16_t delay);
extern void delay_us(uint32_t delay);
void init_SPI1(void);
void Init_SW_SPI(void);
void SPI1_SendByte(uint8_t sendData);
void LCD_OUT (uint8_t Data_LCD, uint8_t Np_LCD);

//uint8_t u8g_com_hw_spi_fn(u8g2_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);
uint8_t u8x8_byte_3wire_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8g2_gpio_and_delay_stm32(U8X8_UNUSED u8x8_t *u8x8, U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int, U8X8_UNUSED void *arg_ptr);

#endif
