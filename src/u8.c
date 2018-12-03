//#include <stdint.h>
#include "u8.h"
#include "printf.h"
//#include "stm32f10x_spi.h"
//#include "SysTick.h"

uint8_t control = 0;

uint8_t u8g2_gpio_and_delay_stm32( u8x8_t *u8x8,  uint8_t msg,  uint8_t arg_int,  void *arg_ptr)
{
  //printf("%d-%d ", msg, arg_int);
  switch(msg) {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:	// called once during init phase of u8g2/u8x8
      //printf("lcd_gpio_init ");
      if (u8x8->byte_cb == u8x8_byte_3wire_sw_spi) Init_SW_SPI();
      //init_SPI1();
      //Delay_mss();  //Ждем, пока контроллер LCD включится.
      delay_ms(5);
      //SystemInit(); //Инициализация настроек кварца
      break;							// can be used to setup pins
    case U8X8_MSG_DELAY_NANO:			// delay arg_int * 1 nano second
      //printf("lcd_gpio_nano %d ", arg_int);
      //delay_us(3 * arg_int);
      delay_us(5);
      //while (arg_int) { __NOP(); arg_int--; }
      break;
    case U8X8_MSG_DELAY_100NANO:		// delay arg_int * 100 nano seconds
      printf("lcd_gpio_100nano ");
      break;
    case U8X8_MSG_DELAY_10MICRO:		// delay arg_int * 10 micro seconds
      printf("lcd_gpio_10micro ");
      delay_us(arg_int * 10);
      break;
    case U8X8_MSG_DELAY_MILLI:			// delay arg_int * 1 milli second
      delay_ms(arg_int);
      break;
    case U8X8_MSG_GPIO_D0:				// D0 or SPI clock pin: Output level in arg_int
        if (arg_int) GPIO_SetBits(GPIOA, GPIO_Pin_5); else GPIO_ResetBits(GPIOA, GPIO_Pin_5);
      break;
    case U8X8_MSG_GPIO_D1:				// D1 or SPI data pin: Output level in arg_int
        if (arg_int) GPIO_SetBits(GPIOA, GPIO_Pin_7); else GPIO_ResetBits(GPIOA, GPIO_Pin_7);
      break;
    case U8X8_MSG_GPIO_E:				// E/WR pin: Output level in arg_int
      printf("lcd_gpio_E ");
      break;
    case U8X8_MSG_GPIO_CS:				// CS (chip select) pin: Output level in arg_int
      //printf("lcd_gpio_cs:%d ", arg_int);
      if (arg_int) CS_ON(); else CS_OFF();
      break;
    case U8X8_MSG_GPIO_DC:				// DC (data/cmd, A0, register select) pin: Output level in arg_int
      //SWITCH FROM DATA TO COMMAND MODE (arg_val == 0 for command mode)
      //управляющий байт переключает режим на устройстве и устанавливается здесь 		
      // define cmd (arg_val = 0) or data mode (arg_val = 1) 
      // cmd - передача команды ,data mode - режим передачи данных
      printf("lcd_gpio_set_dc ");

      if (arg_int == 0)
      {
    	  control = 0xF8;
      }
      else
      {
    	  control = 0xFA;
      }
      //u8g_10MicroDelay();		 
      
      break;
    case U8X8_MSG_GPIO_RESET:			// Reset pin: Output level in arg_int
            //printf("lcd_gpio_reset ");
      break;

    case U8X8_MSG_GPIO_MENU_SELECT:
      u8x8_SetGPIOResult(u8x8, /* get menu select pin state */ 0);
      break;
    case U8X8_MSG_GPIO_MENU_NEXT:
      u8x8_SetGPIOResult(u8x8, /* get menu next pin state */ 0);
      break;
    case U8X8_MSG_GPIO_MENU_PREV:
      u8x8_SetGPIOResult(u8x8, /* get menu prev pin state */ 0);
      break;
    case U8X8_MSG_GPIO_MENU_HOME:
      u8x8_SetGPIOResult(u8x8, /* get menu home pin state */ 0);
      break;
    default:
      printf("Call undefined msg: %d\r\n", msg);
      u8x8_SetGPIOResult(u8x8, 1);			// default return value
      break;
  }
  return 1;
}

uint8_t u8x8_byte_3wire_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  uint32_t i;
  uint8_t *data;
  uint16_t b;
  //static uint8_t last_dc;
 
  switch(msg)
  {
    case U8X8_MSG_BYTE_SEND:
      data = (uint8_t *)arg_ptr;
      //printf("lcd_byte_send:%d %02x %d\r\n", arg_int, *data, control);
      for(i = 0 ; i < arg_int ; i++) {
        b = data[i];
        SPI1_SendByte(b);
      }
      while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) != RESET); // ждем пока данные передадутся до конца
      break;
      
    case U8X8_MSG_BYTE_INIT:
      init_SPI1();
      //printf("lcd_byte_init ");
      break;
    case U8X8_MSG_BYTE_SET_DC:
      printf("lcd_byte_set_dc:%d ", arg_int);
      //u8x8_gpio_SetDC(u8x8, arg_int);
      break;
    case U8X8_MSG_BYTE_START_TRANSFER:
      //printf("lcd_start_transf ");
      u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_enable_level);
      break;
    case U8X8_MSG_BYTE_END_TRANSFER:      
      //printf("lcd_byte_end_trans ");
      while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) != RESET); // ждем пока данные передадутся до конца 
      u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_disable_level);
      break;
    default:
      return 0;
  }
  return 1;
}


//****************************************************************************
void Init_SW_SPI(void) {
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Confugure CS(4) SCK(5) and MOSI(7) pins as Alternate Function Push Pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}
//****************************************************************************
void init_SPI1(void) {
	 /* configure pins used by SPI1
	 * PA5 = SCK
	 * PA6 = MISO
	 * PA7 = MOSI
	 */
	
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI1_Init_LCD12864;                   //Настройка SPI1.
  
  /* Confugure SCK and MOSI pins as Alternate Function Push Pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // CS - pin
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  CS_OFF();                                               //CS=0.

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE); //Вкл. тактирования SPI1.
	
  SPI1_Init_LCD12864.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;//Скорость.
  SPI1_Init_LCD12864.SPI_CPHA = SPI_CPHA_2Edge;//Со 2-го фронта.
  SPI1_Init_LCD12864.SPI_CPOL = SPI_CPOL_Low; //В режиме ожидания SCK - 0.
  SPI1_Init_LCD12864.SPI_CRCPolynomial = 7;//Фигня какая-то.
  SPI1_Init_LCD12864.SPI_DataSize = SPI_DataSize_8b;//Можно и 16!
  SPI1_Init_LCD12864.SPI_Direction = SPI_Direction_1Line_Tx;//Только TX = MOSI = выход.
  SPI1_Init_LCD12864.SPI_FirstBit = SPI_FirstBit_MSB;//Со старшего бита.
  SPI1_Init_LCD12864.SPI_Mode = SPI_Mode_Master;//Мастер.
  SPI1_Init_LCD12864.SPI_NSS = SPI_NSS_Soft;//Програмный NSS (в железе отключено).	
  SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Set);
  SPI_Init(SPI1, &SPI1_Init_LCD12864);
  SPI_Cmd(SPI1, ENABLE);                   //Запуск SPI1.
 
}
//*********************************************************************************
//*********************************************************************************
void SPI1_SendByte(uint8_t sendData) {
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  SPI_I2S_SendData(SPI1, sendData);
}
//**********************************************************************************
//**********************************************************************************
void LCD_OUT(uint8_t Data_LCD, uint8_t Np_LCD)
{
  CS_ON();                                //Передача начата.
  SPI1_SendByte(Np_LCD);           //Передача данных или комманды.
  SPI1_SendByte(Data_LCD & 0xF0);   //Старшая половина и 4 "0".
  SPI1_SendByte(Data_LCD << 4);     //Младшая и 4 "0".
  while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) != RESET); // ждем пока данные передадутся до конца 
  CS_OFF();                               //Передача окончина. 
  //delay_us(2);
}
