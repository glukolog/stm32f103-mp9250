#include <stm32f10x.h>
//#include "stm32f10x_it.h"
//#include "stm32f10x_gpio.h"
//#include "stm32f10x_rcc.h"
//#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "printf.h"

#define putchar _putchar

#include "systick.h"

#define I2C_SOFT

#include <u8g2.h>
#include "u8.h"
#ifdef I2C_SOFT
#include "i2c-soft.h"
#endif
#include "MPU9250.h"
#define BMP280_addr 0xEC
#include "bmp280.h"

#define MPS     10 // Measure per second
u8g2_t u8g2;
char s[80];
uint16_t qs = 0, tqs = 0;
float temp=0, pres;
uint8_t screen = 2;

struct vector_s {
    float x;
    float y;
    float z;
} VM[10], MF, ML;

#define RxBufferSize1 128
uint8_t RxBuffer1[RxBufferSize1];
__IO uint8_t RxCounter1 = 0x00;

void USART1_IRQHandler(void) {
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
    RxBuffer1[RxCounter1++] = USART_ReceiveData(USART1);
    if(RxCounter1 == RxBufferSize1) USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
  }
}

void InitPerGPIO() {
  GPIO_InitTypeDef GPIO_InitStructure;

  // LED
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

#ifdef I2C_SOFT
   // I2C
   GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
   GPIO_Init(GPIOB, &GPIO_InitStructure);
#endif
}

void InitUART(USART_TypeDef * port, uint32_t baud) {
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //USART1 RX
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; // TxD (PA9) push-pull
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  USART_InitTypeDef USART_InitStructure;

  USART_InitStructure.USART_BaudRate = baud;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(port, &USART_InitStructure);
  USART_Cmd(port, ENABLE);
}

void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

  /* Enable the USARTy Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void _putchar(char ch) {
  USART_SendData(USART1, (uint8_t) ch);
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}
}

int MyLowLevelPutchar(int ch)
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART1, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}

  return 0;
}
/*
int _write(int file, char *data, int len)
{
    int bytes_written;
    file = file;

    //if ((file != STDOUT_FILENO) && (file != STDERR_FILENO))
    //{
        //errno = EBADF;
    //    return -1;
    //}

    for (bytes_written = 0; bytes_written < len; bytes_written++)
    {
        MyLowLevelPutchar(*data);
        data++;
    }

    return bytes_written;
}
*/
uint8_t getch(void) {
    if (RxCounter1) return(RxBuffer1[--RxCounter1]);
    return(0);
}

#ifndef I2C_SOFT
void I2C1_init(void)
{
    I2C_InitTypeDef  I2C_InitStructure;
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /* Configure I2C_EE pins: SCL and SDA */
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* I2C configuration */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x38;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 100000;

    /* I2C Peripheral Enable */
    I2C_Cmd(I2C1, ENABLE);
    /* Apply I2C configuration after enabling it */
    I2C_Init(I2C1, &I2C_InitStructure);
}

uint32_t I2C_WaitEvent(I2C_TypeDef* I2Cx, uint32_t flag ) {
  uint32_t timeout = 1000;

  while( !I2C_CheckEvent( I2C1, flag )){// != SUCCESS ) {
     if( !timeout ) {
         //putchar('o');
         I2C_GenerateSTOP(I2C1,ENABLE);
         break;
         //return 0;
     }
     timeout--;
     delay_us(1);
  }

  return 1;
}

void I2C_Transmit(uint8_t addr, uint8_t *tx_buf, uint8_t tx_len, uint8_t *rx_buf, uint8_t rx_len) {
    int i;

//    putchar('t');
    I2C_AcknowledgeConfig(I2C1,ENABLE);
    I2C_GenerateSTART(I2C1,ENABLE);
    //while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    while (!I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
//    putchar('s');

    I2C_Send7bitAddress(I2C1, addr, I2C_Direction_Transmitter);
//    putchar('7');
    while (!I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
//    putchar('a');

    for(i = 0 ; i < tx_len ; i++) {
        I2C_SendData(I2C1, tx_buf[i]);
//        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
        while (!I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    }
    I2C_GenerateSTART(I2C1,ENABLE);
//    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    while (!I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(I2C1, addr, I2C_Direction_Receiver);
//    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    while (!I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    for(i = 0 ; i < rx_len ; i++) {
//        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
        while (!I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
        rx_buf[i] = I2C_ReceiveData(I2C1);
    }

    I2C_GenerateSTOP(I2C1,ENABLE);
    I2C_AcknowledgeConfig(I2C1,DISABLE);

//    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
    while (!I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
    I2C_ReceiveData(I2C1);

//    if (I2C_GetITStatus(I2C1, I2C_IT_TIMEOUT)) I2C_ClearITPendingBit(I2C1, I2C_IT_TIMEOUT);
//    How read|clean error
}

#endif

void MahonyQuaternionUpdateG(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    float norm;
    float hx, hy, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;
    float pa, pb, pc;

    // Auxiliary variables to avoid repeated arithmetic
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f / norm;        // use reciprocal for division
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f / norm;        // use reciprocal for division
    mx *= norm;
    my *= norm;
    mz *= norm;

    //printf("M(%.2f\t%.2f\t%.2f)\t",mx,my,mz);

    // Reference direction of Earth's magnetic field
    hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
    hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
    bx = sqrt((hx * hx) + (hy * hy));
    bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);
    VM[qs].x = ML.x - hx;
    VM[qs].y = ML.y - hy;
    VM[qs].z = ML.z - bz;
    ML.x = hx;
    ML.y = hy;
    ML.z = bz;

    // Estimated direction of gravity and magnetic field
    vx = 2.0f * (q2q4 - q1q3);
    vy = 2.0f * (q1q2 + q3q4);
    vz = q1q1 - q2q2 - q3q3 + q4q4;
    wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
    wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
    wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);
    printf("H(%.3f\t%.3f\t%.3f\t%.3f)\t",hx,hy,bz,bx);
//    printf("V(%.2f\t%.2f\t%.2f)\t",vx,vy,vz);
//    printf("W(%.2f\t%.2f\t%.2f)\t",wx,wy,wz);

    // Error is cross product between estimated direction and measured direction of gravity
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
    //printf("E(%.3f\t%.3f\t%.3f)\t",ex,ey,ez);

    if (Ki > 0.0f)
    {
        eInt[0] += ex;      // accumulate integral error
        eInt[1] += ey;
        eInt[2] += ez;
    }
    else
    {
        eInt[0] = 0.0f;     // prevent integral wind up
        eInt[1] = 0.0f;
        eInt[2] = 0.0f;
    }

    // Apply feedback terms
    gx = gx + Kp * ex + Ki * eInt[0];
    gy = gy + Kp * ey + Ki * eInt[1];
    gz = gz + Kp * ez + Ki * eInt[2];
//    printf("EG: %.3f\t%.3f\t%.3f",gx,gy,gz);
//    printf("\r\n");

    // Integrate rate of change of quaternion
    pa = q2;
    pb = q3;
    pc = q4;
    q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
    q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
    q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
    q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

    // Normalise quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    norm = 1.0f / norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;

}

void MP9250_ReadValues() {
    // If intPin goes high, all data registers have new data
    if(readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt
        readAccelData(accelCount);  // Read the x/y/z adc values
        // Now we'll calculate the accleration value into actual g's
        ax = (float)accelCount[0]*aRes;// - accelBias[0];  // get actual g value, this depends on scale being set
        ay = (float)accelCount[1]*aRes;// - accelBias[1];
        az = (float)accelCount[2]*aRes;// - accelBias[2];

        readGyroData(gyroCount);  // Read the x/y/z adc values
        // Calculate the gyro value into actual degrees per second
        gx = (float)gyroCount[0]*gRes;// - gyroBias[0];  // get actual gyro value, this depends on scale being set
        gy = (float)gyroCount[1]*gRes;// - gyroBias[1];
        gz = (float)gyroCount[2]*gRes;// - gyroBias[2];

        readMagData(magCount);  // Read the x/y/z adc values
        // Calculate the magnetometer values in milliGauss
        // Include factory calibration per data sheet and user environmental corrections
        mx = (float)magCount[0]*mRes*magCalibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
        my = (float)magCount[1]*mRes*magCalibration[1] - magbias[1];
        mz = (float)magCount[2]*mRes*magCalibration[2] - magbias[2];
    }
}

void u8g2_start() {
    u8g2_Setup_st7920_s_128x64_f(&u8g2, U8G2_R2, u8x8_byte_3wire_hw_spi, u8g2_gpio_and_delay_stm32);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);
//    u8g2_SetFont(&u8g2, u8g2_font_6x10_mr);
    u8g2_SetFont(&u8g2, u8g2_font_5x8_mr);
    u8g2_ClearBuffer(&u8g2);
}

void Screen_Dev1() {
    u8g2_DrawStr(&u8g2, 10, 9, "Accel   Gyro  Magnet");
    u8g2_DrawStr(&u8g2, 0, 20, "X");
    u8g2_DrawStr(&u8g2, 0, 30, "Y");
    u8g2_DrawStr(&u8g2, 0, 40, "Z");

    // Serial print and/or display at 0.5 s rate independent of data rates
    //printf("ax = %.3f\t", 1000*ax); printf(" ay = %.3f\t", 1000*ay); printf(" az = %.3f  mg\n\r", 1000*az);
    //printf("gx = %.3f\t", gx); printf(" gy = %.3f\t", gy); printf(" gz = %.3f  deg/s\n\r", gz);
    //printf("mx = %.3f\t", mx); printf(" my = %.3f\t", my); printf(" mz = %.3f  mG\n\r", mz);

    sprintf(s, " %.0f", 1000*ax); u8g2_DrawStr(&u8g2, 42 - u8g2_GetStrWidth(&u8g2, s), 20, s);
    sprintf(s, " %.0f", 1000*ay); u8g2_DrawStr(&u8g2, 42 - u8g2_GetStrWidth(&u8g2, s), 30, s);
    sprintf(s, " %.0f", 1000*az); u8g2_DrawStr(&u8g2, 42 - u8g2_GetStrWidth(&u8g2, s), 40, s);

    sprintf(s, " %.2f", gx); u8g2_DrawStr(&u8g2, 84 - u8g2_GetStrWidth(&u8g2, s), 20, s);
    sprintf(s, " %.2f", gy); u8g2_DrawStr(&u8g2, 84 - u8g2_GetStrWidth(&u8g2, s), 30, s);
    sprintf(s, " %.2f", gz); u8g2_DrawStr(&u8g2, 84 - u8g2_GetStrWidth(&u8g2, s), 40, s);

    sprintf(s, " %.1f", mx); u8g2_DrawStr(&u8g2, 127 - u8g2_GetStrWidth(&u8g2, s), 20, s);
    sprintf(s, " %.1f", my); u8g2_DrawStr(&u8g2, 127 - u8g2_GetStrWidth(&u8g2, s), 30, s);
    sprintf(s, " %.1f", mz); u8g2_DrawStr(&u8g2, 127 - u8g2_GetStrWidth(&u8g2, s), 40, s);

    sprintf(s, "BMP T:%.1f P:%.0f ", temp, pres); u8g2_DrawStr(&u8g2, 0, 55, s);
    sprintf(s, "G=%.3f M=%.1f mG", sqrt(ax*ax+ay*ay+az*az), sqrt(mx*mx+my*my+mz*mz)); u8g2_DrawStr(&u8g2, 0, 63, s);
}

void Screen_Dev2() {
    float l;
    u8g2_ClearBuffer(&u8g2);
    u8g2_DrawStr(&u8g2, 10, 7, "Magnet fluctuation");
    u8g2_DrawStr(&u8g2, 0, 20, "X");
    u8g2_DrawStr(&u8g2, 0, 30, "Y");
    u8g2_DrawStr(&u8g2, 0, 40, "Z");

    // Serial print and/or display at 0.5 s rate independent of data rates
    //printf("ax = %.3f\t", 1000*ax); printf(" ay = %.3f\t", 1000*ay); printf(" az = %.3f  mg\n\r", 1000*az);
    //printf("gx = %.3f\t", gx); printf(" gy = %.3f\t", gy); printf(" gz = %.3f  deg/s\n\r", gz);
    //printf("mx = %.3f\t", mx); printf(" my = %.3f\t", my); printf(" mz = %.3f  mG\n\r", mz);

    sprintf(s, " %.3f", MF.x); u8g2_DrawStr(&u8g2, 38 - u8g2_GetStrWidth(&u8g2, s), 20, s);
    sprintf(s, " %.3f", MF.y); u8g2_DrawStr(&u8g2, 38 - u8g2_GetStrWidth(&u8g2, s), 30, s);
    sprintf(s, " %.3f", MF.z); u8g2_DrawStr(&u8g2, 38 - u8g2_GetStrWidth(&u8g2, s), 40, s);

    sprintf(s, "MF=%.2f ", sqrt(MF.x*MF.x+MF.y*MF.y+MF.z*MF.z)); u8g2_DrawStr(&u8g2, 0, 55, s);
    sprintf(s, "G=%.3f M=%.1f mG", sqrt(ax*ax+ay*ay+az*az), sqrt(mx*mx+my*my+mz*mz)); u8g2_DrawStr(&u8g2, 0, 63, s);

    u8g2_DrawCircle(&u8g2, 64, 32, 20, U8G2_DRAW_ALL);
    u8g2_DrawCircle(&u8g2, 106, 32, 20, U8G2_DRAW_ALL);

    l = sqrt(MF.x*MF.x + MF.y*MF.y + MF.z*MF.z);
    l = (l > 0.2f) ? 20.0f / l : 100.0f;
    //l = 100;
    MF.x *= l;
    MF.y *= l;
    MF.z *= l;

    u8g2_DrawLine(&u8g2, 64, 32, 64 + MF.x, 32 - MF.y);
    u8g2_DrawLine(&u8g2, 106, 32, 106 + MF.y, 32 - MF.z);
}


int main() {
    int i;
    uint8_t pause;//, buf[128]={0};
    uint32_t t0 = 0;

    SysTickInit(1000);
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_USART1 | RCC_APB2Periph_SPI1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_USART1, ENABLE);
    InitPerGPIO();
    NVIC_Configuration();
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    InitUART(USART1, 115200);
#ifndef I2C_SOFT
    I2C1_init();
#endif
/*
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
    // turn off buffers, so IO occurs immediately
*/
    printf("MP-9250!\r\n");
    printf("Last: %X\r\n", I2C_GetLastEvent(I2C1));
    u8g2_start();
    // Read the WHO_AM_I register, this is a good test of communication
    uint8_t whoami = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
    printf("I AM 0x%x\n\r", whoami); printf("I SHOULD BE 0x71\n\r");
    printf("Last: %X\r\n", I2C_GetLastEvent(I2C1));
    BMP280_Init();
    if (whoami == 0x73) // WHO_AM_I should always be 0x68
    {
      printf("MPU9250 is online...\n\r");
      delay_ms(1);

      resetMPU9250(); // Reset registers to default in preparation for device calibration
      calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
      printf("x gyro bias = %f\n\r", gyroBias[0]);
      printf("y gyro bias = %f\n\r", gyroBias[1]);
      printf("z gyro bias = %f\n\r", gyroBias[2]);
      printf("x accel bias = %f\n\r", accelBias[0]);
      printf("y accel bias = %f\n\r", accelBias[1]);
      printf("z accel bias = %f\n\r", accelBias[2]);
      delay_ms(2);
      initMPU9250();
      printf("MPU9250 initialized for active data mode....\n\r"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
      initAK8963(magCalibration);
      printf("AK8963 initialized for active data mode....\n\r"); // Initialize device for active mode read of magnetometer
      printf("Accelerometer full-scale range = %f  g\n\r", 2.0f*(float)(1<<Ascale));
      printf("Gyroscope full-scale range = %f  deg/s\n\r", 250.0f*(float)(1<<Gscale));
      if(Mscale == 0) printf("Magnetometer resolution = 14  bits\n\r");
      if(Mscale == 1) printf("Magnetometer resolution = 16  bits\n\r");
      if(Mmode == 2) printf("Magnetometer ODR = 8 Hz\n\r");
      if(Mmode == 6) printf("Magnetometer ODR = 100 Hz\n\r");
      getAres(); // Get accelerometer sensitivity
      getGres(); // Get gyro sensitivity
      getMres(); // Get magnetometer sensitivity
      printf("Accelerometer sensitivity is %f LSB/g \n\r", 1.0f/aRes);
      printf("Gyroscope sensitivity is %f LSB/deg/s \n\r", 1.0f/gRes);
      printf("Magnetometer sensitivity is %f LSB/G \n\r", 1.0f/mRes);
//      magbias[0] = +550.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
//      magbias[1] = +200.;  // User environmental x-axis correction in milliGauss
//      magbias[2] = -100.;  // User environmental x-axis correction in milliGauss

    } else {
      printf("Could not connect to MPU9250: \n\r");
      printf("%#x \n",  whoami);
    }

    while(1) {
        if (SysTickGet() - t0 > 1000/MPS) {
            t0 = SysTickGet();
            qs++;
            if (qs > 9) qs = 0;
        }
        if (!pause && qs != tqs) {
            tqs = qs;
            BMP280_ReadTP(&temp, &pres);
            //printf("Temperature: %0.2f, Pressure: %.2f\r\n", temp, pres);
            MP9250_ReadValues();
            /*
          Now = t.read_us();
          deltat = (float)((Now - lastUpdate)/1 000 000.0f) ; // set integration time by time elapsed since last filter update
          lastUpdate = Now;

          sum += deltat;
          sumCount++;
          */

            // Pass gyro rate as rad/s
            //MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
            // mpu9250.MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);
            tempCount = readTempData();  // Read the adc values
            temperature = ((float) tempCount) / 333.87f + 21.0f; // Temperature in degrees Centigrade
            //printf(" temperature = %f  C\n\r", temperature);
            //sprintf(s, "Temp: %.1f ", temperature); u8g2_DrawStr(&u8g2, 0, 63, s);

            deltat = 1.0f / MPS;
            //MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
            MahonyQuaternionUpdateG(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);

//            yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
//            pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
//            roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
            pitch *= 180.0f / PI;
            yaw   *= 180.0f / PI;
            roll  *= 180.0f / PI;

            //printf("Yaw, Pitch, Roll: %f %f %f\n\r", yaw, pitch, roll);
            //printf("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\r\n",q[0],q[1],q[2],q[3],q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
            //printf("average rate = %f\n\r", (float) sumCount/sum);

            if ((qs % 5) == 0) {
                memset(&MF, 0, sizeof(MF));
                for(i = 0 ; i < 10 ; i++) {
                    MF.x += VM[i].x;
                    MF.y += VM[i].y;
                    MF.z += VM[i].z;
                }
                switch (screen) {
                case 1: Screen_Dev1(); break;
                case 2: Screen_Dev2(); break;
                default:
                    break;
                }
                printf("MF(%.3f\t%.3f\t%.3f)\t",MF.x,MF.y,MF.z);

                u8g2_SendBuffer(&u8g2);
            }
            printf("\r\n");
            //printf("%d ", qs);
        }
        //        delay_ms(100);
        //        GPIOB->BSRR = 0XFF;
        //        delay_ms(300);
        while (RxCounter1) {
            char c = getch();
            switch (c) {
            case '1' : screen = 1; break;
            case '2' : screen = 2; break;
            case 'p' : pause ^= 1; break;
            default : putchar(c); break;
            }
        }
    }

    return 0;
}
