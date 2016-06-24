#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_i2c.h>

#ifndef Success
#define Success 1
#endif

#ifndef Error
#define Error 0
#endif

#ifndef I2C_NACKPosition_Current
#define I2C_NACKPosition_Current ((uint16_t)0xF7FF)
#endif

#ifndef I2C_NACKPosition_Next
#define I2C_NACKPosition_Next ((uint16_t)0x0800)
#endif

void I2C_LowLevel_Init(I2C_TypeDef* I2Cx, int ClockSpeed , int OwnAddress);

int I2C_Write(I2C_TypeDef* I2Cx, const uint8_t* buf, uint32_t nbyte , uint8_t SlaveAddress);
int I2C_Write_wo_stop(I2C_TypeDef* I2Cx, const uint8_t* buf, uint32_t nbyte , uint8_t SlaveAddress);

int I2C_Read(I2C_TypeDef* I2Cx, uint8_t *buf, uint32_t nbyte , uint8_t SlaveAddress);
void I2C_NACKPositionConfig(I2C_TypeDef* I2Cx, uint16_t I2C_NACKPosition);
