/*******************************************************************************
  * @file       SPI BUS DRIVER APPLICATION       
  * @author 
  * @version
  * @date 
  * @brief
  ******************************************************************************
  * @attention
  *
  *
*******************************************************************************/

/*-------------------------------- Includes ----------------------------------*/
#include "freertos/FreeRTOS.h"

#define SPI_IF_BIT_RATE 5000000 //spi speed rate 5MHz (ADXL345 max 5M)

// #define SET_SPI1_CS_ON() MAP_GPIOPinWrite(spi1_cs_gpio, spi1_cs_pin, spi1_cs_pin); //SPI1 ON
// #define SET_SPI1_CS_OFF() MAP_GPIOPinWrite(spi1_cs_gpio, spi1_cs_pin, 0x00);       //SPI1 OFF

// #define SET_SPI2_CS_ON()        MAP_GPIOPinWrite(spi2_cs_gpio,spi2_cs_pin,spi2_cs_pin); //SPI2 ON
// #define SET_SPI2_CS_OFF()       MAP_GPIOPinWrite(spi2_cs_gpio,spi2_cs_pin,0x00);        //SPI2 OFF

/*******************************************************************************
 FUNCTION PROTOTYPES
*******************************************************************************/
extern void UserSpiInit(void); //spi interface init

void SET_SPI2_CS_ON(void);
void SET_SPI2_CS_OFF(void);
int SPI_Write(uint8_t val);
uint8_t SPI_Read(void);

/*******************************************************************************
                                      END         
*******************************************************************************/
