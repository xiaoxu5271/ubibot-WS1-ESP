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
#include "stdint.h"


#define SPI_IF_BIT_RATE 5000000  //spi speed rate 5MHz (ADXL345 max 5M)

#define spi1_cs_gpio    GPIOA2_BASE
#define spi1_cs_pin     GPIO_PIN_6

#define spi2_cs_gpio    GPIOA1_BASE
#define spi2_cs_pin     GPIO_PIN_4

#define SET_SPI1_CS_ON()        MAP_GPIOPinWrite(spi1_cs_gpio,spi1_cs_pin,spi1_cs_pin); //SPI1 ON
#define SET_SPI1_CS_OFF()       MAP_GPIOPinWrite(spi1_cs_gpio,spi1_cs_pin,0x00);        //SPI1 OFF

#define SET_SPI2_CS_ON()        MAP_GPIOPinWrite(spi2_cs_gpio,spi2_cs_pin,spi2_cs_pin); //SPI2 ON
#define SET_SPI2_CS_OFF()       MAP_GPIOPinWrite(spi2_cs_gpio,spi2_cs_pin,0x00);        //SPI2 OFF


/*******************************************************************************
 FUNCTION PROTOTYPES
*******************************************************************************/
extern void UserSpiInit(void);  //spi interface init

extern uint8_t SPI_SendReciveByte(uint8_t addr);  //spi send and recive a byte


/*******************************************************************************
                                      END         
*******************************************************************************/




