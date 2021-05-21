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
#include "user_spi.h"
#include "hw_types.h"
#include "spi.h"
#include "prcm.h"
#include "rom_map.h"
#include "hw_memmap.h"
#include "hw_mcspi.h"
#include "gpio.h"


/*******************************************************************************
//spi interface init
*******************************************************************************/
void UserSpiInit(void)
{  
  MAP_PRCMPeripheralReset(PRCM_GSPI);  //Reset the peripheral
  
  MAP_SPIReset(GSPI_BASE);  //Reset SPI
  
  //Configure SPI interface
  MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                   SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_3,
                   (SPI_HW_CTRL_CS |
                   SPI_3PIN_MODE |
                   SPI_TURBO_OFF |
                   SPI_CS_ACTIVELOW |
                   SPI_WL_8));

  SET_SPI1_CS_ON();  //adx345 spi cs high
  
  SET_SPI2_CS_ON();  //n25q512a spi cs high 
  
  MAP_SPIEnable(GSPI_BASE);  //Enable SPI for communication
}

/*******************************************************************************
//spi send and recive a byte
*******************************************************************************/
uint8_t SPI_SendReciveByte(uint8_t addr)
{ 
  uint8_t retry=0;
  
  //Wait for space in FIFO
  while(!(HWREG(GSPI_BASE + MCSPI_O_CH0STAT)&MCSPI_CH0STAT_TXS))  
  {
    if(retry++>200)
    {
      break;
    }
  }
  
  retry=0;
  
  HWREG(GSPI_BASE + MCSPI_O_TX0) = addr;  //Write the data
  
  //Wait for Rx data
  while(!(HWREG(GSPI_BASE + MCSPI_O_CH0STAT) & MCSPI_CH0STAT_RXS))  
  { 
    if(retry++>220)
    {
      break;
    }
  }
  return (uint8_t)HWREG(GSPI_BASE + MCSPI_O_RX0);  //Read the value
}


/*******************************************************************************
                                      END         
*******************************************************************************/




