/*****************l3gd20h sensor application*******************/

//driver include
#include "hw_types.h"
#include "spi.h"
#include "rom_map.h"
#include "hw_memmap.h"
#include "gpio.h"
#include "stdint.h"
#include "uart_if.h"

#include "adx345.h"

//Register Address
#define L3GD20H_READ			0XD5    //iic use
#define L3GD20H_WRITE			0XD4    //iic use
#define WHO_AM_I			0X0F
#define CTRL1				0X20
#define CTRL2				0X21
#define CTRL3				0X22
#define CTRL4				0X23
#define CTRL5				0X24
#define REFERENCE			0X25
#define OUT_TEMP			0X26
#define STATUS				0X27
#define OUT_X_L				0X28
#define OUT_X_H				0X29
#define OUT_Y_L				0X2A
#define OUT_Y_H				0X2B
#define OUT_Z_L				0X2C
#define OUT_Z_H				0X2D
#define FIFO_CTRL			0X2E
#define FIFO_SRC			0X2F
#define IG_CFG				0X30
#define IG_SRC				0X31
#define IG_THS_XL			0X33
#define IG_THS_XH			0X32
#define IG_THS_YL			0X35
#define IG_THS_YH			0X34
#define IG_THS_ZL			0X37
#define IG_THS_ZH			0X36
#define IG_DURATION			0X38
#define LOW_ODR				0X39

////user spi write a l3gd20h register 
static void l3gd20h_spi_writeReg(uint8_t addr,uint8_t value)
{ 
  MAP_GPIOPinWrite(GPIOA1_BASE,0x20,0);      //enable the l3gd20h spi cs
  MAP_SPIEnable(GSPI_BASE);     //enable the spi channel
  SPI_SendReciveByte(addr);     //send the address and write single command
  SPI_SendReciveByte(value);    //send write value
  MAP_SPIDisable(GSPI_BASE);    //disable the spi channel
  MAP_GPIOPinWrite(GPIOA1_BASE,0x20,0x20);      //disable the l3gd20h spi cs  
}

//user spi read a l3gd20h register 
static uint8_t l3gd20h_spi_readReg(uint8_t addr)
{
  uint8_t RegValue;
  MAP_GPIOPinWrite(GPIOA1_BASE,0x20,0);      //enable the l3gd20h spi cs
  MAP_SPIEnable(GSPI_BASE);     //enable the spi channel
  SPI_SendReciveByte(addr|0x80);        //send the address and read single command
  RegValue=SPI_SendReciveByte(0x00);    //clk signal
  MAP_SPIDisable(GSPI_BASE);    //disable the spi channel
  MAP_GPIOPinWrite(GPIOA1_BASE,0x20,0x20);      //disable the l3gd20h spi cs 
  return RegValue;
}

//Init L3GD20H device¡ê?successed return 0¡ê?failued return 1
uint8_t L3GD20H_Init(void)
{
  if(l3gd20h_spi_readReg(WHO_AM_I)==0XD7)	//Read device ID
  {  
    l3gd20h_spi_writeReg(CTRL1,0Xcf);	//800Hz 100Hz 30 cut-off,power on,device x y z in enable mode
    l3gd20h_spi_writeReg(CTRL2,0X01);	// external level sensitive trigger disable,High pass filter cut off frequency  30Hz
    l3gd20h_spi_writeReg(CTRL3,0X80);	//interrupt enable on INT1
    l3gd20h_spi_writeReg(CTRL4,0X30);	//2000dps
    l3gd20h_spi_writeReg(CTRL5,0X10);	//High pass filter Enable
    //l3gd20h_spi_writeReg(IG_CFG,0X25);	//x y z axis high event interrupts have been generated
    l3gd20h_spi_writeReg(LOW_ODR,0X08);	//spi only,low speed ODR disable
    return 0;
  }	
  Report("l3gd20h digital gyroscope sensor init failued\n\r");
  return 1;	   					
}

//read three axis digital output data
void l3gd20h_Readxyz(short *x,short *y,short *z)
{
  uint8_t x0,x1,y0,y1,z0,z1;
  
  while((l3gd20h_spi_readReg(STATUS)&0x0F)!=0x0F);
  
  MAP_GPIOPinWrite(GPIOA1_BASE,0x20,0);      //enable the l3gd20h spi cs
  MAP_SPIEnable(GSPI_BASE);     //enable the spi channel
  
  SPI_SendReciveByte(OUT_X_L|0xc0);      //send the address and read multi data commend
  x0=SPI_SendReciveByte(OUT_X_L);        //read x0 data 
  x1=SPI_SendReciveByte(OUT_X_H);        //read x1 data
  y0=SPI_SendReciveByte(OUT_Y_L);        //read y0 data
  y1=SPI_SendReciveByte(OUT_Y_H);        //read y1 data
  z0=SPI_SendReciveByte(OUT_Z_L);        //read z0 data
  z1=SPI_SendReciveByte(OUT_Z_H);        //read z1 data
  
  MAP_SPIDisable(GSPI_BASE);    //disable the channel
  MAP_GPIOPinWrite(GPIOA1_BASE,0x20,0x20);      //disable the l3gd20h spi cs 
  *x=(short)(((uint16_t)x1<<8)+x0);     //get x axis data
  *y=(short)(((uint16_t)y1<<8)+y0);     //get y axis data
  *z=(short)(((uint16_t)z1<<8)+z0);     //get z axis data
}
