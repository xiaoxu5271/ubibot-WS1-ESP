
/*****************co and no2 amp application*******************/

//header include
#include "stdint.h"
#include "hw_types.h"
#include "i2c_if.h"
#include "uart_if.h"
#include "gpio.h"
#include "rom_map.h"
#include "hw_memmap.h"


//register address
#define LMP91002SD_ADDR                 0x48    //hardware iic use
#define LMP91002SD_READ			0X91    //iic use
#define LMP91002SD_WRITE		0x90    //iic use
#define	STATUS				0X00
#define LOCK				0x01
#define TIACN				0X10
#define	REFCN				0x11
#define MODECN				0X12

//writer lem91002sd amp register
//regaddr:register address
//val:write register value
static void Lmp91002sd_WriteReg(uint8_t regaddr,uint8_t val)
{
  uint8_t reg[2];
  reg[0]=regaddr;
  reg[1]=val;
  I2C_IF_Write(LMP91002SD_ADDR,reg,2,0);    //write zhe read register addr and the register value
}

//read lem91002sd amp register
//regaddr:register address
//return:read register value
static uint8_t Lmp91002sd_ReadReg(uint8_t regaddr)
{
  uint8_t reg[1];
  uint8_t regdata[1];
  reg[0]=regaddr;
  I2C_IF_Write(LMP91002SD_ADDR,reg,1,0);    //write zhe read register addr 
  I2C_IF_Read(LMP91002SD_ADDR,regdata,1);
  return regdata[0];
}

//Init LMP91002SD-CO device¡ê?successed return 0,failued return 1
void CO_Lmp91002sdInit(void)
{
  MAP_GPIOPinWrite(GPIOA0_BASE,0x40,0x40);      //disable no2 amp cs
  MAP_GPIOPinWrite(GPIOA0_BASE,0x80,0x00);      //enable co amp cs
  while(Lmp91002sd_ReadReg(STATUS)==0)  //Waite device ready to accept
  {  
  }
  Lmp91002sd_WriteReg(LOCK,0X00);       //TIACN	REFCN register write enable
  while(Lmp91002sd_ReadReg(STATUS)==0)  //Waite device ready to accept
  {  
  }
  Lmp91002sd_WriteReg(TIACN,0X14);       //35k gain ,1.4v max(1000ppm)
  while(Lmp91002sd_ReadReg(STATUS)==0)  //Waite device ready to accept
  {  
  }
  Lmp91002sd_WriteReg(REFCN,0X20);       //reference voltage internal 50%
  while(Lmp91002sd_ReadReg(STATUS)==0)  //Waite device ready to accept
  {  
  }
  Lmp91002sd_WriteReg(MODECN,0X03);       //3-lead amperometric
  MAP_GPIOPinWrite(GPIOA0_BASE,0x80,0x80);      //disable co amp cs
}

//Init LMP91002SD-NO2 device¡ê?successed return 0,failued return 1
void NO2_Lmp91002sdInit(void)
{
  MAP_GPIOPinWrite(GPIOA0_BASE,0x80,0x80);      //disable co amp cs
  MAP_GPIOPinWrite(GPIOA0_BASE,0x40,0x00);      //enable co amp cs
  while(Lmp91002sd_ReadReg(STATUS)==0)  //Waite device ready to accept
  {  
  }
  Lmp91002sd_WriteReg(LOCK,0X00);       //TIACN	REFCN register write enable
  while(Lmp91002sd_ReadReg(STATUS)==0)  //Waite device ready to accept
  {  
  }
  Lmp91002sd_WriteReg(TIACN,0X10);       //14k gain ,1.26v max
  while(Lmp91002sd_ReadReg(STATUS)==0)  //Waite device ready to accept
  {  
  }
  Lmp91002sd_WriteReg(REFCN,0X20);       //reference voltage internal 50%
  while(Lmp91002sd_ReadReg(STATUS)==0)  //Waite device ready to accept
  {  
  }
  Lmp91002sd_WriteReg(MODECN,0X03);       //3-lead amperometric
  MAP_GPIOPinWrite(GPIOA0_BASE,0x40,0x40);      //disable co amp cs
}









