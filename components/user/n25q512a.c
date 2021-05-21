/******************************************************************************/
/******************************************************************************/
/******************** n25q512a memory chip application ************************/
/******************************************************************************/
/******************************************************************************/

//driver include
#include "hw_types.h"
#include "spi.h"
#include "rom_map.h"
#include "hw_memmap.h"
#include "gpio.h"
#include "stdint.h"
#include "uart_if.h"

#include "adx345.h"

//read n25q128a chip status register
static uint8_t N25q_ReadStaReg(void)
{
  uint8_t value=0;
  MAP_GPIOPinWrite(GPIOA2_BASE,GPIO_PIN_6,0x00);      //n25q512a spi cs enable
  MAP_SPIEnable(GSPI_BASE);     //enable the spi channel
  SPI_SendReciveByte(0x05);        //send the status register address read commen
  value=SPI_SendReciveByte(0x00);    //clk signal
  MAP_SPIDisable(GSPI_BASE);    //disable the spi channel
  MAP_GPIOPinWrite(GPIOA2_BASE,GPIO_PIN_6,0x40);     //n25q512a spi cs disable
  return value;
}

//waite write in progress or over
static void N25q_WriteInpro(void)
{
  uint8_t a=1;
  while(a&0x01) //waite status register write in progress bit =0 :Ready,1:Busy
  {
    a=N25q_ReadStaReg();    //read status register
  }
}

/*
//n25q128a write status register
static void N25q_WriterStaReg(void)
{
  MAP_GPIOPinWrite(GPIOA2_BASE,GPIO_PIN_6,0x00);      //n25q512a spi cs enable
  MAP_SPIEnable(GSPI_BASE);     //enable the spi channel
  SPI_SendReciveByte(0x01);        //send the status register address write commend
  SPI_SendReciveByte(0x00);    //status register write enable,top,no area protection,cleared
  MAP_SPIDisable(GSPI_BASE);    //disable the spi channel
  MAP_GPIOPinWrite(GPIOA2_BASE,GPIO_PIN_6,0x40);      //n25q512a spi cs disable
}
*/

//write enable
static void N25q_WriteEnable(void)
{
  MAP_GPIOPinWrite(GPIOA2_BASE,GPIO_PIN_6,0x00);      //n25q512a spi cs enable
  MAP_SPIEnable(GSPI_BASE);     //enable the spi channel
  SPI_SendReciveByte(0x06);        //send the write enable commend
  MAP_SPIDisable(GSPI_BASE);    //disable the spi channel
  MAP_GPIOPinWrite(GPIOA2_BASE,GPIO_PIN_6,0x40);      //n25q512a spi cs disable
  N25q_WriteInpro();        //waite ready
}

//write disable
static void N25q_WriteDisable(void)
{
  MAP_GPIOPinWrite(GPIOA2_BASE,GPIO_PIN_6,0x00);      //n25q512a spi cs enable
  MAP_SPIEnable(GSPI_BASE);     //enable the spi channel
  SPI_SendReciveByte(0x04);        //send the write disable commend
  MAP_SPIDisable(GSPI_BASE);    //disable the spi channel
  MAP_GPIOPinWrite(GPIOA2_BASE,GPIO_PIN_6,0x40);      //n25q512a spi cs disable
  N25q_WriteInpro();        //waite ready
}
//Erase a subsector 4k
///addr:subsector address
void N25q_EraseSubsector(uint32_t addr)
{
  addr*=4096;
  N25q_WriteEnable();	//write enable
  MAP_GPIOPinWrite(GPIOA2_BASE,GPIO_PIN_6,0x00);      //n25q512a spi cs enable
  MAP_SPIEnable(GSPI_BASE);     //enable the spi channel
  SPI_SendReciveByte(0x20);	//subsector eaase code
  SPI_SendReciveByte((uint8_t)((addr&0xFFFFFFFF)>>24));   //first 8 bit address
  SPI_SendReciveByte((uint8_t)((addr&0xFFFFFF)>>16));   //second 8 bit address
  SPI_SendReciveByte((uint8_t)((addr&0xFFFF)>>8));
  SPI_SendReciveByte((uint8_t)(addr&0xFF));	//last 8 bit address
  MAP_SPIDisable(GSPI_BASE);    //disable the spi channel
  MAP_GPIOPinWrite(GPIOA2_BASE,GPIO_PIN_6,0x40);      //n25q512a spi cs disable
  N25q_WriteInpro();		//waite to over
  N25q_WriteDisable();  //write disable
}


//Erase a sector	64k
//addr:sector address
void N25q_EraseSector(uint32_t addr)
{
  addr*=32768;   //8*4096;
  N25q_WriteEnable();	//write enable
  MAP_GPIOPinWrite(GPIOA2_BASE,GPIO_PIN_6,0x00);      //n25q512a spi cs enable
  MAP_SPIEnable(GSPI_BASE);     //enable the spi channel
  SPI_SendReciveByte(0xD8);	//sector eaase code
  SPI_SendReciveByte((uint8_t)((addr&0xFFFFFFFF)>>24));   //first 8 bit address
  SPI_SendReciveByte((uint8_t)((addr&0xFFFFFF)>>16));   //second 8 bit address
  SPI_SendReciveByte((uint8_t)((addr&0xFFFF)>>8));
  SPI_SendReciveByte((uint8_t)(addr&0xFF));	//last 8 bit address
  MAP_SPIDisable(GSPI_BASE);    //disable the spi channel
  MAP_GPIOPinWrite(GPIOA2_BASE,GPIO_PIN_6,0x40);      //n25q512a spi cs disable
  N25q_WriteInpro();		//waite to over
  N25q_WriteDisable();  //write disable
}

//Erase chip 512M
void N25q_EraseChip(void)
{
  N25q_WriteEnable();	//write enable
  MAP_GPIOPinWrite(GPIOA2_BASE,GPIO_PIN_6,0x00);      //n25q512a spi cs enable
  MAP_SPIEnable(GSPI_BASE);     //enable the spi channel
  SPI_SendReciveByte(0xC7);	//chip eaase code bulk erase
  MAP_SPIDisable(GSPI_BASE);    //disable the spi channel
  MAP_GPIOPinWrite(GPIOA2_BASE,GPIO_PIN_6,0x40);      //n25q512a spi cs disable
  N25q_WriteInpro();		//waite to over
  N25q_WriteDisable();  //write disable
}


//read id
uint32_t N25q_ReadId(void)
{
  uint32_t id=0;
  MAP_GPIOPinWrite(GPIOA2_BASE,GPIO_PIN_6,0x00);     //n25q512a spi cs enable
  MAP_SPIEnable(GSPI_BASE);     //enable the spi channel
  SPI_SendReciveByte(0x9f);        //read id code 0x9e/f
  id=SPI_SendReciveByte(0x00);        //manufacturer identification
  id=id<<8;
  id+=SPI_SendReciveByte(0x00); //device identification 1
  id=id<<8;
  id+=SPI_SendReciveByte(0x00); //device identification 2
  MAP_SPIDisable(GSPI_BASE);    //disable the spi channel
  MAP_GPIOPinWrite(GPIOA2_BASE,GPIO_PIN_6,0x40);      //n25q512a spi cs disable
  return id;
}
  
//read memory
void N25q_ReadMemory(uint32_t addr,uint8_t *buffer,uint16_t size)
{
  uint16_t i=0;
  MAP_GPIOPinWrite(GPIOA2_BASE,GPIO_PIN_6,0x00);     //n25q512a spi cs enable
  MAP_SPIEnable(GSPI_BASE);     //enable the spi channel
  SPI_SendReciveByte(0x03);	//read operations code
  SPI_SendReciveByte((uint8_t)((addr&0xFFFFFFFF)>>24));   //first 8 bit address
  SPI_SendReciveByte((uint8_t)((addr&0xFFFFFF)>>16));   //second 8 bit address
  SPI_SendReciveByte((uint8_t)((addr&0xFFFF)>>8));
  SPI_SendReciveByte((uint8_t)(addr&0xFF));	//last 8 bit address
  while(i<size)
  {
    buffer[i]=SPI_SendReciveByte(0x00);	//read one byte
    i++;
  }
  MAP_SPIDisable(GSPI_BASE);    //disable the spi channel
  MAP_GPIOPinWrite(GPIOA2_BASE,GPIO_PIN_6,0x40);      //n25q512a spi cs disable
}

// write memory less then 256 byte a page
//65536 pages,256byte each
//Size<=256
void N25q_WritePage(uint32_t addr,uint8_t *buffer,uint16_t Size)
{
  uint16_t i;
  N25q_WriteEnable();	//Write Enable
  MAP_GPIOPinWrite(GPIOA2_BASE,GPIO_PIN_6,0x00);      //n25q512a spi cs enable
  MAP_SPIEnable(GSPI_BASE);     //enable the spi channel
  SPI_SendReciveByte(0x02);	//page program code 
  SPI_SendReciveByte((uint8_t)((addr&0xFFFFFFFF)>>24));   //first 8 bit address
  SPI_SendReciveByte((uint8_t)((addr&0xFFFFFF)>>16));   //second 8 bit address
  SPI_SendReciveByte((uint8_t)((addr&0xFFFF)>>8));
  SPI_SendReciveByte((uint8_t)(addr&0xFF));	//last 8 bit address
  for(i=0;i<Size;i++)
  SPI_SendReciveByte(buffer[i]);	//write data
  MAP_SPIDisable(GSPI_BASE);    //disable the spi channel
  MAP_GPIOPinWrite(GPIOA2_BASE,GPIO_PIN_6,0x40);      //n25q512a spi cs disable
  N25q_WriteInpro();        //waite ready 
  N25q_WriteDisable();  //write disable
}

//no check write memory 
//the memory is all 0xff
void N25q_NoCheckWrite(uint8_t *buffer,uint32_t addr,uint16_t Size)
{
  uint16_t pageremain;
  pageremain=256-addr%256;	//page remain byte number
  if(Size<=pageremain)
    pageremain=Size;		//less the 256 byte
  while(1)
  {
    N25q_WritePage(addr,buffer,pageremain);	//write page
    if(Size==pageremain)
      break;	//	
    else
    {
      buffer+=pageremain;
      addr+=pageremain;
      Size-=pageremain;
      if(Size>256) 
        pageremain=256;	//write 256 byte
      else
        pageremain=Size;	//less then 256 byte
    }
  }
}

//write memory
void N25q_WriteMemory(uint8_t *buffer,uint32_t addr,uint16_t Size)
{
  uint8_t N25q_buffer[4096];
  uint32_t secpos;
  uint16_t secoff;
  uint16_t secremain;
  uint16_t i;
  secpos=addr/4096;	//subsector address,0-4096
  secoff=addr%4096;	//subsector offset
  secremain=4096-secoff;	//remain size
  
  if(Size<=secremain)
          secremain=Size;	//less then 4096 byte
  while(1)
  {
    N25q_ReadMemory(4096*secpos,N25q_buffer,4096);	//read all sector
    for(i=0;i<secremain;i++)	//data check
    {
      if(N25q_buffer[secoff+i]!=0xff)
              break;	//need to erase
    }
    if(i<secremain)	//erase
    {
      N25q_EraseSubsector(secpos);	//erase the sector
      for(i=0;i<secremain;i++)	//write
      {
              N25q_buffer[secoff+i]=buffer[i];
      }
      N25q_NoCheckWrite(N25q_buffer,secpos*4096,4096);  //write sector
    }
    else
            N25q_NoCheckWrite(buffer,addr,secremain);  //write sector
    if(Size==secremain)
            break;	//write over
    else
    {
      secpos++;
      secoff=0;
      buffer+=secremain;
      addr-=secremain;
      if(Size>4096)
              secremain=4096;
      else
              secremain=Size;
    }
  }
          
}











