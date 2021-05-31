/*******************************************************************************
  * @file       DS18B20 Temperature Sensor Application      
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
#include "ds18b20.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "pin.h"
#include "rom_map.h"
#include "gpio.h"
#include "utils.h"
#include "common.h"

#define PARASITE_POWER_MODE

#define ds18b20_gpio    GPIOA0_BASE 
#define ds18b20_pin     GPIO_PIN_3

#define DATA_IO_ON()            MAP_GPIOPinWrite(ds18b20_gpio,ds18b20_pin,ds18b20_pin);  //SET IO HIGH
#define DATA_IO_OFF()           MAP_GPIOPinWrite(ds18b20_gpio,ds18b20_pin,0x00);         //SET IO LOW

#define ds18b20_io_in()         MAP_GPIODirModeSet(ds18b20_gpio, ds18b20_pin, GPIO_DIR_MODE_IN);  //Configure PIN_18 for GPIO Input mode
#define ds18b20_io_out()        MAP_GPIODirModeSet(ds18b20_gpio, ds18b20_pin, GPIO_DIR_MODE_OUT);  //Configure PIN_18 for GPIO Output mode

/*******************************************************************************
  ds18b20 reset,return:0 reset success,return:-1,failured
*******************************************************************************/
static short ds18b20_reset(void)
{
  uint8_t retry=0;
  
  ds18b20_io_out();
  
  DATA_IO_ON();
  
  MAP_UtilsDelay(80);  //delay about 6 us
  
  DATA_IO_OFF();
  
  MAP_UtilsDelay(10000);  //delay about 750 us
  
  DATA_IO_ON();
  
  MAP_UtilsDelay(200);  //delay about 15 us
  
  ds18b20_io_in();
  
  while(GPIOPinRead(ds18b20_gpio, ds18b20_pin))  //waite ds18b20 respon
  {
    retry++;
    
    if(retry>200)
    {
      return FAILURE;
    }
    
    MAP_UtilsDelay(40);  //delay about 3 us
  }
  
  MAP_UtilsDelay(6400);  //delay about 480 us
  
  ds18b20_io_out();  //data pin out mode
  
  DATA_IO_ON();
  
  return SUCCESS;
}

/*******************************************************************************
  ds18b20 read a bit,return 0/1 
*******************************************************************************/
static uint8_t ds18b20_read_bit(void)
{
  uint8_t data;
  
  ds18b20_io_out();
  
  DATA_IO_ON();
  
  MAP_UtilsDelay(20);  //delay about 1.5 us
  
  DATA_IO_OFF();
  
  MAP_UtilsDelay(40);  //delay about 3 us
  
  DATA_IO_ON();
  
  ds18b20_io_in();
  
  MAP_UtilsDelay(200);  //delay about 15 us
  
  if(GPIOPinRead(ds18b20_gpio, ds18b20_pin))
  {
    data=1;
  }
  else
  {
    data=0;
  }
  
  MAP_UtilsDelay(800);  //delay about 60 us
  
  return data;
}

/*******************************************************************************
  ds18b20 read a byte
*******************************************************************************/
static uint8_t ds18b20_read_byte(void)
{
  uint8_t i,j,data=0;
  
  for(i=0;i<8;i++)
  {
    j=ds18b20_read_bit();
    
    data=(j<<7)|(data>>1);
  }
  
  return data;
}

/*******************************************************************************
  ds18b20 write a byte
*******************************************************************************/
static void ds18b20_write_byte(uint8_t data)
{
  uint8_t i,data_bit;
  
  ds18b20_io_out();  //data pin out mode
  
  for(i=0;i<8;i++)
  {
    data_bit=data&0x01;
    
    if(data_bit)
    {
      DATA_IO_OFF();
  
      MAP_UtilsDelay(40);  //delay about 3 us
      
      DATA_IO_ON();
      
      MAP_UtilsDelay(800);  //delay about 60 us
    }
    else
    {
      DATA_IO_OFF();
  
      MAP_UtilsDelay(800);  //delay about 60 us
      
      DATA_IO_ON();
      
      MAP_UtilsDelay(40);  //delay about 3 us
    }
    data=data>>1;
  }
}

/*******************************************************************************
  ds18b20 start convert
*******************************************************************************/
static void ds18b20_start(void)
{
  if(ds18b20_reset()==0)
  {
    ds18b20_write_byte(0xcc);   //skip rom
    
    ds18b20_write_byte(0x44);   //start convert
    
    #ifdef PARASITE_POWER_MODE 
      
      DATA_IO_ON();
      
      MAP_UtilsDelay(10000000);  //delay about 750ms
      
    #endif
  }
}

/*******************************************************************************
  ds18b20 get temperature value
*******************************************************************************/
float ds18b20_get_temp(void)
{
  uint8_t data_h,data_l;
  short temp;
  float temp_value;
  
  ds18b20_start();  //ds18b20 start convert
  
  if(ds18b20_reset()==0)
  {
    ds18b20_write_byte(0xcc);   //skip rom
    
    ds18b20_write_byte(0xbe);   //read data
    
    data_l=ds18b20_read_byte();  //LSB
    
    data_h=ds18b20_read_byte();  //MSB
    
    if(data_h>7)  //temperature value<0
    {
      data_l=~data_l;
      data_h=~data_h;
      
      temp=data_h;
      temp<<=8;
      temp+=data_l;
      
      temp_value=-(float)temp*0.0625;
      
      if(temp_value >= -55)
      return temp_value;
    }
    else //temperature value>=0
    {
      temp=data_h;
      temp<<=8;
      temp+=data_l;
      
      temp_value=(float)temp*0.0625;
      
      if(temp_value <= 125)
      return temp_value;
    }
  }
  return ERROR_CODE;  //failured to read data
}


/*******************************************************************************
                                      END         
*******************************************************************************/



