/*******************************************************************************
  * @file       AT24C08 EEPROM CHIP DRIVER APPLICATION     
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
#include "string.h"
#include "at24c08.h"
#include "iic.h"
#include "rom_map.h"
#include "utils.h"
#include "common.h"
#include "crc_8_check.h"

/*******************************************************************************
//check two data buffer same or not
*******************************************************************************/
static short Data_Buf_Check(uint8_t * a_buf,uint8_t * b_buf,uint8_t buf_len)
{
  uint8_t i;
  
  for(i=0;i<buf_len;i++)
  {
    if(a_buf[i]!=b_buf[i])
    {
      return FAILURE;
    }
  }
  return SUCCESS;
}

/*******************************************************************************
//save data with crc8 check in at24c08
*******************************************************************************/
static void at24c02_write_check(uint8_t sla_addr,uint8_t reg_addr,uint8_t *val_buf,uint8_t buf_len)
{
  uint8_t i;
  uint8_t retry;
  uint8_t read_buf[5]={0};
  uint8_t write_buf[5]={0};
  
  for(i=0;i<buf_len;i++)
  {
    write_buf[i]=val_buf[i];
  }
  
  write_buf[buf_len]=Data_Crc_Value(val_buf,buf_len);  //crc-8 value
  
  for(retry=0;retry<RETRY_TIME_OUT;retry++)
  {
    MulTry_I2C_WR_mulReg(sla_addr,reg_addr,write_buf,buf_len+1);  //iic write data
    
    MAP_UtilsDelay(67500);  //delay about 5ms
    
    MulTry_I2C_RD_mulReg(sla_addr,reg_addr,read_buf,buf_len+1);  //iic read data
    
    if(Data_Buf_Check(write_buf,read_buf,buf_len+1) == SUCCESS)  //check write and read data
    {
      break;
    }
  }
}

/*******************************************************************************
//read data with crc8 check in at24c08
*******************************************************************************/
static void at24c02_read_check(uint8_t sla_addr,uint8_t reg_addr,uint8_t *val_buf,uint8_t buf_len)
{
  uint8_t i;
  uint8_t retry;
  uint8_t read_buf[5]={0};
  
  for(retry=0;retry<RETRY_TIME_OUT;retry++)
  {
    MulTry_I2C_RD_mulReg(sla_addr,reg_addr,read_buf,buf_len+1);  //read data
    
    if(Data_Crc_Check(read_buf,buf_len+1) == SUCCESS)
    {
      break;
    }
  }
  
  for(i=0;i<buf_len;i++)
  {
    val_buf[i]=read_buf[i];
  }
}

/*******************************************************************************
//write a byte in at24c08
*******************************************************************************/
void at24c08_write_byte(uint16_t reg_addr,uint8_t val)
{
  uint8_t data_buf[1];
  
  data_buf[0]=val;
    
  at24c02_write_check(at24c08_addr0+reg_addr/256,reg_addr%256,data_buf,1);  //save data with crc8 check in at24c08
}

/*******************************************************************************
//read a byte in at24c08
*******************************************************************************/
uint8_t at24c08_read_byte(uint16_t reg_addr)
{
  uint8_t data_buf[1];
  
  at24c02_read_check(at24c08_addr0+reg_addr/256,reg_addr%256,data_buf,1);  //read data with crc8 check in at24c08
  
  return data_buf[0];
}

/******************************************************************************
//write unsigned long data 
******************************************************************************/
void at24c08_write(uint16_t reg_addr,unsigned long val)
{
  uint8_t data_buf[4];
  
  data_buf[0]=(uint8_t)(val>>24);
  data_buf[1]=(uint8_t)(val>>16);
  data_buf[2]=(uint8_t)(val>>8);
  data_buf[3]=(uint8_t)(val);

  at24c02_write_check(at24c08_addr0+reg_addr/256,reg_addr%256,data_buf,4);  //save data with crc8 check in at24c08
}

/******************************************************************************
//read unsigned long data 
******************************************************************************/
unsigned long at24c08_read(uint16_t reg_addr)
{
  uint8_t data_buf[4];
  unsigned long val=0;
  
  at24c02_read_check(at24c08_addr0+reg_addr/256,reg_addr%256,data_buf,4);  //read data with crc8 check in at24c08
  
  val+=(unsigned long)(data_buf[0]<<24);
  val+=(unsigned long)(data_buf[1]<<16);
  val+=(unsigned long)(data_buf[2]<<8);
  val+=(unsigned long)data_buf[3];
  
  return val;
}

/******************************************************************************
//at24c08 write page,addr:0-1023,Size:1-16
******************************************************************************/
static void at24c08_WritePage(uint16_t reg_addr,uint8_t *buffer,uint8_t buf_len)
{
  MulTry_I2C_WR_mulReg(at24c08_addr0+reg_addr/256,reg_addr%256,buffer,buf_len);  //iic multi write
  
  MAP_UtilsDelay(67500);  //delay about 5ms
}

/******************************************************************************
//at24c08 write data
//addr:0-1023,*buffer:write data,Size:1-256
******************************************************************************/
void at24c08_WriteData(uint16_t addr,uint8_t *buf,uint8_t size,bool end_flag)
{
  uint8_t i;
  uint8_t remain;
  
  remain=16-addr%16;
  
  if(remain)
  {
    remain=size>remain?remain:size;
    
    at24c08_WritePage(addr,buf,remain);
    
    addr+=remain;
    
    buf+=remain;
    
    size-=remain;
  }
  
  remain=size/16;
  
  for(i=0;i<remain;i++)
  {
    at24c08_WritePage(addr,buf,16);
      
    addr+=16;
    
    buf+=16;
  }
  
  remain=size%16;
  
  if(remain)
  {
    at24c08_WritePage(addr,buf,remain);
    
    addr+=remain;
  }
  
  if(end_flag)
  {
    at24c08_WritePage(addr,"!",1);  //end flag '!'
  }
}

/*******************************************************************************
//read multi byte from at24c08
*******************************************************************************/
static short I2C_ReadAt24c08(uint8_t sla_addr,uint8_t reg_addr,uint8_t *buf,uint8_t size,bool end_flag) 		
{
  uint8_t i;
  
  IIC_Start();  
  
  IIC_Send_Byte(2*sla_addr);  //send write command
  
  if(IIC_Wait_Ack()<0)  //wait device ack
  {
    IIC_Stop();  //IIC stop
    
    return FAILURE;
  }
  
  IIC_Send_Byte(reg_addr);  //send register address
  
  if(IIC_Wait_Ack()<0)  //wait device ack
  {
    IIC_Stop();  //IIC stop
    
    return FAILURE;
  }
  
  IIC_Start();  //IIC start
  
  IIC_Send_Byte(2*sla_addr+1);  //send read command 
  
  if(IIC_Wait_Ack()<0)  //wait device ack
  {
    IIC_Stop();  //IIC stop
    
    return FAILURE;
  }
  
  for(i=0;i<size;i++)
  {
    if(i == size-1)
    {
      buf[i]=IIC_Read_Byte(0);  //read a byte
    } 
    else
    {
      buf[i]=IIC_Read_Byte(1);  //read a byte
    }

    if((buf[i]=='!')&&(end_flag))  //end flag
    {
      buf[i]=IIC_Read_Byte(0);  //read a byte
      
      buf[i]='\0';
      
      break;
    }  
  }

  if(i == size)
  {
    return FAILURE;
  }
  
  IIC_Stop();  //IIC stop
  
  return SUCCESS;
}

/*******************************************************************************
//at24c08 read data
*******************************************************************************/
void at24c08_ReadData(uint16_t reg_addr,uint8_t *buf,uint8_t size,bool end_flag)
{
  uint8_t n_try;
  
  for(n_try=0;n_try<RETRY_TIME_OUT;n_try++)
  {
    if(I2C_ReadAt24c08(at24c08_addr0+reg_addr/256,reg_addr%256,buf,size,end_flag) == SUCCESS)
    {
      break;
    }
    else
    {
      MAP_UtilsDelay(40000);  //delay about 3ms
    }
  }
}

/*******************************************************************************
                                      END         
*******************************************************************************/




