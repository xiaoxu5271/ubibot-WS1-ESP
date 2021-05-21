/*******************************************************************************
  * @file       IIC BUS DRIVER APPLICATION       
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
#include "iic.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "rom_map.h"
#include "gpio.h"
#include "pin.h"
#include "utils.h"
#include "common.h"

/*******************************************************************************
  delay about nus*3us
*******************************************************************************/
void delay_us(uint8_t nus)
{
  MAP_UtilsDelay(40*nus);  //40
}

/*******************************************************************************
  iic start:when CLK is high,SDA change form high to low
*******************************************************************************/
void IIC_Start(void)
{
  SDA_OUT();  //sda out mode
  
  IIC_SDA_ON();
  
  IIC_SCL_ON();
  
  delay_us(5);  //delay about 5us
  
  IIC_SDA_OFF();
  
  delay_us(5);  //delay about 5us
  
  IIC_SCL_OFF();
}

/*******************************************************************************
  iic stop:when CLK is high SDA change form low to high
*******************************************************************************/
void IIC_Stop(void)
{
  SDA_OUT();  //sda out mode
  
  IIC_SCL_OFF();
  
  IIC_SDA_OFF(); 
  
  IIC_SCL_ON();
  
  delay_us(5);  //delay about 5us
  
  IIC_SDA_ON();
  
  delay_us(5);	//delay about 5us						   	
}

/*******************************************************************************
  Master send ack
*******************************************************************************/
static void IIC_Ack(void)
{
  SDA_OUT();    //sda out mode
  
  IIC_SCL_OFF();
  
  IIC_SDA_OFF();
  
  delay_us(2);  //delay about 2us
  
  IIC_SCL_ON();
  
  delay_us(5);  //delay about 5us
  
  IIC_SCL_OFF();
}

/*******************************************************************************
  Master send no ack
*******************************************************************************/
static void IIC_NAck(void)
{
  SDA_OUT();  //sda out mode
  
  IIC_SCL_OFF();
  
  IIC_SDA_ON();
  
  delay_us(2);  //delay about 2us
  
  IIC_SCL_ON();
  
  delay_us(5);  //delay about 5us
  
  IIC_SCL_OFF();
}

/*******************************************************************************
//init the iic bus
*******************************************************************************/
void I2C_Init(void)
{
  SDA_OUT();  //sda out mode
  
  IIC_SCL_ON();
  
  IIC_SDA_ON();
}

/*******************************************************************************
  iic bus wait ack,acked return SUCCESS,no acked return FAILURE
*******************************************************************************/
short IIC_Wait_Ack(void)
{
  uint8_t retry=0;
  
  SDA_IN();  //SDA IN
  
  IIC_SDA_ON();
  
  delay_us(1);  //delay about 1us
  
  IIC_SCL_ON();
  
  delay_us(1);  //delay about 1us
  
  while(GPIOPinRead(iic_sda_gpio,iic_sda_pin))  //waite for ack
  {
    if(retry++>200)  //200us time out
    {
      IIC_Stop();  //iic bus stop
      
      return FAILURE;
    }
    delay_us(2);  //delay about 2us
  }
  
  IIC_SCL_OFF();
  
  return SUCCESS;  
} 

/*******************************************************************************
  iic send a byte
*******************************************************************************/
void IIC_Send_Byte(uint8_t txd)
{                        
  uint8_t t;   
  
  SDA_OUT(); 	    
  
  IIC_SCL_OFF();
  
  for(t=0;t<8;t++)
  {      
    if(txd&0x80)
    {
      IIC_SDA_ON();
    }
    else
    {
      IIC_SDA_OFF();
    }
    
    txd<<=1; 
    
    IIC_SCL_ON();
    
    delay_us(2);  //delay about 2us
    
    IIC_SCL_OFF();
    
    delay_us(2);  //delay about 2us
  }
} 

/*******************************************************************************
  IIC Read a Byte,ack:1-send ack,0-no ack
*******************************************************************************/
uint8_t IIC_Read_Byte(uint8_t ack)
{
  uint8_t i;
  uint8_t receive=0;
  
  SDA_IN();  //sda in mode
  
  for(i=0;i<8;i++)
  {
    IIC_SCL_ON();
    
    delay_us(2);  //delay about 2us
    
    receive<<=1;
    
    if(GPIOPinRead(iic_sda_gpio,iic_sda_pin))  //read 1 bit data
    {
      receive++;
    }
    
    IIC_SCL_OFF(); 
    
    delay_us(1);  //delay about 1us
  }
  
  if (ack)
  {
    IIC_Ack();  //send ack
  }
  else
  {
    IIC_NAck();  //send no ack
  }
  
  return receive;
}

/*******************************************************************************
  write a byte to slave register
*******************************************************************************/
static short IIC_WR_Reg(uint8_t sla_addr,uint8_t reg_addr,uint8_t val)
{
  IIC_Start();  //IIC start 	
  
  IIC_Send_Byte(2*sla_addr);  //send write command
  
  if(IIC_Wait_Ack()<0)  //wait device ack
  {
    return FAILURE;
  }
  
  IIC_Send_Byte(reg_addr);  //send register address
  
  if(IIC_Wait_Ack()<0)  //wait device ack
  {
    return FAILURE;
  }
  
  IIC_Send_Byte(val);  //send data value
  
  if(IIC_Wait_Ack()<0)  //wait device ack
  {
    return FAILURE;
  }
   
  IIC_Stop();   //IIC stop	
  
  return SUCCESS;
}

/*******************************************************************************
  write a byte to slave register whit multiple try
*******************************************************************************/
void MulTry_IIC_WR_Reg(uint8_t sla_addr,uint8_t reg_addr,uint8_t val)
{
  uint8_t n_try;
  
  for(n_try=0;n_try<RETRY_TIME_OUT;n_try++)
  {
    if(IIC_WR_Reg(sla_addr,reg_addr,val)==SUCCESS)
    {
      break;
    }
    MAP_UtilsDelay(80000);  //delay about 6ms
  }
}

/*******************************************************************************
  Read a byte from slave register
*******************************************************************************/
static short IIC_RD_Reg(uint8_t sla_addr,uint8_t reg_addr,uint8_t *val)		
{
  IIC_Start();  //IIC start  	
  
  IIC_Send_Byte(2*sla_addr);  //send write command
  
  if(IIC_Wait_Ack()<0)  //wait device ack
  {
    return FAILURE;
  }
  
  IIC_Send_Byte(reg_addr);  //send register address
  
  if(IIC_Wait_Ack()<0)  //wait device ack
  {
    return FAILURE;
  }
  
  IIC_Start();  //IIC start
  
  IIC_Send_Byte(2*sla_addr+1);  //send read command
  
  if(IIC_Wait_Ack()<0)  //wait device ack
  {
    return FAILURE;
  }
  
  *val=IIC_Read_Byte(0);  //read a byte
  
  IIC_Stop();   //IIC stop
  
  return SUCCESS;
}

/*******************************************************************************
  Read a byte from slave register whit multiple try
*******************************************************************************/
void MulTry_IIC_RD_Reg(uint8_t sla_addr,uint8_t reg_addr,uint8_t *val)
{
  uint8_t n_try;
  
  for(n_try=0;n_try<RETRY_TIME_OUT;n_try++)
  {
    if(IIC_RD_Reg(sla_addr,reg_addr,val)==SUCCESS)
    {
      break;
    }
    MAP_UtilsDelay(40000);  //delay about 3ms
  }
}

/*******************************************************************************
  write multi byte to slave register
*******************************************************************************/
static short I2C_WR_mulReg(uint8_t sla_addr,uint8_t reg_addr,uint8_t *buf,uint8_t len) 
{
  IIC_Start();  //IIC start 	
  
  IIC_Send_Byte(2*sla_addr);  //send write command	
  
  if(IIC_Wait_Ack()<0)  //wait device ack
  {
    return FAILURE;
  }
  
  IIC_Send_Byte(reg_addr);  //send register address
  
  if(IIC_Wait_Ack()<0)  //wait device ack
  {
    return FAILURE;
  }
  
  while(len)
  {
    IIC_Send_Byte(*buf);  //send data value
    
    buf++;
    
    len--;
    
    if(IIC_Wait_Ack()<0)  //wait device ack
    {
      return FAILURE;
    }
  }

  IIC_Stop();   //IIC stop
  
  return SUCCESS;
}

/*******************************************************************************
  write multi byte to slave register whit multiple try
*******************************************************************************/
void MulTry_I2C_WR_mulReg(uint8_t sla_addr,uint8_t reg_addr,uint8_t *buf,uint8_t len) 
{
  uint8_t n_try;
  
  for(n_try=0;n_try<RETRY_TIME_OUT;n_try++)
  {
    if(I2C_WR_mulReg(sla_addr,reg_addr,buf,len)==SUCCESS)
    {
      break;
    }
    MAP_UtilsDelay(80000);  //delay about 6ms
  }
}

/*******************************************************************************
  read multiple byte from slave register
*******************************************************************************/
static short I2C_RD_mulReg(uint8_t sla_addr,uint8_t reg_addr,uint8_t *buf,uint8_t len) 		
{
  uint8_t i;
  
  IIC_Start();  //IIC start  	
  
  IIC_Send_Byte(2*sla_addr);  //send write command 
  
  if(IIC_Wait_Ack()<0)  //wait device ack
  {
    return FAILURE;
  }
  
  IIC_Send_Byte(reg_addr);  //send register address
  
  if(IIC_Wait_Ack()<0)  //wait device ack
  {  
    return FAILURE;
  }	
  
  IIC_Start();  //IIC start
  
  IIC_Send_Byte(2*sla_addr+1);  //send read command
  
  if(IIC_Wait_Ack()<0)  //wait device ack
  {
    return FAILURE;
  }
  
  for(i=0;i<len;i++)
  {
    if(i==len-1)
    {
      buf[i]=IIC_Read_Byte(0);  //read a byte no ack
    }
    else
    {
      buf[i]=IIC_Read_Byte(1);  //read a byte ack
    }
  }

  IIC_Stop();  //IIC stop
  
  return SUCCESS;
}

/*******************************************************************************
  read multiple byte from slave register whit multiple try
*******************************************************************************/
void MulTry_I2C_RD_mulReg(uint8_t sla_addr,uint8_t reg_addr,uint8_t *buf,uint8_t len) 
{
  uint8_t n_try;
  
  for(n_try=0;n_try<RETRY_TIME_OUT;n_try++)
  {
    if(I2C_RD_mulReg(sla_addr,reg_addr,buf,len)==SUCCESS)
    {
      break;
    }
    MAP_UtilsDelay(40000);  //delay about 3ms
  }
}


/*******************************************************************************
                                      END         
*******************************************************************************/




