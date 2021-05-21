/*******************************************************************************
  * @file       Temperature and Humility Sensor Application Driver     
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
#include "sht30dis.h"
#include "iic.h"
#include "rom_map.h"
#include "utils.h"
#include "common.h"
#include "crc_8_check.h"

#define sht30dis_addr           0x44    //7 MSB address 0x44


/*******************************************************************************
  SHT30DIS Single Shot Measure
  Command:0x2400-Repeatability:high,clock stretching:disable
*******************************************************************************/
void sht30_SingleShotMeasure(float *temp,float *humi)
{
  uint16_t retry;
  uint8_t recive[6];
  uint16_t tempval,humival;
  
  *temp=ERROR_CODE;
  
  *humi=ERROR_CODE;
  
  IIC_Start();  //iic bus start
  
  IIC_Send_Byte(2*sht30dis_addr);  //send sht30dis write command 
  
  if(IIC_Wait_Ack()<0)  //wait device ack
  {
    goto end;
  }
  
  IIC_Send_Byte(0x24);  //send command MSB
  
  if(IIC_Wait_Ack()<0)  //wait device ack
  {
    goto end;
  }
  
  IIC_Send_Byte(0x00);  //send command LSB
  
  if(IIC_Wait_Ack()<0)  //wait device ack
  {
    goto end;
  }
  
  IIC_Stop();  //IIC stop
  
  for(retry=0;retry<1000;retry++)
  {
    IIC_Start();  //iic bus start
    
    IIC_Send_Byte(2*sht30dis_addr+1);  //send sht30dis read command
    
    if(IIC_Wait_Ack()<0)  //wait device ack
    {
      IIC_Stop();  //IIC stop
    }
    else  //ACK:measurement completed
    {
      for(retry=0;retry<6;retry++)
      {
        if(retry==5)
        {
          recive[retry]=IIC_Read_Byte(0);  //read a byte,NACK
        }
        else
        {
          recive[retry]=IIC_Read_Byte(1);  //read a byte,ACK
        }
      }
      
      if(Data_Crc_Check(&recive[0],3)==0)
      {
        tempval=recive[0];
        
        tempval=tempval<<8;
        
        tempval+=recive[1];

        *temp=175*(float)tempval/65535-45;
      }
      
      if(Data_Crc_Check(&recive[3],3)==0)
      {
        humival=recive[3];
        
        humival=humival<<8;
        
        humival+=recive[4];
        
        *humi=100*humival/65535; 
      }
      
      break;
    }
    
    MAP_UtilsDelay(60000);  //delay about 4.5ms
  }
 
end:
  
  IIC_Stop();  //IIC stop
}

/*******************************************************************************
  sht30dis sensor reset,command:0x30a2
*******************************************************************************/
short sht30dis_reset(void)
{
  IIC_Start();  //iic bus start
  
  IIC_Send_Byte(2*sht30dis_addr);  //send sht30dis write command
  
  if(IIC_Wait_Ack()<0)  //wait device ack
  {
    IIC_Stop();  //IIC stop
    
    return FAILURE;
  }
  
  IIC_Send_Byte(0x30);  //send command MSB
  
  if(IIC_Wait_Ack()<0)  //wait device ack
  {
    IIC_Stop();  //IIC stop
    
    return FAILURE;
  }
  
  IIC_Send_Byte(0xa2);  //send command LSB
  
  if(IIC_Wait_Ack()<0)  //wait device ack
  {
    IIC_Stop();  //IIC stop
    
    return FAILURE;
  }
  
  IIC_Stop();  //IIC stop
  
  return SUCCESS;
}


/*******************************************************************************
                                      END         
*******************************************************************************/




