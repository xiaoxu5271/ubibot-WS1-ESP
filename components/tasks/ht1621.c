/*******************************************************************************
  * @file       HT1621 DRIVER APPLICATION      
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
#include "stdbool.h"
#include "math.h"
#include "iic.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "rom_map.h"
#include "gpio.h"
#include "pin.h"
#include "utils.h"
#include "common.h"
#include "ht1621.h"

/*******************************************************************************
  Ht1621 send byte,write data high len bit
*******************************************************************************/
void SendBit_1621(uint8_t data,uint8_t len)      
{
  uint8_t i;
  
  HT1621_DATA_OUT();
    
  for(i=0;i<len;i++)
  {
    HT1621_WR_OFF();            //wr off
    
    MAP_UtilsDelay(200);        //delay 15us
    
    if((data&0x80)==0)
    {
      HT1621_DATA_OFF();        //data off
    }
    else
    {
      HT1621_DATA_ON();         //data on
    }

    MAP_UtilsDelay(200);        //delay 15us
    
    HT1621_WR_ON();             //wr on
    
    MAP_UtilsDelay(200);        //delay 15us
    
    data<<=1;
  }
}

/*******************************************************************************
  Ht1621 send byte,write data low len bit
*******************************************************************************/
void SendDataBit_1621(uint8_t data,uint8_t len)      
{
  uint8_t i;

  HT1621_DATA_OUT();
  
  for(i=0;i<len;i++)
  {
    HT1621_WR_OFF();            //wr off
    
    MAP_UtilsDelay(200);        //delay 15us
    
    if((data&0x01)==0)
    {
      HT1621_DATA_OFF();        //data off
    }
    else
    {
      HT1621_DATA_ON();         //data on
    }
   
    MAP_UtilsDelay(200);        //delay 15us
    
    HT1621_WR_ON();             //wr on
    
    MAP_UtilsDelay(200);        //delay 15us
    
    data>>=1;
  }
}

/*******************************************************************************
  Ht1621 read byte
*******************************************************************************/
uint8_t HT1621_Read_Bit(void)
{
  uint8_t i,read_val=0;

  HT1621_DATA_ON();             //data
  
  HT1621_DATA_IN();
    
  for(i=0;i<4;i++)
  {
    HT1621_RD_OFF();            //rd
    
    MAP_UtilsDelay(200);        //15us
    
    if(GPIOPinRead(HT1621_DATA_GPIO,HT1621_DATA_PIN))
    {
      read_val+=1<<i;
    }

    MAP_UtilsDelay(200);        //15us
    
    HT1621_RD_ON();             //wr
    
    MAP_UtilsDelay(200);        //15us
    
  }
  
  return read_val;
}

/*******************************************************************************
  Ht1621 write command
*******************************************************************************/
void Ht1621_SendCmd(uint8_t command)
{
  HT1621_CS_OFF(); //gpio15 cs=0
  
  SendBit_1621(0x80,3); //command oeration id 100
  
  SendBit_1621(command,9);      //write 9bit data, 9bit can be any
  
  HT1621_CS_ON(); //gpio15 cs=1
}

/*******************************************************************************
  Ht1621 write data
*******************************************************************************/
void Ht1621_Write(uint8_t addr,uint8_t data)
{
  addr<<=2;
    
  HT1621_CS_OFF(); //gpio15 cs=0
  
  SendBit_1621(0xa0,3); //command oeration id 101
  
  SendBit_1621(addr,6); //write 6 bit address
  
  SendDataBit_1621(data,4);     //write low 4 bit data
  
  HT1621_CS_ON(); //gpio15 cs=1
}

/*******************************************************************************
  Ht1621 read data
*******************************************************************************/
uint8_t Ht1621_Read(uint8_t addr)
{
  uint8_t read_val;
  
  addr<<=2;
    
  HT1621_CS_OFF(); //gpio15 cs=0
  
  SendBit_1621(0xc0,3); //command oeration id 101
  
  SendBit_1621(addr,6); //write 6 bit address
  
  read_val=HT1621_Read_Bit();  //read low 4 bit data
  
  HT1621_CS_ON(); //gpio15 cs=1
  
  return read_val;
}

/*******************************************************************************
  Ht1621 off
*******************************************************************************/
void Ht1621_Off(void)
{
  Ht1621_SendCmd(LCDOFF);
}

/*******************************************************************************
  Ht1621 init
*******************************************************************************/
void Ht1621_On(void)
{
  Ht1621_SendCmd(SYSEN);        //open HT1621B osi
  
  Ht1621_SendCmd(LCDON);        //start HT1621B
  
  Ht1621_SendCmd(RCosc);
  
  Ht1621_SendCmd(BIAS);         //1/3 BIAS
  
  Ht1621_Write(7,0x01);
  
  Ht1621_Write(22,0x00);
  
  Ht1621_Write(23,0x00);
  
  Ht1621_Display_Err_Val(0x00);
}

/*******************************************************************************
  Display the Temp number
*******************************************************************************/
static void Display_Temp_Data_Number(uint8_t addr1,uint8_t addr2,uint8_t data_val,bool end_flag)
{
  switch(data_val)
  {
    case 0:
    {
      Ht1621_Write(addr1,0x0f);
      
      if(end_flag)
      {
        Ht1621_Write(addr2,0x0b);
      }
      else
      {
        Ht1621_Write(addr2,0x0a);
      }
        
      break;
    }
    case 1:
    {
      Ht1621_Write(addr1,0x06);
      
      if(end_flag)
      {
        Ht1621_Write(addr2,0x01);
      }
      else
      {
        Ht1621_Write(addr2,0x00);
      }
      
      break;
    }
    case 2:
    {
      Ht1621_Write(addr1,0x0d);
      
      if(end_flag)
      {
        Ht1621_Write(addr2,0x07);
      }
      else
      {
        Ht1621_Write(addr2,0x06);
      }
      
      break;
    }
    case 3:
    {
      Ht1621_Write(addr1,0x0f);
      
      if(end_flag)
      {
        Ht1621_Write(addr2,0x05);
      }
      else
      {
        Ht1621_Write(addr2,0x04);
      }
      
      break;
    }
    case 4:
    {
      Ht1621_Write(addr1,0x06);
      
      if(end_flag)
      {
        Ht1621_Write(addr2,0x0d);
      }
      else
      {
        Ht1621_Write(addr2,0x0c);
      }
      
      break;
    }
    case 5:
    {
      Ht1621_Write(addr1,0x0b);
      
      if(end_flag)
      {
        Ht1621_Write(addr2,0x0d);
      }
      else
      {
        Ht1621_Write(addr2,0x0c);
      }
      
      break;
    }
    case 6:
    {
      Ht1621_Write(addr1,0x0b);
      
      if(end_flag)
      {
        Ht1621_Write(addr2,0x0f);
      }
      else
      {
        Ht1621_Write(addr2,0x0e);
      }
      
      break;
    }
    case 7:
    {
      Ht1621_Write(addr1,0x0e);
      
      if(end_flag)
      {
        Ht1621_Write(addr2,0x01);
      }
      else
      {
        Ht1621_Write(addr2,0x00);
      }
      
      break;
    }
    case 8:
    {
      Ht1621_Write(addr1,0x0f);
      
      if(end_flag)
      {
        Ht1621_Write(addr2,0x0f);
      }
      else
      {
        Ht1621_Write(addr2,0x0e);
      }
      
      break;
    }
    case 9:
    {
      Ht1621_Write(addr1,0x0f);
      
      if(end_flag)
      {
        Ht1621_Write(addr2,0x0d);
      }
      else
      {
        Ht1621_Write(addr2,0x0c);
      }
      
      break;
    }
    default:
    {
      break;
    }
  }
}

/*******************************************************************************
  Display the Humi number
*******************************************************************************/
static void Display_Humi_Data_Number(uint8_t addr1,uint8_t addr2,uint8_t data_val)
{
  switch(data_val)
  {
    case 0:
    {
      Ht1621_Write(addr1,0x0f);
      
      Ht1621_Write(addr2,0x0b);

      break;
    }
    case 1:
    {
      Ht1621_Write(addr1,0x06);
      
      Ht1621_Write(addr2,0x01);
      
      break;
    }
    case 2:
    {
      Ht1621_Write(addr1,0x0b);
      
      Ht1621_Write(addr2,0x0d);
      
      break;
    }
    case 3:
    {
      Ht1621_Write(addr1,0x0f);
      
      Ht1621_Write(addr2,0x05);
      
      break;
    }
    case 4:
    {
      Ht1621_Write(addr1,0x06);
      
      Ht1621_Write(addr2,0x07);
      
      break;
    }
    case 5:
    {
      Ht1621_Write(addr1,0x0d);
      
      Ht1621_Write(addr2,0x07);
      
      break;
    }
    case 6:
    {
      Ht1621_Write(addr1,0x0d);
      
      Ht1621_Write(addr2,0x0f);
      
      break;
    }
    case 7:
    {
      Ht1621_Write(addr1,0x07);
      
      Ht1621_Write(addr2,0x01);
      
      break;
    }
    case 8:
    {
      Ht1621_Write(addr1,0x0f);
      
      Ht1621_Write(addr2,0x0f);
      
      break;
    }
    case 9:
    {
      Ht1621_Write(addr1,0x0f);
      
      Ht1621_Write(addr2,0x07);
      
      break;
    }
    default:
    {
      break;
    }
  }
}

/*******************************************************************************
  Display the Light number
*******************************************************************************/
static void Display_Light_Data_Number(uint8_t addr1,uint8_t addr2,uint8_t data_val,bool end_flag)
{
  switch(data_val)
  {
    case 0:
    {
      Ht1621_Write(addr1,0x0f);
      
      if(end_flag)
      {
        Ht1621_Write(addr2,0x0d);
      }
      else
      {
        Ht1621_Write(addr2,0x05);
      }
        
      break;
    }
    case 1:
    {
      Ht1621_Write(addr1,0x06);
      
      if(end_flag)
      {
        Ht1621_Write(addr2,0x08);
      }
      else
      {
        Ht1621_Write(addr2,0x00);
      }
      
      break;
    }
    case 2:
    {
      Ht1621_Write(addr1,0x0d);
      
      if(end_flag)
      {
        Ht1621_Write(addr2,0x0b);
      }
      else
      {
        Ht1621_Write(addr2,0x03);
      }
      
      break;
    }
    case 3:
    {
      Ht1621_Write(addr1,0x0f);
      
      if(end_flag)
      {
        Ht1621_Write(addr2,0x0a);
      }
      else
      {
        Ht1621_Write(addr2,0x02);
      }
      
      break;
    }
    case 4:
    {
      Ht1621_Write(addr1,0x06);
      
      if(end_flag)
      {
        Ht1621_Write(addr2,0x0e);
      }
      else
      {
        Ht1621_Write(addr2,0x06);
      }
      
      break;
    }
    case 5:
    {
      Ht1621_Write(addr1,0x0b);
      
      if(end_flag)
      {
        Ht1621_Write(addr2,0x0e);
      }
      else
      {
        Ht1621_Write(addr2,0x06);
      }
      
      break;
    }
    case 6:
    {
      Ht1621_Write(addr1,0x0b);
      
      if(end_flag)
      {
        Ht1621_Write(addr2,0x0f);
      }
      else
      {
        Ht1621_Write(addr2,0x07);
      }
      
      break;
    }
    case 7:
    {
      Ht1621_Write(addr1,0x0e);
      
      if(end_flag)
      {
        Ht1621_Write(addr2,0x08);
      }
      else
      {
        Ht1621_Write(addr2,0x00);
      }
      
      break;
    }
    case 8:
    {
      Ht1621_Write(addr1,0x0f);
      
      if(end_flag)
      {
        Ht1621_Write(addr2,0x0f);
      }
      else
      {
        Ht1621_Write(addr2,0x07);
      }
      
      break;
    }
    case 9:
    {
      Ht1621_Write(addr1,0x0f);
      
      if(end_flag)
      {
        Ht1621_Write(addr2,0x0e);
      }
      else
      {
        Ht1621_Write(addr2,0x06);
      }
      
      break;
    }
    default:
    {
      break;
    }
  }
}

/*******************************************************************************
  Display the temprature value
*******************************************************************************/
void Ht1621_Display_Temp_Val(float temp_val,bool ext_flag)
{
  bool val_flag=0;
  uint16_t data_val;
  uint8_t f_val,s_val,t_val;
  
  if(temp_val<0)
  {
    val_flag=1;
    
    temp_val=fabs(temp_val);
  }
  
  data_val=(uint16_t)(10*temp_val);
  
  f_val=data_val/100;
  
  data_val=data_val%100;
  
  s_val=data_val/10;
  
  t_val=data_val%10;
  
  if(f_val>0)
  {
    Display_Temp_Data_Number(9,8,f_val,ext_flag);
  }
  else
  {
    Ht1621_Write(9,0x00);
    
    if(ext_flag&&val_flag)
    {
      val_flag=0;
      
      Ht1621_Write(8,0x05);
    }
    else if(ext_flag)
    {
      Ht1621_Write(8,0x01);
    }
    else if(val_flag)
    {
      val_flag=0;
      
      Ht1621_Write(8,0x04);
    }
    else
    {
      Ht1621_Write(8,0x00);
    }
  }
  
  Display_Temp_Data_Number(11,10,s_val,val_flag);
  
  Display_Temp_Data_Number(13,12,t_val,0);
}

/*******************************************************************************
  Display the Humility value
*******************************************************************************/
void Ht1621_Display_Humi_Val(uint8_t humi_val)
{
  uint8_t f_val,s_val;
  
  f_val=humi_val/10;
  
  s_val=humi_val%10;
  
  if(f_val>0)
  {
    Display_Humi_Data_Number(6,7,f_val);
  }
  else
  {
    Ht1621_Write(6,0x00);
    
    Ht1621_Write(7,0x01);
  }
  
  Display_Humi_Data_Number(4,5,s_val);
}

/*******************************************************************************
  Display the error value
*******************************************************************************/
void Ht1621_Display_Err_Val(uint8_t err_val)
{
  uint8_t f_val,s_val;
  
  f_val=err_val/10;
  
  s_val=err_val%10;
  
  Display_Humi_Data_Number(2,3,f_val);
  
  Display_Humi_Data_Number(0,1,s_val);
}

/*******************************************************************************
  Display the light value
*******************************************************************************/
void Ht1621_Display_Light_Val(float light_val,bool T1_val,bool T2_val,bool T3_val,uint8_t l_val)
{
  bool max_flag=0;
  bool mid_flag=0;
  unsigned long data_val;
  uint8_t f_val,s_val,t_val,p_val;
  
  data_val=(unsigned long)(10*light_val);
  
  if(data_val>100000)
  {
    max_flag=1;
    
    f_val=data_val/100000;
    
    data_val=data_val%100000;
    
    s_val=data_val/10000;
    
    data_val=data_val%10000;
    
    t_val=data_val/1000;
    
    data_val=data_val%1000;
    
    p_val=data_val/100;
  }
  else if(data_val>10000)
  {
    mid_flag=1;
    
    f_val=data_val/10000;
    
    data_val=data_val%10000;
    
    s_val=data_val/1000;
    
    data_val=data_val%1000;
    
    t_val=data_val/100;
    
    data_val=data_val%100;
    
    p_val=data_val/10;
  }
  else
  {
    f_val=data_val/1000;
    
    data_val=data_val%1000;
    
    s_val=data_val/100;
    
    data_val=data_val%100;
    
    t_val=data_val/10;
    
    p_val=data_val%10;
  }
  
  if(f_val>0)
  {
    Display_Light_Data_Number(15,14,f_val,T1_val);
  }
  else
  {
    Ht1621_Write(15,0x00);
    if(T1_val)
    {
      Ht1621_Write(14,0x08);
    }
    else
    {
      Ht1621_Write(14,0x00);
    }
  }
  
  if((s_val>0)||(f_val>0))
  {
    Display_Light_Data_Number(17,16,s_val,T2_val);
  }
  else
  {
    Ht1621_Write(17,0x00);
    if(T2_val)
    {
      Ht1621_Write(16,0x08);
    }
    else
    {
      Ht1621_Write(16,0x00);
    }
  }
  
  Display_Light_Data_Number(19,18,t_val,T3_val);
  
  if(mid_flag||max_flag)
  {
    Display_Light_Data_Number(21,20,p_val,1);
    
    if(max_flag)
    {
      Ht1621_Write(24,0x08|l_val);
    }
    else
    {
      Ht1621_Write(24,l_val);
    }
  }
  else
  {
    Ht1621_Write(24,l_val);
    
    Display_Light_Data_Number(21,20,p_val,0);
  }
  
}


/*******************************************************************************
                                      END         
*******************************************************************************/




