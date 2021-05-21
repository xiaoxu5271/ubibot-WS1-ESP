/*******************************************************************************
  * @file       PCF8563 Driver  
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
#include "PCF8563.h"
#include "iic.h"
#include "time.h"
#include "string.h"
#include "stdlib.h"
#include "rom_map.h"
#include "utils.h"
#include "MsgType.h"
#include "at24c08.h"

/*******************************************************************************
//PCF_Format_BCD to PCF_Format_BIN
*******************************************************************************/
static uint8_t Bin_To_Bcd(uint8_t bin_val)
{
  return 16*(bin_val/10)+bin_val%10;
}

/*******************************************************************************
//PCF_Format_BIN to PCF_Format_BCD
*******************************************************************************/
static uint8_t Bcd_To_Bin(uint8_t bcd_val)
{
  return 10*(bcd_val/16)+bcd_val%16;
}

/*******************************************************************************
//Return Unix time
*******************************************************************************/
unsigned long Return_UnixTime(uint8_t *t_buf)  
{
  struct tm ts;

  ts.tm_sec = t_buf[0];  //second

  ts.tm_min = t_buf[1];  //minite

  ts.tm_hour = t_buf[2];  //hour

  ts.tm_mday = t_buf[3];  //date
  
  ts.tm_mon = t_buf[5]-1;  //month
  
  ts.tm_year = t_buf[6]+100;  //2000-1900 year
  
  ts.tm_isdst=0;  //do not use Daylight Saving Time
  
  return mktime(&ts);  //unix time
}

/*******************************************************************************
//PCF8563 update time
*******************************************************************************/
void Update_UTCtime(char *time)
{
  char *InpString;
  uint8_t i;
  uint8_t reg_buf[7] = {0};
  uint8_t time_buf[7] = {0};
  unsigned long net_time = 0;
  unsigned long sys_time = 0;
  
  InpString = strtok(time, "-");
  time_buf[6] = (uint8_t)((uint16_t)strtoul(InpString, 0, 10)-2000);  //year

  InpString = strtok(NULL, "-");
  time_buf[5]=(uint8_t)strtoul(InpString, 0, 10);  //month
  
  InpString = strtok(NULL, "T");
  time_buf[3]=(uint8_t)strtoul(InpString, 0, 10);  //day
  
  InpString = strtok(NULL, ":");
  time_buf[2]=(uint8_t)strtoul(InpString, 0, 10);  //hour

  InpString = strtok(NULL, ":");
  time_buf[1]=(uint8_t)strtoul(InpString, 0, 10);  //min

  InpString = strtok(NULL, "Z");
  time_buf[0]=(uint8_t)strtoul(InpString, 0, 10);  //sec
  
  net_time = Return_UnixTime(time_buf);

  sys_time = Read_UnixTime();
  
  if((sys_time > (net_time+UPDATE_TIME_SIZE))||(sys_time < (net_time-UPDATE_TIME_SIZE)))
  {
    reg_buf[0]=Bin_To_Bcd(time_buf[0])&0x7f;  //second
      
    reg_buf[1]=Bin_To_Bcd(time_buf[1])&0x7f;  //minute
    
    reg_buf[2]=Bin_To_Bcd(time_buf[2])&0x3f;  //hour
    
    reg_buf[3]=Bin_To_Bcd(time_buf[3])&0x3f;  //date
    
    reg_buf[5]=Bin_To_Bcd(time_buf[5])&0x1f;  //month
    
    reg_buf[6]=Bin_To_Bcd(time_buf[6]);  //year
    
    for(i=0;i<RETRY_TIME_OUT;i++)
    {
      MulTry_I2C_WR_mulReg(PCF8563_ADDR,VL_seconds,reg_buf,sizeof(reg_buf));  //Write Timer Register
      
      sys_time = Read_UnixTime();
      
      if((sys_time < (net_time+UPDATE_TIME_SIZE))&&(sys_time > (net_time-UPDATE_TIME_SIZE)))
      {
        at24c08_write(LAST_UPDATE_TIME_ADDR,sys_time);  //sys_time
        
        break;
      }
      else
      {
        MAP_UtilsDelay(200000);  //delay about 15ms
      }
    }
  }
}

/*******************************************************************************
//PCF8563 read UTC time
*******************************************************************************/
void Read_UTCtime(char *buffer,uint8_t buf_size)  
{
  struct tm ts;
  uint8_t  read_time[7];
  
  MulTry_I2C_RD_mulReg(PCF8563_ADDR,VL_seconds,read_time,sizeof(read_time));  //Read Timer Register

  read_time[0]&=0x7f;
  ts.tm_sec=Bcd_To_Bin(read_time[0]);  //second value
  
  read_time[1]&=0x7f;
  ts.tm_min=Bcd_To_Bin(read_time[1]);  //minite value
  
  read_time[2]&=0x3f;
  ts.tm_hour=Bcd_To_Bin(read_time[2]);  //hour value
  
  read_time[3]&=0x3f;
  ts.tm_mday=Bcd_To_Bin(read_time[3]);  //day value
  
  read_time[5]&=0x1f;
  ts.tm_mon=Bcd_To_Bin(read_time[5])-1;  //month value
  
  ts.tm_year=Bcd_To_Bin(read_time[6])+100;  //year value
  
  ts.tm_isdst=0;  //do not use Daylight Saving Time
  
  strftime(buffer,buf_size,"%Y-%m-%dT%H:%M:%SZ",&ts);  //UTC time
}

/*******************************************************************************
  PCF8563 read Unix time
*******************************************************************************/
unsigned long Read_UnixTime(void)  
{
  struct tm ts;
  uint8_t  read_time[7];
  
  MulTry_I2C_RD_mulReg(PCF8563_ADDR,VL_seconds,read_time,sizeof(read_time));  //Read Timer Register

  read_time[0]&=0x7f;
  ts.tm_sec=Bcd_To_Bin(read_time[0]);  //second
  
  read_time[1]&=0x7f;
  ts.tm_min=Bcd_To_Bin(read_time[1]);  //minite
  
  read_time[2]&=0x3f;
  ts.tm_hour=Bcd_To_Bin(read_time[2]);  //hour
  
  read_time[3]&=0x3f;
  ts.tm_mday=Bcd_To_Bin(read_time[3]);  //date
  
  read_time[5]&=0x1f;
  ts.tm_mon=Bcd_To_Bin(read_time[5])-1;  //month
  
  ts.tm_year=Bcd_To_Bin(read_time[6])+100;  //2000-1900 year
  
  ts.tm_isdst=0;  //do not use Daylight Saving Time
  
  return mktime(&ts);  //unix time
}

/*******************************************************************************
//PCF8563 init
*******************************************************************************/
void Timer_IC_Init(void)
{
  uint8_t status_val[2]={0x00,0x00};
  
  uint8_t alarm_setval[4]={0x80,0x80,0x80,0x80};  //MIN/HOUR/DAY/WEEDDAY ALARM DISABLED
  
  uint8_t timer_set[3]={0x00,0x00,0x00};  //CLKOUT INHIBITED/TIMER DISABLED/TIMER VALUE
  
  MulTry_I2C_WR_mulReg(PCF8563_ADDR,Control_status_1,status_val,sizeof(status_val));  //Write Timer Register
  
  MulTry_I2C_WR_mulReg(PCF8563_ADDR,Minute_alarm,alarm_setval,sizeof(alarm_setval));  //Write Timer Register
  
  MulTry_I2C_WR_mulReg(PCF8563_ADDR,CLKOUT_control,timer_set,sizeof(timer_set));  //Write Timer Register
}

/*******************************************************************************
//PCF8563 Reset Time
*******************************************************************************/
void Timer_IC_Reset_Time(void)
{
  uint8_t time_val[7]={0x00,0x00,0x00,0x01,0x00,0x01,0x17};
  
  MulTry_I2C_WR_mulReg(PCF8563_ADDR,VL_seconds,time_val,sizeof(time_val));  //Write Timer Register
}


/*******************************************************************************
                                      END         
*******************************************************************************/




