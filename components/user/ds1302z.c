

//
//head include//
//
#include "ds1302z.h"
#include "time.h"
#include "string.h"
#include "stdlib.h"


//
//ds1302 send a byte//
//
static void ds1302_WriteByte(uint8_t txd)
{
  uint8_t i;
  
  SDA_OUT();  //set sda pin out mode//
  
  for(i=0;i<8;i++)
  {  
    IIC_SCL_OFF();    //DATA can change and send//
    
    delay_us(2);  //delay about 2us//
    
    if(txd&0x01)
    {
      IIC_SDA_ON();
    }
    else
    {
      IIC_SDA_OFF();
    }
    
    txd>>=1; 	  
    delay_us(2);  //delay about 2us//
    
    IIC_SCL_ON();  //send one bit//
    delay_us(2); //delay about 2us//
  }	
}


//
//DS1302 read a byte//
//
uint8_t ds1302_ReadByte(void)
{
  uint8_t i,recive=0;
  
  SDA_IN();  //SDA IN mode//
  
  for(i=0;i<8;i++ )
  {
    recive>>=1;
    IIC_SCL_OFF(); 
    delay_us(2);  //delay about 2us//
    
    if(GPIOPinRead(iic_sda_gpio,iic_sda_pin))
    {
      recive=recive|0x80;
    }
    
    IIC_SCL_ON();  //send one bit//
    delay_us(2);  //delay about 2us//
  }					 
  return recive;
}


//
//DS1302 write register//
//
void ds1302_WriteReg(uint8_t addr,uint8_t tex)
{
  RST_OFF();
  IIC_SCL_OFF();
  delay_us(2);  //delay about 2us//
  
  RST_ON();
  delay_us(2);  //delay about 2us//
  
  ds1302_WriteByte(addr);  //write ds1302 address//
  ds1302_WriteByte(tex);  //write data//
  
  RST_OFF();
}


//
//DS1302 read register//
//
uint8_t ds1302_ReadReg(uint8_t addr)
{
  uint8_t recive;
  
  RST_OFF();
  IIC_SCL_OFF();
  delay_us(2);  //delay about 2us//
  
  RST_ON();
  delay_us(2);  //delay about 2us//
  
  ds1302_WriteByte(addr);  //write ds1302 address//
  recive=ds1302_ReadByte();  //read data//
  
  IIC_SCL_OFF();
  delay_us(2);  //delay about 2us//
  
  RST_OFF();
  
  return recive;
}

//
//ds1302 read UTC time//
//
void Read_UTCtime(char *buffer)  
{
  struct tm ts;
  uint8_t  time[7];
  
  time[0]=ds1302_ReadReg(r_sec_addr);  //read second register//
  
  time[1]=ds1302_ReadReg(r_min_addr);  //read minite register//
  
  time[2]=ds1302_ReadReg(r_hour_addr);  //read hour register//
  
  time[3]=ds1302_ReadReg(r_date_addr);  //read date register//
  
  time[4]=ds1302_ReadReg(r_month_addr);  //read month register//
  
  time[5]=ds1302_ReadReg(r_year_addr);  //read year register//

  
  ts.tm_sec=10*(time[0]/16)+time[0]%16;  //second//
  ts.tm_min=10*(time[1]/16)+time[1]%16;  //minite//
  
  ts.tm_hour=10*(time[2]/16)+time[2]%16;  //hour//
  ts.tm_mday=10*(time[3]/16)+time[3]%16;  //date//
  
  ts.tm_mon=10*(time[4]/16)+time[4]%16-1;  //month//
  ts.tm_year=10*(time[5]/16)+time[5]%16+100;  //year//
  
  strftime(buffer,21,"%Y-%m-%dT%H:%M:%SZ",&ts);  //UTC time//
}


//
//ds1302 update time//
//
void Update_UTCtime(char *time)
{
  uint16_t year;
  char *InpString;
  uint8_t timedata[6];
  uint8_t mon,day,hour,min,sec;
  
  InpString = strtok(time, "-");
  year = (uint16_t)strtoul(InpString, 0, 10)-2000;  //year//

  InpString = strtok(NULL, "-");
  mon=(uint8_t)strtoul(InpString, 0, 10);  //mon
  
  InpString = strtok(NULL, "T");
  day=(uint8_t)strtoul(InpString, 0, 10);  //day//
  
  InpString = strtok(NULL, ":");
  hour=(uint8_t)strtoul(InpString, 0, 10);  //hour//

  InpString = strtok(NULL, ":");
  min=(uint8_t)strtoul(InpString, 0, 10);  //min//

  InpString = strtok(NULL, "Z");
  sec=(uint8_t)strtoul(InpString, 0, 10);  //sec//
  

  timedata[0]=(sec/10)*16+sec%10;  //second//
  
  timedata[1]=(min/10)*16+min%10;  //minute//
  
  timedata[2]=(hour/10)*16+hour%10;  //hour//
  
  timedata[3]=(day/10)*16+day%10;  //date//
  
  timedata[4]=(mon/10)*16+mon%10;  //month//
  
  timedata[5]=(year/10)*16+year%10;  //year//
  
  ds1302_WriteReg(w_write_addr,0x00);  //write enable//
  
  ds1302_WriteReg(w_sec_addr,timedata[0]);  //write second register,start the oscilltor//
  
  ds1302_WriteReg(w_min_addr,timedata[1]);  //write minite register//
  
  ds1302_WriteReg(w_hour_addr,timedata[2]);  //write hour register,24 hour mode//
  
  ds1302_WriteReg(w_date_addr,timedata[3]);  //write date register//
  
  ds1302_WriteReg(w_month_addr,timedata[4]);  //write month register//
  
  ds1302_WriteReg(w_year_addr,timedata[5]);  //write year register//
  
  ds1302_WriteReg(w_write_addr,0x80);  //write disable//
}


//
//ds1302 read Unix time//
//
unsigned long Read_UnixTime(void)  
{
  struct tm ts;
  uint8_t  time[7];
  
  time[0]=ds1302_ReadReg(r_sec_addr);  //read second register//
  
  time[1]=ds1302_ReadReg(r_min_addr);  //read minite register//
  
  time[2]=ds1302_ReadReg(r_hour_addr);  //read hour register//
  
  time[3]=ds1302_ReadReg(r_date_addr);  //read date register//
  
  time[4]=ds1302_ReadReg(r_month_addr);  //read month register//
  
  time[5]=ds1302_ReadReg(r_year_addr);  //read year register//

  
  ts.tm_sec=10*(time[0]/16)+time[0]%16;  //second//
  
  ts.tm_min=10*(time[1]/16)+time[1]%16;  //minite//
  
  ts.tm_hour=10*(time[2]/16)+time[2]%16;  //hour//
  
  ts.tm_mday=10*(time[3]/16)+time[3]%16;  //date//
  
  ts.tm_mon=10*(time[4]/16)+time[4]%16-1;  //month//
  
  ts.tm_year=10*(time[5]/16)+time[5]%16+100;  //2000-1900 year//
  
  return mktime(&ts);  //unix time//
}

//******************************************************************************
//ds1302 init//
//******************************************************************************
void DS1302_Init(void)
{
  uint8_t regval;

  ds1302_WriteReg(w_write_addr,0x00);  //write enable//
  
  
  regval=ds1302_ReadReg(r_sec_addr);
  
  regval&=0x7f;  //bit7 CN=0//
  
  ds1302_WriteReg(w_sec_addr,regval);  //start the oscilltor//
  
  
  regval=ds1302_ReadReg(r_hour_addr);
  
  regval&=0x3f;  //bit7=0,24 hour mode//
  
  ds1302_WriteReg(w_hour_addr,regval);  //24h mode//
  
  
  ds1302_WriteReg(w_write_addr,0x80);  //write disable//
}


//-------------------------------------END------------------------------------//



