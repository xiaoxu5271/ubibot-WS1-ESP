/******************************************************************************/
/******************************************************************************/
/************************** ds1308 time application ***************************/
/******************************************************************************/
/******************************************************************************/

//header include
#include "stdlib.h"
#include "stdint.h"
#include "string.h"
#include "time.h"
#include "hw_types.h"
#include "rom_map.h"
#include "hw_memmap.h"
#include "gpio.h"
#include "i2c_if.h"
#include "uart_if.h"

//assi addr
#define Dsaddr          0x68

//write register comand
static void IIC_WriteRegAddr(uint8_t slaaddr,uint8_t regaddr)
{
  uint8_t reg[1];
  reg[0]=regaddr;
  I2C_IF_Write(slaaddr,reg,1,0);    //write zhe read register addr 
}

//DS1308 Init 2016-1-1-1-0-0
void DS1308_Init(void)
{
  uint8_t timeset[9];
  MAP_GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_1,0x02);  //power on
  timeset[0]=0x00;     //first register address,second register
  timeset[1]=0x00;     //oscillator enable,0 seconds
  timeset[2]=0x00;     //0minutes
  timeset[3]=0x01;     //24h,1hour
  timeset[4]=0x00;     //day 0
  timeset[5]=0x01;     //date 1
  timeset[6]=0x01;     //month 1
  timeset[7]=0x16;     //year 16
  timeset[8]=0x00;     //ds1308 sqw/slkin disable
  I2C_IF_Write(Dsaddr,timeset,9,0);    //write the read register addr and data
  MAP_GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_1,0x00);  //power off
}

//DS1308 Read 
//retuan UTC time
void Read_UTCtime(char *buffer)
{
  struct tm ts;
  uint8_t  time[7];
  uint8_t mon,day,hour,min,sec;
  uint16_t year;
  
  MAP_GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_1,0x02);  //power on
  IIC_WriteRegAddr(Dsaddr,0x00);    //write the start read register address
  I2C_IF_Read(Dsaddr,time,7);
  MAP_GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_1,0x00);  //power off
 
  sec=time[0];				//read sec register
  sec=10*(sec/16)+sec%16;
  ts.tm_sec=(int)sec;
  
  min=time[1];			 //read min register
  min=10*(min/16)+min%16;
  ts.tm_min=min;
  
  hour=time[2];		//read hour register
  hour=10*(hour/16)+hour%16;
  ts.tm_hour=hour;
  
  day=time[4];			 //read day register
  day=10*(day/16)+day%16;
  ts.tm_mday=day;
  
  mon=time[5];		 //read mon register
  mon=10*(mon/16)+mon%16;
  ts.tm_mon=mon-1;
  
  year=time[6];		//read year register
  year=2000+10*(year/16)+year%16;
  ts.tm_year=year-1900;
  
  //Report("%4d-%2d-%2dT%2d:%2d:%2dZ\n\r",ts.tm_year,ts.tm_mon,ts.tm_mday,ts.tm_hour,ts.tm_min,ts.tm_sec);
  
  strftime(buffer,30,"%Y-%m-%dT%H:%M:%SZ",&ts);
  //sprintf(buffer,"%4d-%2d-%2dT%2d:%2d:%2dZ",year,mon,day,hour,min,sec);
  //Report("Buffer:%s\n\r",buffer);
}

void Update_UTCtime(char *time)
{
  uint8_t timedata[8];
  uint8_t mon,day,hour,min,sec;
  uint16_t year;
  char *InpString;
  char *ErrPtr;
  
  InpString = strtok(time, "-");
  //Report("Buffer:%s\n\r",InpString);
  if(InpString != NULL)
  {
    year = (uint16_t)strtoul(InpString, &ErrPtr, 10)-2000;
  }
  InpString = strtok(NULL, "-");
  //Report("Buffer:%s\n\r",InpString);
  if(InpString != NULL)
  {
    mon=(uint8_t)strtoul(InpString, &ErrPtr, 10);
  }
  InpString = strtok(NULL, "T");
  //Report("Buffer:%s\n\r",InpString);
  if(InpString != NULL)
  {
    day=(uint8_t)strtoul(InpString, &ErrPtr, 10);
  }
  InpString = strtok(NULL, ":");
  //Report("Buffer:%s\n\r",InpString);
  if(InpString != NULL)
  {
    hour=(uint8_t)strtoul(InpString, &ErrPtr, 10);
  }
  InpString = strtok(NULL, ":");
  //Report("Buffer:%s\n\r",InpString);
  if(InpString != NULL)
  {
    min=(uint8_t)strtoul(InpString, &ErrPtr, 10);
  }
  InpString = strtok(NULL, "Z");
  //Report("Buffer:%s\n\r",InpString);
  if(InpString != NULL)
  {
    sec=(uint8_t)strtoul(InpString, &ErrPtr, 10);
  }
  timedata[0]=0x00;     //first register address,second register
  timedata[1]=(sec/10)*16+sec%10;     //oscillator enable,0 seconds
  timedata[2]=(min/10)*16+min%10;     //0minutes
  timedata[3]=(hour/10)*16+hour%10;     //24h,1hour
  timedata[4]=0x00;     //day 0
  timedata[5]=(day/10)*16+day%10;     //date 1
  timedata[6]=(mon/10)*16+mon%10;     //month 1
  timedata[7]=(year/10)*16+year%10;     //year 15
  MAP_GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_1,0x02);  //power on
  I2C_IF_Write(Dsaddr,timedata,8,0);    //write the read register addr
  MAP_GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_1,0x00);  //power off
}

    
//DS1308 Read 
//retuan unix time
uint32_t DS1308_ReadTime(void)
{
  uint8_t  time[7];
  uint8_t mon,day,hour,min,sec;
  uint16_t year;
  uint32_t sumsec=0;
  
  MAP_GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_1,0x02);  //power on
  IIC_WriteRegAddr(Dsaddr,0x00);    //write the start read register address
  I2C_IF_Read(Dsaddr,time,7);
  MAP_GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_1,0x00);  //power off
 
  sec=time[0];				//read sec register
  sec=10*(sec/16)+sec%16;
  //Report("sec:%d\n\r",sec);     //for debug
  
  min=time[1];;			 //read min register
  min=10*(min/16)+min%16;
  //Report("min:%d\n\r",min);     //for debug
  
  hour=time[2];;		//read hour register
  hour=10*(hour/16)+hour%16;
  //Report("hour:%d\n\r",hour);   //for debug
  
  day=time[4];;			 //read day register
  day=10*(day/16)+day%16;
  //Report("day:%d\n\r",day);     //for debug
  
  mon=time[5];;			 //read mon register
  mon=10*(mon/16)+mon%16;
  //Report("mon:%d\n\r",mon);     //for debug
  
  year=time[6];;		//read year register
  year=2000+10*(year/16)+year%16;
  //Report("year:%d\n\r",year);   //for debug
  
  //1970+4x
  if((year-1970)%4==0)
  {
    sumsec+=((year-1970)/4)*126230400;	//126230400sec four years
    if(mon==1)
    {
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;		//hour
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==2)
    {
      sumsec+=31*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==3)
    {
      sumsec+=59*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==4)
    {
      sumsec+=90*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==5)
    {
      sumsec+=120*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==6)
    {
      sumsec+=151*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==7)
    {
      sumsec+=181*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==8)
    {
      sumsec+=212*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==9)
    {
      sumsec+=243*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==10)
    {
      sumsec+=273*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==11)
    {
      sumsec+=304*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==12)
    {
      sumsec+=334*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    else 
      return 0;
  }
	
  //1970+4x+1	
  if((year-1970)%4==1)
  {
    sumsec+=((year-1970)/4)*126230400;//126230400sec four years
    sumsec+=31536000;
    if(mon==1)
    {
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==2)
    {
      sumsec+=31*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==3)
    {
      sumsec+=59*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==4)
    {
      sumsec+=90*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==5)
    {
      sumsec+=120*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==6)
    {
      sumsec+=151*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==7)
    {
      sumsec+=181*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==8)
    {
      sumsec+=212*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==9)
    {
      sumsec+=243*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==10)
    {
      sumsec+=273*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==11)
    {
      sumsec+=304*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==12)
    {
      sumsec+=334*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    else 
      return 0;
  }

  //1970+4x+2	
  if((year-1970)%4==2)
  {
    sumsec+=((year-1970)/4)*126230400;//126230400sec four years
    sumsec+=2*31536000;
    if(mon==1)
    {
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==2)
    {
      sumsec+=31*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==3)
    {
      sumsec+=60*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==4)
    {
      sumsec+=91*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==5)
    {
      sumsec+=121*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==6)
    {
      sumsec+=152*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==7)
    {
      sumsec+=182*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==8)
    {
      sumsec+=213*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==9)
    {
      sumsec+=244*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==10)
    {
      sumsec+=274*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==11)
    {
      sumsec+=305*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==12)
    {
      sumsec+=335*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    else 
      return 0;
  }

  //1970+4x+3		
  if((year-1970)%4==3)
  {
    sumsec+=((year-1970)/4)*126230400;//126230400sec four years
    sumsec+=2*31536000+31622400;
    if(mon==1)
    {
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==2)
    {
      sumsec+=31*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==3)
    {
      sumsec+=59*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==4)
    {
      sumsec+=90*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==5)
    {
      sumsec+=120*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==6)
    {
      sumsec+=151*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==7)
    {
      sumsec+=181*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==8)
    {
      sumsec+=212*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==9)
    {
      sumsec+=243*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==10)
    {
      sumsec+=273*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==11)
    {
      sumsec+=304*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    if(mon==12)
    {
      sumsec+=334*86400;
      sumsec+=(day-1)*86400;		//86400sec every day
      sumsec+=3600*hour;
      sumsec+=60*min+sec;
      return sumsec;
    }
    else 
      return 0;
  }       
  else
    return 0;
}

//DS1302 update time
void DS1302_UpdateTime(uint32_t nowtime)
{
  uint8_t timedata[8];
  uint8_t sec,min,hour,mday,mon,year;
  
  year=(nowtime/126230400)*4-30;	//1970-2000,start with 2000
  nowtime=nowtime%126230400;
	
  if(nowtime<31536000)		//1970+4x
  {
    if(nowtime<31*86400)
    {
      mon=1;
      mday=nowtime/86400;		//86400sec a day
      nowtime=nowtime%86400;	//time in a day
      hour=nowtime/3600;	//3600 sec a hour
      nowtime=nowtime%3600;	//time in hour
      min=nowtime/60;		//60sec a min
      sec=nowtime%60;	//sec			
    }
    else
    {
      nowtime-=31*86400;
      if(nowtime<28*86400)
      {
        mon=2;
        mday=nowtime/86400;	//86400sec a day
        nowtime=nowtime%86400;	//time in a day
        hour=nowtime/3600;	//3600 sec a hour
        nowtime=nowtime%3600;	//time in hour
        min=nowtime/60;		//60sec a min
        sec=nowtime%60;	//sec			
      }
      else
      {
        nowtime-=28*86400;
        if(nowtime<31*86400)
        {
          mon=3;
          mday=nowtime/86400;	//86400sec a day
          nowtime=nowtime%86400;	//time in a day
          hour=nowtime/3600;	//3600 sec a hour
          nowtime=nowtime%3600;	//time in hour
          min=nowtime/60;		//60sec a min
          sec=nowtime%60;	//sec			
        }
        else
        {
           nowtime-=31*86400;
           if(nowtime<30*86400)
           {
             mon=4;
             mday=nowtime/86400;	//86400sec a day
             nowtime=nowtime%86400;	//time in a day
             hour=nowtime/3600;	//3600 sec a hour
             nowtime=nowtime%3600;	//time in hour
             min=nowtime/60;		//60sec a min
             sec=nowtime%60;	//sec			
           }
          else
          {
            nowtime-=30*86400;
            if(nowtime<31*86400)
            {
              mon=5;
              mday=nowtime/86400;	//86400sec a day
              nowtime=nowtime%86400;	//time in a day
              hour=nowtime/3600;	//3600 sec a hour
              nowtime=nowtime%3600;	//time in hour
              min=nowtime/60;		//60sec a min
              sec=nowtime%60;	//sec			
            }
            else
            {
              nowtime-=31*86400;
              if(nowtime<30*86400)
              {
                mon=6;
                mday=nowtime/86400;	//86400sec a day
                nowtime=nowtime%86400;	//time in a day
                hour=nowtime/3600;	//3600 sec a hour
                nowtime=nowtime%3600;	//time in hour
                min=nowtime/60;		//60sec a min
                sec=nowtime%60;	//sec			
              }
              else
              {
                nowtime-=30*86400;
                if(nowtime<31*86400)
                {
                  mon=7;
                  mday=nowtime/86400;	//86400sec a day
                  nowtime=nowtime%86400;	//time in a day
                  hour=nowtime/3600;	//3600 sec a hour
                  nowtime=nowtime%3600;	//time in hour
                  min=nowtime/60;		//60sec a min
                  sec=nowtime%60;	//sec			
                }
                else
                  {
                    nowtime-=31*86400;
                    if(nowtime<31*86400)
                    {
                      mon=8;
                      mday=nowtime/86400;	//86400sec a day
                      nowtime=nowtime%86400;	//time in a day
                      hour=nowtime/3600;	//3600 sec a hour
                      nowtime=nowtime%3600;	//time in hour
                      min=nowtime/60;		//60sec a min
                      sec=nowtime%60;	//sec			
                    }
                    else
                    {
                      nowtime-=31*86400;
                      if(nowtime<30*86400)
                      {
                        mon=9;
                        mday=nowtime/86400;	//86400sec a day
                        nowtime=nowtime%86400;	//time in a day
                        hour=nowtime/3600;	//3600 sec a hour
                        nowtime=nowtime%3600;	//time in hour
                        min=nowtime/60;		//60sec a min
                        sec=nowtime%60;	//sec			
                      }
                      else
                      {
                        nowtime-=30*86400;
                        if(nowtime<31*86400)
                        {
                          mon=10;
                          mday=nowtime/86400;	//86400sec a day
                          nowtime=nowtime%86400;	//time in a day
                          hour=nowtime/3600;	//3600 sec a hour
                          nowtime=nowtime%3600;	//time in hour
                          min=nowtime/60;		//60sec a min
                          sec=nowtime%60;	//sec			
                        }
                        else
                        {
                          nowtime-=31*86400;
                          if(nowtime<30*86400)
                          {
                            mon=11;
                            mday=nowtime/86400;	//86400sec a day
                            nowtime=nowtime%86400;	//time in a day
                            hour=nowtime/3600;	//3600 sec a hour
                            nowtime=nowtime%3600;	//time in hour
                            min=nowtime/60;		//60sec a min
                            sec=nowtime%60;	//sec			
                          }
                          else
                            {
                              nowtime-=30*86400;
                              mon=12;
                              mday=nowtime/86400;	//86400sec a day
                              nowtime=nowtime%86400;	//time in a day
                              hour=nowtime/3600;	//3600 sec a hour
                              nowtime=nowtime%3600;	//time in hour
                              min=nowtime/60;		//60sec a min
                              sec=nowtime%60;	//sec
                            }
                         }
                      }
                    }
                  }
                }
              }
            }
          }
        }			
      }				
    }	
  else
  {
    nowtime-=31536000;
    if(nowtime<31536000)		//1970+4x+1
    {
      year+=1;
      if(nowtime<31*86400)
      {
        mon=1;
        mday=nowtime/86400;	//86400sec a day
        nowtime=nowtime%86400;	//time in a day
        hour=nowtime/3600;	//3600 sec a hour
        nowtime=nowtime%3600;	//time in hour
        min=nowtime/60;		//60sec a min
        sec=nowtime%60;	//sec			
      }
    else
      {
        nowtime-=31*86400;
        if(nowtime<28*86400)
        {
          mon=2;
          mday=nowtime/86400;	//86400sec a day
          nowtime=nowtime%86400;	//time in a day
          hour=nowtime/3600;	//3600 sec a hour
          nowtime=nowtime%3600;	//time in hour
          min=nowtime/60;		//60sec a min
          sec=nowtime%60;	//sec			
        }
        else
        {
          nowtime-=28*86400;
          if(nowtime<31*86400)
          {
            mon=3;
            mday=nowtime/86400;	//86400sec a day
            nowtime=nowtime%86400;	//time in a day
            hour=nowtime/3600;	//3600 sec a hour
            nowtime=nowtime%3600;	//time in hour
            min=nowtime/60;		//60sec a min
            sec=nowtime%60;	//sec			
          }
          else
          {
            nowtime-=31*86400;
            if(nowtime<30*86400)
            {
              mon=4;
              mday=nowtime/86400;	//86400sec a day
              nowtime=nowtime%86400;	//time in a day
              hour=nowtime/3600;	//3600 sec a hour
              nowtime=nowtime%3600;	//time in hour
              min=nowtime/60;		//60sec a min
              sec=nowtime%60;	//sec			
             }
             else
             {
                nowtime-=30*86400;
                if(nowtime<31*86400)
                {
                  mon=5;
                  mday=nowtime/86400;	//86400sec a day
                  nowtime=nowtime%86400;	//time in a day
                  hour=nowtime/3600;	//3600 sec a hour
                  nowtime=nowtime%3600;	//time in hour
                  min=nowtime/60;		//60sec a min
                  sec=nowtime%60;	//sec			
                }
                else
                {
                  nowtime-=31*86400;
                  if(nowtime<30*86400)
                  {
                    mon=6;
                    mday=nowtime/86400;	//86400sec a day
                    nowtime=nowtime%86400;	//time in a day
                    hour=nowtime/3600;	//3600 sec a hour
                    nowtime=nowtime%3600;	//time in hour
                    min=nowtime/60;		//60sec a min
                    sec=nowtime%60;	//sec			
                  }
                  else
                  {
                    nowtime-=30*86400;
                    if(nowtime<31*86400)
                    {
                      mon=7;
                      mday=nowtime/86400;	//86400sec a day
                      nowtime=nowtime%86400;	//time in a day
                      hour=nowtime/3600;	//3600 sec a hour
                      nowtime=nowtime%3600;	//time in hour
                      min=nowtime/60;		//60sec a min
                      sec=nowtime%60;	//sec			
                    }
                    else
                    {
                      nowtime-=31*86400;
                      if(nowtime<31*86400)
                      {
                        mon=8;
                        mday=nowtime/86400;	//86400sec a day
                        nowtime=nowtime%86400;	//time in a day
                        hour=nowtime/3600;	//3600 sec a hour
                        nowtime=nowtime%3600;	//time in hour
                        min=nowtime/60;		//60sec a min
                        sec=nowtime%60;	//sec			
                      }
                      else
                      {
                        nowtime-=31*86400;
                        if(nowtime<30*86400)
                        {
                          mon=9;
                          mday=nowtime/86400;	//86400sec a day
                          nowtime=nowtime%86400;	//time in a day
                          hour=nowtime/3600;	//3600 sec a hour
                          nowtime=nowtime%3600;	//time in hour
                          min=nowtime/60;		//60sec a min
                          sec=nowtime%60;	//sec			
                        }
                        else
                        {
                          nowtime-=30*86400;
                          if(nowtime<31*86400)
                          {
                            mon=10;
                            mday=nowtime/86400;	//86400sec a day
                            nowtime=nowtime%86400;	//time in a day
                            hour=nowtime/3600;	//3600 sec a hour
                            nowtime=nowtime%3600;	//time in hour
                            min=nowtime/60;		//60sec a min
                            sec=nowtime%60;	//sec			
                          }
                          else
                          {
                            nowtime-=31*86400;
                            if(nowtime<30*86400)
                            {
                              mon=11;
                              mday=nowtime/86400;	//86400sec a day
                              nowtime=nowtime%86400;	//time in a day
                              hour=nowtime/3600;	//3600 sec a hour
                              nowtime=nowtime%3600;	//time in hour
                              min=nowtime/60;		//60sec a min
                              sec=nowtime%60;	//sec			
                            }
                            else
                            {
                              nowtime-=30*86400;
                              mon=12;
                              mday=nowtime/86400;	//86400sec a day
                              nowtime=nowtime%86400;	//time in a day
                              hour=nowtime/3600;	//3600 sec a hour
                              nowtime=nowtime%3600;	//time in hour
                              min=nowtime/60;		//60sec a min
                              sec=nowtime%60;	//sec
                            }
                          }
                        }
                      }
                    }
                  }
                }
              }
            }
          }  
        }
      }	
    else
    {
      nowtime-=31536000;
      if(nowtime<31622400)			//1970+4x+2
      {
        year+=2;
        if(nowtime<31*86400)
        {
          mon=1;
          mday=nowtime/86400;	//86400sec a day
          nowtime=nowtime%86400;	//time in a day
          hour=nowtime/3600;	//3600 sec a hour
          nowtime=nowtime%3600;	//time in hour
          min=nowtime/60;		//60sec a min
          sec=nowtime%60;	//sec			
        }
        else
        {
          nowtime-=31*86400;
          if(nowtime<29*86400)
          {
            mon=2;
            mday=nowtime/86400;	//86400sec a day
            nowtime=nowtime%86400;	//time in a day
            hour=nowtime/3600;	//3600 sec a hour
            nowtime=nowtime%3600;	//time in hour
            min=nowtime/60;		//60sec a min
            sec=nowtime%60;	//sec			
          }
          else
          {
            nowtime-=29*86400;
            if(nowtime<31*86400)
            {
              mon=3;
              mday=nowtime/86400;	//86400sec a day
              nowtime=nowtime%86400;	//time in a day
              hour=nowtime/3600;	//3600 sec a hour
              nowtime=nowtime%3600;	//time in hour
              min=nowtime/60;		//60sec a min
              sec=nowtime%60;	//sec			
            }
            else
            {
               nowtime-=31*86400;
               if(nowtime<30*86400)
               {
                 mon=4;
                 mday=nowtime/86400;	//86400sec a day
                 nowtime=nowtime%86400;	//time in a day
                 hour=nowtime/3600;	//3600 sec a hour
                 nowtime=nowtime%3600;	//time in hour
                 min=nowtime/60;		//60sec a min
                 sec=nowtime%60;	//sec			
               }
               else
               {
                  nowtime-=30*86400;
                  if(nowtime<31*86400)
                  {
                    mon=5;
                    mday=nowtime/86400;	//86400sec a day
                    nowtime=nowtime%86400;	//time in a day
                    hour=nowtime/3600;	//3600 sec a hour
                    nowtime=nowtime%3600;	//time in hour
                    min=nowtime/60;		//60sec a min
                    sec=nowtime%60;	//sec			
                  }
                  else
                  {
                    nowtime-=31*86400;
                    if(nowtime<30*86400)
                    {
                      mon=6;
                      mday=nowtime/86400;	//86400sec a day
                      nowtime=nowtime%86400;	//time in a day
                      hour=nowtime/3600;	//3600 sec a hour
                      nowtime=nowtime%3600;	//time in hour
                      min=nowtime/60;		//60sec a min
                      sec=nowtime%60;	//sec			
                    }
                    else
                    {
                      nowtime-=30*86400;
                      if(nowtime<31*86400)
                      {
                        mon=7;
                        mday=nowtime/86400;	//86400sec a day
                        nowtime=nowtime%86400;	//time in a day
                        hour=nowtime/3600;	//3600 sec a hour
                        nowtime=nowtime%3600;	//time in hour
                        min=nowtime/60;		//60sec a min
                        sec=nowtime%60;	//sec			
                      }
                      else
                      {
                      nowtime-=31*86400;
                      if(nowtime<31*86400)
                      {
                        mon=8;
                        mday=nowtime/86400;	//86400sec a day
                        nowtime=nowtime%86400;	//time in a day
                        hour=nowtime/3600;	//3600 sec a hour
                        nowtime=nowtime%3600;	//time in hour
                        min=nowtime/60;		//60sec a min
                        sec=nowtime%60;	//sec			
                      }
                      else
                      {
                        nowtime-=31*86400;
                        if(nowtime<30*86400)
                        {
                          mon=9;
                          mday=nowtime/86400;	//86400sec a day
                          nowtime=nowtime%86400;	//time in a day
                          hour=nowtime/3600;	//3600 sec a hour
                          nowtime=nowtime%3600;	//time in hour
                          min=nowtime/60;		//60sec a min
                          sec=nowtime%60;	//sec			
                        }
                        else
                        {
                          nowtime-=30*86400;
                          if(nowtime<31*86400)
                          {
                            mon=10;
                            mday=nowtime/86400;	//86400sec a day
                            nowtime=nowtime%86400;	//time in a day
                            hour=nowtime/3600;	//3600 sec a hour
                            nowtime=nowtime%3600;	//time in hour
                            min=nowtime/60;		//60sec a min
                            sec=nowtime%60;	//sec			
                          }
                          else
                          {
                            nowtime-=31*86400;
                            if(nowtime<30*86400)
                            {
                              mon=11;
                              mday=nowtime/86400;	//86400sec a day
                              nowtime=nowtime%86400;	//time in a day
                              hour=nowtime/3600;	//3600 sec a hour
                              nowtime=nowtime%3600;	//time in hour
                              min=nowtime/60;		//60sec a min
                              sec=nowtime%60;	//sec			
                            }
                            else
                            {
                              nowtime-=30*86400;
                              mon=12;
                              mday=nowtime/86400;	//86400sec a day
                              nowtime=nowtime%86400;	//time in a day
                              hour=nowtime/3600;	//3600 sec a hour
                              nowtime=nowtime%3600;	//time in hour
                              min=nowtime/60;		//60sec a min
                              sec=nowtime%60;	//sec
                            }
                          }
                        }
                      }
                    }
                  }
                }
              }
            }
          }
        }          
      }

      else
      {
        nowtime-=31622400;
        year+=3;
        if(nowtime<31*86400)		//1970+4x+3
        {
          mon=1;
          mday=nowtime/86400;	//86400sec a day
          nowtime=nowtime%86400;	//time in a day
          hour=nowtime/3600;	//3600 sec a hour
          nowtime=nowtime%3600;	//time in hour
          min=nowtime/60;		//60sec a min
          sec=nowtime%60;	//sec			
        }
        else
        {
          nowtime-=31*86400;
          if(nowtime<28*86400)
          {
            mon=2;
            mday=nowtime/86400;	//86400sec a day
            nowtime=nowtime%86400;	//time in a day
            hour=nowtime/3600;	//3600 sec a hour
            nowtime=nowtime%3600;	//time in hour
            min=nowtime/60;		//60sec a min
            sec=nowtime%60;	//sec			
          }
          else
          {
            nowtime-=28*86400;
            if(nowtime<31*86400)
            {
              mon=3;
              mday=nowtime/86400;	//86400sec a day
              nowtime=nowtime%86400;	//time in a day
              hour=nowtime/3600;	//3600 sec a hour
              nowtime=nowtime%3600;	//time in hour
              min=nowtime/60;		//60sec a min
              sec=nowtime%60;	//sec			
            }
            else
            {
              nowtime-=31*86400;
              if(nowtime<30*86400)
              {
                mon=4;
                mday=nowtime/86400;	//86400sec a day
                nowtime=nowtime%86400;	//time in a day
                hour=nowtime/3600;	//3600 sec a hour
                nowtime=nowtime%3600;	//time in hour
                min=nowtime/60;		//60sec a min
                sec=nowtime%60;	//sec			
              }
              else
              {
                nowtime-=30*86400;
                if(nowtime<31*86400)
                {
                  mon=5;
                  mday=nowtime/86400;	//86400sec a day
                  nowtime=nowtime%86400;	//time in a day
                  hour=nowtime/3600;	//3600 sec a hour
                  nowtime=nowtime%3600;	//time in hour
                  min=nowtime/60;		//60sec a min
                  sec=nowtime%60;	//sec			
                }
                else
                {
                  nowtime-=31*86400;
                  if(nowtime<30*86400)
                  {
                    mon=6;
                    mday=nowtime/86400;	//86400sec a day
                    nowtime=nowtime%86400;	//time in a day
                    hour=nowtime/3600;	//3600 sec a hour
                    nowtime=nowtime%3600;	//time in hour
                    min=nowtime/60;		//60sec a min
                    sec=nowtime%60;	//sec			
                  }
                  else
                  {
                    nowtime-=30*86400;
                    if(nowtime<31*86400)
                    {
                      mon=7;
                      mday=nowtime/86400;	//86400sec a day
                      nowtime=nowtime%86400;	//time in a day
                      hour=nowtime/3600;	//3600 sec a hour
                      nowtime=nowtime%3600;	//time in hour
                      min=nowtime/60;		//60sec a min
                      sec=nowtime%60;	//sec			
                    }
                    else
                    {
                      nowtime-=31*86400;
                      if(nowtime<31*86400)
                      {
                        mon=8;
                        mday=nowtime/86400;	//86400sec a day
                        nowtime=nowtime%86400;	//time in a day
                        hour=nowtime/3600;	//3600 sec a hour
                        nowtime=nowtime%3600;	//time in hour
                        min=nowtime/60;		//60sec a min
                        sec=nowtime%60;	//sec			
                      }
                      else
                      {
                        nowtime-=31*86400;
                        if(nowtime<30*86400)
                        {
                          mon=9;
                          mday=nowtime/86400;	//86400sec a day
                          nowtime=nowtime%86400;	//time in a day
                          hour=nowtime/3600;	//3600 sec a hour
                          nowtime=nowtime%3600;	//time in hour
                          min=nowtime/60;		//60sec a min
                          sec=nowtime%60;	//sec			
                        }
                        else
                        {
                          nowtime-=30*86400;
                          if(nowtime<31*86400)
                          {
                            mon=10;
                            mday=nowtime/86400;	//86400sec a day
                            nowtime=nowtime%86400;	//time in a day
                            hour=nowtime/3600;	//3600 sec a hour
                            nowtime=nowtime%3600;	//time in hour
                            min=nowtime/60;		//60sec a min
                            sec=nowtime%60;	//sec			
                          }
                          else
                          {
                            nowtime-=31*86400;
                            if(nowtime<30*86400)
                            {
                              mon=11;
                              mday=nowtime/86400;	//86400sec a day
                              nowtime=nowtime%86400;	//time in a day
                              hour=nowtime/3600;	//3600 sec a hour
                              nowtime=nowtime%3600;	//time in hour
                              min=nowtime/60;		//60sec a min
                              sec=nowtime%60;	//sec			
                            }
                            else
                            {
                              nowtime-=30*86400;
                              mon=12;
                              mday=nowtime/86400;	//86400sec a day
                              nowtime=nowtime%86400;	//time in a day
                              hour=nowtime/3600;	//3600 sec a hour
                              nowtime=nowtime%3600;	//time in hour
                              min=nowtime/60;		//60sec a min
                              sec=nowtime%60;	//sec
                            }
                          }
                        }
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }
 
  mday+=1;	//have no 0 day every mon
	 
  timedata[0]=0x00;     //first register address,second register
  timedata[1]=(sec/10)*16+sec%10;     //oscillator enable,0 seconds
  timedata[2]=(min/10)*16+min%10;     //0minutes
  timedata[3]=(hour/10)*16+hour%10;     //24h,1hour
  timedata[4]=0x00;     //day 0
  timedata[5]=(mday/10)*16+mday%10;     //date 1
  timedata[6]=(mon/10)*16+mon%10;     //month 1
  timedata[7]=(year/10)*16+year%10;     //year 15
  MAP_GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_1,0x02);  //power on
  I2C_IF_Write(Dsaddr,timedata,8,0);    //write the read register addr
  MAP_GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_1,0x00);  //power off
}


