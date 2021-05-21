/******************************************************************************/
/******************************************************************************/
/**************** HTS221 temperature and humidity sensor application **********/
/******************************************************************************/
/******************************************************************************/

//header include
#include "stdint.h"
#include "i2c_if.h"
#include "uart_if.h"

//define sensor register address
#define HTS221ADDR              0x5F
#define WHO_AM_I		0X0F
#define AV_CONF			0X10
#define CTRL_REG1		0X20
#define CTRL_REG2		0X21
#define CTRL_REG3		0X22
#define STATUS_REG		0X27
#define HUMIDITY_OUT_L	        0X28
#define HUMIDITY_OUT_H	        0X29
#define TEMP_OUT_L		0X2A
#define TEMP_OUT_H		0X2B
#define H0_rH_x2		0x30
#define H1_rH_x2		0X31
#define H0_T0_OUT_L 		0X36
#define H0_T0_OUT_H 		0X37
#define H1_T0_OUT_L 		0X3A
#define H1_T0_OUT_H 		0X3B
#define T0_degC_x8		0x32
#define T1_degC_x8		0x33
#define T0_OUT_L		0X3C
#define T0_OUT_H		0X3D
#define T1_OUT_L		0X3E
#define T1_OUT_H		0X3F
#define T1_T0msb		0x35

//data used for calculate the temperature and humidity value 
//y=kx+c
float ktemp,ctemp,khumi,chumi;

//write register comand
//slaaddr:slave address
//regaddr:register address
static void IIC_WriteRegAddr(uint8_t slaaddr,uint8_t regaddr)
{
  uint8_t reg[1];
  reg[0]=regaddr;
  I2C_IF_Write(slaaddr,reg,1,0);    //write the read register addr 
}

//writer HTS221 sensor register
//regaddr:register address
//val:write register value
static void HTS221TR_WR_Reg(uint8_t regaddr,uint8_t val)
{
  uint8_t reg[2];
  reg[0]=regaddr;
  reg[1]=val;
  I2C_IF_Write(HTS221ADDR,reg,2,0);    //write zhe read register addr and the register value
}

//read HTS221 sensor register
//regaddr:register address
//return:read register value
static uint8_t HTS221TR_RD_Reg(uint8_t regaddr)
{
  uint8_t reg[1];
  uint8_t regdata[1];
  reg[0]=regaddr;
  I2C_IF_Write(HTS221ADDR,reg,1,0);    //write the read register addr 
  I2C_IF_Read(HTS221ADDR,regdata,1);    //read the register
  return regdata[0];
}

//Init HTS221TR device/successed return 0/failured return -1
uint8_t HTS221TR_Init(void)
{
  uint8_t h0_rh,h1_rh;
  uint16_t t0_degc,t1_degc;
  short h0_out,h1_out,t0_out,t1_out;

  if(HTS221TR_RD_Reg(WHO_AM_I)==0XBC)	//read device identification
  {  
    HTS221TR_WR_Reg(AV_CONF,0X1B);	//AVGT:16,AVGH:32,Temperature Noise 0.03C Humidity Noise0.15% IDD 1Hz 2.1uA
    HTS221TR_WR_Reg(CTRL_REG1,0X87);	//POWER on ,block data not updated until MSB and LSB reading, 12.5HZ
    HTS221TR_WR_Reg(CTRL_REG2,0X80);	//reboot memory content ,heater disable,waiting for start of conversion
    HTS221TR_WR_Reg(CTRL_REG3,0X00);	//DRDY_L Data ready disable
    
    t0_degc=((HTS221TR_RD_Reg(T1_T0msb)&0x03)<<8)+HTS221TR_RD_Reg(T0_degC_x8);	//read t0_degc_x8 register
    t1_degc=((HTS221TR_RD_Reg(T1_T0msb)&0x0c)<<6)+HTS221TR_RD_Reg(T1_degC_x8);	//read t1_degc_x8 register
    t0_out=(short)((HTS221TR_RD_Reg(T0_OUT_H)<<8)+HTS221TR_RD_Reg(T0_OUT_L));	//read T0_OUT register data
    t1_out=(short)((HTS221TR_RD_Reg(T1_OUT_H)<<8)+HTS221TR_RD_Reg(T1_OUT_L));	//read T1_OUT register data 
    ktemp=(float)(t1_degc-t0_degc)/(float)(t1_out-t0_out)/8;//y=kx+c,k=(y1-y0)/(x1-x0)
    ctemp=(float)t1_degc/8-ktemp*(float)t1_out;//y=kx+c,c=y1-x1*k

    h0_rh=HTS221TR_RD_Reg(H0_rH_x2);	//read H0_rH_x2 register data
    h1_rh=HTS221TR_RD_Reg(H1_rH_x2);	//read H1_rH_x2 register data
    h0_out=(short)((HTS221TR_RD_Reg(H0_T0_OUT_H)<<8)+HTS221TR_RD_Reg(H0_T0_OUT_L));	//read H0_T0_OUT register data
    h1_out=(short)((HTS221TR_RD_Reg(H1_T0_OUT_H)<<8)+HTS221TR_RD_Reg(H1_T0_OUT_L));	////read H0_T1_OUT register data
    khumi=(float)(h1_rh-h0_rh)/(float)(h1_out-h0_out)/2;//y=kx+c,k=(y1-y0)/(x1-x0)
    chumi=(float)h1_rh/2-khumi*(float)h1_out;//y=kx+c,c=y1-x1*k
     
    return 0;
  }
  Report("hts221 tempeture humidity sensor init failued\n\r");
  return -1;	   					
}

float GetTemperatureValue(void)
{
  uint8_t tempdata[2];
  short temp;
  float value;
  HTS221TR_WR_Reg(CTRL_REG2,0X01);	//start for a new data set
  while((HTS221TR_RD_Reg(STATUS_REG)&0x03)!=0x03)       //Wait for data ready
  {
    
  }
  IIC_WriteRegAddr(HTS221ADDR,(TEMP_OUT_L+0x80));       //multiple read addr 7bit should be 1
  I2C_IF_Read(HTS221ADDR,tempdata,2);   //read temp data
  HTS221TR_WR_Reg(CTRL_REG2,0X00);	//waiting for a new data conversion
  temp=(short)((tempdata[1]<<8)+tempdata[0]);//read temperature data
  value=ktemp*(float)temp+ctemp; //y=kx+c 
  
  return value;
}

float GetHumidityValue(void)
{
  uint8_t humidata[2];
  short humi;
  float value;
  HTS221TR_WR_Reg(CTRL_REG2,0X01);	//start for a new data set
  while((HTS221TR_RD_Reg(STATUS_REG)&0x03)!=0x03)       //Wait for data ready
  {
    
  }
  IIC_WriteRegAddr(HTS221ADDR,(HUMIDITY_OUT_L+0x80));    //multiple read addr 7bit should be 1
  I2C_IF_Read(HTS221ADDR,humidata,2);  
  HTS221TR_WR_Reg(CTRL_REG2,0X00);	//waiting for a new data conversion
  humi=(short)((humidata[1]<<8)+humidata[0]);//read humidity data
  value=(float)humi*khumi+chumi;       ////y=kx+c
  
  return value;
}

