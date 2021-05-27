/*******************************************************************************
  * @file       ADXL345 Acceleration Sensor DRIVER APPLICATION      
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

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "iic.h"
#include "MsgType.h"
#include "user_spi.h"

#include "adxl345.h"

struct Init_Data
{
  uint8_t reg_addr;
  uint8_t reg_val;
};

extern volatile uint8_t thres_acc_min;

/*******************************************************************************
  adx345 write register
*******************************************************************************/
void adx345_writeReg(uint8_t addr, uint8_t value)
{
#ifdef ADXL345_SPI
  SET_SPI1_CS_OFF();         //enable the adx345 spi cs
  MAP_SPIEnable(GSPI_BASE);  //enable the spi channel
  SPI_SendReciveByte(addr);  //send register address
  SPI_SendReciveByte(value); //send write value
  MAP_SPIDisable(GSPI_BASE); //disable the channel
  SET_SPI1_CS_ON();          //disable the adx345 spi cs
#endif
#ifdef ADXL345_IIC
  MulTry_IIC_WR_Reg(ADXL345_IIC_ADDR, addr, value);
#endif
}

/*******************************************************************************
  adx345 read register
*******************************************************************************/
uint8_t adx345_readReg(uint8_t addr)
{
  uint8_t RegValue = 0;

#ifdef ADXL345_SPI
  SET_SPI1_CS_OFF();                   //enable the adx345 spi cs
  MAP_SPIEnable(GSPI_BASE);            //enable the spi channel
  SPI_SendReciveByte(addr | 0x80);     //send the register address and read single command
  RegValue = SPI_SendReciveByte(0x00); //clk signal get the value
  MAP_SPIDisable(GSPI_BASE);           //disable the channel
  SET_SPI1_CS_ON();                    //disable the adx345 spi cs
#endif
#ifdef ADXL345_IIC
  MulTry_IIC_RD_Reg(ADXL345_IIC_ADDR, addr, &RegValue);
#endif

  return RegValue;
}

/*******************************************************************************
  adxl245 sensor init
*******************************************************************************/
short ADXL345_Init(void)
{
  uint8_t i;
  uint8_t n_i;

  if (adx345_readReg(DEVICE_ID) == 0xE5) //read the sensor device id
  {
    struct Init_Data Acce_Init_Data[] = {{POWER_CTL, 0x20}, {INT_ENABLE, 0x00}, {THRESH_TAP, 0x16}, {OFSX, 0x00}, {OFSY, 0x00}, {OFSZ, 0x00}, {DUR, 0x25}, {Latent, 0x78}, {Window, 0xff}, {THRESH_ACT, thres_acc_min}, {THRESH_INACT, thres_acc_min}, {TIME_INACT, 0x05}, {ACT_INACT_CTL, 0xff}, {THRESH_FF, 0x08}, {TIME_FF, 0x28}, {TAP_AXES, 0x07}, {BW_RATE, 0x0a}, {INT_MAP, 0x87}, {DATA_FORMAT, 0x2b}, {FIFO_CTL, 0xa8}};

    n_i = sizeof(Acce_Init_Data) / sizeof(struct Init_Data);

    for (i = 0; i < n_i; i++)
    {
      adx345_writeReg(Acce_Init_Data[i].reg_addr, Acce_Init_Data[i].reg_val); //adx345 write register
    }
    return SUCCESS;
  }
  return FAILURE;
}

/*******************************************************************************
  read three Axis data
*******************************************************************************/
void ADXL345_RD_xyz(short *x_val, short *y_val, short *z_val)
{
  uint8_t read_val[6];

#ifdef ADXL345_SPI
  SET_SPI1_CS_OFF();                 //enable the adx345 spi cs
  MAP_SPIEnable(GSPI_BASE);          //enable the spi channel
  SPI_SendReciveByte(DATAX0 | 0xc0); //send the address and read multi data command
  uint8_t i;
  for (i = 0; i < 6; i++)
  {
    read_val[i] = SPI_SendReciveByte(0x00); //read data
  }
  MAP_SPIDisable(GSPI_BASE); //disable the channel
  SET_SPI1_CS_ON();          //disable the adx345 spi cs
#endif
#ifdef ADXL345_IIC
  MulTry_I2C_RD_mulReg(ADXL345_IIC_ADDR, DATAX0, read_val, 6);
#endif

  *x_val = (short)(((uint16_t)read_val[1] << 8) + read_val[0]); //x axis data
  *y_val = (short)(((uint16_t)read_val[3] << 8) + read_val[2]); //y axis data
  *z_val = (short)(((uint16_t)read_val[5] << 8) + read_val[4]); //z axis data
}

/*******************************************************************************
  adxl245 sensor reset
*******************************************************************************/
//static void ADXL345_Reset(void)
//{
//  uint8_t i;
//  uint8_t n_i;
//
//  if(adx345_readReg(DEVICE_ID)==0xE5)   //read the sensor device id
//  {
//    struct Init_Data Acce_Init_Data[]={{POWER_CTL,0x00},{THRESH_TAP,0x00},{OFSX,0x00},{OFSY,0x00},{OFSZ,0x00},
//                                       {DUR,0x00},{Latent,0x00},{Window,0x00},{THRESH_ACT,0x00},{THRESH_INACT,0x00},
//                                       {TIME_INACT,0x00},{ACT_INACT_CTL,0x00},{THRESH_FF,0x00},{TIME_FF,0x00},{TAP_AXES,0x00},{BW_RATE,0x0a},
//                                       {INT_ENABLE,0x00},{INT_MAP,0x00},{DATA_FORMAT,0x00},{FIFO_CTL,0x00}};
//
//    n_i=sizeof(Acce_Init_Data)/sizeof(struct Init_Data);
//
//    for(i=0;i<n_i;i++)
//    {
//      adx345_writeReg(Acce_Init_Data[i].reg_addr,Acce_Init_Data[i].reg_val);  //adx345 write register
//    }
//  }
//}

/*******************************************************************************
  Set ADXL345 In Hibernate
*******************************************************************************/
void ADXL345_Power_Down(void)
{
  //  if(adx345_readReg(DEVICE_ID)==0xE5)  //Check Acceleration Sensor Exist
  {
    //    ADXL345_Reset();

    adx345_writeReg(POWER_CTL, 0x00); //standby mode
  }
}

/*******************************************************************************
  adx345 sensor check id
*******************************************************************************/
short Adxl345_Check_Id(void)
{
  if (adx345_readReg(DEVICE_ID) == 0xE5) //read the sensor device id
  {
    return SUCCESS;
  }
  return FAILURE;
}

/*******************************************************************************
                                      END         
*******************************************************************************/
