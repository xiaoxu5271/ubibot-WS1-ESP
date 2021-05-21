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
#include "stdint.h"

#define ADXL345_IIC
#define ADXL345_IIC_ADDR        0X53    

#define DEVICE_ID		0X00 	
#define THRESH_TAP		0X1D 
#define OFSX			0X1E
#define OFSY			0X1F
#define OFSZ			0X20
#define DUR			0X21
#define Latent			0X22
#define Window  		0X23 
#define THRESH_ACT		0X24
#define THRESH_INACT		0X25 
#define TIME_INACT		0X26
#define ACT_INACT_CTL		0X27	 
#define THRESH_FF		0X28	
#define TIME_FF			0X29 
#define TAP_AXES		0X2A  
#define ACT_TAP_STATUS 	 	0X2B 
#define BW_RATE			0X2C 
#define POWER_CTL		0X2D 
#define INT_ENABLE		0X2E
#define INT_MAP			0X2F
#define INT_SOURCE  		0X30
#define DATA_FORMAT	    	0X31
#define DATAX0			0X32
#define DATAX1			0X33
#define DATAY0			0X34
#define DATAY1			0X35
#define DATAZ0			0X36
#define DATAZ1			0X37
#define FIFO_CTL		0X38
#define FIFO_STATUS		0X39


/*******************************************************************************
 FUNCTION PROTOTYPES
*******************************************************************************/
extern void adx345_writeReg(uint8_t addr,uint8_t value);  //adx345 write register

extern uint8_t adx345_readReg(uint8_t addr);  //adx345 read register

extern short ADXL345_Init(void);  //adxl245 sensor init

extern void ADXL345_RD_xyz(short *x_val,short *y_val,short *z_val);  //read three Axis data

extern void ADXL345_Power_Down(void);  //Set ADXL345 In Hibernate

extern short Adxl345_Check_Id(void);  //adx345 sensor check id


/*******************************************************************************
                                      END         
*******************************************************************************/




