/******************************************************************************/
/******************************************************************************/
/**************** mag3110 digital magnetometer sensor application *************/
/******************************************************************************/
/******************************************************************************/

//header include
#include "stdint.h"
#include "i2c_if.h"
#include "uart_if.h"
#include "math.h"
#include "adx345.h"

//mag3110 sensor register address
#define MAG3110_ADDR	        0X0E
#define DR_STATUS 		0X00
#define OUT_X_MSB 		0X01
#define OUT_X_LSB 		0X02
#define OUT_Y_MSB 		0X03
#define OUT_Y_LSB 		0X04
#define OUT_Z_MSB 		0X05
#define OUT_Z_LSB 		0X06
#define MAG_WHO_AM_I 		0X07
#define SYSMOD 			0X08
#define OFF_X_MSB 		0X09
#define OFF_X_LSB 		0X0A
#define OFF_Y_MSB 		0X0B
#define OFF_Y_LSB 		0X0C
#define OFF_Z_MSB 		0X0D
#define OFF_Z_LSB 		0X0E
#define DIE_TEMP 		0X0F
#define MAG_CTRL_REG1 		0X10
#define MAG_CTRL_REG2 		0X11

//write register comand
static void IIC_WriteRegAddr(uint8_t slaaddr,uint8_t regaddr)
{
  uint8_t reg[1];
  reg[0]=regaddr;
  I2C_IF_Write(slaaddr,reg,1,0);    //write zhe read register addr 
}

//writer mag3110 sensor register
//regaddr:register address
//val:write register value
void MAG3110_WR_Reg(uint8_t regaddr,uint8_t val)
{
  uint8_t reg[2];
  reg[0]=regaddr;
  reg[1]=val;
  I2C_IF_Write(MAG3110_ADDR,reg,2,0);    //write zhe read register addr and the register value
}

//read mag3110 sensor register
//regaddr:register address
//return:read register value
uint8_t MAG3110_RD_Reg(uint8_t regaddr)
{
  uint8_t reg[1];
  uint8_t regdata[1];
  reg[0]=regaddr;
  I2C_IF_Write(MAG3110_ADDR,reg,1,0);    //write zhe read register addr 
  I2C_IF_Read(MAG3110_ADDR,regdata,1);
  return regdata[0];
}

//Init MAG3110 device/successed return 0,failued return 1
uint8_t MAG3110_Init(void)
{	
  if(MAG3110_RD_Reg(MAG_WHO_AM_I)==0XC4)	//read device ID
  {  
    MAG3110_WR_Reg(MAG_CTRL_REG1,0x61);	//10Hz,137.5uA,0.4uT;standby mode
    MAG3110_WR_Reg(MAG_CTRL_REG2,0XA0);	//resets enabled,not corrected by the user offset register values
    return 0;
  }
  Report("mag3110 magnetometer sensor init failued\n\r");
  return 1;	   					
}

//read 3 axis data
//returan:x,y,z data value
void Read_AcceMagValue(float *acce,float *mag)
{
  uint16_t i,j;
  short x,y,z;
  short x1,y1,z1;
  short x2,y2,z2;
  float sumacce,summag;
  uint8_t XYZbuffer[6];
  
  //MAG3110_WR_Reg(MAG_CTRL_REG1,0x19);	//10Hz,900uA,0.25uT;perform continuous measurements
  for(j=0;j<7;j++)
  {
    while((MAG3110_RD_Reg(DR_STATUS)&0X0f)!=0X0f)   //waite for data is ready
    {
      i++;
      ADXL345_RD_xyz(&x1,&y1,&z1);
      ADXL345_RD_xyz(&x2,&y2,&z2);
      sumacce+=(float)sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)+(z2-z1)*(z2-z1));
    }
    IIC_WriteRegAddr(MAG3110_ADDR,OUT_X_MSB);     //write the read address
    I2C_IF_Read(MAG3110_ADDR,XYZbuffer,6);        //read the xyz register data
    //MAG3110_WR_Reg(MAG_CTRL_REG1,0x18);	//10Hz,900uA,0.25uT;standby mode
    
    x=(short)(((uint16_t)XYZbuffer[0]<<8)+XYZbuffer[1]);         //x value	    
    y=(short)(((uint16_t)XYZbuffer[2]<<8)+XYZbuffer[3]); 	//y value    
    z=(short)(((uint16_t)XYZbuffer[4]<<8)+XYZbuffer[5]); 	//z value
    summag+=(float)sqrt(x*x+y*y+z*z);
  }
  *acce=(float)(sumacce/i);
  Report("i:%d\n\r",i);
  *mag=(float)(summag/j);
}





