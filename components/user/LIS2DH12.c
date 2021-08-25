/*******************************************************************************
  * @file       LIS2DH12 Sensor Application      
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
#include "stdio.h"
#include "iic.h"
#include "math.h"
#include "MsgType.h"
#include "user_spi.h"
#include "LIS2DH12.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#define TAG "LIS2DH12"

#define STATUS_REG_AUX 0x07 //
#define OUT_TEMP_L 0x0C     //
#define OUT_TEMP_H 0x0D     //
#define WHO_AM_I 0x0F
#define CTRL_REG0 0x1E    //Default
#define TEMP_CFG_REG 0x1F //
#define CTRL_REG1 0x20    //0x57
#define CTRL_REG2 0x21    //
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24
#define CTRL_REG6 0x25
#define REFERENCE 0x26
#define STATUS_REG 0x27
#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D
#define FIFO_CTRL_REG 0x2E
#define FIFO_SRC_REG 0x2F
#define INT1_CFG 0x30
#define INT1_SRC 0x31
#define INT1_THS 0x32
#define INT1_DURATION 0x33
#define INT2_CFG 0x34
#define INT2_SRC 0x35
#define INT2_THS 0x36
#define INT2_DURATION 0x37
#define CLICK_CFG 0x38
#define CLICK_SRC 0x39
#define CLICK_THS 0x3A
#define TIME_LIMIT 0x3B
#define TIME_LATENCY 0x3C
#define TIME_WINDOW 0x3D
#define ACT_THS 0x3E
#define ACT_DUR 0x3F

//extern uint8_t Read_sec_val(void);

/**
  * @brief  Read generic device register
  *
  * @param  reg   register to read
  * @param  data  pointer to buffer that store the data read(ptr)
  * @param  len   number of consecutive register to read
  *
  */
void lis2dh12_read_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
  MulTry_I2C_RD_mulReg(LIS2DH12_ADDR, reg | 0x80, data, len);
}

/**
  * @brief  Write generic device register
  *
  * @param  reg   register to write
  * @param  data  pointer to data to write in register reg(ptr)
  * @param  len   number of consecutive register to write
  *
  */
void lis2dh12_write_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
  MulTry_I2C_WR_mulReg(LIS2DH12_ADDR, reg | 0x80, data, len);
}

/**
  * @brief  DeviceWhoamI .[get]
  * @param  buff     buffer that stores data read
  *
  */
void lis2dh12_device_id_get(uint8_t *buff)
{
  lis2dh12_read_reg(WHO_AM_I, buff, 1);
}

void lis2dh12_init(void)
{
  uint8_t reg_val;

  vTaskDelay(5 / portTICK_RATE_MS);

  reg_val = 0x00;
  lis2dh12_write_reg(INT1_CFG, &reg_val, 1); //Disable all interrupt sources

  lis2dh12_read_reg(INT1_SRC, &reg_val, 1);

  reg_val = 0x00;
  lis2dh12_write_reg(CTRL_REG1, &reg_val, 1); //Put device into power-down

  vTaskDelay(5 / portTICK_RATE_MS);
}

void lis2dh12_detect_init(void)
{
  uint8_t reg_val;

  reg_val = 0x57;
  lis2dh12_write_reg(CTRL_REG1, &reg_val, 1); // 100HZ,X_EN,Y_EN,Z_EN

  reg_val = 0x01;                             //
  lis2dh12_write_reg(CTRL_REG2, &reg_val, 1); //High-pass filter enabled for AOI function on Interrupt 1

  reg_val = 0x40;                             //
  lis2dh12_write_reg(CTRL_REG3, &reg_val, 1); //IA1 interrupt on INT1 pin

  reg_val = 0x28;                             //High-resolution
  lis2dh12_write_reg(CTRL_REG4, &reg_val, 1); //¡À8 g

  reg_val = 0x08;                             //interrupt request latched
  lis2dh12_write_reg(CTRL_REG5, &reg_val, 1); //

  reg_val = 0x00;
  lis2dh12_write_reg(CTRL_REG6, &reg_val, 1); //0: active-high

  reg_val = 0x04;
  lis2dh12_write_reg(INT1_THS, &reg_val, 1); //4*62 mg

  reg_val = 0x00;
  lis2dh12_write_reg(INT1_DURATION, &reg_val, 1); //

  //  reg_val = 0x84;                            //LIR_Click bit is set
  //  lis2dh12_write_reg(CLICK_THS,&reg_val,1);  //4*62 mg
  //  reg_val = 0x10;                            //
  //  lis2dh12_write_reg(TIME_LIMIT,&reg_val,1);  //1 LSB = 1/ODR
  //  reg_val = 0x40;                            //
  //  lis2dh12_write_reg(TIME_LATENCY,&reg_val,1);  //1 LSB = 1/ODR
  //  reg_val = 0x40;                            //
  //  lis2dh12_write_reg(TIME_WINDOW,&reg_val,1);  //

  //  reg_val = 0x00;                            //FIFO ctrl.
  //  lis2dh12_write_reg(FIFO_CTRL_REG,&reg_val,1);  //

  //  reg_val = 0x02;                          //Sleep-to-wake
  //  lis2dh12_write_reg(ACT_THS,&reg_val,1);  //2*62 mg
  //  reg_val = 0x12;                          //Sleep-to-wake
  //  lis2dh12_write_reg(ACT_DUR,&reg_val,1);  //1 LSb = (8*1[LSb]+1)/ODR

  lis2dh12_read_reg(REFERENCE, &reg_val, 1);

  reg_val = 0x2a;
  lis2dh12_write_reg(INT1_CFG, &reg_val, 1); //Enable interrupt generation on XYZ high event or on direction recognition.

  //  reg_val = 0x3f;
  //  lis2dh12_write_reg(CLICK_CFG,&reg_val,1);  //Enable interrupt single/double-click
}

uint8_t lis2dh12_clear_int(void)
{
  uint8_t reg_val;

  lis2dh12_read_reg(INT1_SRC, &reg_val, 1);
  //  printf("INT1_SRC=%x.\n\r",reg_val);

  //  lis2dh12_read_reg(CLICK_SRC,&reg_val,1);
  //  printf("CLICK_SRC=%x.\n\r",reg_val);
  return reg_val;
}

float lis2dh12_data(void)
{
  uint8_t sec_val[2] = {0};
  uint8_t reg_val, data_num = 0;
  float x_axle, y_axle, z_axle;
  float acc_val1 = 0, acc_val2 = 0, sum_val = 0;
  sec_val[0] = (uint8_t)(esp_timer_get_time() / 1000000);
  sec_val[1] = sec_val[0];
  while (sec_val[0] == sec_val[1])
  {
    lis2dh12_read_reg(STATUS_REG, &reg_val, 1);
    //    printf("STATUS_REG=%x.\n\r",reg_val);
    if (reg_val & 0x08)
    {
      lis2dh12_RD_xyz(&x_axle, &y_axle, &z_axle);
      acc_val2 = sqrt(x_axle * x_axle + y_axle * y_axle + z_axle * z_axle) - 1000;
      if (acc_val1 != 0)
      {
        sum_val += fabs(acc_val2 - acc_val1);
        //        sum_val += fabs(acc_val2*acc_val2 - acc_val1*acc_val1);
        //        sum_val += acc_val2 - acc_val1;

        data_num += 1;
      }
      acc_val1 = acc_val2;
      //      printf("x_axle=%f,y_axle=%f,z_axle=%f.\n\r",x_axle,y_axle,z_axle);
      //        vTaskDelay(5 / portTICK_RATE_MS);  //wait 5 msec
    }
    vTaskDelay(5 / portTICK_RATE_MS); //wait 5 msec
    sec_val[1] = (uint8_t)(esp_timer_get_time() / 1000000);
    if (data_num > 100)
      break;
  }
  //  printf("num=%d,avg_val=%f.\n\r",data_num,sum_val/data_num);
  return sum_val / data_num;
}

float lis2dh12_from_fs2_hr_to_mg(int16_t lsb)
{
  return ((float)lsb / 16.0f) * 1.0f;
}

float lis2dh12_from_fs4_hr_to_mg(int16_t lsb)
{
  return ((float)lsb / 16.0f) * 2.0f;
}

float lis2dh12_from_fs8_hr_to_mg(int16_t lsb)
{
  return ((float)lsb / 16.0f) * 4.0f;
}

float lis2dh12_from_fs16_hr_to_mg(int16_t lsb)
{
  return ((float)lsb / 16.0f) * 12.0f;
}

/*******************************************************************************
//  read three Axis data
*******************************************************************************/
void lis2dh12_RD_xyz(float *x_val, float *y_val, float *z_val)
{
  uint8_t read_val[6];
  int16_t xval, yval, zval;
  lis2dh12_read_reg(OUT_X_L, read_val, 6);

  xval = (short)(((int16_t)read_val[1] * 256) + (int16_t)read_val[0]); //x axis data
  yval = (short)(((int16_t)read_val[3] * 256) + (int16_t)read_val[2]); //y axis data
  zval = (short)(((int16_t)read_val[5] * 256) + (int16_t)read_val[4]); //z axis data

  *x_val = lis2dh12_from_fs8_hr_to_mg(xval);
  *y_val = lis2dh12_from_fs8_hr_to_mg(yval);
  *z_val = lis2dh12_from_fs8_hr_to_mg(zval);
}

void lis2dh12_Power_Down(void)
{
  uint8_t reg_val = 0x00;
  lis2dh12_write_reg(CTRL_REG1, &reg_val, 1); //power-down mode
}
