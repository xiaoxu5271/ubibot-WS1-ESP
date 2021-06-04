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
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#include "iic.h"
#include "MsgType.h"
#include "crc_8_check.h"

#include "sht30dis.h"

#define TAG "SHT30"

#define sht30dis_addr 0x44 //7 MSB address 0x44

//SHT30
#define SHT30_WRITE_ADDR 0x44 //地址
#define CMD_FETCH_DATA_H 0x22 //循环采样，参考sht30 datasheet           1S两次
#define CMD_FETCH_DATA_L 0x36

#define CMD_MSB_0_5MSP 0x20 //循环采样，参考sht30 datasheet          0.5MPS
#define CMD_LSB_0_5MSP 0x32

#define CMD_MSB_NO_CLK 0x2C //单次测量，参考sht30 datasheet          no clock
#define CMD_LSB_HIGH 0x06   //Repeatability HIGH

extern SemaphoreHandle_t xMutex8; //Used for I2C lock

/*******************************************************************************
  SHT30DIS Single Shot Measure
  Command:0x2400-Repeatability:high,clock stretching:disable
*******************************************************************************/
void sht30_SingleShotMeasure(float *temp, float *humi)
{

  xSemaphoreTake(xMutex8, -1);
  uint16_t retry;
  uint8_t recive[6];
  uint16_t tempval, humival;

  *temp = ERROR_CODE;

  *humi = ERROR_CODE;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (2 * sht30dis_addr), ACK_CHECK_EN);
  i2c_master_write_byte(cmd, 0x24, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN);

  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  vTaskDelay(50 / portTICK_RATE_MS);

  cmd = i2c_cmd_link_create();
  for (retry = 0; retry < 1000; retry++)
  {
    i2c_master_start(cmd);

    if (i2c_master_write_byte(cmd, (2 * sht30dis_addr + 1), ACK_CHECK_EN) != ESP_OK)
    {
      i2c_master_stop(cmd);
    }
    else //ACK:measurement completed
    {
      for (retry = 0; retry < 6; retry++)
      {
        if (retry == 5)
        {
          // recive[retry] = IIC_Read_Byte(0); //read a byte,NACK
          i2c_master_read_byte(cmd, &recive[retry], NACK_VAL);
        }
        else
        {
          i2c_master_read_byte(cmd, &recive[retry], ACK_VAL);
        }
      }

      if (Data_Crc_Check(&recive[0], 3) == 0)
      {
        tempval = recive[0];

        tempval = tempval << 8;

        tempval += recive[1];

        *temp = 175 * (float)tempval / 65535 - 45;
      }

      if (Data_Crc_Check(&recive[3], 3) == 0)
      {
        humival = recive[3];

        humival = humival << 8;

        humival += recive[4];

        *humi = 100 * humival / 65535;
      }

      break;
    }

    // MAP_UtilsDelay(60000); //delay about 4.5ms
    vTaskDelay(5 / portTICK_RATE_MS);
  }

  // end:

  // IIC_Stop(); //IIC stop
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  vTaskDelay(5 / portTICK_RATE_MS);
  xSemaphoreGive(xMutex8);
}

/*******************************************************************************
  sht30dis sensor reset,command:0x30a2
*******************************************************************************/
short sht30dis_reset(void)
{

  xSemaphoreTake(xMutex8, -1);
  // IIC_Start(); //iic bus start
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (2 * sht30dis_addr), ACK_CHECK_EN);
  i2c_master_write_byte(cmd, 0x30, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, 0xa2, ACK_CHECK_EN);

  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  vTaskDelay(5 / portTICK_RATE_MS);
  xSemaphoreGive(xMutex8);

  if (ret == ESP_OK)
  {
    return SUCCESS;
  }
  else
  {
    return FAILURE;
  }
}

/*******************************************************************************
                                      END         
*******************************************************************************/

unsigned char SHT3X_CheckCrc(unsigned char *pdata, unsigned char nbrOfBytes, unsigned char checksum)
{
  unsigned char crc;
  crc = Data_Crc_Check(pdata, nbrOfBytes); // calculates 8-Bit checksum
  if (crc != checksum)
  {
    return 1;
  }
  return 0;
}
/*
* 获取sht30温湿度
* @param[in]   void  		       :无
* @retval      void                :无
* @note        修改日志 
*               Ver0.0.1:
*/
int sht30_SS_get_value(float *temp, float *humi)
{
  xSemaphoreTake(xMutex8, -1);
  int ret;
  *temp = ERROR_CODE;
  *humi = ERROR_CODE;

  uint8_t sht30_buf[6] = {0};
  i2c_cmd_handle_t cmd = i2c_cmd_link_create(); //新建操作I2C句柄
  i2c_master_start(cmd);                        //启动I2C

  i2c_master_start(cmd);                                           //启动I2C
  i2c_master_write_byte(cmd, SHT30_WRITE_ADDR << 1, ACK_CHECK_EN); //发地址+写+检查ack
                                                                   // i2c_master_write_byte(cmd, CMD_MSB_0_5MSP, ACK_CHECK_EN);                    //发数据高8位+检查ack
                                                                   // i2c_master_write_byte(cmd, CMD_LSB_0_5MSP, ACK_CHECK_EN);                    //发数据低8位+检查ack
  i2c_master_write_byte(cmd, 0x24, ACK_CHECK_EN);                  //发数据高8位+检查ack
  i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN);                  //发数据低8位+检查ack
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS); //I2C发送
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "%d, i2c_master_cmd_begin ERR : %s\n", __LINE__, esp_err_to_name(ret));
    // return ret;
  }
  i2c_cmd_link_delete(cmd); //删除I2C句柄

  vTaskDelay(50 / portTICK_RATE_MS);

  cmd = i2c_cmd_link_create();                                              //新建操作I2C句柄
  i2c_master_start(cmd);                                                    //启动I2C
  i2c_master_write_byte(cmd, (SHT30_WRITE_ADDR << 1) | 0x01, ACK_CHECK_EN); //发地址+读+检查ack
  i2c_master_read_byte(cmd, &sht30_buf[0], ACK_VAL);                        //读取数据+回复ack
  i2c_master_read_byte(cmd, &sht30_buf[1], ACK_VAL);                        //读取数据+回复ack
  i2c_master_read_byte(cmd, &sht30_buf[2], ACK_VAL);                        //读取数据+回复ack
  i2c_master_read_byte(cmd, &sht30_buf[3], ACK_VAL);                        //读取数据+回复ack
  i2c_master_read_byte(cmd, &sht30_buf[4], ACK_VAL);                        //读取数据+回复ack
  i2c_master_read_byte(cmd, &sht30_buf[5], NACK_VAL);                       //读取数据+不回复ack
  i2c_master_stop(cmd);                                                     //停止I2C

  ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS); //I2C发送
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "%d, i2c_master_cmd_begin ERR : %s\n", __LINE__, esp_err_to_name(ret));
    // return ret;
  }
  i2c_cmd_link_delete(cmd); //删除I2C句柄

  vTaskDelay(5 / portTICK_RATE_MS);
  xSemaphoreGive(xMutex8);
  //校验读出来的数据，算法参考sht30 datasheet
  if ((!SHT3X_CheckCrc(sht30_buf, 2, sht30_buf[2])) && (!SHT3X_CheckCrc(sht30_buf + 3, 2, sht30_buf[5])))
  {
    *temp = ((((sht30_buf[0] * 256) + sht30_buf[1]) * 175) / 65535.0 - 45);
    *humi = (((sht30_buf[3] * 256) + (sht30_buf[4])) * 100 / 65535.0);
    // ESP_LOGI("SHT30", "temp:%4.2f C \r\n", *temp); //℃打印出来是乱码,所以用s
    // ESP_LOGI("SHT30", "hum:%4.2f %%RH \r\n", *humi);
    ret = ESP_OK; //成功
  }
  else
  {
    ret = ESP_FAIL;
  }
  return ret;
}