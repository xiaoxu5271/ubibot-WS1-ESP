/*******************************************************************************
  * @file       IIC BUS DRIVER APPLICATION       
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
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
// #include "Json_parse.h"
// #include "Http.h"
// #include "user_key.h"
// #include "Led.h"
// #include "crc8_16.h"
// #include "ota.h"

#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "MsgType.h"
#include "iic.h"

/*******************************************************************************
  delay about nus*3us
*******************************************************************************/
void delay_us(uint8_t nus)
{
  MAP_UtilsDelay(40 * nus); //40
}

/*******************************************************************************
//init the iic bus
*******************************************************************************/
void I2C_Init(void)
{
  // SDA_OUT(); //sda out mode

  // IIC_SCL_ON();

  // IIC_SDA_ON();
  int i2c_master_port = I2C_MASTER_NUM;
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = I2C_MASTER_SDA_IO;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = I2C_MASTER_SCL_IO;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
  i2c_param_config(i2c_master_port, &conf);
  i2c_driver_install(i2c_master_port, conf.mode,
                     I2C_MASTER_RX_BUF_DISABLE,
                     I2C_MASTER_TX_BUF_DISABLE, 0);
}

/*******************************************************************************
  write a byte to slave register
*******************************************************************************/
static short IIC_WR_Reg(uint8_t sla_addr, uint8_t reg_addr, uint8_t val)
{
  // IIC_Start(); //IIC start
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);

  // IIC_Send_Byte(2 * sla_addr); //send write command

  // if (IIC_Wait_Ack() < 0) //wait device ack
  // {
  //   return FAILURE;
  // }
  i2c_master_write_byte(cmd, (2 * sla_addr), ACK_CHECK_EN);

  // IIC_Send_Byte(reg_addr); //send register address

  // if (IIC_Wait_Ack() < 0) //wait device ack
  // {
  //   return FAILURE;
  // }
  i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);

  // IIC_Send_Byte(val); //send data value
  i2c_master_write_byte(cmd, val, ACK_CHECK_EN);

  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

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
  write a byte to slave register whit multiple try
*******************************************************************************/
void MulTry_IIC_WR_Reg(uint8_t sla_addr, uint8_t reg_addr, uint8_t val)
{
  uint8_t n_try;

  for (n_try = 0; n_try < RETRY_TIME_OUT; n_try++)
  {
    if (IIC_WR_Reg(sla_addr, reg_addr, val) == SUCCESS)
    {
      break;
    }
    MAP_UtilsDelay(80000); //delay about 6ms
  }
}

/*******************************************************************************
  Read a byte from slave register
*******************************************************************************/
static short IIC_RD_Reg(uint8_t sla_addr, uint8_t reg_addr, uint8_t *val)
{
  // IIC_Start(); //IIC start
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);

  // IIC_Send_Byte(2 * sla_addr); //send write command

  // if (IIC_Wait_Ack() < 0) //wait device ack
  // {
  //   return FAILURE;
  // }
  i2c_master_write_byte(cmd, 2 * sla_addr, ACK_CHECK_EN);

  // IIC_Send_Byte(reg_addr); //send register address

  // if (IIC_Wait_Ack() < 0) //wait device ack
  // {
  //   return FAILURE;
  // }
  i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);

  // IIC_Start(); //IIC start
  i2c_master_start(cmd);

  // IIC_Send_Byte(2 * sla_addr + 1); //send read command

  // if (IIC_Wait_Ack() < 0) //wait device ack
  // {
  //   return FAILURE;
  // }
  i2c_master_write_byte(cmd, 2 * sla_addr + 1, ACK_CHECK_EN);

  // *val = IIC_Read_Byte(0); //read a byte
  i2c_master_read_byte(cmd, val, NACK_VAL); //只读1 byte 不需要应答

  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

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
  Read a byte from slave register whit multiple try
*******************************************************************************/
void MulTry_IIC_RD_Reg(uint8_t sla_addr, uint8_t reg_addr, uint8_t *val)
{
  uint8_t n_try;

  for (n_try = 0; n_try < RETRY_TIME_OUT; n_try++)
  {
    if (IIC_RD_Reg(sla_addr, reg_addr, val) == SUCCESS)
    {
      break;
    }
    MAP_UtilsDelay(40000); //delay about 3ms
  }
}

/*******************************************************************************
  write multi byte to slave register
*******************************************************************************/
static short I2C_WR_mulReg(uint8_t sla_addr, uint8_t reg_addr, uint8_t *buf, uint8_t len)
{
  // IIC_Start(); //IIC start
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);

  // IIC_Send_Byte(2 * sla_addr); //send write command

  // if (IIC_Wait_Ack() < 0) //wait device ack
  // {
  //   return FAILURE;
  // }
  i2c_master_write_byte(cmd, 2 * sla_addr, ACK_CHECK_EN);

  // IIC_Send_Byte(reg_addr); //send register address

  // if (IIC_Wait_Ack() < 0) //wait device ack
  // {
  //   return FAILURE;
  // }
  i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);

  while (len)
  {
    // IIC_Send_Byte(*buf); //send data value
    i2c_master_write_byte(cmd, *buf, ACK_CHECK_EN);
    buf++;
    len--;
  }

  // IIC_Stop(); //IIC stop
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

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
  write multi byte to slave register whit multiple try
*******************************************************************************/
void MulTry_I2C_WR_mulReg(uint8_t sla_addr, uint8_t reg_addr, uint8_t *buf, uint8_t len)
{
  uint8_t n_try;

  for (n_try = 0; n_try < RETRY_TIME_OUT; n_try++)
  {
    if (I2C_WR_mulReg(sla_addr, reg_addr, buf, len) == SUCCESS)
    {
      break;
    }
    MAP_UtilsDelay(80000); //delay about 6ms
  }
}

/*******************************************************************************
  read multiple byte from slave register
*******************************************************************************/
static short I2C_RD_mulReg(uint8_t sla_addr, uint8_t reg_addr, uint8_t *buf, uint8_t len)
{
  uint8_t i;

  // IIC_Start(); //IIC start
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);

  // IIC_Send_Byte(2 * sla_addr); //send write command

  // if (IIC_Wait_Ack() < 0) //wait device ack
  // {
  //   return FAILURE;
  // }
  i2c_master_write_byte(cmd, 2 * sla_addr, ACK_CHECK_EN);

  // IIC_Send_Byte(reg_addr); //send register address

  // if (IIC_Wait_Ack() < 0) //wait device ack
  // {
  //   return FAILURE;
  // }
  i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);

  // IIC_Start(); //IIC start
  i2c_master_start(cmd);

  // IIC_Send_Byte(2 * sla_addr + 1); //send read command

  // if (IIC_Wait_Ack() < 0) //wait device ack
  // {
  //   return FAILURE;
  // }
  i2c_master_write_byte(cmd, 2 * sla_addr + 1, ACK_CHECK_EN);

  for (i = 0; i < len; i++)
  {
    if (i == len - 1)
    {
      // buf[i] = IIC_Read_Byte(0); //read a byte no ack
      i2c_master_read_byte(cmd, &buf[i], NACK_VAL);
    }
    else
    {
      // buf[i] = IIC_Read_Byte(1); //read a byte ack
      i2c_master_read_byte(cmd, &buf[i], ACK_VAL);
    }
  }

  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

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
  read multiple byte from slave register whit multiple try
*******************************************************************************/
void MulTry_I2C_RD_mulReg(uint8_t sla_addr, uint8_t reg_addr, uint8_t *buf, uint8_t len)
{
  uint8_t n_try;

  for (n_try = 0; n_try < RETRY_TIME_OUT; n_try++)
  {
    if (I2C_RD_mulReg(sla_addr, reg_addr, buf, len) == SUCCESS)
    {
      break;
    }
    MAP_UtilsDelay(40000); //delay about 3ms
  }
}

/*******************************************************************************
                                      END         
*******************************************************************************/
