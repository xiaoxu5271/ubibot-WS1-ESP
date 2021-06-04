/*******************************************************************************
  * @file       OPT3001 Light Sensor DRIVER APPLICATION      
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
#include "esp_log.h"

#include "opt3001.h"

#define TAG "opt3001"
/*******************************************************************************
  write opt3001 light sensor register
*******************************************************************************/
static void OPT3001_WriteReg(uint8_t regaddr, uint16_t val)
{
  uint8_t reg[2];

  reg[0] = val / 256; //value high 8bit

  reg[1] = val % 256; //value low 8bit

  MulTry_I2C_WR_mulReg(OPT3001_ADDR, regaddr, reg, 2); //write data
}

/*******************************************************************************
  read write opt3001 light sensor register
*******************************************************************************/
static uint16_t OPT3001_ReadReg(uint8_t regaddr)
{
  uint8_t regdata[2];

  MulTry_I2C_RD_mulReg(OPT3001_ADDR, regaddr, regdata, 2); //read data

  return (uint16_t)((regdata[0] << 8) + regdata[1]);
}

/*******************************************************************************
  opt3001 light sensor init
*******************************************************************************/
short OPT3001_Init(void)
{
  if (OPT3001_ReadReg(DeviceID) == 0x3001) //read device id
  {
    OPT3001_WriteReg(LowLimit, 0x0064); //low limit 1.00-lux

    OPT3001_WriteReg(HighLimit, 0xBFFF); //high limit 83865.60-lux

    OPT3001_WriteReg(Configuration, 0xCE1A); //15:12-C: automatic full-scale,800ms conversion time field,continuous

    ESP_LOGI(TAG, "%d", __LINE__);
    return SUCCESS;
  }
  ESP_LOGE(TAG, "%d", __LINE__);
  return FAILURE;
}

/*******************************************************************************
//measure the light value
*******************************************************************************/
void OPT3001_value(float *lightvalue)
{
  double e_val;
  float lsbsize;
  uint16_t retry;
  uint16_t read_val;

  if (OPT3001_ReadReg(DeviceID) == 0x3001) //read device id
  {
    for (retry = 0; retry < 1000; retry++)
    {
      if ((OPT3001_ReadReg(Configuration) & 0x0080) == 0x0080) //wait the conversion completed
      {
        read_val = OPT3001_ReadReg(Result); //read the result register

        e_val = (read_val >> 12) & 0x000f;

        lsbsize = 0.01 * pow(2.0, (double)e_val);

        *lightvalue = (float)lsbsize * (float)(read_val & 0x0fff); //read result

        break;
      }
      else
      {
        *lightvalue = ERROR_CODE;
      }

      // MAP_UtilsDelay(60000); //delay about 4.5ms
      vTaskDelay(5 / portTICK_RATE_MS);
    }
  }
  else
  {
    *lightvalue = ERROR_CODE;
  }
}

/*******************************************************************************
                                      END         
*******************************************************************************/
