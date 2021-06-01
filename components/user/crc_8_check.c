/*******************************************************************************
  * @file       CRC-8 Check Driver       
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
#include "freertos/FreeRTOS.h"

#define polynomial 0x131 //P(x)=x^8+x^5+x^4+1=100110001

/*******************************************************************************
  CRC-8 Check,No Error-Return 0
*******************************************************************************/
uint8_t Data_Crc_Check(uint8_t *data_buf, uint8_t bytes)
{
  uint8_t bit, byte;
  uint8_t crc_val = 0xff; //calculated checksum

  for (byte = 0; byte < bytes; byte++)
  {
    crc_val ^= *data_buf++;

    for (bit = 0; bit < 8; bit++)
    {
      if (crc_val & 0x80)
      {
        crc_val = (crc_val << 1) ^ polynomial;
      }
      else
      {
        crc_val = (crc_val << 1);
      }
    }
  }
  return crc_val;
}

/*******************************************************************************
  get CRC-8 value
*******************************************************************************/
uint8_t Data_Crc_Value(uint8_t *data_buf, uint8_t bytes)
{
  uint8_t bit, byte;
  uint8_t crc_val = 0xff; //calculated checksum

  for (byte = 0; byte < bytes; byte++)
  {
    crc_val ^= *data_buf++;

    for (bit = 0; bit < 8; bit++)
    {
      if (crc_val & 0x80)
      {
        crc_val = (crc_val << 1) ^ polynomial;
      }
      else
      {
        crc_val = crc_val << 1;
      }
    }
  }
  return crc_val;
}

/*******************************************************************************
                                      END         
*******************************************************************************/
