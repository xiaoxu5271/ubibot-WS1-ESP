/*******************************************************************************
  * @file       Power Measure Application Task
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
#include <stdlib.h>
#include "osi.h"
#include "MsgType.h"
#include "hw_types.h"
#include "rom_map.h"
#include "utils.h"
#include "adc.h"
#include "hw_memmap.h"
#include "gpio_hal.h"
#include "pin.h"
#include "at24c08.h"
#include "PeripheralDriver.h"

#define USB_PORT GPIOA0_BASE
#define USB_PIN GPIO_PIN_4

extern OsiSyncObj_t xBinary7; //For Power Measure Task
extern OsiMsgQ_t xQueue0;     //Used for cjson and memory save

extern float f4_a, f4_b;

/*******************************************************************************
//power voltage value
*******************************************************************************/
float power_adcValue(void)
{
  uint8_t i = 0;
  float p_val = 0;
  float p_value;
  uint8_t uiIndex = 0;
  uint32_t ulSample;
  uint32_t pulAdcSamples[10];
  uint8_t err_num = 0;

  MAP_PinTypeADC(PIN_60, PIN_MODE_255); //adc in mode

  MAP_ADCTimerConfig(ADC_BASE, 2 ^ 17); //Configure ADC timer

  MAP_ADCTimerEnable(ADC_BASE); //Enable ADC timer

  MAP_ADCEnable(ADC_BASE); //Enable ADC module

  MAP_ADCChannelEnable(ADC_BASE, ADC_CH_3); //Enable ADC channel

  while (uiIndex < 10)
  {
    if (MAP_ADCFIFOLvlGet(ADC_BASE, ADC_CH_3))
    {
      ulSample = MAP_ADCFIFORead(ADC_BASE, ADC_CH_3);

      pulAdcSamples[uiIndex++] = ulSample;
    }
    if (err_num++ == 0xff)
    {
      break;
    }
  }

  MAP_ADCChannelDisable(ADC_BASE, ADC_CH_3); //disable the adc channel

  uiIndex = 0;

  while (uiIndex < 5)
  {
    p_value = ((float)((pulAdcSamples[5 + uiIndex] >> 2) & 0x0FFF)) * 1.467 / 4096;

    uiIndex++;

    if ((p_value <= 1.467) && (p_value > 0))
    {
      p_val += p_value;

      i++;
    }
  }

  if (i)
  {
    return 3.7 * p_val / i;
  }
  else
  {
    return 0;
  }
}

/*******************************************************************************
//Power Measure Task
*******************************************************************************/
void PowerMeasureTask(void *pvParameters)
{
  uint8_t err_val;
  float p_value;
  SensorMessage pMsg;

  for (;;)
  {
    // osi_SyncObjWait(&xBinary7,OSI_WAIT_FOREVER);  //Wait Timer Interrupt Message
    ulTaskNotifyTake(pdTRUE, -1);

    for (err_val = 0; err_val < 3; err_val++)
    {
      p_value = power_adcValue(); //Read Noise Value

      if ((p_value >= 0.9) && (p_value <= 6.5))
      {
        if (p_value <= 4.2)
        {
          if (!(GPIOPinRead(USB_PORT, USB_PIN)))
          {
            MAP_UtilsDelay(100000); //Delay About 7.5ms
            if (!(GPIOPinRead(USB_PORT, USB_PIN)))
            {
              break;
            }
          }
        }
        else if (p_value >= 4.2)
        {
          if (GPIOPinRead(USB_PORT, USB_PIN))
          {
            MAP_UtilsDelay(100000); //Delay About 7.5ms

            if (GPIOPinRead(USB_PORT, USB_PIN))
            {
              p_value = 5;

              break;
            }
          }
        }
      }

      MAP_UtilsDelay(20000000); //Delay About 1.5s
    }

#ifdef USE_LCD
    if (p_value > 3.0)
    {
      osi_at24c08_write_byte(POWER_DATA_ADDR, 0x07);
    }
    else if (p_value > 2.5)
    {
      osi_at24c08_write_byte(POWER_DATA_ADDR, 0x06);
    }
    else
    {
      osi_at24c08_write_byte(POWER_DATA_ADDR, 0x04);
    }
#endif

    pMsg.sensornum = BAT_NUM;                    //Message Number
    pMsg.sensorval = f4_a * p_value + f4_b;      //Message Value
    osi_MsgQWrite(&xQueue0, &pMsg, OSI_NO_WAIT); //send power data message
  }
}

/*******************************************************************************
                                      END         
*******************************************************************************/
