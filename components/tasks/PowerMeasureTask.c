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
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"

#include "MsgType.h"
#include "at24c08.h"
#include "PeripheralDriver.h"

#define TAG "Power_ADC"

extern TaskHandle_t xBinary7; //For Power Measure Task
extern QueueHandle_t xQueue0; //Used for cjson and memory save
extern SemaphoreHandle_t xMutex2;

extern float f4_a, f4_b;

#define ADC1_TEST_CHANNEL (6)
#define DEFAULT_VREF 1100 //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 64  //Multisampling

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_0; //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_atten_t atten = ADC_ATTEN_DB_6;    //ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;

/*******************************************************************************
//power voltage value
*******************************************************************************/
float power_adcValue(void)
{
  float adc_reading = 0;

  for (int i = 0; i < NO_OF_SAMPLES; i++)
  {
    adc_reading += adc1_get_raw((adc1_channel_t)channel);
  }
  adc_reading /= NO_OF_SAMPLES;
  //Convert adc_reading to voltage in mV
  float voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
  voltage = (float)voltage * 37 / 10 / 1000;
  ESP_LOGI(TAG, "voltage=%.4f", voltage);

  return voltage;
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
    // osi_SyncObjWait(&xBinary7,-1);  //Wait Timer Interrupt Message
    xEventGroupSetBits(Task_Group, SENTASK_7);
    ulTaskNotifyTake(pdTRUE, -1);
    xEventGroupClearBits(Task_Group, SENTASK_7);

    xSemaphoreTake(xMutex2, -1);
    //initialize ADC
    adc1_config_width(width);
    adc1_config_channel_atten(channel, atten);
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);

    for (err_val = 0; err_val < 3; err_val++)
    {
      p_value = power_adcValue(); //Read Noise Value

      if ((p_value >= 0.9) && (p_value <= 6.5))
      {
        if (p_value <= 4.2)
        {
          if (!(gpio_get_level(USB_PIN)))
          {
            // MAP_UtilsDelay(100000); //Delay About 7.5ms
            vTaskDelay(5 / portTICK_RATE_MS);
            if (!(gpio_get_level(USB_PIN)))
            {
              break;
            }
          }
        }
        else if (p_value >= 4.2)
        {
          if (gpio_get_level(USB_PIN))
          {
            // MAP_UtilsDelay(100000); //Delay About 7.5ms
            vTaskDelay(5 / portTICK_RATE_MS);

            if (gpio_get_level(USB_PIN))
            {
              p_value = 5;
              break;
            }
          }
        }
      }

      // MAP_UtilsDelay(20000000); //Delay About 1.5s
      vTaskDelay(1500 / portTICK_RATE_MS);
    }
    xSemaphoreGive(xMutex2);

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

    pMsg.sensornum = BAT_NUM;               //Message Number
    pMsg.sensorval = f4_a * p_value + f4_b; //Message Value
    xQueueSend(xQueue0, &pMsg, 0);          //send power data message
  }
}

/*******************************************************************************
                                      END         
*******************************************************************************/
