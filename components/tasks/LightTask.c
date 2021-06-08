/*******************************************************************************
  * @file       Light Sensor Application Task
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
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "MsgType.h"
#include "opt3001.h"
// #include "ht1621.h"
#include "PeripheralDriver.h"

#define TAG "LightSensorTask"

extern TaskHandle_t xBinary3; //For Light Sensor Task
extern QueueHandle_t xQueue0; //Used for cjson and memory save

#ifdef USE_LCD
extern OsiSyncObj_t xMutex6; //Used for Ht1621 LCD Driver
extern volatile uint8_t power_flag;
#endif

extern float f3_a, f3_b;

/*******************************************************************************
//light Sensor Task
*******************************************************************************/
void LightSensorTask(void *pvParameters)
{
  float lightvalue;
  SensorMessage lMsg;

  for (;;)
  {
    // osi_SyncObjWait(&xBinary3,-1);  //Wait Timer Interrupt Message
    xEventGroupSetBits(Task_Group, SENTASK_3);
    ulTaskNotifyTake(pdTRUE, -1);
    xEventGroupClearBits(Task_Group, SENTASK_3);

    osi_OPT3001_value(&lightvalue); //Read Light Value
    ESP_LOGI(TAG, "lightvalue=%04f", lightvalue);

    if (lightvalue != ERROR_CODE)
    {
#ifdef USE_LCD
      xSemaphoreTake(xMutex6, -1); //LCD Semaphore Take
      Ht1621_Display_Light_Val(lightvalue, 0, 0, 0, power_flag);
      xSemaphoreGive(xMutex6); //LCD Semaphore Give
#endif

      lMsg.sensornum = LIGHT_NUM;                //Message Number
      lMsg.sensorval = f3_a * lightvalue + f3_b; //Message Value
      xQueueSend(xQueue0, &lMsg, 0);             //Send Light Data Message
    }
  }
}

/*******************************************************************************
                                      END         
*******************************************************************************/
