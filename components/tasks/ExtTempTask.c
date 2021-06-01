/*******************************************************************************
  * @file       Ext Temperature Sensor Application Task   
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

#include "MsgType.h"
#include "ds18b20.h"
// #include "ht1621.h"
#include "PeripheralDriver.h"

extern TaskHandle_t xBinary6; //For external Temprature Measure
extern QueueHandle_t xQueue0; //Used for cjson and memory save

extern float f8_a, f8_b;

/*******************************************************************************
//External temperature measure task 
*******************************************************************************/
void ExtTempMeasureTask(void *pvParameters)
{
  float water_temp;
  SensorMessage wMsg;

  for (;;)
  {
    // osi_SyncObjWait(&xBinary6,-1);  //Wait Timer Interrupt Message
    ulTaskNotifyTake(pdTRUE, -1);

    water_temp = osi_ds18b20_get_temp(); //measure the temperature

    if (water_temp != ERROR_CODE)
    {
#ifdef USE_LCD
      xSemaphoreTake(xMutex6, -1); //LCD Semaphore Take
      Ht1621_Display_Temp_Val(water_temp, 1);
      xSemaphoreGive(xMutex6); //LCD Semaphore Give
#endif

      wMsg.sensornum = EXT_NUM;                  //Message Number
      wMsg.sensorval = f8_a * water_temp + f8_b; //Message Value
      xQueueSend(xQueue0, &wMsg, 0);             //send noise data message
    }
  }
}

/*******************************************************************************
                                      END         
*******************************************************************************/
