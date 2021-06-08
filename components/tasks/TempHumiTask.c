/*******************************************************************************
  * @file       Temperature and Humility Sensor Application Task
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
#include "esp_log.h"

#include "MsgType.h"
#include "sht30dis.h"
// #include "ht1621.h"
#include "PeripheralDriver.h"

#define TAG "TempHumiSensorTask"

extern QueueHandle_t xQueue0; //Used for cjson and memory save
extern TaskHandle_t xBinary2; //For Temp&Humi Sensor Task

#ifdef USE_LCD
extern OsiSyncObj_t xMutex6; //Used for Ht1621 LCD Driver
#endif

extern float f1_a, f1_b, f2_a, f2_b;

/*******************************************************************************
//Temperature and Humility Sensor Task 
*******************************************************************************/
void TempHumiSensorTask(void *pvParameters)
{
  float tempvalue;
  float humivalue;
  SensorMessage thMsg;

  for (;;)
  {
    // osi_SyncObjWait(&xBinary2,-1);  //Wait Sensor Task Operate Message

    xEventGroupSetBits(Task_Group, SENTASK_2);
    ulTaskNotifyTake(pdTRUE, -1);
    xEventGroupClearBits(Task_Group, SENTASK_2);

    osi_sht30_SingleShotMeasure(&tempvalue, &humivalue); //read temperature humility data

    ESP_LOGI(TAG, "tempvalue=%04f,humivalue=%04f", tempvalue, humivalue);

    if (tempvalue != ERROR_CODE)
    {
#ifdef USE_LCD
      xSemaphoreTake(xMutex6, -1);           //LCD Semaphore Take
      Ht1621_Display_Temp_Val(tempvalue, 0); //Display Temprature value
      xSemaphoreGive(xMutex6);               //LCD Semaphore Give
#endif

      thMsg.sensornum = TEMP_NUM;                //Message Number
      thMsg.sensorval = f1_a * tempvalue + f1_b; //Message Value
      xQueueSend(xQueue0, &thMsg, 0);            //Send Temperature Data Message
    }

    if (humivalue != ERROR_CODE)
    {
#ifdef USE_LCD
      xSemaphoreTake(xMutex6, -1);                 //LCD Semaphore Take
      Ht1621_Display_Humi_Val((uint8_t)humivalue); //Display Humility value
      xSemaphoreGive(xMutex6);                     //LCD Semaphore Give
#endif

      thMsg.sensornum = HUMI_NUM;                //Message Number
      thMsg.sensorval = f2_a * humivalue + f2_b; //Message Value
      xQueueSend(xQueue0, &thMsg, 0);            //Send Humility Data Message
    }
  }
}

/*******************************************************************************
                                      END         
*******************************************************************************/
