/*******************************************************************************
  * @file       Data Save Application Task   
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
#include "string.h"
#include "stdbool.h"
#include "stdio.h"
#include "cJSON.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "MsgType.h"
#include "w25q128.h"
#include "at24c08.h"
#include "PCF8563.h"
#include "HttpClientTask.h"
#include "PeripheralDriver.h"

#define RETRY_TIME_OUT 3

extern QueueHandle_t xQueue0;     //Used for cjson and memory save
extern TaskHandle_t xBinary0;     //For DataPostTask interrupt
extern TaskHandle_t xBinary11;    //For Memory Delete Task
extern SemaphoreHandle_t xMutex1; //Used for SPI Lock
extern SemaphoreHandle_t xMutex3; //Used for cJSON Lock

extern volatile uint8_t save_addr_flag;
extern volatile bool data_post; //need post data immediately
extern volatile unsigned long POST_NUM;
extern volatile unsigned long DELETE_ADDR, POST_ADDR, WRITE_ADDR;

extern void osi_Erase_Memory(void);

/*******************************************************************************
//save sensor data
*******************************************************************************/
static void DataSave(char *buffer, uint8_t size)
{
#ifdef DEBUG_SAVE

  osi_UartPrint(buffer);

#endif

  osi_w25q_Write_Addr_Check(WRITE_ADDR); //w25q128 check write address with locked

  xSemaphoreTake(xMutex1, -1); //SPI Semaphore Take

  w25q_WriteData(WRITE_ADDR, buffer, size); //write data in nor-flash

  xSemaphoreGive(xMutex1); //SPI Semaphore Give

  portENTER_CRITICAL(0); //enter critical

  POST_NUM += 1;

  WRITE_ADDR += 1 + size; //End with a '!'

  portEXIT_CRITICAL(0); //exit crtitcal

  if (save_addr_flag == 0)
  {
    osi_at24c08_write(DATA_WRITE_ADDR1, WRITE_ADDR); //save the write address

    osi_at24c08_write(DATA_AMOUNT_ADDR1, POST_NUM); //save the post data amount
  }
  else
  {
    osi_at24c08_write(DATA_WRITE_ADDR2, WRITE_ADDR); //save the write address

    osi_at24c08_write(DATA_AMOUNT_ADDR2, POST_NUM); //save the post data amount
  }

  if (data_post) //need post data immediately
  {
    data_post = 0;

    // osi_SyncObjSignalFromISR(&xBinary0);  //send message to data post task
    vTaskNotifyGiveFromISR(xBinary0, NULL);
  }

#ifdef DEBUG_SAVE

  osi_UartPrint_Val("POST_NUM:", POST_NUM);

  osi_UartPrint_Val("WRITE_ADDR:", WRITE_ADDR);

#endif
}

/*******************************************************************************
//Data Save Task
*******************************************************************************/
void DataSaveTask(void *pvParameters)
{
  char fields[7];
  char utctime[21];
  char *OutBuffer;
  cJSON *pJsonRoot;
  SensorMessage xMsg;
  char SaveBuffer[SAVE_DATA_SIZE];
  unsigned long d_addr, p_addr, w_addr;
  uint16_t det_data_sum;
  uint16_t det_data_num;
  unsigned long det_data_end_addr;

  for (;;)
  {
    // osi_MsgQRead(&xQueue0, &xMsg, -1); //Wait Sensor Value Message
    xQueueReceive(xQueue0, &xMsg, portMAX_DELAY);

    osi_Read_UTCtime(utctime, sizeof(utctime)); //read time

    snprintf(fields, sizeof(fields), "field%d", xMsg.sensornum); //fields number

    xSemaphoreTake(xMutex3, -1); //cJSON Semaphore Take

    pJsonRoot = cJSON_CreateObject();

    cJSON_AddStringToObject(pJsonRoot, "created_at", utctime);

    cJSON_AddNumberToObject(pJsonRoot, fields, (float)xMsg.sensorval);

    OutBuffer = cJSON_PrintUnformatted(pJsonRoot); //cJSON_Print(Root)

    cJSON_Delete(pJsonRoot); //delete cjson root

    memcpy(SaveBuffer, OutBuffer, SAVE_DATA_SIZE);

    free(OutBuffer);

    xSemaphoreGive(xMutex3); //cJSON Semaphore Give

    if ((POST_NUM >= Memory_Max_Addr) || (WRITE_ADDR > Memory_Max_Addr) || (POST_ADDR > Memory_Max_Addr) || (DELETE_ADDR > Memory_Max_Addr))
    {
      // osi_SyncObjSignalFromISR(&xBinary11); //start delete task
      vTaskNotifyGiveFromISR(xBinary11, NULL);
    }
    else
    {
      d_addr = DELETE_ADDR; //delete pointer variable value

      w_addr = WRITE_ADDR; //write pointer variable value

      p_addr = POST_ADDR; //post pointer variable value

      if ((w_addr >= p_addr) && (w_addr >= d_addr) && (p_addr >= d_addr)) //w_addr>d_addr
      {
        if ((w_addr + SAVE_DATA_SIZE) <= Memory_Max_Addr) //Memory Chip max address,end with a '!'
        {
          DataSave(SaveBuffer, strlen(SaveBuffer)); //save data
        }
        else
        {
          WRITE_ADDR = 0; //first Sector

          if (d_addr == 0)
          {
            det_data_sum = POST_DATA_NUMBER;

            while (det_data_sum)
            {
              Read_PostDataLen(POST_ADDR, &det_data_end_addr, det_data_sum, &det_data_num, NULL);

              PostAddrChage(det_data_num, det_data_end_addr); //change the point

              if (det_data_sum > det_data_num)
              {
                det_data_sum -= det_data_num;
              }
              else
              {
                break;
              }
            }
            osi_Erase_Memory(); //Erase memory
          }
          DataSave(SaveBuffer, strlen(SaveBuffer)); //save data
        }
      }
      else if (((p_addr >= d_addr) && (p_addr >= w_addr) && (d_addr >= w_addr)) || ((d_addr >= w_addr) && (d_addr >= p_addr) && (w_addr >= p_addr))) //w_addr<d_addr
      {
        if ((w_addr + SAVE_DATA_SIZE) >= d_addr) //WRITE_ADDR pointer variable can not be equal to DELETED pointer variable//
        {
          det_data_sum = POST_DATA_NUMBER;

          while (det_data_sum)
          {
            Read_PostDataLen(POST_ADDR, &det_data_end_addr, det_data_sum, &det_data_num, NULL);

            PostAddrChage(det_data_num, det_data_end_addr); //change the point

            if (det_data_sum > det_data_num)
            {
              det_data_sum -= det_data_num;
            }
            else
            {
              break;
            }
          }
          osi_Erase_Memory(); //Erase memory
        }
        DataSave(SaveBuffer, strlen(SaveBuffer)); //save data
      }
      else //pointer variable wrong,need delete all sensor data
      {
        // osi_SyncObjSignalFromISR(&xBinary11); //start delete task
        vTaskNotifyGiveFromISR(xBinary11, NULL);
      }
    }
  }
}

/*******************************************************************************
                                      END         
*******************************************************************************/
