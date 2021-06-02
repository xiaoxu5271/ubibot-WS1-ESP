/*******************************************************************************
  * @file       Uart Application Task  
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
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "sht30dis.h"
#include "opt3001.h"
#include "at24c08.h"
#include "w25q128.h"
#include "adxl345.h"
#include "PCF8563.h"
#include "JsonParse.h"
#include "MsgType.h"
#include "PeripheralDriver.h"
#include "HttpClientTask.h"
#include "app_config.h"

#include "UartTask.h"

extern TaskHandle_t GR_LED_FAST_TaskHandle;
QueueHandle_t Log_uart_queue;

extern SemaphoreHandle_t xMutex1; //Used for SPI Lock
extern SemaphoreHandle_t xMutex3; //Used for cJSON Lock
extern SemaphoreHandle_t xMutex4; //Used for UART Lock

extern TaskHandle_t xBinary0;     //For post Task
extern TaskHandle_t UART_xBinary; //For UART Parse Task
extern TaskHandle_t xBinary16;    //USB activate

uint16_t iLen = 0;
char UartGet[UART_REV_BUF_LEN];
bool uart_pares_status = 0;

extern volatile bool ap_mode_status;
extern volatile bool f_reset_status;
extern volatile bool POST_TASK_END_FLAG;
extern volatile bool UPDATETIME_TASK_END_FLAG;
extern volatile bool APIGET_TASK_END_FLAG;
extern volatile uint16_t sys_run_time;

extern volatile unsigned long POST_NUM;
extern volatile unsigned long DELETE_ADDR, POST_ADDR, WRITE_ADDR;

#define UART0_TXD (UART_PIN_NO_CHANGE)
#define UART0_RXD (UART_PIN_NO_CHANGE)
#define UART0_RTS (UART_PIN_NO_CHANGE)
#define UART0_CTS (UART_PIN_NO_CHANGE)

#define TAG "UartTask"

//UART INIT
void Set_Uart_Int_Source(void)
{
  //uart0,log
  uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_APB,
  };
  //Install UART driver, and get the queue.
  uart_driver_install(UART_NUM_0, UART_REV_BUF_LEN * 2, 0, 2, &Log_uart_queue, 0);
  uart_param_config(UART_NUM_0, &uart_config);
  uart_set_pin(UART_NUM_0, UART0_TXD, UART0_TXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}
/*******************************************************************************
//uart read data
*******************************************************************************/
static void UartReadData(unsigned long addr)
{
  char read_buf[65];

  unsigned long save_data_num;

  osi_at24c08_read_addr(); //Read Post Data Amount/Write Data/Post Data/Delete Data Address

  save_data_num = POST_NUM;

  //  xSemaphoreTake(xMutex4, -1); //UART Semaphore Take
  xSemaphoreTake(xMutex4, -1);

  printf("{\"save_data_sum\":%ld}\r\n", save_data_num);

  //  xSemaphoreGive(xMutex4); //UART Semaphore Give
  xSemaphoreGive(xMutex4);

  while (save_data_num--)
  {
    osi_w25q_ReadData(addr, read_buf, sizeof(read_buf), NULL);

    osi_UartPrint_Mul(read_buf, "\r\n");

    addr += strlen(read_buf) + 1; //end whit '!'

    sys_run_time = 0; //clear system time out
  }
}

/*******************************************************************************
//UART GET TASK
*******************************************************************************/
//void UartRevTask(void *pvParameters)
//{
//  char cChar;
//
//  for(;;)
//  {
//    osi_SyncObjWait(&xBinary10,-1);  //waite UART0 interrupt message
//
//     xSemaphoreTake(xMutex4,-1);  //UART Semaphore Take
//
//    while(UARTCharsAvail(UARTA0_BASE))
//    {
//      cChar=UARTCharGetNonBlocking(UARTA0_BASE);  //none blocking get
//
//      UARTCharPutNonBlocking(UARTA0_BASE,cChar);  //none blocking put
//
//      if(uart_pares_status == 0)
//      {
//        iLen=iLen>=UART_REV_BUF_LEN?0:iLen;  //buffer max
//
//        UartGet[iLen++]=cChar;
//
//        if((cChar=='\n')||(cChar=='\r'))  //end with '\r\n'
//        {
//          UartGet[--iLen]='\0';               //end flag
//
//          if(iLen)
//          {
//            osi_SyncObjSignalFromISR(&UART_xBinary);
//          }
//        }
//      }
//    }
//
//     xSemaphoreGive(xMutex4);  //UART Semaphore Give
//  }
//}

/*******************************************************************************
//UART Parse TASK
*******************************************************************************/
void UartParseTask(void *pvParameters)
{
  short cmdprase;
  short set_val = -1;

  uart_event_t event;
  // uint8_t UartGet[BUF_SIZE] = {0};
  uint16_t all_read_len = 0;
  while (1)
  {
    ESP_LOGI(TAG, "%d", __LINE__);
    if (xQueueReceive(Log_uart_queue, (void *)&event, (portTickType)portMAX_DELAY))
    {
      switch (event.type)
      {
      case UART_DATA:
        if (event.size >= 1)
        {
          if (all_read_len + event.size >= UART_REV_BUF_LEN)
          {
            ESP_LOGE(TAG, "read len flow");
            all_read_len = 0;
            memset(UartGet, 0, UART_REV_BUF_LEN);
          }
          uart_read_bytes(UART_NUM_0, (uint8_t *)UartGet + all_read_len, event.size, portMAX_DELAY);

          uxQueueMessagesWaiting(Log_uart_queue);

          all_read_len += event.size;
          UartGet[all_read_len] = 0; //去掉字符串结束符，防止字符串拼接不成功

          if ((UartGet[all_read_len - 1] == '\r') || (UartGet[all_read_len - 1] == '\n'))
          {
            ESP_LOGI(TAG, "uart0 recv,  len:%d,%s", strlen((const char *)UartGet), UartGet);

            xSemaphoreTake(xMutex3, -1);
            cmdprase = ParseTcpUartCmd(UartGet);
            xSemaphoreGive(xMutex3);

            switch (cmdprase)
            {
            case 0:
            {
              osi_UartPrint_Mul(SUCCESSED_CODE, "\r\n");
              break;
            }
            case 1: //Command:SetupWifi
            {
              osi_UartPrint_Mul(SUCCESSED_CODE, "\r\n");
              osi_bell_makeSound(200);

              if (ap_mode_status || POST_TASK_END_FLAG || UPDATETIME_TASK_END_FLAG || APIGET_TASK_END_FLAG)
              {
                osi_UartPrint_Mul(FAILURED_CODE, "\r\n");
              }
              else
              {
                my_xTaskCreate(Green_Red_Led_FastFlashed_Task, "Green_Red_Led_FastFlashed_Task", 256, NULL, 9, &GR_LED_FAST_TaskHandle); //Create Green and Red Led Fast Flash Task
                set_val = osi_WiFi_Connect_Test();
                vTaskDelete(GR_LED_FAST_TaskHandle); //delete Green and Red Led Fast Flash Task
                SET_RED_LED_OFF();                   //Set Red Led Off
                SET_GREEN_LED_OFF();                 //Set Green Led Off
                osi_bell_makeSound(200);
                if (set_val < 0)
                {
                  Red_Led_Flashed(3, 3);
                  osi_UartPrint_Mul(FAILURED_CODE, "\r\n");
                }
                else
                {
                  Green_Led_Flashed(3, 3);
                  osi_UartPrint_Mul(SUCCESSED_CODE, "\r\n");
                  // osi_SyncObjSignalFromISR(&xBinary16); //start device activate
                  if (xBinary16 != NULL)
                    vTaskNotifyGiveFromISR(xBinary16, NULL);
                  // osi_SyncObjSignalFromISR(&xBinary0);  //Post Task
                  if (xBinary0 != NULL)
                    vTaskNotifyGiveFromISR(xBinary0, NULL);
                }
              }
              break;
            }
            case 2: //Command:ReadProduct
            {
              memset(UartGet, 0, sizeof(UartGet));
              Read_Product_Set(UartGet, UART_REV_BUF_LEN);
              osi_UartPrint_Mul(UartGet, "\r\n");
              break;
            }
            case 3: //Command:ReadWifi
            {
              memset(UartGet, 0, sizeof(UartGet));
              Read_Wifi_Set(UartGet, UART_REV_BUF_LEN);
              osi_UartPrint_Mul(UartGet, "\r\n");
              break;
            }
            case 4: //Command:GetLastError
            {
              memset(UartGet, 0, sizeof(UartGet));
              Read_System_ERROR_Code(UartGet, UART_REV_BUF_LEN);
              osi_UartPrint_Mul(UartGet, "\r\n");
              break;
            }
            case 6: //Command:ReadMetaData
            {
              memset(UartGet, 0, sizeof(UartGet));
              Cmd_Read_MetaData(UartGet, UART_REV_BUF_LEN);
              osi_UartPrint_Mul(UartGet, "\r\n");
              break;
            }
            case 12: //Command:ReadCali
            {
              memset(UartGet, 0, sizeof(UartGet));
              Read_cali(UartGet, UART_REV_BUF_LEN);
              osi_UartPrint_Mul(UartGet, "\r\n");
              break;
            }
            case 7: //Command:ScanWifiList
            {
              if (ap_mode_status || POST_TASK_END_FLAG || UPDATETIME_TASK_END_FLAG || APIGET_TASK_END_FLAG)
              {
                osi_UartPrint_Mul(FAILURED_CODE, "\r\n");
              }
              else
              {
                osi_Scan_Wifi_List(NULL, NULL, 1);
              }
              break;
            }
            case 8: //Command:CheckSensors
            {
              memset(UartGet, 0, sizeof(UartGet));
              Cmd_System_TestData(UartGet, UART_REV_BUF_LEN);
              osi_UartPrint_Mul(UartGet, "\r\n");
              break;
            }
            case 9: //Command:ReadData
            {
              UartReadData(POST_ADDR);
              break;
            }
            case 10: //Command:ClearData
            {
              if (f_reset_status)
              {
                osi_UartPrint_Mul(FAILURED_CODE, "\r\n");
              }
              else
              {
                SET_GREEN_LED_OFF();   //Set Green Led Off
                osi_Save_Data_Reset(); //Nor Flash Memory Chip Reset
                osi_UartPrint_Mul(SUCCESSED_CODE, "\r\n");
              }
              break;
            }
            default:
            {
              break;
            }
            }

            all_read_len = 0;
            memset(UartGet, 0, UART_REV_BUF_LEN);
            uart_flush(UART_NUM_0);
          }
        }
        break;

      case UART_FIFO_OVF:
        ESP_LOGI(TAG, "hw fifo overflow");
        uart_flush_input(UART_NUM_0);
        xQueueReset(Log_uart_queue);
        break;

      case UART_BUFFER_FULL:
        ESP_LOGI(TAG, "ring buffer full");
        uart_flush_input(UART_NUM_0);
        xQueueReset(Log_uart_queue);
        break;

      //Others
      default:
        // ESP_LOGI(TAG, "uart type: %d", event.type);
        break;
      }
    }
  }
}

/*******************************************************************************
                                      END         
*******************************************************************************/
