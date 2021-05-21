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
#include "stdlib.h"
#include "osi.h"
#include "string.h"
#include "stdint.h"
#include "stdint.h"
#include "stdbool.h"
#include "stdio.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "rom_map.h"
#include "gpio.h"
#include "uart.h"
#include "uart_if.h"
#include "common.h"
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
#include "UartTask.h"

extern OsiTaskHandle            GR_LED_FAST_TaskHandle;

extern OsiSyncObj_t             xMutex1;        //Used for SPI Lock
extern OsiSyncObj_t             xMutex3;        //Used for cJSON Lock
extern OsiSyncObj_t             xMutex4;        //Used for UART Lock

extern OsiSyncObj_t             xBinary0;      //For post Task
extern OsiSyncObj_t             UART_xBinary; //For UART Parse Task
extern OsiSyncObj_t             xBinary16;      //USB activate

uint16_t iLen = 0;
char UartGet[UART_REV_BUF_LEN];
bool uart_pares_status = 0;

extern volatile bool ap_mode_status;
extern volatile bool f_reset_status;
extern volatile bool POST_TASK_END_FLAG;
extern volatile bool UPDATETIME_TASK_END_FLAG;
extern volatile bool APIGET_TASK_END_FLAG;
extern volatile uint16_t sys_run_time;

extern volatile unsigned long   POST_NUM;
extern volatile unsigned long   DELETE_ADDR,POST_ADDR,WRITE_ADDR;

/*******************************************************************************
//uart read data
*******************************************************************************/
static void UartReadData(unsigned long addr)
{
  char read_buf[65];

  unsigned long save_data_num;
  
  osi_at24c08_read_addr();  //Read Post Data Amount/Write Data/Post Data/Delete Data Address
  
  save_data_num=POST_NUM;
  
  osi_SyncObjWait(&xMutex4,OSI_WAIT_FOREVER);   //UART Semaphore Take
    
  UART_PRINT("{\"save_data_sum\":%d}\r\n",save_data_num);
  
  osi_SyncObjSignal(&xMutex4);  //UART Semaphore Give
    
  while(save_data_num--)
  {
    osi_w25q_ReadData(addr,read_buf,sizeof(read_buf),NULL);

    osi_UartPrint_Mul(read_buf,"\r\n");
    
    addr+=strlen(read_buf)+1;  //end whit '!'
    
    sys_run_time = 0;  //clear system time out
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
//    osi_SyncObjWait(&xBinary10,OSI_WAIT_FOREVER);  //waite UART0 interrupt message
//
//    osi_SyncObjWait(&xMutex4,OSI_WAIT_FOREVER);  //UART Semaphore Take
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
//    osi_SyncObjSignal(&xMutex4);  //UART Semaphore Give
//  }
//}

/*******************************************************************************
//UART Parse TASK
*******************************************************************************/
void UartParseTask(void *pvParameters)
{
  short cmdprase;
  short set_val=-1;
  
  for(;;)
  {
    osi_SyncObjWait(&UART_xBinary,OSI_WAIT_FOREVER);  //waite UART0 interrupt message

    uart_pares_status = 1;
    osi_UartPrint_Mul(UartGet,"\r");
    
    osi_SyncObjWait(&xMutex3,OSI_WAIT_FOREVER);  //cJSON Semaphore Take
    cmdprase=ParseTcpUartCmd(UartGet);
    osi_SyncObjSignal(&xMutex3);  //cJSON Semaphore Give
    
    switch(cmdprase)
    {
      case 0:
      {
        osi_UartPrint_Mul(SUCCESSED_CODE,"\r\n");
        break;
      }
      case 1:  //Command:SetupWifi
      {
        osi_UartPrint_Mul(SUCCESSED_CODE,"\r\n");
        osi_bell_makeSound(200);
        
        if(ap_mode_status||POST_TASK_END_FLAG||UPDATETIME_TASK_END_FLAG||APIGET_TASK_END_FLAG)
        {
          osi_UartPrint_Mul(FAILURED_CODE,"\r\n");
        }
        else
        {
          osi_TaskCreate( Green_Red_Led_FastFlashed_Task, "Green_Red_Led_FastFlashed_Task",256, NULL, 9,&GR_LED_FAST_TaskHandle);  //Create Green and Red Led Fast Flash Task
          set_val=osi_WiFi_Connect_Test();
          osi_TaskDelete(&GR_LED_FAST_TaskHandle);  //delete Green and Red Led Fast Flash Task
          SET_RED_LED_OFF();  //Set Red Led Off
          SET_GREEN_LED_OFF();  //Set Green Led Off
          osi_bell_makeSound(200);
          if(set_val<0)
          {
            Red_Led_Flashed(3,3);
            osi_UartPrint_Mul(FAILURED_CODE,"\r\n");
          }
          else
          {
            Green_Led_Flashed(3,3);
            osi_UartPrint_Mul(SUCCESSED_CODE,"\r\n");
            osi_SyncObjSignalFromISR(&xBinary16);  //start device activate
            osi_SyncObjSignalFromISR(&xBinary0);  //Post Task
          }
        }
        break;
      }
      case 2:  //Command:ReadProduct
      {
        memset(UartGet,0,sizeof(UartGet));
        Read_Product_Set(UartGet,UART_REV_BUF_LEN);
        osi_UartPrint_Mul(UartGet,"\r\n");
        break;
      }
      case 3:  //Command:ReadWifi
      {
        memset(UartGet,0,sizeof(UartGet));
        Read_Wifi_Set(UartGet,UART_REV_BUF_LEN);
        osi_UartPrint_Mul(UartGet,"\r\n");
        break;
      }
      case 4:  //Command:GetLastError
      {
        memset(UartGet,0,sizeof(UartGet));
        Read_System_ERROR_Code(UartGet,UART_REV_BUF_LEN);
        osi_UartPrint_Mul(UartGet,"\r\n");
        break;
      }
      case 6:  //Command:ReadMetaData
      {
        memset(UartGet,0,sizeof(UartGet));
        Cmd_Read_MetaData(UartGet,UART_REV_BUF_LEN);
        osi_UartPrint_Mul(UartGet,"\r\n");
        break;
      }
      case 12:  //Command:ReadCali
      {
        memset(UartGet,0,sizeof(UartGet));
        Read_cali(UartGet,UART_REV_BUF_LEN);
        osi_UartPrint_Mul(UartGet,"\r\n");
        break;
      }
      case 7:  //Command:ScanWifiList
      {
        if(ap_mode_status||POST_TASK_END_FLAG||UPDATETIME_TASK_END_FLAG||APIGET_TASK_END_FLAG)
        {
          osi_UartPrint_Mul(FAILURED_CODE,"\r\n");
        }
        else
        {
          osi_Scan_Wifi_List(NULL,NULL,1);
        }
        break;
      }
      case 8:  //Command:CheckSensors
      {
        memset(UartGet,0,sizeof(UartGet));
        Cmd_System_TestData(UartGet,UART_REV_BUF_LEN);
        osi_UartPrint_Mul(UartGet,"\r\n");
        break;
      }
      case 9:  //Command:ReadData
      {
        UartReadData(POST_ADDR);
        break;
      }
      case 10:  //Command:ClearData
      {
        if(f_reset_status)
        {
          osi_UartPrint_Mul(FAILURED_CODE,"\r\n");
        }
        else
        {
          SET_GREEN_LED_OFF();  //Set Green Led Off
          osi_Save_Data_Reset();  //Nor Flash Memory Chip Reset
          osi_UartPrint_Mul(SUCCESSED_CODE,"\r\n");
        }
        break;
      }
      default:
      {
        break;
      }
    } 
    uart_pares_status = 0;
    iLen=0;
  }
}


/*******************************************************************************
                                      END         
*******************************************************************************/




