/*******************************************************************************
  * @file       Nor Flash Data Deleted Application Task   
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
#include "stdint.h"
#include "stdbool.h"
#include <stdlib.h>
#include "osi.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "rom_map.h"
#include "gpio.h"
#include "utils.h"
#include "w25q128.h"
#include "at24c08.h"
#include "MsgType.h"
#include "PeripheralDriver.h"

extern volatile uint8_t save_addr_flag;
extern volatile unsigned long POST_NUM;
extern volatile unsigned long DELETE_ADDR, POST_ADDR, WRITE_ADDR;
extern OsiSyncObj_t xMutex1;   //Used for SPI Lock
extern OsiSyncObj_t xBinary11; //For Memory Delete Task

/*******************************************************************************
//Erase Flash 4k memory
*******************************************************************************/
static void N25q_EraseMemory(unsigned long addr)
{
  osi_SyncObjWait(&xMutex1, OSI_WAIT_FOREVER); //SPI Semaphore Take

  w25q_EraseSubsector(addr); //erase the subsector

  osi_SyncObjSignal(&xMutex1); //SPI Semaphore Give

  osi_EnterCritical(); //enter critical

  DELETE_ADDR += 4096;

  osi_ExitCritical(0); //exit critical

  if (DELETE_ADDR >= Memory_Max_Addr)
  {
    DELETE_ADDR = 0; //subsector 0
  }
  if (save_addr_flag == 0)
  {
    osi_at24c08_write(DATA_DELETE_ADDR1, DELETE_ADDR); //save data delete address
  }
  else
  {
    osi_at24c08_write(DATA_DELETE_ADDR2, DELETE_ADDR); //save data delete address
  }

#ifdef DEBUG

  osi_UartPrint_Val("DELETE_ADDR:", DELETE_ADDR); //print the data delete address

#endif
}

/*******************************************************************************
//Erase memory
*******************************************************************************/
void osi_Erase_Memory(void)
{
  uint16_t i_sub, n_sub;
  unsigned long d_addr, p_addr, w_addr;

  if ((POST_NUM >= Memory_Max_Addr) || (WRITE_ADDR > Memory_Max_Addr) || (POST_ADDR > Memory_Max_Addr) || (DELETE_ADDR > Memory_Max_Addr))
  {
    osi_Save_Data_Reset(); //Nor Flash Memory Chip Reset
  }
  else
  {
    d_addr = DELETE_ADDR; //delete pointer variable value

    p_addr = POST_ADDR; //post pointer variable value

    w_addr = WRITE_ADDR; //write pointer variable value

    if (((w_addr >= p_addr) && (w_addr >= d_addr) && (p_addr >= d_addr)) || ((p_addr >= d_addr) && (p_addr >= w_addr) && (d_addr >= w_addr))) //p_addr >= d_addr
    {
      n_sub = (p_addr - d_addr) / 4096; //delete 4k/a subsector

      for (i_sub = 0; i_sub < n_sub; i_sub++)
      {
        N25q_EraseMemory(DELETE_ADDR);
      }
    }
    else if ((d_addr >= w_addr) && (d_addr >= p_addr) && (w_addr >= p_addr)) //d_addr>=p_addr
    {
      n_sub = (Memory_Max_Addr + 1 - d_addr) / 4096; //max 0x00ffffff,4k a subsector

      for (i_sub = 0; i_sub < n_sub; i_sub++)
      {
        N25q_EraseMemory(DELETE_ADDR);
      }

      DELETE_ADDR = 0; //subsector 0

      n_sub = p_addr / 4096; //4k/a subsector

      for (i_sub = 0; i_sub < n_sub; i_sub++)
      {
        N25q_EraseMemory(DELETE_ADDR);
      }
    }
    else //pointer variable wrong,delete all sensor data
    {
      osi_Save_Data_Reset(); //Nor Flash Memory Chip Reset
    }
  }
}

/*******************************************************************************
//nor flash memory chip,data deleted task
*******************************************************************************/
void Memory_DeleteTask(void *pvParameters)
{
  for (;;)
  {
    // osi_SyncObjWait(&xBinary11,OSI_WAIT_FOREVER);  //wait task start message
    ulTaskNotifyTake(pdTRUE, -1);

    osi_Erase_Memory(); //Erase memory
  }
}

/*******************************************************************************
                                      END         
*******************************************************************************/
