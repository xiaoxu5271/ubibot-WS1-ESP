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
#include <stdlib.h>
#include "osi.h"
#include "rom_map.h"
#include "utils.h"
#include "MsgType.h"
#include "opt3001.h"
#include "ht1621.h"
#include "PeripheralDriver.h"

extern OsiSyncObj_t xBinary3; //For Light Sensor Task
extern OsiMsgQ_t xQueue0;     //Used for cjson and memory save

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
    // osi_SyncObjWait(&xBinary3,OSI_WAIT_FOREVER);  //Wait Timer Interrupt Message
    ulTaskNotifyTake(pdTRUE, -1);

    osi_OPT3001_value(&lightvalue); //Read Light Value

    if (lightvalue != ERROR_CODE)
    {
#ifdef USE_LCD
      osi_SyncObjWait(&xMutex6, OSI_WAIT_FOREVER); //LCD Semaphore Take
      Ht1621_Display_Light_Val(lightvalue, 0, 0, 0, power_flag);
      osi_SyncObjSignal(&xMutex6); //LCD Semaphore Give
#endif

      lMsg.sensornum = LIGHT_NUM;                  //Message Number
      lMsg.sensorval = f3_a * lightvalue + f3_b;   //Message Value
      osi_MsgQWrite(&xQueue0, &lMsg, OSI_NO_WAIT); //Send Light Data Message
    }
  }
}

/*******************************************************************************
                                      END         
*******************************************************************************/
