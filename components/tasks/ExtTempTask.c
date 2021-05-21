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
#include <stdlib.h>
#include "osi.h"
#include "rom_map.h"
#include "utils.h"
#include "MsgType.h"
#include "ds18b20.h"
#include "ht1621.h"
#include "PeripheralDriver.h"

extern OsiSyncObj_t     xBinary6;       //For external Temprature Measure
extern OsiMsgQ_t        xQueue0;        //Used for cjson and memory save

#ifdef USE_LCD
  extern OsiSyncObj_t     xMutex6;        //Used for Ht1621 LCD Driver
#endif

extern float f8_a,f8_b;

/*******************************************************************************
//External temperature measure task 
*******************************************************************************/
void ExtTempMeasureTask(void *pvParameters)
{
  float water_temp;
  SensorMessage wMsg;
  
  for(;;)
  {
    osi_SyncObjWait(&xBinary6,OSI_WAIT_FOREVER);  //Wait Timer Interrupt Message
    
    water_temp=osi_ds18b20_get_temp();  //measure the temperature

    if(water_temp!=ERROR_CODE)
    {
      #ifdef USE_LCD
        osi_SyncObjWait(&xMutex6,OSI_WAIT_FOREVER);   //LCD Semaphore Take
        Ht1621_Display_Temp_Val(water_temp,1);
        osi_SyncObjSignal(&xMutex6);                  //LCD Semaphore Give
      #endif
      
      wMsg.sensornum=EXT_NUM;           //Message Number 
      wMsg.sensorval=f8_a*water_temp + f8_b;        //Message Value
      osi_MsgQWrite(&xQueue0,&wMsg,OSI_NO_WAIT);   //send noise data message
    }
  }
}


/*******************************************************************************
                                      END         
*******************************************************************************/




