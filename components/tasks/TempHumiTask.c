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
#include <stdlib.h>
#include "osi.h"
#include "rom_map.h"
#include "utils.h"
#include "MsgType.h"
#include "sht30dis.h"
#include "ht1621.h"
#include "PeripheralDriver.h"

extern OsiMsgQ_t        xQueue0;        //Used for cjson and memory save
extern OsiSyncObj_t     xBinary2;       //For Temp&Humi Sensor Task

#ifdef USE_LCD
  extern OsiSyncObj_t     xMutex6;      //Used for Ht1621 LCD Driver
#endif

extern float f1_a,f1_b,f2_a,f2_b;
  
/*******************************************************************************
//Temperature and Humility Sensor Task 
*******************************************************************************/
void TempHumiSensorTask(void *pvParameters)
{
  float tempvalue;
  float humivalue;
  SensorMessage thMsg;
  
  for(;;)
  {
    osi_SyncObjWait(&xBinary2,OSI_WAIT_FOREVER);  //Wait Sensor Task Operate Message
    
    osi_sht30_SingleShotMeasure(&tempvalue,&humivalue);  //read temperature humility data

    if(tempvalue!=ERROR_CODE)
    {
      #ifdef USE_LCD
        osi_SyncObjWait(&xMutex6,OSI_WAIT_FOREVER);     //LCD Semaphore Take
        Ht1621_Display_Temp_Val(tempvalue,0);           //Display Temprature value
        osi_SyncObjSignal(&xMutex6);                    //LCD Semaphore Give
      #endif
        
      thMsg.sensornum=TEMP_NUM;  //Message Number
      thMsg.sensorval=f1_a*tempvalue + f1_b;  //Message Value
      osi_MsgQWrite(&xQueue0,&thMsg,OSI_NO_WAIT);  //Send Temperature Data Message
    }
    
    if(humivalue!=ERROR_CODE)
    {
      #ifdef USE_LCD
        osi_SyncObjWait(&xMutex6,OSI_WAIT_FOREVER);     //LCD Semaphore Take
        Ht1621_Display_Humi_Val((uint8_t)humivalue);    //Display Humility value
        osi_SyncObjSignal(&xMutex6);                    //LCD Semaphore Give
      #endif
        
      thMsg.sensornum=HUMI_NUM;  //Message Number
      thMsg.sensorval=f2_a*humivalue + f2_b;  //Message Value
      osi_MsgQWrite(&xQueue0,&thMsg,OSI_NO_WAIT);  //Send Humility Data Message
    }
  }
}


/*******************************************************************************
                                      END         
*******************************************************************************/




