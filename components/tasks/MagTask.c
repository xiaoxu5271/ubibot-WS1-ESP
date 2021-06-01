/*******************************************************************************
  * @file       Mag Sensor Application Task
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
#include "MsgType.h"

#ifdef MAG_SENSOR

#include "stdbool.h"
#include <stdlib.h>
#include "osi.h"
#include "rom_map.h"
#include "utils.h"
#include "at24c08.h"
#include "MagTask.h"
#include "PeripheralDriver.h"

extern OsiSyncObj_t xBinary4; //For Magnetic Sensor Task
extern OsiMsgQ_t xQueue0;     //Used for cjson and memory save

extern volatile uint8_t door_status;
extern volatile bool data_post;     //need post data immediately
extern volatile uint8_t fn_mag_int; //0:no data save,1:save data,2:save data and post
extern float f9_a, f9_b;
/*******************************************************************************
//Magnetic Sensor Status
*******************************************************************************/
uint8_t MagSensor_Status(void)
{
  if (GPIOPinRead(MAG_PORT, MAG_PIN))
  {
    return DOOR_OPEN;
  }
  else
  {
    return DOOR_CLOSED;
  }
}

/*******************************************************************************
//Magnetic Sensor Task
*******************************************************************************/
void MagneticSensorTask(void *pvParameters)
{
  SensorMessage mMsg;

  for (;;)
  {
    // osi_SyncObjWait(&xBinary4,-1);        //Wait Magnetic Sensor Interrupt Message
    ulTaskNotifyTake(pdTRUE, -1);

    door_status = MagSensor_Status();

    osi_at24c08_write_byte(DOOR_STATUS_ADDR, door_status); //save door status

    if (fn_mag_int)
    {
      if (fn_mag_int == 2)
      {
        data_post = 1; //Need Post Data Immediately
      }

      mMsg.sensornum = MAG_NUM;                   //Message Number
      mMsg.sensorval = f9_a * door_status + f9_b; //Message Value
      xQueueSend(xQueue0, &mMsg, 0);              //Send Magnetic Sensor Data Message
    }
  }
}
#endif

/*******************************************************************************
                                      END         
*******************************************************************************/
