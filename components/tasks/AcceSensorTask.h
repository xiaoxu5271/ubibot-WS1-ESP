/*******************************************************************************
  * @file       Acce Sensor Application Task    
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

#define ACCE_PORT       GPIOA2_BASE
#define ACCE_PIN        GPIO_PIN_1


/*******************************************************************************
 FUNCTION PROTOTYPES
*******************************************************************************/
extern uint8_t osi_adx345_readReg(uint8_t addr);  //adx345 read register with locked

extern void acce_sensor_reset(void);  //reset the acce sensor

extern void AcceSensor_Int_Task(void *pvParameters);  //Acceleration Interrupt Application Task

extern void AccelerationSensorTask(void *pvParameters);  //ACCELERATION SENSOR TASK


/*******************************************************************************
                                      END         
*******************************************************************************/




