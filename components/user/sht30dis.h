/*******************************************************************************
  * @file       Temperature and Humility Sensor Application Driver     
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
#include "freertos/FreeRTOS.h"

#define ERROR_CODE 0xffff

/*******************************************************************************
 FUNCTION PROTOTYPES
*******************************************************************************/
extern short sht30dis_reset(void); //sht30dis sensor reset

extern void sht30_SingleShotMeasure(float *temp, float *humi); //sht30dis single shot measure
int sht30_SS_get_value(float *temp, float *humi);

/*******************************************************************************
                                      END         
*******************************************************************************/
