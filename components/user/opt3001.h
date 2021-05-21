/*******************************************************************************
  * @file       OPT3001 Light Sensor DRIVER APPLICATION      
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


#define OPT3001_ADDR		0X45
#define Result			0x00
#define Configuration		0x01
#define LowLimit		0x02
#define HighLimit		0x03
#define ManufacturerID		0x7e
#define DeviceID		0x7f

#define ERROR_CODE              0xffff


/*******************************************************************************
 FUNCTION PROTOTYPES
*******************************************************************************/
extern short OPT3001_Init(void);  //opt3001 light sensor init

extern void OPT3001_value(float *lightvalue);  //measure the light value


/*******************************************************************************
                                      END         
*******************************************************************************/




