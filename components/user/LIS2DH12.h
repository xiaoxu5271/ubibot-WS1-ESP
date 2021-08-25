/*******************************************************************************
  * @file       LIS2DH12 Sensor Application      
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

#define LIS2DH12_ADDR 0x19
#define LIS2DH12_ID 0x33

extern void lis2dh12_device_id_get(uint8_t *buff);
extern void lis2dh12_init(void);
extern float lis2dh12_data(void);
extern void lis2dh12_detect_init(void);
extern void lis2dh12_RD_xyz(float *x_val, float *y_val, float *z_val);

extern uint8_t lis2dh12_clear_int(void);
extern void lis2dh12_Power_Down(void);
