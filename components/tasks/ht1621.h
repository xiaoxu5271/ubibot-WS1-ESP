/*******************************************************************************
  * @file       HT1621 DRIVER APPLICATION      
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

#define BIAS                    0x28  //1/3bias 4common
#define RCosc                   0x18
#define SYSEN                   0x01
#define LCDOFF                  0x02
#define LCDON                   0x03

#define HT1621_CS_GPIO          GPIOA3_BASE        
#define HT1621_CS_PIN           GPIO_PIN_4

#define HT1621_WR_GPIO          GPIOA1_BASE
#define HT1621_WR_PIN           GPIO_PIN_1

#define HT1621_RD_GPIO          GPIOA1_BASE
#define HT1621_RD_PIN           GPIO_PIN_1

#define HT1621_DATA_GPIO        GPIOA1_BASE
#define HT1621_DATA_PIN         GPIO_PIN_2

#define HT1621_CS_ON()          MAP_GPIOPinWrite(HT1621_CS_GPIO ,HT1621_CS_PIN,HT1621_CS_PIN);
#define HT1621_CS_OFF()         MAP_GPIOPinWrite(HT1621_CS_GPIO ,HT1621_CS_PIN,0x00);

#define HT1621_WR_ON()          MAP_GPIOPinWrite(HT1621_WR_GPIO ,HT1621_WR_PIN,HT1621_WR_PIN);
#define HT1621_WR_OFF()         MAP_GPIOPinWrite(HT1621_WR_GPIO ,HT1621_WR_PIN,0x00);

#define HT1621_RD_ON()          MAP_GPIOPinWrite(HT1621_RD_GPIO ,HT1621_RD_PIN,HT1621_RD_PIN);
#define HT1621_RD_OFF()         MAP_GPIOPinWrite(HT1621_RD_GPIO ,HT1621_RD_PIN,0x00);

#define HT1621_DATA_OUT()       MAP_GPIODirModeSet(HT1621_DATA_GPIO,HT1621_DATA_PIN, GPIO_DIR_MODE_OUT);  //set sda pin out mode
#define HT1621_DATA_IN()        MAP_GPIODirModeSet(HT1621_DATA_GPIO,HT1621_DATA_PIN, GPIO_DIR_MODE_IN);  //set sda pin in mode

#define HT1621_DATA_ON()        MAP_GPIOPinWrite(HT1621_DATA_GPIO ,HT1621_DATA_PIN,HT1621_DATA_PIN);
#define HT1621_DATA_OFF()       MAP_GPIOPinWrite(HT1621_DATA_GPIO ,HT1621_DATA_PIN,0x00);


/*******************************************************************************
 FUNCTION PROTOTYPES
*******************************************************************************/
extern void Ht1621_SendCmd(uint8_t command);

extern void Ht1621_Write(uint8_t addr,uint8_t data);

extern uint8_t Ht1621_Read(uint8_t addr);

extern void Ht1621_Off(void);

extern void Ht1621_On(void);

extern void Ht1621_Display_Temp_Val(float temp_val,bool ext_flag);  //Display the temprature value

extern void Ht1621_Display_Humi_Val(uint8_t humi_val);  //Display the Humility value

extern void Ht1621_Display_Err_Val(uint8_t err_val);  //Display the error value

extern void Ht1621_Display_Light_Val(float light_val,bool T1_val,bool T2_val,bool T3_val,uint8_t l_val);  //Display the light value


/*******************************************************************************
                                      END         
*******************************************************************************/




