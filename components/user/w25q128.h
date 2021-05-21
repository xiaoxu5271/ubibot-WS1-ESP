/*******************************************************************************
  * @file       W25Q128 NOR FLASH CHIP DRIVER APPLICATION      
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

#define SAVE_DATA_SIZE          65
#define Memory_Min_Addr         0
#define Memory_Max_Addr         0xffffff

#define W25Q80                  0xef13
#define W25Q16                  0xef14
#define W25Q32                  0xef15
#define W25Q64                  0xef16
#define W25Q128                 0xef17

#define READ_STATUS_REGISTER    0x05
#define WRITE_STATUS_REGISTER   0x10
#define WRITE_ENABLE            0x06
#define W25Q_DEVICE_ID          0x90
#define READ_DATA               0x03
#define PAGE_PROGRAM            0x02
#define SECTOR_ERASE            0x20
#define CHIP_ERASE              0xc7
#define POWER_DOWN              0xb9
#define RELEASE_POWER_DOWN      0xab


/*******************************************************************************
 FUNCTION PROTOTYPES
*******************************************************************************/
extern short w25q_Init(void);  //init memory ic

extern uint8_t w25q_ReadReg(uint8_t com_val);  //w25q128 read register

extern void w25q_WriteReg(uint8_t com_val,uint8_t value);  //w25q128 write register

extern void w25q_WriteCommand(uint8_t com_val);  //w25q128 write register

extern short w25q_ReadData(uint32_t addr,char *buffer,uint8_t size,uint8_t *read_size);  //w25q128 read data

extern void w25q_WriteData(uint32_t addr,char *buffer,uint8_t Size);  //w25q128 write data

extern void w25q_EraseSubsector(uint32_t addr);  //w25q128 erase subsector-4k

extern void w25q_EraseChip(void);  //w25q128 erase chip

extern void w25q_PowerDown(void);  //w25q128 power down

extern void w25q_WakeUp(void);  //w25q128 wake up

extern void osi_w25q_Write_Addr_Check(unsigned long w_Addr);  //w25q128 check write address with locked


/*******************************************************************************
                                      END         
*******************************************************************************/




