/******************************************************************************/
/******************************************************************************/
/************************** ds1308 time application ***************************/
/******************************************************************************/
/******************************************************************************/

#include "stdint.h"

extern void DS1308_Init(void);
extern void Read_UTCtime(char *buffer);
extern void Update_UTCtime(char *time);
extern uint32_t DS1308_ReadTime(void);
extern void DS1302_UpdateTime(uint32_t nowtime);






