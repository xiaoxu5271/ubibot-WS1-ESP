/******************************************************************************/
/******************************************************************************/
/******************** n25q512a memory chip application ************************/
/******************************************************************************/
/******************************************************************************/

#include "stdint.h"

extern uint32_t N25q_ReadId(void);
extern void N25q_WritePage(uint32_t addr,uint8_t *buffer,uint16_t Size);
extern void N25q_ReadMemory(uint32_t addr,uint8_t *buffer,uint16_t size);
extern void N25q_EraseSubsector(uint32_t addr);
extern void N25q_EraseSector(uint32_t addr);
extern void N25q_EraseChip(void);

