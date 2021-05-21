

//
//header include//
//
#include "iic.h"

//
//ds1302 register address//
//
#define w_sec_addr      0x80
#define r_sec_addr      0x81
#define w_min_addr      0x82
#define r_min_addr      0x83
#define w_hour_addr     0x84
#define r_hour_addr     0x85
#define w_date_addr     0x86
#define r_date_addr     0x87
#define w_month_addr    0x88
#define r_month_addr    0x89
#define w_year_addr     0x8c
#define r_year_addr     0x8d

#define w_write_addr    0x8e
#define r_write_addr    0x8f


//
//only need change gpio port and pin suit other application//
//
#define ds_rst_gpio    GPIOA1_BASE     //RST GPIO PORT//
#define ds_rst_pin     GPIO_PIN_0      //RST GPIO PIN//

//
//gpio pin operation,set on or off//
//
#define RST_ON()        MAP_GPIOPinWrite(ds_rst_gpio,ds_rst_pin,ds_rst_pin)	//RST high//
#define RST_OFF() 	MAP_GPIOPinWrite(ds_rst_gpio,ds_rst_pin,0x00)	//RST low//


//
//read ds1302 register//
//return:register value//
extern uint8_t ds1302_ReadReg(uint8_t addr);


//
//ds1302 init//
//start xtal//
//
extern void DS1302_Init(void);


//
//ds1302 read UTC time//
//
extern void Read_UTCtime(char *buffer);


//
//ds1302 update time//
//
extern void Update_UTCtime(char *time);


//
//ds1302 read Unix time//
//return: Unix time//
//
extern unsigned long Read_UnixTime(void);
	
//---end---//

