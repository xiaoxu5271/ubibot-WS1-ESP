/*******************************************************************************
  * @file       AT24C08 EEPROM CHIP DRIVER APPLICATION     
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


/*******************************************************************************
 at24c08 chip address
*******************************************************************************/
//#define at24c08_addr0           0x50    //8bit:0xa0,0-255
//#define at24c08_addr1           0x51    //8bit:0xa2,256-511
//#define at24c08_addr2           0x52    //8bit:0xa4,512-767
//#define at24c08_addr3           0x53    //8bit:0xa6,768-1023

#define at24c08_addr0           0x54    //8bit:0xa0,0-255
#define at24c08_addr1           0x55    //8bit:0xa2,256-511
#define at24c08_addr2           0x56    //8bit:0xa4,512-767
#define at24c08_addr3           0x57    //8bit:0xa6,768-1023

#define DATA_AMOUNT_ADDR1       0       //unsigned long + 2(crc)
#define DATA_WRITE_ADDR1        8       //unsigned long + 2(crc)
#define DATA_POST_ADDR1         16      //unsigned long + 2(crc)
#define DATA_DELETE_ADDR1       24      //unsigned long + 2(crc)

#define DATA_AMOUNT_ADDR2       32      //unsigned long + 2(crc)
#define DATA_WRITE_ADDR2        40      //unsigned long + 2(crc)
#define DATA_POST_ADDR2         48      //unsigned long + 2(crc)
#define DATA_DELETE_ADDR2       56      //unsigned long + 2(crc)

#define FN_TH_ADDR              64      //unsigned long + 2(crc)
#define FN_TH_T_ADDR            72      //unsigned long + 2(crc)
#define FN_LIGHT_ADDR           80      //unsigned long + 2(crc)
#define FN_LIGHT_T_ADDR         88      //unsigned long + 2(crc)
#define FN_MAG_ADDR             96      //unsigned long + 2(crc)
#define FN_MAG_T_ADDR           104     //unsigned long + 2(crc)
#define FN_BT_ADDR              112     //unsigned long + 2(crc)
#define FN_BT_T_ADDR            120     //unsigned long + 2(crc)
#define FN_EXT_ADDR             128     //unsigned long + 2(crc)
#define FN_EXT_T_ADDR           136     //unsigned long + 2(crc)
#define FN_BATTERY_ADDR         144     //unsigned long + 2(crc)
#define FN_BATTERY_T_ADDR       152     //unsigned long + 2(crc)
#define FN_DP_ADDR              160     //unsigned long + 2(crc)
#define FN_DP_T_ADDR            168     //unsigned long + 2(crc)

#define FN_MAG_INT_ADDR         176     //unsigned char + 2(crc)
#define FN_ACC_TAP1_ADDR        180     //unsigned char + 2(crc)
#define FN_ACC_TAP2_ADDR        184     //unsigned char + 2(crc)
#define FN_ACC_ACT_ADDR         188     //unsigned char + 2(crc)
#define THRES_ACC_MIN_ADDR      192     //unsigned char + 2(crc)
#define CG_LED_DATA_ADDR        196     //unsigned char + 2(crc)

#define DATA_SAVE_FLAG_ADDR     200     //unsigned char + 2(crc)
#define DOOR_STATUS_ADDR        204     //unsigned char + 2(crc)
#define POWER_DATA_ADDR         208     //unsigned char + 2(crc)
#define USB_FLAG_ADDR           212     //unsigned char + 2(crc)
#define HOST_NAME_IP_ADDR       216     //unsigned char + 2(crc)
#define NO_NET_FN_ADDR          220     //unsigned char + 2(crc)
#define WIFI_MODE_ADDR          224     //unsigned char + 2(crc)

#define MAC_ADDR                232     //8 byte
#define SSID_LEN_ADDR           240     //unsigned char + 2(crc)
#define PASSWORD_LEN_ADDR       244     //unsigned char + 2(crc)
#define SYSTEM_STATUS_ADDR      248     //8 byte

#define PRODUCT_ID_ADDR         256     //32 byte
#define SERISE_NUM_ADDR         288     //32 byte
#define HOST_ADDR               320     //32 byte
#define HOST_IP_ADDR            352     //8 byte
#define SSID_FLAG_ADDR          360     //8 byte
#define SSID_ADDR               368     //32 byte
#define PASSWORD_ADDR           400     //password 64 bute
#define SECTYPE_ADDR            464     //8 byte

#define PRODUCTURI_FLAG_ADDR    512     //8 byte
//#define PRODUCTURI_ADDR         520     //120 byte
#define DHCP_MODE_ADDR          520     //4 byte
#define STATIC_IP_ADDR          528     //8 byte
#define STATIC_SN_ADDR          536     //8 byte
#define STATIC_GW_ADDR          544     //8 byte
#define STATIC_DNS_ADDR         552     //8 byte
#define STATIC_SDNS_ADDR        560     //8 byte
#define HOST_PORT_ADDR          568     //8 byte

#define F1_A_ADDR               576
#define F1_B_ADDR               584
#define F2_A_ADDR               592
#define F2_B_ADDR               600
#define F3_A_ADDR               608
#define F3_B_ADDR               616
#define F4_A_ADDR               624
#define F4_B_ADDR               632
#define F5_A_ADDR               640
#define F5_B_ADDR               648
#define F6_A_ADDR               656
#define F6_B_ADDR               664
#define F7_A_ADDR               672
#define F7_B_ADDR               680
#define F8_A_ADDR               688
#define F8_B_ADDR               696
#define F9_A_ADDR               704
#define F9_B_ADDR               712
#define F10_A_ADDR              720
#define F10_B_ADDR              728

#define CHANNEL_ID_ADDR         736     //unsigned long + 2(crc)
#define USER_ID_ADDR            744     //44 byte

#define DATAURI_FLAG_ADDR       788     //8 byte
#define DATAURI_ADDR            796     //120 byte

//#define MALLOC_FLR_ERR          920     //unsigned long + 2(crc)
//#define STATCK_OVER_FLOW        928     //unsigned long + 2(crc)
#define SCN_WF_FLR_ERR          920     //unsigned long + 2(crc)
#define WF_PWD_WR_ERR           928     //unsigned long + 2(crc)
#define WD_RESET_ERR            936     //unsigned long + 2(crc)          
#define CNT_WIFI_FLR_ERR        944     //unsigned long + 2(crc)
#define RSL_HOST_FLR_ERR        952     //unsigned long + 2(crc)
#define CNT_SERVER_FLR_ERR      960     //unsigned long + 2(crc)
#define API_GET_FLR_ERR         968     //unsigned long + 2(crc)
#define UPDATE_TIME_FLR_ERR     976     //unsigned long + 2(crc)
#define MEMORY_SAVE_DATA_ERR    984     //unsigned long + 2(crc)
#define POST_DATA_JSON_FAULT    992     //unsigned long + 2(crc)
#define POST_DATA_FLR_ERR       1000    //unsigned long + 2(crc)

#define LAST_UPDATE_TIME_ADDR   1016    //unsigned long + 2(crc)


/*******************************************************************************
//FUNCTION PROTOTYPES
*******************************************************************************/
extern void at24c08_write_byte(uint16_t reg_addr,uint8_t val);  //write a byte in at24c08

extern uint8_t at24c08_read_byte(uint16_t reg_addr);  //read a byte in at24c08

extern void at24c08_write(uint16_t reg_addr,unsigned long val);  //write unsigned long data 

extern  unsigned long at24c08_read(uint16_t reg_addr);  //read unsigned long data 

extern void at24c08_WriteData(uint16_t addr,uint8_t *buf,uint8_t size,bool end_flag);  //at24c08 write data

extern void at24c08_ReadData(uint16_t addr,uint8_t *buf,uint8_t size,bool end_flag);  //at24c08 read data


/*******************************************************************************
                                      END         
*******************************************************************************/




