/*******************************************************************************
  * @file           
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
#include "freertos/task.h"
#include "freertos/event_groups.h"

//#define DEBUG
//#define DEBUG_SAVE      //memory save debug
//#define DEBUG_RESPONSE  //http respon debug
#define USE_WD
//#define USE_LCD
#define HIB_MODE
//#ifdef CHECK_WATER_MARK

#define SUCCESS 0
#define FAILURE -1

#define TEMP_NUM 1
#define HUMI_NUM 2
#define LIGHT_NUM 3
#define BAT_NUM 4
#define RSSI_NUM 5
#define ACCE_NUM 6
#define TAP_NUM 7
#define EXT_NUM 8
#define MAG_NUM 9

#define DEFAULT_FN_TH 300
#define DEFAULT_FN_LIGHT 300
#define DEFAULT_FN_MAG 86400
#define DEFAULT_FN_MAG_INT 2
#define DEFAULT_FN_ACC_TAP1 1
#define DEFAULT_FN_ACC_TAP2 1
#define DEFAULT_FN_ACC_ACT 1
#define DEFAULT_THRES_ACC_MIN 5
#define DEFAULT_FN_BT 300
#define DEFAULT_FN_EXT 300
#define DEFAULT_FN_BATTERY 10800 //3*60*60s
#define DEFAULT_FN_DP 900
#define DEFAULT_CG_DATA_LED 1
#define DEFAULT_NO_NET_FN 1
#define DEFAULT_WIFI_MODE 1

#define DEFAULT_CALI_A 1
#define DEFAULT_CALI_B 0

#define NO_NET_FN_DP 3600 //60*60s

#define DOOR_OPEN 1
#define DOOR_CLOSED 0
#define SYSTEM_ON "ON"   //Power ON
#define SYSTEM_OFF "OFF" //Power OFF

#define SEND_DATA_NUMBER 20
#define POST_DATA_NUMBER 2000

#define RESET_SLEEP_TIME 2      //2*0.1s
#define RESET_WATCH_DOG_TIME 50 //50*0.1s
#define MIN_SLP_TIME 1          //1s
#define MAX_SLP_TIME 86400      //24*60*60s

//#define HOST_PORT               80
//#define MAX_BUFF_SIZE           1460
//#define OSI_STACK_SIZE          1024
#define PORT_NUM 5001
#define WLAN_DEL_ALL_PROFILES 0xFF
#define HOST_HTTP_PORT_NUM 80
#define RESP_BUF_LEN 1024

#define MSG_SLP_VAL 0x01
#define MSG_WD_VAL 0x02

#define UPDATE_TIME_SIZE 30  //1min
#define SYS_RUN_TIMEOUT 1200 //60s
#define USB_TIME_OUT 100     //15min 750

#define DEFAULT_IP_ADDR1 101
#define DEFAULT_IP_ADDR2 201
#define DEFAULT_IP_ADDR3 30
#define DEFAULT_IP_ADDR4 5
#define DNS_WRONG_NUM -159
#define DEFAULT_HOST_NAME_IP 0 //USE HOST NAME

#define TEST_NUM 12345678
#define PRODUCT_URI "URI_P"
#define DATA_URI "URI_D"

#define ProductURI1 "/products/"
#define ProductURI2 "/devices/"
#define ProductURI3 "/activate"
#define DataURI1 "/update.json?execute=true&metadata=true&api_key="

#define SUCCESSED_CODE "{\"status\":0,\"code\":0}"
#define FAILURED_CODE "{\"status\":0,\"code\":4}"

#define FIRMWARENUM "ws1_e_v1.0.9"

#define MAP_UtilsDelay(num) ets_delay_us((uint32_t)(num * 3 / 40))

#define my_xTaskCreate(TaskCode, Name, Depth, Parameters, Priority, Handle) xTaskCreate(TaskCode, Name, Depth * 5, Parameters, Priority + 2, Handle)

EventGroupHandle_t Task_Group;
#define POST_TASK_BIT (1 << 0)       //
#define UPDATETIME_TASK_BIT (1 << 1) //
#define APIGET_TASK_BIT (1 << 2)     //
#define AP_MODE_END_BIT (1 << 3)     //
#define MAIN_INIT_BIT (1 << 4)       //
#define USB_MODE_BIT (1 << 5)        //
#define TIMER_CHECK_BIT (1 << 6)     //
#define BUTTON_INT_BIT (1 << 7)      //
#define SENTASK_2 (1 << 8)           //
#define SENTASK_3 (1 << 9)           //
#define SENTASK_5 (1 << 10)          //
#define SENTASK_6 (1 << 11)          //
#define SENTASK_7 (1 << 12)          //
#define SENTASK_12 (1 << 13)         //
#define SAVE_TASK_BIT (1 << 14)      //
#define DELE_TASK_BIT (1 << 15)      //
#define FREST_TASK_BIT (1 << 16)     //
#define USB_SET_BIT (1 << 17)        //
#define OTA_TASK_BIT (1 << 18)       //

#define ALL_BIT (POST_TASK_BIT |       \
                 UPDATETIME_TASK_BIT | \
                 APIGET_TASK_BIT |     \
                 AP_MODE_END_BIT |     \
                 MAIN_INIT_BIT |       \
                 USB_MODE_BIT |        \
                 TIMER_CHECK_BIT |     \
                 BUTTON_INT_BIT |      \
                 SENTASK_2 |           \
                 SENTASK_3 |           \
                 SENTASK_5 |           \
                 SENTASK_6 |           \
                 SENTASK_7 |           \
                 SENTASK_12 |          \
                 SAVE_TASK_BIT |       \
                 DELE_TASK_BIT |       \
                 FREST_TASK_BIT |      \
                 USB_SET_BIT |         \
                 OTA_TASK_BIT)

typedef struct
{
  uint8_t sensornum; //sensor field
  float sensorval;   //sensor value
} SensorMessage;

typedef struct
{
  uint8_t pid[64];
  uint8_t psn[64];
  uint8_t hst[64];
  uint8_t mac[64];
  uint8_t pfw[64];
  uint8_t ssid[64];
} web_product;

typedef struct
{
  uint8_t wrt[32];
  uint8_t cwf[32];
  uint8_t csf[32];
  uint8_t agf[32];
  uint8_t mde[32];
  uint8_t pde[32];
  uint8_t pdf[32];
  uint8_t swf[32];
  uint8_t wpw[32];
} web_errcode;

typedef struct
{
  uint8_t wst[64];
  uint8_t wpw[64];
} web_post_wifi;

typedef union
{
  float cali_val;
  unsigned long cali_buf;
} f_cali;

/*******************************************************************************
                                      END         
*******************************************************************************/
