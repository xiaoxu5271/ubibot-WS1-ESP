/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_sleep.h"
#include "esp_task_wdt.h"

#include "app_config.h"

/*******************************************************************************
  * @file       MAIN FUNCTION PROTOTYPES      
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
#include "string.h"
#include "math.h"

// #include "stdlib.h"
// #include "FreeRTOS.h"
// #include "task.h"
// #include "osi.h" //Free-RTOS includes

// #include "hw_ints.h" //driverlib includes
// #include "hw_types.h"
// #include "hw_memmap.h"
// #include "interrupt.h"
// #include "prcm.h"
// #include "utils.h"
// #include "rom.h"
// #include "rom_map.h"

// #include "simplelink.h" //Simplelink includes

// #include "cc_types.h" //middleware includes
// #include "rtc_hal.h"
// #include "gpio_hal.h"
// #include "uart_drv.h"
// #include "cc_timer.h"
// #include "cc_pm_ops.h"

// #include "uart.h" //Common driver
// #include "uart_if.h"
// #include "timer.h"
// #include "timer_if.h"
// #include "common.h"
// #include "wdt.h"
// #include "wdt_if.h"
// #include "pinmux.h"

// #include <http/client/httpcli.h> //HTTP Client lib
// #include <http/client/common.h>

#include "cJSON.h" //JSON Parser
#include "JsonParse.h"

#include "iic.h" //user driver header
#include "user_spi.h"
#include "ht1621.h"
#include "sht30dis.h"
#include "opt3001.h"
#include "at24c08.h"
#include "w25q128.h"
#include "adxl345.h"
#include "PCF8563.h"
#include "MsgType.h"
#include "UartTask.h"
#include "TempHumiTask.h"
#include "LightTask.h"
#include "MagTask.h"
#include "AcceSensorTask.h"
#include "ExtTempTask.h"
#include "DataSaveTask.h"
#include "DataDeleteTask.h"
#include "HttpClientTask.h"
#include "PowerMeasureTask.h"
#include "PeripheralDriver.h"

/*******************************************************************************
//Global Variables
*******************************************************************************/
TaskHandle_t xBinary0; //For DataPostTask interrupt
TaskHandle_t xBinary1; //For GPIO interrupt task
TaskHandle_t xBinary2; //For Temp&Humi Sensor Task
TaskHandle_t xBinary3; //For Light Sensor Task

#ifdef MAG_SENSOR
TaskHandle_t xBinary4; //For Magnetic Sensor Task
#endif

TaskHandle_t xBinary5; //For Acceleration SensorTask
TaskHandle_t xBinary6; //For external water Temprature Measure
TaskHandle_t xBinary7; //For Power Measure Task
TaskHandle_t xBinary8; //For body Temprature Measure
TaskHandle_t xBinary9; //For Timer interrupt task
//TaskHandle_t            xBinary10;      //For UART interrupt Task
TaskHandle_t UART_xBinary; //For UART Parse Task
TaskHandle_t xBinary11;	   //For Memory Delete Task
TaskHandle_t xBinary12;	   //For Acce Sensor Interrupt Task
TaskHandle_t xBinary13;	   //Task End Check
#ifdef CHECK_WATER_MARK
TaskHandle_t xBinary14; //WaterMark_Check
#endif
TaskHandle_t xBinary15; //USB Mode
TaskHandle_t xBinary16; //USB activate
TaskHandle_t xBinary17; //Internet Application

SemaphoreHandle_t xMutex1; //Used for SPI Lock
SemaphoreHandle_t xMutex2; //Used for SimpleLink Lock
SemaphoreHandle_t xMutex3; //Used for cJSON Lock
SemaphoreHandle_t xMutex4; //Used for UART Lock
SemaphoreHandle_t xMutex5; //Used for Post_Data_Buffer Lock
#ifdef USE_LCD
SemaphoreHandle_t xMutex6; //Used for Ht1621 LCD Driver
#endif

QueueHandle_t xQueue0;		 //Used for cjson and memory save
QueueHandle_t xQueue1;		 //Used for LED control task
QueueHandle_t xQueue2;		 //Used for bell conctrol task
QueueHandle_t xQueue3;		 //Used for WatchDog and Sleep Timer Application
QueueHandle_t HttpMsg_Queue; //Used for http server resolve

//OsiTaskHandle           R_LED_TaskHandle;
TaskHandle_t GR_LED_TaskHandle;
TaskHandle_t GR_LED_FAST_TaskHandle;
//OsiTaskHandle           UpdateTimeTaskHandle;
//OsiTaskHandle           UartParseTaskHandle;
//OsiTaskHandle           ButtonPush_Int_TaskHandle;
//OsiTaskHandle           Green_LedControl_TaskHandle;
//OsiTaskHandle           Bell_Control_TaskHandle;
//OsiTaskHandle           TempHumiTaskHandle;
//OsiTaskHandle           LightTaskHandle;
//OsiTaskHandle           MagneticSensorTaskHandle;
//OsiTaskHandle           ExtTempMeasureTaskHandle;
//OsiTaskHandle           PowerMeasureTaskHandle;
//OsiTaskHandle           DataSaveTaskHandle;
//OsiTaskHandle           DataPostTaskHandle;
//OsiTaskHandle           Memory_DeleteTaskHandle;
//OsiTaskHandle           Timer_Check_TaskHandle;
//OsiTaskHandle           F_RESET_TaskHandle;

volatile unsigned long g_ulStatus = 0; //SimpleLink Status
unsigned long g_ulStaIp = 0;
unsigned long g_ulDestinationIP;					//IP address of destination server
unsigned long g_ulGatewayIP = 0;					//Network Gateway IP address
unsigned char g_ucConnectionSSID[SSID_LEN_MAX + 1]; //Connection SSID
unsigned char g_ucConnectionBSSID[BSSID_LEN_MAX];	//Connection BSSID MAC 地址
//unsigned char           g_buff[MAX_BUFF_SIZE+1];

volatile bool POST_TASK_END_FLAG = 0;
volatile bool UPDATETIME_TASK_END_FLAG = 0;
volatile bool APIGET_TASK_END_FLAG = 0;
volatile bool AP_MODE_END_FLAG = 0;
volatile bool AP_MODE_END = 0;

volatile bool ap_mode_status = 0;
volatile bool f_reset_status = 0;
volatile bool data_post = 0; //need post data immediately

#ifdef MAG_SENSOR
volatile uint8_t door_status;
volatile uint8_t fn_mag_int; //0:no data save,1:save data,2:save data and post
#endif

volatile bool acce_act = 0;		  //acceleration sensor active
volatile bool PostSetData = 0;	  //post url data
volatile bool update_time = 0;	  //update time
volatile bool usb_status_val = 0; //usb status

volatile uint8_t fn_acc_tap1;	//0:closed single tap,1:single tap interrupt,2:single tap interrupt and post
volatile uint8_t fn_acc_tap2;	//0:closed double tap,1:double tap interrupt,2:double tap interrupt and post
volatile uint8_t fn_acc_act;	//0:cloused act interrupt,1:act interrupt
volatile uint8_t thres_acc_min; //min value for acceleration sensor act interrupt
volatile uint8_t cg_data_led;	//Led ON or OFF when data post
volatile uint8_t no_net_fn;		//change fn_dp when no net
volatile uint8_t wifi_mode;		//1:connect wifi immediately,0:connect wifi when scaned

volatile uint16_t sys_run_time = 0;

#ifdef USE_LCD
volatile uint8_t power_flag;
#endif

volatile uint8_t SYS_WD_SEC = 0;	 //Watch Dog Timer mSec
volatile unsigned long slp_t_ms = 0; //Watch Dog Timer mSec
volatile unsigned long sleep_time = 0;
unsigned long now_unix_t;					 //now unix time
volatile unsigned long fn_th, fn_th_t;		 //Temp&Humi sensor frequence
volatile unsigned long fn_light, fn_light_t; //Light sensor frequence
#ifdef MAG_SENSOR
volatile unsigned long fn_mag, fn_mag_t; //Magnetic sensor frequence
#endif
volatile unsigned long fn_bt, fn_bt_t;			 //Body Temperature Measure frequence
volatile unsigned long fn_ext, fn_ext_t;		 //Noise Measure frequence
volatile unsigned long fn_battery, fn_battery_t; //PowerMeasure frequence
volatile unsigned long fn_dp, fn_dp_t;			 //data post frequence

float f1_a, f1_b, f2_a, f2_b, f3_a, f3_b, f4_a, f4_b, f5_a, f5_b, f6_a, f6_b, f7_a, f7_b, f8_a, f8_b, f9_a, f9_b, f10_a, f10_b;

volatile unsigned long POST_NUM;
volatile unsigned long DELETE_ADDR, POST_ADDR, WRITE_ADDR;

extern uint16_t iLen;
extern char UartGet[UART_REV_BUF_LEN];
extern bool uart_pares_status;
short flash_set_val = -1;

/*******************************************************************************
//Buffer Size
*******************************************************************************/
volatile uint8_t usb_status;
volatile uint8_t save_addr_flag;

uint8_t Host_Flag = 0;
uint8_t HOST_IP[4];
char SEC_TYPE[8];
char SSID_NAME[32];
char PASS_WORD[64];
char HOST_NAME[64];
//char    UartGet[255];
char POST_REQUEST_URI[250];
char Post_Data_Buffer[4096];
char Read_Response_Buffer[RESP_BUF_LEN];

/*******************************************************************************
//Local Function Prototypes
*******************************************************************************/
// extern int platform_init();

// extern void lp3p0_setup_power_policy(int power_policy);

/*******************************************************************************
//The Function Handles WLAN Events
*******************************************************************************/

unsigned char __SL_G_PID[] = "__SL_G_PID";
unsigned char __SL_G_PSN[] = "__SL_G_PSN";
unsigned char __SL_G_HST[] = "__SL_G_HST";
unsigned char __SL_G_MAC[] = "__SL_G_MAC";
unsigned char __SL_G_PFW[] = "__SL_G_PFW";
unsigned char __SL_G_TWS[] = "__SL_G_TWS";

unsigned char __SL_G_WF0[] = "__SL_G_WF0";
unsigned char __SL_G_WF1[] = "__SL_G_WF1";
unsigned char __SL_G_WF2[] = "__SL_G_WF2";
unsigned char __SL_G_WF3[] = "__SL_G_WF3";
unsigned char __SL_G_WF4[] = "__SL_G_WF4";
unsigned char __SL_G_WF5[] = "__SL_G_WF5";
unsigned char __SL_G_WF6[] = "__SL_G_WF6";
unsigned char __SL_G_WF7[] = "__SL_G_WF7";
unsigned char __SL_G_WF8[] = "__SL_G_WF8";
unsigned char __SL_G_WF9[] = "__SL_G_WF9";

unsigned char __SL_G_WRT[] = "__SL_G_WRT";
unsigned char __SL_G_CWF[] = "__SL_G_CWF";
unsigned char __SL_G_CSF[] = "__SL_G_CSF";
unsigned char __SL_G_AGF[] = "__SL_G_AGF";
unsigned char __SL_G_MDE[] = "__SL_G_MDE";
unsigned char __SL_G_PDE[] = "__SL_G_PDE";
unsigned char __SL_G_PDF[] = "__SL_G_PDF";
unsigned char __SL_G_SWF[] = "__SL_G_SWF";
unsigned char __SL_G_WPW[] = "__SL_G_WPW";

unsigned char __SL_G_RSP[] = "__SL_G_RSP";

unsigned char __SL_P_WST[] = "__SL_P_WST";
unsigned char __SL_P_WPW[] = "__SL_P_WPW";

char ssid_buf[10][64] = {0};
extern Sl_WlanNetworkEntry_t netEntries[20]; // Wlan Network Entry
web_product web_product_msg;
web_errcode web_errcode_msg;
web_post_wifi web_post_wifi_msg;
char web_resp_buf[24] = {0};

void UpdateTimeTask(void *pvParameters);
void ApikeyGetTask(void *pvParameters);

extern void WlanAPMode(void *pvParameters);

/*******************************************************************************
//read product 
*******************************************************************************/
void Web_read_product(void)
{
	uint8_t read_len;
	char mac_addr[8] = {0};
	uint8_t read_buf[64];

	memset(&web_product_msg, 0, sizeof(web_product_msg));

	memset(read_buf, 0, sizeof(read_buf));

	osi_at24c08_ReadData(PRODUCT_ID_ADDR, (uint8_t *)read_buf, sizeof(read_buf), 1);

	snprintf((char *)web_product_msg.pid, sizeof(web_product_msg.pid), "{\"ProductID\":\"%s\"}", read_buf);

	memset(read_buf, 0, sizeof(read_buf));

	osi_at24c08_ReadData(SERISE_NUM_ADDR, (uint8_t *)read_buf, sizeof(read_buf), 1);

	snprintf((char *)web_product_msg.psn, sizeof(web_product_msg.psn), "{\"SeriesNumber\":\"%s\"}", read_buf);

	memset(read_buf, 0, sizeof(read_buf));

	osi_at24c08_ReadData(HOST_ADDR, (uint8_t *)read_buf, sizeof(read_buf), 1);

	snprintf((char *)web_product_msg.hst, sizeof(web_product_msg.hst), "{\"Host\":\"%s\"}", read_buf);

	memset(read_buf, 0, sizeof(read_buf));

	// osi_at24c08_ReadData(MAC_ADDR, (uint8_t *)mac_addr, SL_MAC_ADDR_LEN, 0); //Read MAC
	esp_read_mac(mac_addr, 0); //获取芯片内部默认出厂MAC

	snprintf((char *)read_buf, sizeof(read_buf), "%02x:%02x:%02x:%02x:%02x:%02x", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

	snprintf((char *)web_product_msg.mac, sizeof(web_product_msg.mac), "{\"MAC\":\"%s\"}", read_buf);

	snprintf((char *)web_product_msg.pfw, sizeof(web_product_msg.pfw), "{\"set_wifi\":1,\"firmware\":\"%s\"}", FIRMWARENUM);

	mem_set(SSID_NAME, 0, sizeof(SSID_NAME));

	read_len = osi_at24c08_read_byte(SSID_LEN_ADDR);

	if (read_len > sizeof(SSID_NAME))
	{
		read_len = sizeof(SSID_NAME);
	}

	osi_at24c08_ReadData(SSID_ADDR, (uint8_t *)SSID_NAME, read_len, 0); //Read Wifi SSID

	snprintf((char *)web_product_msg.ssid, sizeof(web_product_msg.ssid), "{\"ssid\":\"%s\"}", SSID_NAME);
}

/*******************************************************************************
//read error code 
*******************************************************************************/
void Web_read_errcode(void)
{
	unsigned long err_time;

	memset(&web_errcode_msg, 0, sizeof(web_errcode_msg));

	err_time = osi_at24c08_read(WD_RESET_ERR);
	if (err_time)
	{
		snprintf((char *)web_errcode_msg.wrt, sizeof(web_errcode_msg.wrt), "{\"wd_rst\":%d}", err_time);
	}

	err_time = osi_at24c08_read(CNT_WIFI_FLR_ERR);
	if (err_time)
	{
		snprintf((char *)web_errcode_msg.cwf, sizeof(web_errcode_msg.cwf), "{\"c_wf_flr\":%d}", err_time);
	}

	err_time = osi_at24c08_read(CNT_SERVER_FLR_ERR);
	if (err_time)
	{
		snprintf((char *)web_errcode_msg.csf, sizeof(web_errcode_msg.csf), "{\"c_srv_flr\":%d}", err_time);
	}

	err_time = osi_at24c08_read(API_GET_FLR_ERR);
	if (err_time)
	{
		snprintf((char *)web_errcode_msg.agf, sizeof(web_errcode_msg.agf), "{\"api_gt_flr\":%d}", err_time);
	}

	err_time = osi_at24c08_read(MEMORY_SAVE_DATA_ERR);
	if (err_time)
	{
		snprintf((char *)web_errcode_msg.mde, sizeof(web_errcode_msg.mde), "{\"mry_dt_err\":%d}", err_time);
	}

	err_time = osi_at24c08_read(POST_DATA_JSON_FAULT);
	if (err_time)
	{
		snprintf((char *)web_errcode_msg.pde, sizeof(web_errcode_msg.pde), "{\"pt_dt_js_err\":%d}", err_time);
	}

	err_time = osi_at24c08_read(POST_DATA_FLR_ERR);
	if (err_time)
	{
		snprintf((char *)web_errcode_msg.pdf, sizeof(web_errcode_msg.pdf), "{\"pt_dt_flr\":%d}", err_time);
	}

	err_time = osi_at24c08_read(SCN_WF_FLR_ERR);
	if (err_time)
	{
		snprintf((char *)web_errcode_msg.swf, sizeof(web_errcode_msg.swf), "{\"scn_wf_flr\":%d}", err_time);
	}

	err_time = osi_at24c08_read(WF_PWD_WR_ERR);
	if (err_time)
	{
		snprintf((char *)web_errcode_msg.wpw, sizeof(web_errcode_msg.wpw), "{\"wf_pwd_wr\":%d}", err_time);
	}
}

/*******************************************************************************
//Routine to feed to watchdog
*******************************************************************************/
// void WatchdogAck(void)
// {
// #ifdef USE_WD

// 	WatchdogIntClear(WDT_BASE); //Acknowledge watchdog by clearing interrupt

// #endif
// }

/*******************************************************************************
//calculat the fn min sleep time
*******************************************************************************/
static long fn_sleep_time_min(void)
{
	uint8_t fn_i, fn_ix;
	unsigned long slp_t = 0;
#ifdef MAG_SENSOR
	unsigned long fn_x[] = {fn_dp, fn_th, fn_light, fn_mag, fn_ext, fn_battery};
#else
	unsigned long fn_x[] = {fn_dp, fn_th, fn_light, fn_ext, fn_battery};
#endif

	fn_ix = sizeof(fn_x) / sizeof(unsigned long);

	for (fn_i = 0; fn_i < fn_ix; fn_i++) //get a fn value >0
	{
		if (fn_x[fn_i] > 0)
		{
			slp_t = fn_x[fn_i];

			break;
		}
	}

	for (fn_i = 0; fn_i < fn_ix; fn_i++)
	{
		if (fn_x[fn_i] > 0)
		{
			slp_t = fn_x[fn_i] < slp_t ? fn_x[fn_i] : slp_t;
		}
	}

	return slp_t;
}

/*******************************************************************************
//calculat the fn_t min sleep time
*******************************************************************************/
static long fn_t_sleep_time_min(void)
{
	long fn_min;
	long slp_t = 0;
	uint8_t fn_t_i, fn_t_ix;
#ifdef MAG_SENSOR
	unsigned long fn_x_t[] = {fn_dp_t, fn_th_t, fn_light_t, fn_mag_t, fn_ext_t, fn_battery_t};
#else
	unsigned long fn_x_t[] = {fn_dp_t, fn_th_t, fn_light_t, fn_ext_t, fn_battery_t};
#endif
	fn_t_ix = sizeof(fn_x_t) / sizeof(unsigned long);

	for (fn_t_i = 0; fn_t_i < fn_t_ix; fn_t_i++)
	{
		if (fn_x_t[fn_t_i] > now_unix_t)
		{
			slp_t = fn_x_t[fn_t_i] - now_unix_t;

			break;
		}
	}

	for (fn_t_i = 0; fn_t_i < fn_t_ix; fn_t_i++)
	{
		if (fn_x_t[fn_t_i] > now_unix_t)
		{
			slp_t = fn_x_t[fn_t_i] - now_unix_t < slp_t ? fn_x_t[fn_t_i] - now_unix_t : slp_t;
		}
	}

	fn_min = fn_sleep_time_min();

	slp_t = slp_t > fn_min ? fn_min : slp_t;

	slp_t = slp_t < MIN_SLP_TIME ? MIN_SLP_TIME : slp_t;

	slp_t = slp_t > MAX_SLP_TIME ? MAX_SLP_TIME : slp_t;

	return slp_t;
}

/*******************************************************************************
//callback function for timer interrupt handler
CC3200 设置睡眠时间，是在运行中设置，设置后立即即使，如果还未进入睡眠而计时到，则进入此函数
*******************************************************************************/
// void TimerCallback(void *vParam)
// {
// #ifdef DEBUG

// 	UART_PRINT("timer wake up\r\n");

// #endif

// #ifdef CHECK_WATER_MARK

// 	// osi_SyncObjSignalFromISR(&xBinary14);
// 	vTaskNotifyGiveFromISR(xBinary14, NULL);

// #endif

// 	// osi_SyncObjSignalFromISR(&xBinary9); //check Task start time
// 	vTaskNotifyGiveFromISR(xBinary9, NULL);

// 	return;
// }

// cc_hndl tTimerHndl = NULL; //handle for the Timer

/*******************************************************************************
//set Timer as a wake up source from low power modes
*******************************************************************************/
// int SetTimerAsWkUp(long slp_time, bool call_back)
// {
// 	int Set_val = -1;
// 	struct cc_timer_cfg sRealTimeTimer;
// 	struct u64_time sIntervalTimer;
// 	struct u64_time sInitTime;

// 	if (tTimerHndl != NULL)
// 	{
// 		Set_val = cc_timer_stop(tTimerHndl); //Stop an active timer
// 		if (Set_val < 0)
// 		{
// 			return FAILURE;
// 		}

// 		Set_val = cc_timer_delete(tTimerHndl); //Deletes an inactive timer
// 		if (Set_val < 0)
// 		{
// 			return FAILURE;
// 		}
// 	}

// 	sInitTime.secs = 0;
// 	sInitTime.nsec = 0;
// 	Set_val = cc_rtc_set(&sInitTime);
// 	if (Set_val < 0)
// 	{
// 		return FAILURE;
// 	}

// 	sRealTimeTimer.source = HW_REALTIME_CLK;
// 	if (call_back)
// 	{
// 		sRealTimeTimer.timeout_cb = TimerCallback;
// 	}
// 	else
// 	{
// 		sRealTimeTimer.timeout_cb = NULL;
// 	}
// 	sRealTimeTimer.cb_param = NULL;
// 	tTimerHndl = cc_timer_create(&sRealTimeTimer);
// 	if (tTimerHndl == NULL)
// 	{
// 		return FAILURE;
// 	}

// 	sIntervalTimer.secs = slp_time;
// 	sIntervalTimer.nsec = 0;
// 	Set_val = cc_timer_start(tTimerHndl, &sIntervalTimer, OPT_TIMER_PERIODIC);
// 	if (Set_val < 0)
// 	{
// 		return FAILURE;
// 	}

// 	return SUCCESS;
// }

/*******************************************************************************
//set GPIOs as a wake up source from low power modes
*******************************************************************************/
void Set_GPIO_AsWkUp(void)
{
	// 	SetButtonAsWkUp(); //set Button Int Pin as wake up source

	// #ifdef MAG_SENSOR
	// 	SetMAGAsWkUp(); //set Mag Int pin as wake up source
	// #endif

	// 	SetACCEAsWkUp(); //set AcceSensor Int pin as wake up source

	// 	SetUSBAsWkUp(); //set USB Int Pin as wake up source

	const int ext_wakeup_pin_1 = BUTTON_PIN;
	const uint64_t ext_wakeup_pin_1_mask = 1ULL << ext_wakeup_pin_1;
	const int ext_wakeup_pin_2 = ACCE_SRC_WKUP;
	const uint64_t ext_wakeup_pin_2_mask = 1ULL << ext_wakeup_pin_2;

	printf("Enabling EXT1 wakeup on pins GPIO%d, GPIO%d\n", ext_wakeup_pin_1, ext_wakeup_pin_2);
	esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask | ext_wakeup_pin_2_mask, ESP_EXT1_WAKEUP_ALL_LOW);
}

/*******************************************************************************
  Set Sensors In Power Down  Mode 
*******************************************************************************/
static void Sensor_Power_OFF(void)
{
	ADXL345_Power_Down(); //Acce Sensor power down

#ifdef USE_LCD

	Ht1621_Off(); //LCD driver power down

#endif
}

/*******************************************************************************
//Check and Update Task Operate Time
*******************************************************************************/
void Timer_Check_Task(void *pvParameters)
{
	uint8_t msg_slp_val = MSG_SLP_VAL;

	for (;;)
	{
		// osi_SyncObjWait(&xBinary9, OSI_WAIT_FOREVER); //Wait Task Operation Message
		ulTaskNotifyTake(pdTRUE, -1);

		osi_Read_UnixTime(); //update system unix time

#ifdef DEBUG

		osi_UartPrint_Val("now_unix_t:", now_unix_t);

#endif

#ifdef DEBUG

		osi_UartPrint_Val("fn_dp:", fn_dp);

		osi_UartPrint_Val("fn_dp_t:", fn_dp_t);

#endif

		if (((now_unix_t >= fn_dp_t) || (fn_dp_t >= (now_unix_t + MAX_SLP_TIME))) && fn_dp) //fn_dp_t
		{
			fn_dp_t = now_unix_t + fn_dp;

			// osi_SyncObjSignalFromISR(&xBinary0); //Data Post Task
			vTaskNotifyGiveFromISR(xBinary0, NULL);

			osi_at24c08_write(FN_DP_T_ADDR, fn_dp_t);
		}

#ifdef DEBUG

		osi_UartPrint_Val("fn_th:", fn_th);

		osi_UartPrint_Val("fn_th_t:", fn_th_t);

#endif

		if (((now_unix_t >= fn_th_t) || (fn_th_t >= (now_unix_t + MAX_SLP_TIME))) && fn_th) //fn_th_t
		{
			fn_th_t = now_unix_t + fn_th;

			// osi_SyncObjSignalFromISR(&xBinary2); //Temp&Humi Sensor Task
			vTaskNotifyGiveFromISR(xBinary2, NULL);

			osi_at24c08_write(FN_TH_T_ADDR, fn_th_t);
		}

#ifdef DEBUG

		osi_UartPrint_Val("fn_light:", fn_light);

		osi_UartPrint_Val("fn_light_t:", fn_light_t);

#endif

		if (((now_unix_t >= fn_light_t) || (fn_light_t >= (now_unix_t + MAX_SLP_TIME))) && fn_light) //fn_light_t
		{
			fn_light_t = now_unix_t + fn_light;

			// osi_SyncObjSignalFromISR(&xBinary3); //Light Sensor Task
			vTaskNotifyGiveFromISR(xBinary3, NULL);

			osi_at24c08_write(FN_LIGHT_T_ADDR, fn_light_t);
		}

#ifdef MAG_SENSOR
#ifdef DEBUG

		osi_UartPrint_Val("fn_mag:", fn_mag);

		osi_UartPrint_Val("fn_mag_t:", fn_mag_t);

#endif

		if (((now_unix_t >= fn_mag_t) || (fn_mag_t >= (now_unix_t + MAX_SLP_TIME))) && fn_mag) //fn_mag_t
		{
			fn_mag_t = now_unix_t + fn_mag;

			// osi_SyncObjSignalFromISR(&xBinary4); //Magnetic Sensor Task
			vTaskNotifyGiveFromISR(xBinary4, NULL);

			osi_at24c08_write(FN_MAG_T_ADDR, fn_mag_t);
		}
#endif

#ifdef DEBUG

		osi_UartPrint_Val("fn_ext:", fn_ext);

		osi_UartPrint_Val("fn_ext_t:", fn_ext_t);

#endif

		if (((now_unix_t >= fn_ext_t) || (fn_ext_t >= (now_unix_t + MAX_SLP_TIME))) && fn_ext) //fn_ext_t
		{
			fn_ext_t = now_unix_t + fn_ext;

			// osi_SyncObjSignalFromISR(&xBinary6); //Extern Temperature Measure Task
			vTaskNotifyGiveFromISR(xBinary6, NULL);

			osi_at24c08_write(FN_EXT_T_ADDR, fn_ext_t);
		}

#ifdef DEBUG

		osi_UartPrint_Val("fn_battery:", fn_battery);

		osi_UartPrint_Val("fn_battery_t:", fn_battery_t);

#endif

		if (((now_unix_t >= fn_battery_t) || (fn_battery_t >= (now_unix_t + MAX_SLP_TIME))) && fn_battery) //fn_battery_t
		{
			fn_battery_t = now_unix_t + fn_battery;

			// osi_SyncObjSignalFromISR(&xBinary7); //Power Measure Task
			vTaskNotifyGiveFromISR(xBinary7, NULL);

			osi_at24c08_write(FN_BATTERY_T_ADDR, fn_battery_t);
		}

		// osi_MsgQWrite(&xQueue3, &msg_slp_val, OSI_NO_WAIT); //send to WatchDog and SleepTimer message
		xQueueSend(xQueue3, &msg_slp_val, -1);
	}
}

#ifdef CHECK_WATER_MARK

/*******************************************************************************
//check the tasks water mark
*******************************************************************************/
void WaterMark_Check(void *pvParameters)
{
	UBaseType_t watermark;

	for (;;)
	{
		// osi_SyncObjWait(&xBinary14, OSI_WAIT_FOREVER); //Wait Task Operation Message
		ulTaskNotifyTake(pdTRUE, -1);

		watermark = uxTaskGetStackHighWaterMark(UpdateTimeTaskHandle);

		osi_UartPrint_Val("UpdateTimeTaskWaterMark:", watermark);

		watermark = uxTaskGetStackHighWaterMark(UartParseTaskHandle);

		osi_UartPrint_Val("UartParseTaskWaterMark:", watermark);

		watermark = uxTaskGetStackHighWaterMark(ButtonPush_Int_TaskHandle);

		osi_UartPrint_Val("ButtonPush_Int_TaskWaterMark:", watermark);

		watermark = uxTaskGetStackHighWaterMark(MagneticSensorTaskHandle);

		osi_UartPrint_Val("MagneticSensorTaskWaterMark:", watermark);

		watermark = uxTaskGetStackHighWaterMark(Green_LedControl_TaskHandle);

		osi_UartPrint_Val("Green_LedControl_TaskWaterMark:", watermark);

		watermark = uxTaskGetStackHighWaterMark(Bell_Control_TaskHandle);

		osi_UartPrint_Val("Bell_Control_TaskWaterMark:", watermark);

		watermark = uxTaskGetStackHighWaterMark(TempHumiTaskHandle);

		osi_UartPrint_Val("TempHumiTaskWaterMark:", watermark);

		watermark = uxTaskGetStackHighWaterMark(LightTaskHandle);

		osi_UartPrint_Val("LightTaskWaterMark:", watermark);

		watermark = uxTaskGetStackHighWaterMark(ExtTempMeasureTaskHandle);

		osi_UartPrint_Val("ExtTempMeasureTaskWaterMark:", watermark);

		watermark = uxTaskGetStackHighWaterMark(PowerMeasureTaskHandle);

		osi_UartPrint_Val("PowerMeasureTaskWaterMark:", watermark);

		watermark = uxTaskGetStackHighWaterMark(DataSaveTaskHandle);

		osi_UartPrint_Val("DataSaveTaskWaterMark:", watermark);

		watermark = uxTaskGetStackHighWaterMark(DataPostTaskHandle);

		osi_UartPrint_Val("DataPostTaskWaterMark:", watermark);

		watermark = uxTaskGetStackHighWaterMark(Memory_DeleteTaskHandle);

		osi_UartPrint_Val("Memory_DeleteTaskWaterMark:", watermark);

		watermark = uxTaskGetStackHighWaterMark(Timer_Check_TaskHandle);

		osi_UartPrint_Val("Timer_Check_TaskWaterMark:", watermark);
	}
}

#endif

/*******************************************************************************
//USB Mode Led Flash Task
*******************************************************************************/
void Usb_Mode_Task(void *pvParameters)
{
	uint16_t usb_t;

	usb_status = osi_at24c08_read_byte(USB_FLAG_ADDR);

	if (usb_status == 0)
	{
		// if (gpio_get_level(USB_PIN)) //USB Mode GPIO Wake Up
		if (gpio_get_level(USB_PIN))
		{
			usb_status_val = 1;

			// osi_SyncObjSignalFromISR(&xBinary15);
			vTaskNotifyGiveFromISR(xBinary15, NULL);
		}
	}
	else
	{
		// if (!gpio_get_level(USB_PIN)) //USB Mode GPIO Wake Up
		if (!gpio_get_level(USB_PIN))
		{
			usb_status = 0;

			usb_status_val = 0;

			osi_at24c08_write_byte(USB_FLAG_ADDR, usb_status);

			data_post = 1; //Save Then Post Data

			// osi_SyncObjSignalFromISR(&xBinary7); //Power Measure Task
			vTaskNotifyGiveFromISR(xBinary7, NULL);
		}
		else
		{
			usb_status_val = 1;
		}
	}

	for (;;)
	{
		// osi_SyncObjWait(&xBinary15, OSI_WAIT_FOREVER); //Wait Task Operation Message
		ulTaskNotifyTake(pdTRUE, -1);

		usb_t = 0;

		data_post = 1; //Save Then Post Data

		// osi_SyncObjSignalFromISR(&xBinary7); //Power Measure Task
		vTaskNotifyGiveFromISR(xBinary7, NULL);

		for (;;)
		{
			usb_t += 1;

			Green_Led_Flashed(1, 4); //Green Led Flashed 0.6s

			// if (!gpio_get_level(USB_PIN)) //read usb pin status
			if (!gpio_get_level(USB_PIN))
			{
				usb_status = 0;

				usb_status_val = 0;

				osi_at24c08_write_byte(USB_FLAG_ADDR, usb_status);

				data_post = 1; //Save Then Post Data

				// osi_SyncObjSignalFromISR(&xBinary7); //Power Measure Task
				vTaskNotifyGiveFromISR(xBinary7, NULL);

				break;
			}
			else
			{
				usb_status_val = 1;
			}

			if (usb_t >= USB_TIME_OUT)
			{
				usb_status = 1;

				osi_at24c08_write_byte(USB_FLAG_ADDR, usb_status);

				break;
			}

			sys_run_time = 0; //clear system time out
		}
	}
}

/*******************************************************************************
//GPIO or Timer Wake Up Process
*******************************************************************************/
static void WakeUp_Process(void)
{
	// usb_status_val = gpio_get_level(USB_PIN);
	usb_status_val = gpio_get_level(USB_PIN);

	// if (!gpio_get_level(BUTTON_PIN)) //BUTTON GPIO Wake Up
	if (!gpio_get_level(BUTTON_PIN))
	{
		// osi_SyncObjSignalFromISR(&xBinary1);
		vTaskNotifyGiveFromISR(xBinary1, NULL);
	}

	// if (!GPIOPinRead(ACCE_PORT, ACCE_PIN)) //Acceleration Sensor GPIO Wake Up
	if (!gpio_get_level(ACCE_SRC_WKUP))
	{
		// osi_SyncObjSignalFromISR(&xBinary12);
		vTaskNotifyGiveFromISR(xBinary12, NULL);
	}

#ifdef MAG_SENSOR
	if (!GPIOPinRead(MAG_PORT, MAG_PIN)) //MAG Sensor GPIO Wake Up
	{
		if (door_status)
		{
			// osi_SyncObjSignalFromISR(&xBinary4);
			vTaskNotifyGiveFromISR(xBinary4, NULL);
		}
	}
	else
	{
		if (!door_status)
		{
			// osi_SyncObjSignalFromISR(&xBinary4);
			vTaskNotifyGiveFromISR(xBinary4, NULL);
		}
	}
#endif

	// osi_SyncObjSignalFromISR(&xBinary9); //Start TIMERA0 Interrupt Task
	vTaskNotifyGiveFromISR(xBinary9, NULL);
}

/*******************************************************************************
//Create The System Work Task
*******************************************************************************/
void MainTask_Create(void *pvParameters)
{
	// OsiTaskHandle Create_TaskHandle = NULL;

	WakeUp_Process(); //GPIO or Timer Wake Up Process

	osi_at24c08_read_addr(); //Read Post Data Amount/Write Data/Post Data/Delete Data Address

	xTaskCreate(Timer_Check_Task, "Timer_Check_Task", 384, NULL, 9, &xBinary9); //Check Tasks Operate Time

	xTaskCreate(TempHumiSensorTask, "TempHumiSensorTask", 448, NULL, 7, &xBinary2); //Create TempHumiSensor Task

	xTaskCreate(LightSensorTask, "LightSensorTask", 512, NULL, 7, &xBinary3); //Create LightSensor Task

#ifdef MAG_SENSOR
	xTaskCreate(MagneticSensorTask, "MagneticSensorTask", 320, NULL, 7, &xBinary4); //Create MagneticSensor Task
#endif

	xTaskCreate(AcceSensor_Int_Task, "AcceSensor_Int_Task", 384, NULL, 5, &xBinary12); //Create System_Interrupt_Task

	xTaskCreate(AccelerationSensorTask, "AccelerationSensorTask", 512, NULL, 5, &xBinary5); //Create AccelerationSensor Task

	xTaskCreate(ExtTempMeasureTask, "ExtTempMeasureTask", 512, NULL, 7, &xBinary6); //Create Noise Measure Task

	xTaskCreate(PowerMeasureTask, "PowerMeasureTask", 448, NULL, 7, &xBinary7); //Create Power Measure Task

	xTaskCreate(DataSaveTask, "DataSaveTask", 1024, NULL, 7, NULL); //Create Data Save Task

	xTaskCreate(DataPostTask, "DataPostTask", 2048, NULL, 5, &xBinary0); //Create Data Post Task

	xTaskCreate(Green_LedControl_Task, "Green_LedControl_Task", 192, NULL, 5, NULL); //Create Green Led control task

	xTaskCreate(Bell_Control_Task, "Bell_Control_Task", 192, NULL, 5, NULL); //Create bell control task

	xTaskCreate(Memory_DeleteTask, "Memory_DeleteTask", 640, NULL, 5, &xBinary11); //Create Memory_DeleteTask

#ifdef CHECK_WATER_MARK

	xTaskCreate(WaterMark_Check, "WaterMark_Check", 512, NULL, 7, &xBinary14);

#endif

	// osi_TaskDelete(&Create_TaskHandle); //delete Main Task_Create TASK
	vTaskDelete(NULL);
}

/*******************************************************************************
//API KEY GET TASK 
*******************************************************************************/
void ApikeyGetTask(void *pvParameters)
{
	uint8_t re_try;
	int lRetVal = -1;

	// osi_SyncObjWait(&xMutex2, OSI_WAIT_FOREVER); //SimpleLink Semaphore Take
	xSemaphoreTake(xMutex2, -1);

	PostSetData = 1;

	APIGET_TASK_END_FLAG = 1;

	// osi_SyncObjSignalFromISR(&xBinary13); //Start Tasks End Check
	vTaskNotifyGiveFromISR(xBinary13, NULL);

	// osi_SyncObjSignalFromISR(&xBinary17); //Start LED Blink When Internet Application
	vTaskNotifyGiveFromISR(xBinary17, NULL);

	for (re_try = 0; re_try < RETRY_TIME_OUT; re_try++)
	{
		sys_run_time = 0; //clear system time out

		lRetVal = Scan_Wifi_List(NULL, NULL, 0);

		if ((lRetVal != ERROR_CODE) || (wifi_mode == 1))
		{
			if (lRetVal == ERROR_CODE)
			{
				osi_at24c08_ReadData(SECTYPE_ADDR, (uint8_t *)SEC_TYPE, sizeof(SEC_TYPE), 1); //Read Wifi Sectype
			}

			lRetVal = WlanConnect(); //wlan connect to the accesspoint
			if (lRetVal >= 0)
			{
				lRetVal = Http_Get_Function(); //HTTP Get Form Server

				Wlan_Disconnect_AP(); //wlan disconnect form the ap

				// sl_Stop(SL_STOP_TIMEOUT); //stop the simple link

				// MAP_UtilsDelay(80000); //delay about 6ms

				if (lRetVal == SUCCESS)
				{
					break;
				}
			}
			else
			{
				Wlan_Disconnect_AP(); //wlan disconnect form the ap

				// sl_Stop(SL_STOP_TIMEOUT); //stop the simple link

				// MAP_UtilsDelay(80000); //delay about 6ms
			}
		}
	}

	PostSetData = 0;

	APIGET_TASK_END_FLAG = 0;

	// osi_SyncObjSignal(&xMutex2); //SimpleLink Semaphore Give
	xSemaphoreGive(xMutex2);

	if (lRetVal == SUCCESS)
	{
		osi_Read_UnixTime();

		OperateData_Init(); //Operate Data Init//

		osi_OperateData_Save();

		acce_sensor_reset();

		data_post = 1; //Save Then Post Data

		// osi_SyncObjSignalFromISR(&xBinary2); //Temp&Humi Sensor Task
		vTaskNotifyGiveFromISR(xBinary2, NULL);

		// osi_SyncObjSignalFromISR(&xBinary3); //Light Sensor Task
		vTaskNotifyGiveFromISR(xBinary3, NULL);

		// osi_SyncObjSignalFromISR(&xBinary6); //Extern Temperature Measure Task
		vTaskNotifyGiveFromISR(xBinary6, NULL);

		// osi_SyncObjSignalFromISR(&xBinary7); //Power Measure Task
		vTaskNotifyGiveFromISR(xBinary7, NULL);

		xTaskCreate(MainTask_Create, "MainTask_Create Task", 512, NULL, 9, NULL); //create the system work task
	}
	else
	{
		osi_System_ERROR_Save(API_GET_FLR_ERR); //save ERROR data

		xTaskCreate(st_Green_Red_LedFlashed_Task, "st_Green_Red_LedFlashed_Task", 256, NULL, 7, NULL); //Create Red Led Flash task
	}

	vTaskDelete(NULL);
}

/*******************************************************************************
//update time task
*******************************************************************************/
void UpdateTimeTask(void *pvParameters)
{
	// ;
	int lRetVal = -1;
	// OsiTaskHandle upd_TaskHandle = NULL;

	// osi_SyncObjWait(&xMutex2, OSI_WAIT_FOREVER); //SimpleLink Semaphore Take
	xSemaphoreTake(xMutex2, -1);

	update_time = 1;

	UPDATETIME_TASK_END_FLAG = 1;

	// osi_SyncObjSignalFromISR(&xBinary13);
	vTaskNotifyGiveFromISR(xBinary13, NULL);

	// osi_SyncObjSignalFromISR(&xBinary17); //Start LED Blink
	vTaskNotifyGiveFromISR(xBinary17, NULL);

	sys_run_time = 0; //clear wifi post time

	lRetVal = Scan_Wifi_List(NULL, NULL, 0);

	if ((lRetVal != ERROR_CODE) || (wifi_mode == 1))
	{
		if (lRetVal == ERROR_CODE)
		{
			osi_at24c08_ReadData(SECTYPE_ADDR, (uint8_t *)SEC_TYPE, sizeof(SEC_TYPE), 1); //Read Wifi Sectype
		}

		lRetVal = WlanConnect(); //wlan connect to the accesspoint
		if (lRetVal >= 0)
		{
			lRetVal = Http_Get_Function(); //HTTP Get Form Server
			if (lRetVal < 0)
			{
				Red_Led_On_time(10); //RED LED ON 1.5s

				osi_System_ERROR_Save(UPDATE_TIME_FLR_ERR); //save ERROR data
			}
		}

		Wlan_Disconnect_AP(); //wlan disconnect form the ap

		// sl_Stop(SL_STOP_TIMEOUT); //stop the simple link
		// MAP_UtilsDelay(80000); //delay about 6ms
	}

	update_time = 0;

	UPDATETIME_TASK_END_FLAG = 0;

	// osi_SyncObjSignal(&xMutex2); //SimpleLink Semaphore Give
	xSemaphoreGive(xMutex2);

	osi_Read_UnixTime(); //read unix time

	OperateData_Init(); //Operate Data Init

	osi_OperateData_Save();

	acce_sensor_reset();

	data_post = 1; //Save Then Post Data

	// osi_SyncObjSignalFromISR(&xBinary2); //Temp&Humi Sensor Task
	vTaskNotifyGiveFromISR(xBinary2, NULL);

	// osi_SyncObjSignalFromISR(&xBinary3); //Light Sensor Task
	vTaskNotifyGiveFromISR(xBinary3, NULL);

	// osi_SyncObjSignalFromISR(&xBinary6); //Extern Temperature Measure Task
	vTaskNotifyGiveFromISR(xBinary6, NULL);

	// osi_SyncObjSignalFromISR(&xBinary7); //Power Measure Task
	vTaskNotifyGiveFromISR(xBinary7, NULL);

	xTaskCreate(MainTask_Create, "MainTask_Create Task", 512, NULL, 9, NULL); //create the system work task

	// osi_TaskDelete(&upd_TaskHandle); //delete Update Time TASK
	vTaskDelete(NULL);
}

/*******************************************************************************
  USB Write Command Task
*******************************************************************************/
void Usb_Set_Mode(void *pvParameters)
{
	// OsiTaskHandle WUS_TaskHandle = NULL;

	// osi_SyncObjWait(&xBinary16, OSI_WAIT_FOREVER); //Wait Task Operation Message
	ulTaskNotifyTake(pdTRUE, -1);

	xTaskCreate(ApikeyGetTask, "ApikeyGetTask", 1024, NULL, 7, NULL); //Create ApiKeyGetTask

	MAP_UtilsDelay(80000); //delay about 6ms

	vTaskDelete(NULL);
}

/*******************************************************************************
  Factory Reset Task
*******************************************************************************/
void F_ResetTask(void *pvParameters)
{
	// char mac_addr[6] = {0};
	// uint8_t buf_len = sizeof(mac_addr);
	uint8_t rest_buf[64];

	f_reset_status = 1;

	MetaData_Init();	 //Metadata init
	osi_MetaData_Save(); //at24c08 save metadata

	CaliData_Init();	 //Calidata init
	osi_CaliData_Save(); //at24c08 save Calidata

	osi_Error_Code_Init(); //Error Code Init Save in At24c08 whit locked

	//重置网络以及MAC esp32 应该不需要
	// osi_SyncObjWait(&xMutex2, OSI_WAIT_FOREVER); //SimpleLink Semaphore Take
	// APIGET_TASK_END_FLAG = 1;
	// // osi_SyncObjSignalFromISR(&xBinary13); //Start Tasks End Check
	// vTaskNotifyGiveFromISR(xBinary13, NULL);
	// sl_Start(0, 0, 0); //start the simple link
	// sl_NetCfgGet(SL_MAC_ADDRESS_GET, NULL, &buf_len, (unsigned char *)mac_addr);
	// sl_Stop(SL_STOP_TIMEOUT); //stop the simple link
	// MAP_UtilsDelay(80000);	  //delay about 6ms
	// APIGET_TASK_END_FLAG = 0;
	// osi_SyncObjSignal(&xMutex2);
	//SimpleLink Semaphore Give
	// esp_read_mac(mac_addr, 0);
	// osi_at24c08_WriteData(MAC_ADDR, (uint8_t *)mac_addr, SL_MAC_ADDR_LEN, 0); //save type

	SET_GREEN_LED_OFF(); //LED OFF
	acce_sensor_reset(); //check acce sensor

	mem_set(rest_buf, 0, sizeof(rest_buf));

	osi_at24c08_WriteData(SSID_FLAG_ADDR, rest_buf, 7, 1); //deleted ssid flag
	osi_at24c08_write_byte(SSID_LEN_ADDR, 0);
	osi_at24c08_WriteData(SSID_ADDR, rest_buf, 32, 0); //deleted ssid
	osi_at24c08_write_byte(PASSWORD_LEN_ADDR, 0);
	osi_at24c08_WriteData(PASSWORD_ADDR, rest_buf, 64, 0);	  //deleted password
	osi_at24c08_WriteData(SECTYPE_ADDR, rest_buf, 7, 1);	  //deleted sec type
	osi_at24c08_WriteData(CHANNEL_ID_ADDR, rest_buf, 7, 1);	  //deleted channel id
	osi_at24c08_WriteData(USER_ID_ADDR, rest_buf, 43, 1);	  //deleted user id
	osi_at24c08_WriteData(DATAURI_FLAG_ADDR, rest_buf, 7, 1); //deleted datauri

	//Rest Host Name and IP Data
	Host_Flag = DEFAULT_HOST_NAME_IP;
	osi_at24c08_write_byte(HOST_NAME_IP_ADDR, Host_Flag);

	HOST_IP[0] = DEFAULT_IP_ADDR1;
	HOST_IP[1] = DEFAULT_IP_ADDR2;
	HOST_IP[2] = DEFAULT_IP_ADDR3;
	HOST_IP[3] = DEFAULT_IP_ADDR4;
	osi_at24c08_WriteData(HOST_IP_ADDR, HOST_IP, sizeof(HOST_IP), 1); //Save Host IP

	osi_at24c08_write(HOST_PORT_ADDR, HOST_HTTP_PORT_NUM); //HOST_HTTP_PORT_NUM

	uint8_t ip[4] = {192, 168, 1, 64};							 //< Source IP Address
	uint8_t sn[4] = {255, 255, 255, 0};							 //< Subnet Mask
	uint8_t gw[4] = {192, 168, 1, 1};							 //< Gateway IP Address
	uint8_t dns[4] = {192, 168, 1, 1};							 //< DNS server IP Address
	osi_at24c08_write_byte(DHCP_MODE_ADDR, 1);					 //0:disable dhcp,1:enable dhcp
	osi_at24c08_WriteData(STATIC_IP_ADDR, ip, sizeof(ip), 1);	 //reset static ip
	osi_at24c08_WriteData(STATIC_SN_ADDR, sn, sizeof(sn), 1);	 //reset static sn
	osi_at24c08_WriteData(STATIC_GW_ADDR, gw, sizeof(gw), 1);	 //reset static gw
	osi_at24c08_WriteData(STATIC_DNS_ADDR, dns, sizeof(dns), 1); //reset static dns

	osi_Save_Data_Reset(); //Nor Flash Memory Chip Reset with SPI Locked

	if (gpio_get_level(USB_PIN)) //read usb pin status
	{
		xTaskCreate(Usb_Set_Mode, "Usb_Set_Mode", 512, NULL, 5, NULL); //Create Wait Uart Set Task task
	}
	else
	{
		// xTaskCreate(Httpserver_parse_Task, "Httpserver_parse_Task", 896, NULL, 5, NULL);

		xTaskCreate(WlanAPMode, "WlanAPMode", 1600, NULL, 7, NULL); //Create Wlan AP Mode task
	}

	f_reset_status = 0;

	vTaskDelete(NULL); //Delete Factory Reset Task
}

/*******************************************************************************
//Button Interrupt Application Task
*******************************************************************************/
void ButtonPush_Int_Task(void *pvParameters)
{
	uint8_t Button_Push;

	for (;;)
	{
		// osi_SyncObjWait(&xBinary1, OSI_WAIT_FOREVER); //Waite Button GPIO Interrupt Message
		ulTaskNotifyTake(pdTRUE, -1);

		// if (!gpio_get_level(BUTTON_PIN))
		if (!gpio_get_level(BUTTON_PIN))
		{
			Button_Push = 0;

			while (!gpio_get_level(BUTTON_PIN)) //waite button up,read push time
			{
				Button_Push += 1;

				Button_Push = Button_Push > 250 ? 205 : Button_Push;

				if (Button_Push == 200)
				{
					SET_RED_LED_ON(); //RED LED ON

					vTaskSuspendAll(); //disable the tasks

					bell_makeSound(2000);

					xTaskResumeAll(); //enable all tasks
				}

				MAP_UtilsDelay(200000); //delay about 15ms

				sys_run_time = 0; //clear system time out
			}

			if (Button_Push >= 200) //power off
			{
				osi_at24c08_WriteData(SYSTEM_STATUS_ADDR, SYSTEM_OFF, strlen(SYSTEM_OFF), 1); //Restor The System Status-OFF

				for (;;) //never loop
				{
					cc_idle_task_pm(); //Enter Hiberate Mode
				}
			}
			else
			{
				osi_bell_makeSound(200); //Bell make a sound,200 times

				data_post = 1; //Save Then Post Data

				// osi_SyncObjSignalFromISR(&xBinary2); //Temp&Humi Sensor Task
				vTaskNotifyGiveFromISR(xBinary2, NULL);

				// osi_SyncObjSignalFromISR(&xBinary3); //Light Sensor Task
				vTaskNotifyGiveFromISR(xBinary3, NULL);

				// osi_SyncObjSignalFromISR(&xBinary6); //Extern Temperature Measure Task
				vTaskNotifyGiveFromISR(xBinary6, NULL);

				// osi_SyncObjSignalFromISR(&xBinary7); //Power Measure Task
				vTaskNotifyGiveFromISR(xBinary7, NULL);
			}
		}
	}
}

/*******************************************************************************
//uart interrupt hander//
*******************************************************************************/
// void UARTA0IntHandler(void)
// {
// 	char cChar;
// 	unsigned long ulStatus = MAP_UARTIntStatus(UARTA0_BASE, true);

// 	if (ulStatus & UART_INT_RX)
// 	{
// 		//    osi_SyncObjSignalFromISR(&xBinary10);

// 		while (UARTCharsAvail(UARTA0_BASE))
// 		{
// 			cChar = UARTCharGetNonBlocking(UARTA0_BASE); //none blocking get

// 			//      UARTCharPutNonBlocking(UARTA0_BASE,cChar);  //none blocking put

// 			if (uart_pares_status == 0)
// 			{
// 				iLen = iLen >= UART_REV_BUF_LEN ? 0 : iLen; //buffer max

// 				UartGet[iLen++] = cChar;

// 				if ((cChar == '\n') || (cChar == '\r')) //end with '\r\n'
// 				{
// 					UartGet[--iLen] = '\0'; //end flag

// 					if (iLen)
// 					{
// 						osi_SyncObjSignalFromISR(&UART_xBinary);
// 					}
// 				}
// 			}
// 		}
// 	}

// 	MAP_UARTIntClear(UARTA0_BASE, ulStatus); //Clear UART INTERRUPT BIT
// }

/*******************************************************************************
//Power OFF
设置关机，非立即关机
*******************************************************************************/
static void SET_Power_OFF(void)
{
	at24c08_WriteData(SYSTEM_STATUS_ADDR, SYSTEM_OFF, strlen(SYSTEM_OFF), 1); //Restor The System Status-OFF

	Sensor_Power_OFF(); //Set Sensors In Power Down  Mode

	// SetButtonAsWkUp(); // set Button as wakeup source

	// lp3p0_setup_power_policy(POWER_POLICY_HIBERNATE); //Setting up HIBERNATE mode for the system

	// MAP_UtilsDelay(2000000); //delay about 150ms
}

/*******************************************************************************
//Check Tasks End Task
*******************************************************************************/
void Tasks_Check_Task(void *pvParameters)
{
	uint32_t w_t_out;

	for (;;)
	{
		// osi_SyncObjWait(&xBinary13, OSI_WAIT_FOREVER); //Wait Task Start Message
		ulTaskNotifyTake(pdTRUE, -1);

		w_t_out = 0;

		while (POST_TASK_END_FLAG || UPDATETIME_TASK_END_FLAG || APIGET_TASK_END_FLAG || AP_MODE_END_FLAG)
		{
			if (w_t_out++ > 3000000) //15min time out
			{
				break;
			}
			MAP_UtilsDelay(4000); //delay about 0.3ms
		}

		if (cg_data_led)
		{
			SET_GREEN_LED_OFF();
		}
	}
}

/*******************************************************************************
//Senosrs Init when power on
*******************************************************************************/
static void Sensors_Init(void)
{

	OPT3001_Init(); //opt3001 light sensor init

	ADXL345_Init(); //adxl245 sensor init

	flash_set_val = w25q_Init();

	while (w25q_Init() != SUCCESS) //Init W25Q128 Memory Chip
	{
		Red_Led_Flashed(1, 6); //check flash
	}

#ifdef USE_LCD

	float temp_val, humi_val, light_val, p_value;

	Ht1621_On();

	sht30_SingleShotMeasure(&temp_val, &humi_val); //read temperature humility data

	Ht1621_Display_Temp_Val(temp_val, 0); //Display the temprature value

	Ht1621_Display_Humi_Val((uint8_t)humi_val); //Display the Humility value

	p_value = power_adcValue(); //Read power Value

	if (p_value > 3.0)
	{
		power_flag = 0x07;

		at24c08_write_byte(POWER_DATA_ADDR, power_flag);
	}
	else if (p_value > 2.5)
	{
		power_flag = 0x06;

		at24c08_write_byte(POWER_DATA_ADDR, power_flag);
	}
	else
	{
		power_flag = 0x04;

		at24c08_write_byte(POWER_DATA_ADDR, power_flag);
	}

	OPT3001_value(&light_val); //Read Light Value

	Ht1621_Display_Light_Val(light_val, 0, 0, 0, power_flag); //Display the light value

#endif

#ifdef MAG_SENSOR
	door_status = GPIOPinRead(MAG_PORT, MAG_PIN) > 0 ? DOOR_OPEN : DOOR_CLOSED;

	at24c08_write_byte(DOOR_STATUS_ADDR, door_status); //save door status
#endif
}

/*******************************************************************************
//System Variables Init
*******************************************************************************/
void System_Variables_Init(void)
{
	// 	osi_SyncObjCreate(&xBinary0); //Data Post Task

	// 	osi_SyncObjCreate(&xBinary1); //Button Interrupt Task

	// 	osi_SyncObjCreate(&xBinary2); //Temp&Humi Sensor Task

	// 	osi_SyncObjCreate(&xBinary3); //Light Sensor Task

	// #ifdef MAG_SENSOR
	// 	osi_SyncObjCreate(&xBinary4); //Magnetic Sensor Task
	// #endif

	// 	osi_SyncObjCreate(&xBinary5); //Acceleration Sensor Task

	// 	osi_SyncObjCreate(&xBinary6); //Power Measure Task

	// 	osi_SyncObjCreate(&xBinary7); //Extern Temprature Measure

	// 	osi_SyncObjCreate(&xBinary8); //Body Temprature Measure

	// 	osi_SyncObjCreate(&xBinary9); //Timer Interrupt task

	// 	//  osi_SyncObjCreate(&xBinary10);        //UART Interrupt Task

	// 	osi_SyncObjCreate(&UART_xBinary); //UART Parse Task

	// 	osi_SyncObjCreate(&xBinary11); //Delete task

	// 	osi_SyncObjCreate(&xBinary12); //cce Sensor Interrupt Task

	// 	osi_SyncObjCreate(&xBinary13); //Check tasks End

	// #ifdef CHECK_WATER_MARK

	// 	osi_SyncObjCreate(&xBinary14); //Check Water Mark

	// #endif

	// 	osi_SyncObjCreate(&xBinary15); //USB Mode

	// 	osi_SyncObjCreate(&xBinary16); //USB activate

	// 	osi_SyncObjCreate(&xBinary17); //Internet Application

	Net_sta_group = xEventGroupCreate();

	// osi_LockObjCreate(&xMutex1); //Used For SPI Bus
	xMutex1 = xSemaphoreCreateMutex();

	// osi_LockObjCreate(&xMutex2); //Used For SimpleLink
	xMutex2 = xSemaphoreCreateMutex();

	// osi_LockObjCreate(&xMutex3); //Used For cJSON
	xMutex3 = xSemaphoreCreateMutex();

	// osi_LockObjCreate(&xMutex4); //Used For UART
	xMutex4 = xSemaphoreCreateMutex();

	// osi_LockObjCreate(&xMutex5); //Used for Post_Data_Buffer Lock
	xMutex5 = xSemaphoreCreateMutex();

	// osi_MsgQCreate(&xQueue0, "xQueue0", sizeof(SensorMessage), 8); //create queue used for sensor value save
	xQueue0 = xQueueCreate(8, sizeof(SensorMessage));

	// osi_MsgQCreate(&xQueue1, "xQueue1", sizeof(uint8_t), 1); //Used for LED control task
	xQueue1 = xQueueCreate(1, sizeof(uint8_t));

	// osi_MsgQCreate(&xQueue2, "xQueue2", sizeof(uint16_t), 1); //Used for bell conctrol task
	xQueue2 = xQueueCreate(1, sizeof(uint16_t));

	// osi_MsgQCreate(&xQueue3, "xQueue3", sizeof(uint8_t), 2); //Used for WatchDog and Sleep Timer Application
	xQueue3 = xQueueCreate(2, sizeof(uint8_t));

	// osi_MsgQCreate(&HttpMsg_Queue, "HttpMsg_Queue", sizeof(uint8_t), 3); //Used for http server resolve
	HttpMsg_Queue = xQueueCreate(3, sizeof(uint8_t));
}

/*******************************************************************************
//Read System Status                             
*******************************************************************************/
static short Read_System_Status(uint8_t *read_flag, uint8_t buf_len)
{
	uint8_t n_read;

	for (n_read = 0; n_read < RETRY_TIME_OUT; n_read++)
	{
		at24c08_ReadData(SYSTEM_STATUS_ADDR, read_flag, buf_len, 1); //Read System Status

		if ((!strcmp((char const *)read_flag, SYSTEM_ON)) || (!strcmp((char const *)read_flag, SYSTEM_OFF))) //Check System Status
		{
			return SUCCESS;
		}
		else
		{
			MAP_UtilsDelay(2000000); //Delay About 150ms
		}
	}

	return FAILURE;
}

/*******************************************************************************
//Watch Dog Rest Task
*******************************************************************************/
void WatchDog_Reset_Task(void *pvParameters)
{
	uint8_t msg_val;
	int rtc_result = 0;

	for (;;)
	{
		// osi_MsgQRead(&xQueue3, &msg_val, OSI_WAIT_FOREVER); //Wait WatchDog and SleepTimer Task Start Message
		xQueueReceive(xQueue3, &msg_val, portMAX_DELAY);

		if (msg_val == MSG_SLP_VAL)
		{
			sleep_time = fn_t_sleep_time_min();

			slp_t_ms = 0;

			//设置休眠唤醒时间，执行即计时，非休眠则进入回调
			// rtc_result = SetTimerAsWkUp(sleep_time, 1); //set Timer as a wake up source from low power modes
			// if (rtc_result < 0)
			// {
			// 	Reboot_MCU(); //Reboot the MCU
			// }
		}
		// else if (msg_val == MSG_WD_VAL)
		// {
		// 	osi_EnterCritical(); //enter critical

		// 	WatchdogAck(); //Acknowledge the watchdog

		// 	osi_ExitCritical(0); //exit crtitcal
		// }

		if (sys_run_time > SYS_RUN_TIMEOUT)
		{
			Reboot_MCU(); //Reboot the MCU
		}
	}
}

esp_timer_handle_t http_timer_suspend_p = NULL;
void timer_app_cb(void *arg);
esp_timer_handle_t timer_app_handle = NULL; //定时器句柄
esp_timer_create_args_t timer_app_arg = {
	.callback = &timer_app_cb,
	.arg = NULL,
	.name = "App_Timer"};

/*******************************************************************************
//TimerA0 Interrupt Handler
//Reset The Watch Dog
*******************************************************************************/
void timer_app_cb(*arg)
{
	// Timer_IF_InterruptClear(TIMERA0_BASE); //Clear the timer interrupt bit
	sys_run_time += 1;

	SYS_WD_SEC += 1;

	if (SYS_WD_SEC > RESET_WATCH_DOG_TIME)
	{
		SYS_WD_SEC = 0;

		uint8_t msg_wd_val = MSG_WD_VAL;

		// osi_MsgQWrite(&xQueue3, &msg_wd_val, OSI_NO_WAIT); //send to WatchDog and SleepTimer message
		xQueueSendFromISR(xQueue3, &msg_wd_val, NULL);
	}

	if (sleep_time > 0)
	{
		slp_t_ms += 1;

		if ((slp_t_ms + RESET_SLEEP_TIME) >= (10 * sleep_time))
		{
			slp_t_ms = 0;

			// osi_SyncObjSignalFromISR(&xBinary9); //check Task start time
			vTaskNotifyGiveFromISR(xBinary9, NULL);
		}
	}
}

/*******************************************************************************
//callback function for gpio interrupt handler
*******************************************************************************/
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
	uint32_t gpio_num = (uint32_t)arg;

	if (gpio_num == BUTTON_PIN) //button push wake up
	{
		// osi_SyncObjSignalFromISR(&xBinary1);
		vTaskNotifyGiveFromISR(xBinary1, NULL);
	}

	else if (gpio_num == ACCE_SRC_WKUP) //acce sensor int
	{
		// osi_SyncObjSignalFromISR(&xBinary12);
		vTaskNotifyGiveFromISR(xBinary12, NULL);
	}
	else if (gpio_num == USB_SRC_WKUP) //usb int wake up
	{
		// osi_SyncObjSignalFromISR(&xBinary15);
		vTaskNotifyGiveFromISR(xBinary15, NULL);
	}
}
/*******************************************************************************
//IO init
*******************************************************************************/
void PinMuxConfig(void)
{
	gpio_config_t io_conf;

	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = (1ULL << G_LED_PIN) | (1ULL << R_LED_PIN) | (1ULL << BELL_PIN);
	io_conf.pull_down_en = 1;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);

	io_conf.intr_type = GPIO_INTR_ANYEDGE;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 0;
	io_conf.pin_bit_mask = ((1ULL << BUTTON_PIN) | (1ULL << USB_SRC_WKUP) | (1ULL << ACCE_SRC_WKUP));
	gpio_config(&io_conf);

	gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
	gpio_isr_handler_add(BUTTON_PIN, gpio_isr_handler, (void *)BUTTON_PIN);
	gpio_isr_handler_add(USB_SRC_WKUP, gpio_isr_handler, (void *)USB_SRC_WKUP);
	gpio_isr_handler_add(ACCE_SRC_WKUP, gpio_isr_handler, (void *)ACCE_SRC_WKUP);
}

/*******************************************************************************
                               MAIN FUNCTION                                
*******************************************************************************/
void app_main(void)
{
	uint8_t push_time = 0;
	uint8_t read_flag[16] = {0};
	uint8_t read_flag2[16] = {0};
	unsigned long ulResetCause = 0;

	//ESP32 无关
	// BoardInit();	//Board Initialization
	//所有IO初始化
	PinMuxConfig(); //Configure The Peripherals

	//ESP 在任务中初始化
	I2C_Init(); //Configuring IIC Bus

	UserSpiInit(); //Configuring SPI Bus

#ifdef USE_WD
	// 看门狗 esp在系统中设置
	// WDT_IF_Init(NULL, 80000000 * 10); //Initialize WDT 10s
	// CHECK_ERROR_CODE(esp_task_wdt_init(10, false), ESP_OK);
#endif

	// ulResetCause = PRCMSysResetCauseGet(); //Get the reset cause
	ulResetCause = esp_reset_reason();
	if (ulResetCause == ESP_RST_WDT) //clean boot the system
	{
		Red_Led_Flashed(1, 2);
		Green_Led_Flashed(1, 2);
		System_ERROR_Save(WD_RESET_ERR); //save ERROR data
		esp_restart();
		// Reboot_MCU();					 //Reboot the MCU
	}
	else if (ulResetCause == ESP_RST_POWERON) //power on wakeup
	{
		Timer_IC_Init(); //PCF8563 init
		unsigned long time_ic_t = Read_UnixTime();
		unsigned long last_update_t = at24c08_read(LAST_UPDATE_TIME_ADDR); //read time
		if ((time_ic_t > (last_update_t + MAX_SLP_TIME)) || (time_ic_t < last_update_t))
		{
			Timer_IC_Reset_Time(); //PCF8563 Reset Time 2017-01-01 00:00:00
		}

		usb_status = 0;
		at24c08_write_byte(USB_FLAG_ADDR, usb_status);
	}

	// platform_init();												 //Initialize the platform
	if (Read_System_Status(read_flag, sizeof(read_flag)) == SUCCESS) //Check System Status
	{
		//二值化初始化 ，ESP 使用任务通知
		System_Variables_Init(); //System Variables Init

		OperateData_Read(); //Read Operate Data in At24c08
		MetaData_Read();	//Read Metadata in At24c08
		CaliData_Read();	//Read Calidata in At24c08

		//UART初始化
		Set_Uart_Int_Source(); //Set Peripheral Interrupt Source

		//wifi初始化
		init_wifi();
		// VStartSimpleLinkSpawnTask(SPAWN_TASK_PRIORITY); //Start the SimpleLink Host

		if (!strcmp((char const *)read_flag, SYSTEM_ON)) //"SYSTEM ON"
		{
			//初始化睡眠唤醒，ESP可在进入中断前再进行设置
			// Set_GPIO_AsWkUp();								  //setting GPIO wakeup source
			// lp3p0_setup_power_policy(POWER_POLICY_HIBERNATE); //lowest power mode

			xTaskCreate(Tasks_Check_Task, "Tasks_Check_Task", 256, NULL, 1, &xBinary13);		   //Create Check Tasks End
			xTaskCreate(Green_LedFlashed_Task, "Green_LedFlashed_Task", 256, NULL, 7, &xBinary17); //Create GREEN LED Blink Task When Internet Application
			xTaskCreate(Usb_Mode_Task, "Usb_Mode_Task", 320, NULL, 3, &xBinary15);				   //USB Mode Led Flash Task
			xTaskCreate(WatchDog_Reset_Task, "WatchDog_Reset_Task", 384, NULL, 9, NULL);		   //Watch Dog Rest Task
																								   //      xTaskCreate( UartRevTask, "UartRevTask",512,NULL, 9, NULL );  //Create UART recive Task
			xTaskCreate(UartParseTask, "UartParseTask", 1280, NULL, 7, NULL);					   //Create UART Task
			xTaskCreate(ButtonPush_Int_Task, "ButtonPush_Int_Task", 384, NULL, 9, NULL);		   //Create Button Interrupt_Task

			if (ulResetCause == ESP_RST_POWERON) // Check the wakeup source-except PRCM_HIB_EXIT
			{
				MAP_UtilsDelay(10000000); //Delay About 750ms
				Green_Led_Bell_Sound(200);
				Sensors_Init();														//Senosrs Init when power on
				xTaskCreate(UpdateTimeTask, "UpdateTimeTask", 1024, NULL, 7, NULL); //Create UpdataTime Task
			}
			else
			{
				xTaskCreate(MainTask_Create, "MainTask_Create Task", 512, NULL, 9, NULL); //create the system work task
			}

			goto end;
		}
		else //"SYSTEM OFF"
		{
			push_time = 0;
			while (!gpio_get_level(BUTTON_PIN)) //Wait Button Up
			{
				if (push_time++ == 20) //About 3s,POWER ON
				{
					SET_GREEN_LED_ON();
					bell_makeSound(200);
				}

				if (push_time == 50) //About 7.5s,Creat AP Mode Task
				{
					bell_makeSound(200);
					while (!gpio_get_level(BUTTON_PIN)) //Wait Button Up
					{
						Green_Red_Led_Flashed(1, 6);
						push_time += 12;
						if (push_time >= 100)
						{
							break;
						}
					}
				}

				if (push_time >= 100) //About 15s,Creat Factory Reset Task
				{
					SET_GREEN_LED_OFF();
					bell_makeSound(200);
					while (!gpio_get_level(BUTTON_PIN)) //Wait Button Up
					{
						// WatchdogAck(); //Acknowledge the watchdog
						Red_Led_Flashed(1, 3);
					}
					break;
				}
				MAP_UtilsDelay(2000000); //Delay About 150ms
										 // WatchdogAck();			 //Acknowledge the watchdog
			}

			if (push_time >= 20) //Push Time>=3s or USB connected Power ON
			{
				SET_GREEN_LED_OFF();													//LED OFF
				at24c08_WriteData(SYSTEM_STATUS_ADDR, SYSTEM_ON, strlen(SYSTEM_ON), 1); //Restor The System Status-ON
				Sensors_Init();															//Senosrs Init when power on
																						// Set_GPIO_AsWkUp();														// setting GPIO wakeup source
																						// lp3p0_setup_power_policy(POWER_POLICY_HIBERNATE);						//Setting up HIBERNATE mode for the system

				xTaskCreate(Tasks_Check_Task, "Tasks_Check_Task", 256, NULL, 1, &xBinary13);		   //Create Check Tasks End
				xTaskCreate(Green_LedFlashed_Task, "Green_LedFlashed_Task", 256, NULL, 7, &xBinary17); //Create GREEN LED Blink Task When Internet Application
				xTaskCreate(Usb_Mode_Task, "Usb_Mode_Task", 320, NULL, 3, &xBinary15);				   //USB Mode Led Flash Task
				xTaskCreate(WatchDog_Reset_Task, "WatchDog_Reset_Task", 384, NULL, 9, NULL);		   //Watch Dog Rest Task
																									   //        xTaskCreate( UartRevTask, "UartRevTask",512,NULL, 9, NULL );  //Create UART recive Task
				xTaskCreate(UartParseTask, "UartParseTask", 1280, NULL, 7, NULL);					   //Create UART Task
				xTaskCreate(ButtonPush_Int_Task, "ButtonPush_Int_Task", 384, NULL, 9, NULL);		   //Create Button Interrupt_Task

				if (push_time >= 100) //Push Time>=15s,Creat Factory Reset Task
				{
					xTaskCreate(F_ResetTask, "FactoryResetTask", 768, NULL, 7, NULL); //Device Factory Reset
					goto end;
				}
				else if (push_time >= 50) //15s<=Push Time>=5s
				{
					// xTaskCreate(Httpserver_parse_Task, "Httpserver_parse_Task", 896, NULL, 5, NULL);
					xTaskCreate(WlanAPMode, "WlanAPMode", 1600, NULL, 7, NULL); //Create Wlan AP Mode task
					goto end;
				}
				else //7.5s<=Push Time>=3s,Power ON
				{
					at24c08_ReadData(PRODUCTURI_FLAG_ADDR, read_flag, sizeof(read_flag), 1); //read product id flag
					if (!strcmp((char const *)read_flag, PRODUCT_URI))
					{
						at24c08_ReadData(SSID_FLAG_ADDR, read_flag, sizeof(read_flag), 1);		//read ssid flag
						at24c08_ReadData(DATAURI_FLAG_ADDR, read_flag2, sizeof(read_flag2), 1); //read datapost addr
						if ((!strcmp((char const *)read_flag, "SSID")) && (!strcmp((char const *)read_flag2, DATA_URI)))
						{
							xTaskCreate(UpdateTimeTask, "UpdateTimeTask", 1024, NULL, 7, NULL); //Create UpdataTime Task
							goto end;
						}
						else if (!strcmp((char const *)read_flag, "SSID"))
						{
							xTaskCreate(ApikeyGetTask, "ApikeyGetTask", 1024, NULL, 7, NULL); //Create ApiKeyGetTask
							goto end;
						}
						else if (!strcmp((char const *)read_flag2, DATA_URI))
						{
							xTaskCreate(MainTask_Create, "MainTask_Create Task", 512, NULL, 9, NULL); //create the system work task
							goto end;
						}
						else if (gpio_get_level(USB_PIN)) //read usb pin status
						{
							xTaskCreate(Usb_Set_Mode, "Usb_Set_Mode", 512, NULL, 5, NULL); //Create Wait Uart Set Task task
							goto end;
						}
						else //check ssid addr
						{
							// xTaskCreate(Httpserver_parse_Task, "Httpserver_parse_Task", 896, NULL, 5, NULL);
							xTaskCreate(WlanAPMode, "WlanAPMode", 1600, NULL, 7, NULL); //Create Wlan AP Mode task
							goto end;
						}
					}
					else //check product id
					{
						xTaskCreate(F_ResetTask, "FactoryResetTask", 768, NULL, 7, NULL); //Device Factory Reset
						goto end;
					}
				}
			}
			else //push time<3s
			{
				SET_Power_OFF(); //Power OFF
				goto end;
			}
		}
	}
	else //except system ON and OFF
	{
		at24c08_write(LAST_UPDATE_TIME_ADDR, 0); //reset last update time
		SET_Power_OFF();						 //Power OFF
		goto end;
	}

end:

	// Timer_IF_Init(PRCM_TIMERA0, TIMERA0_BASE, TIMER_CFG_PERIODIC, TIMER_A, 0);
	// Timer_IF_IntSetup(TIMERA0_BASE, TIMER_A, TIMERA0IntHandler);
	// Timer_IF_InterruptClear(TIMERA0_BASE);		//Clear the timer interrupt bit
	// Timer_IF_Start(TIMERA0_BASE, TIMER_A, 100); //Start TIMERA0 values in nSec
	// osi_start(); // Start the task scheduler

	esp_err_t err = esp_timer_create(&timer_app_arg, &timer_app_handle);
	err = esp_timer_start_periodic(timer_app_handle, 10000); //创建定时器，单位us，定时10ms
	if (err != ESP_OK)
	{
		ESP_LOGI(TAG, "timer heart create err code:%d\n", err);
	}
	for (;;)
		; //never go here
}

/*******************************************************************************
                                      END         
*******************************************************************************/