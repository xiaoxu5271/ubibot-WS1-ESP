#ifndef __S_C
#define __S_C

#include "freertos/event_groups.h"
#include "freertos/task.h"
// #include "Bluetooth.h"
#include "esp_wifi.h"

void init_wifi(void);
void start_user_wifi(void);
void stop_user_wifi(void);
// void Net_Switch(void);
void start_softap(void);
// bool Check_Wifi(uint8_t *ssid, int8_t *rssi);
// int Tcp_Send(int sock, char *Send_Buff);

void osi_Scan_Wifi_List(char *rssi_ssid, int *rssi_val, bool uart_printf); //Scan WIFI LIST with locked//
short WiFi_Connect_Test(void);                                             //WiFi connect test
short osi_WiFi_Connect_Test(void);                                         //WiFi connect test with locked
void Wlan_Disconnect_AP(void);                                             //wlan disconnect form the ap//
int Scan_Wifi_List(char *rssi_ssid, int *rssi_val, bool uart_printf);      //Scan WIFI LIST//
int WlanConnect(void);                                                     //Wlan Connect To The Accesspoint//

// extern uint8_t wifi_work_sta;    //wifi开启状态
extern uint8_t start_AP;
extern uint16_t Net_ErrCode; //
extern bool WIFI_STA;

EventGroupHandle_t Net_sta_group;
#define CONNECTED_BIT (1 << 0)    //网络连接
#define ACTIVED_BIT (1 << 1)      //激活
#define WIFI_S_I_BIT (1 << 2)     //wifi是否进入初始化状态
#define WIFI_I_BIT (1 << 3)       //wifi初始化完成状态
#define MQTT_W_S_BIT (1 << 4)     //WIFI MQTT 启动
#define MQTT_W_C_BIT (1 << 5)     //WIFI MQTT 连接
#define WIFI_S_BIT (1 << 6)       //wifi启动状态
#define WIFI_C_BIT (1 << 7)       //wifi连接状态
#define MQTT_INITED_BIT (1 << 14) //MQTT初始化完成
#define ACTIVE_S_BIT (1 << 15)    //激活中
#define TIME_CAL_BIT (1 << 16)    //时间校准成功
#define BLE_RESP_BIT (1 << 17)    //蓝牙超时回复标志

#define TCP_PORT 5001

// static const int CONNECTED_BIT = BIT0;
// static const int AP_STACONNECTED_BIT = BIT0;
// extern TaskHandle_t my_tcp_connect_Handle;

#define SOFT_AP_MAX_CONNECT 2 //最多的连接点

#endif