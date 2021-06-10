#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "tcpip_adapter.h"
// #include "esp_wpa2.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_smartconfig.h"
/*  user include */
#include "esp_log.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "MsgType.h"
#include "at24c08.h"
#include "JsonParse.h"
#include "PeripheralDriver.h"
#include "UartTask.h"

#include "app_config.h"
#define TAG "NET_CFG"

// TaskHandle_t my_tcp_connect_Handle;
// EventGroupHandle_t tcp_event_group;

uint16_t Net_ErrCode = 0;
bool scan_flag = false;
char AP_SSID[15] = {0};

#define WLAN_SCAN_COUNT 20
#define ERROR_CODE 0xffff

extern char SEC_TYPE[8];
extern char SSID_NAME[32];
extern char PASS_WORD[64];

extern bool ap_mode_status;
extern bool AP_MODE_END_FLAG;
extern TaskHandle_t xBinary13; //Task End Check
extern TaskHandle_t GR_LED_TaskHandle;
extern TaskHandle_t GR_LED_FAST_TaskHandle;
extern char UartGet[UART_REV_BUF_LEN];
extern volatile unsigned long POST_NUM;
extern volatile unsigned long DELETE_ADDR, POST_ADDR, WRITE_ADDR;
extern volatile uint16_t sys_run_time;

extern void UpdateTimeTask(void *pvParameters);
extern void ApikeyGetTask(void *pvParameters);

extern SemaphoreHandle_t xMutex2; //Used for SimpleLink Lock
extern SemaphoreHandle_t xMutex4; //Used for UART Lock

char wifi_buf[180];

void WlanAPMode(void *pvParameters);

void timer_wifi_cb(void *arg);
esp_timer_handle_t timer_wifi_handle = NULL; //定时器句柄
esp_timer_create_args_t timer_wifi_arg = {
    .callback = &timer_wifi_cb,
    .arg = NULL,
    .name = "Wifi_Timer"};

void timer_wifi_cb(void *arg)
{
    //超时未获取到IP
    start_user_wifi();
}

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        if (scan_flag == false)
        {
            esp_wifi_connect();
        }
    }

    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED)
    {
        esp_timer_start_once(timer_wifi_handle, 10000 * 1000);
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {

        // Net_sta_flag = false;
        xEventGroupClearBits(Net_sta_group, CONNECTED_BIT);
        // Start_Active();

        wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t *)event_data;
        if (event->reason >= 200)
        {
            Net_ErrCode = event->reason;
        }
        ESP_LOGI(TAG, "Wi-Fi disconnected,reason:%d", event->reason);
        if (scan_flag == false)
        {
            esp_wifi_connect();
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        esp_timer_stop(timer_wifi_handle);
        xEventGroupSetBits(Net_sta_group, CONNECTED_BIT);
        // Start_Active();
        // ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        // ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    }

    else if (event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
    else
    {
        ESP_LOGI(TAG, "event_base=%s,event_id=%d", event_base, event_id);
    }
}

void init_wifi(void) //
{
    xEventGroupSetBits(Net_sta_group, WIFI_S_I_BIT);
    char temp[6] = {0};
    char series_number[16] = {0};
    // tcpip_adapter_init();
    // Net_sta_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_timer_create(&timer_wifi_arg, &timer_wifi_handle));

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));
    // ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MAX_MODEM)); //最大省电
    // ESP_ERROR_CHECK(esp_wifi_get_config(ESP_IF_WIFI_STA, &s_staconf));
    // wifi_config_t s_staconf;
    // memset(&s_staconf.sta, 0, sizeof(s_staconf));
    // strcpy((char *)s_staconf.sta.ssid, wifi_data.wifi_ssid);
    // strcpy((char *)s_staconf.sta.password, wifi_data.wifi_pwd);

    // ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &s_staconf));

    // xEventGroupSetBits(Net_sta_group, WIFI_S_BIT);
    // start_user_wifi();
    osi_at24c08_ReadData(SERISE_NUM_ADDR, (uint8_t *)series_number, sizeof(series_number), 1);
    memcpy(temp, series_number, 5);
    snprintf(AP_SSID, 15, "Ubibot-%s", temp);
    ESP_LOGI(TAG, "AP_SSID:%s", AP_SSID);

    xEventGroupSetBits(Net_sta_group, WIFI_I_BIT);
}

void stop_user_wifi(void)
{
    if ((xEventGroupGetBits(Net_sta_group) & WIFI_S_BIT) == WIFI_S_BIT)
    {
        xEventGroupClearBits(Net_sta_group, WIFI_S_BIT);
        esp_err_t err = esp_wifi_stop();
        if (err == ESP_ERR_WIFI_NOT_INIT)
        {
            return;
        }
        ESP_ERROR_CHECK(err);
        ESP_LOGI(TAG, "turn off WIFI! \n");
    }
    else
    {
        ESP_LOGI(TAG, "WIFI not start! \n");
    }
}

void start_user_wifi(void)
{
    //是否开启初始化
    if ((xEventGroupGetBits(Net_sta_group) & WIFI_S_I_BIT) == WIFI_S_I_BIT)
    {
        xEventGroupWaitBits(Net_sta_group, WIFI_I_BIT, false, false, -1);
    }
    else
    {
        init_wifi();
    }

    ESP_LOGI(TAG, "%d,start_user_wifi", __LINE__);
    uint8_t read_len;
    if ((xEventGroupGetBits(Net_sta_group) & WIFI_S_BIT) == WIFI_S_BIT)
    {
        esp_err_t err = esp_wifi_stop();
        if (err == ESP_ERR_WIFI_NOT_INIT)
        {
            return;
        }
        ESP_ERROR_CHECK(err);
    }
    xEventGroupSetBits(Net_sta_group, WIFI_S_BIT);

    // esp_netif_create_default_wifi_sta();
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    wifi_config_t s_staconf;
    memset(&s_staconf.sta, 0, sizeof(s_staconf));
    memset(SSID_NAME, 0, sizeof(SSID_NAME));
    memset(PASS_WORD, 0, sizeof(PASS_WORD));

    read_len = osi_at24c08_read_byte(SSID_LEN_ADDR);
    if (read_len > sizeof(SSID_NAME))
    {
        read_len = sizeof(SSID_NAME);
    }
    osi_at24c08_ReadData(SSID_ADDR, (uint8_t *)SSID_NAME, read_len, 0); //Read Wifi SSID
    ESP_LOGI(TAG, "%d,SSID_NAME=%s", __LINE__, SSID_NAME);
    read_len = osi_at24c08_read_byte(PASSWORD_LEN_ADDR);
    if (read_len > sizeof(PASS_WORD))
    {
        read_len = sizeof(PASS_WORD);
    }
    osi_at24c08_ReadData(PASSWORD_ADDR, (uint8_t *)PASS_WORD, read_len, 0); //Read Wifi Password

    strcpy((char *)s_staconf.sta.ssid, SSID_NAME);
    strcpy((char *)s_staconf.sta.password, PASS_WORD);
    s_staconf.sta.scan_method = 1;
    s_staconf.sta.sort_method = 0;

    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &s_staconf));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "turn on WIFI! \n");
}

#define DEFAULT_SCAN_LIST_SIZE 5

/* The examples use WiFi configuration that you can set via project configuration menu.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/

void start_softap(void)
{
    //是否开启初始化
    if ((xEventGroupGetBits(Net_sta_group) & WIFI_S_I_BIT) == WIFI_S_I_BIT)
    {
        xEventGroupWaitBits(Net_sta_group, WIFI_I_BIT, false, false, -1);
    }
    else
    {
        init_wifi();
    }

    if ((xEventGroupGetBits(Net_sta_group) & WIFI_S_BIT) == WIFI_S_BIT)
    {
        esp_err_t err = esp_wifi_stop();
        if (err == ESP_ERR_WIFI_NOT_INIT)
        {
            return;
        }
        ESP_ERROR_CHECK(err);
    }
    xEventGroupSetBits(Net_sta_group, WIFI_S_BIT);

    // esp_netif_create_default_wifi_ap();

    wifi_config_t wifi_config = {
        .ap = {
            // .ssid = AP_SSID,
            // .ssid_len = 5,
            // .channel = EXAMPLE_ESP_WIFI_CHANNEL,
            // .password = "12345678",
            .max_connection = SOFT_AP_MAX_CONNECT,
            .authmode = WIFI_AUTH_OPEN},
    };

    memcpy(wifi_config.ap.ssid, AP_SSID, sizeof(AP_SSID));
    // wifi_config.ap.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    tcpip_adapter_ip_info_t ip_info = {
        .ip.addr = ipaddr_addr("192.168.1.1"),
        .netmask.addr = ipaddr_addr("255.255.255.0"),
        .gw.addr = ipaddr_addr("192.168.1.1"),
    };

    ESP_ERROR_CHECK(tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP));
    ESP_ERROR_CHECK(tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_AP, &ip_info));
    ESP_ERROR_CHECK(tcpip_adapter_dhcps_start(TCPIP_ADAPTER_IF_AP));

    ESP_LOGI(TAG, "wifi_init_softap finished. ");
}

int Tcp_Send(int sock, const char *Send_Buff, uint16_t Send_Buff_len, uint8_t flag)
{
    int to_write = Send_Buff_len;
    int written = 0;
    // ESP_LOGI(TAG, "Send_Buff:%s,Len:%d,sock:%d", Send_Buff, Send_Buff_len, sock);
    while (to_write > 0)
    {
        written = send(sock, Send_Buff + (Send_Buff_len - to_write), to_write, flag);
        if (written < 0)
        {
            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
            return written;
        }
        to_write -= written;
    }

    return written;
}

//TCP 收发数据
static void do_retransmit(const int sock)
{
    int len;
    esp_err_t ret;
    char rx_buffer[1024];

    do
    {
        len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        if (len < 0)
        {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
        }
        else if (len == 0)
        {
            ESP_LOGW(TAG, "Connection closed");
        }
        else
        {
            rx_buffer[len] = 0; // Null-terminate whatever is received and treat it like a string
            ESP_LOGI(TAG, "Received %d,strlen:%d, bytes: %s,sock:%d", len, strlen(rx_buffer), rx_buffer, sock);
            // send(sock, rx_buffer, len, 0);
            ret = ParseTcpUartCmd(rx_buffer);
            // ESP_LOGI(TAG, "TCP ret:%d", ret);
        }
    } while (len > 0);
}

static void tcp_server_task(void *pvParameters)
{
    char addr_str[128];

    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

    struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
    dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr_ip4->sin_family = AF_INET;
    dest_addr_ip4->sin_port = htons(TCP_PORT);
    ip_protocol = IPPROTO_IP;

    int listen_sock = socket(AF_INET, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0)
    {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0)
    {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(TAG, "IPPROTO: %d", AF_INET);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", TCP_PORT);

    err = listen(listen_sock, 1);
    if (err != 0)
    {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    while (1)
    {

        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
        uint addr_len = sizeof(source_addr);
        ESP_LOGI(TAG, "%d", __LINE__);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        ESP_LOGI(TAG, "%d", __LINE__);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        // Convert ip address to string
        if (source_addr.sin6_family == PF_INET)
        {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
            ESP_LOGI(TAG, "%d", __LINE__);
        }
        else if (source_addr.sin6_family == PF_INET6)
        {
            inet6_ntoa_r(source_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
            ESP_LOGI(TAG, "%d", __LINE__);
        }
        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

        do_retransmit(sock);

        shutdown(sock, 0);
        close(sock);
    }

CLEAN_UP:
    esp_restart();
    close(listen_sock);
    vTaskDelete(NULL);
}

/******************************************************************************/

/*******************************************************************************
//wlan disconnect form the ap
*******************************************************************************/
void Wlan_Disconnect_AP(void)
{
    stop_user_wifi();
}

/*******************************************************************************
//Scan WIFI LIST
//return:rssi value
*******************************************************************************/
int Scan_Wifi_List(char *rssi_ssid, int *rssi_val, bool uart_printf)
{
    uint8_t i;
    uint8_t read_len;
    uint16_t ScanAP = 0;
    int Wifi_List_Val = -127;
    char scan_buf[96];
    char utctime[21] = {0};
    char bssid_buf[5][18] = {0};
    signed char bssid_rssi_val[5] = {0};

    uint16_t number = DEFAULT_SCAN_LIST_SIZE;
    wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE];
    memset(ap_info, 0, sizeof(ap_info));
    scan_flag = true;

    memset(SSID_NAME, 0, sizeof(SSID_NAME));

    read_len = osi_at24c08_read_byte(SSID_LEN_ADDR);
    ESP_LOGI(TAG, "%d,read_len=%d", __LINE__, read_len);

    if (read_len > sizeof(SSID_NAME))
    {
        read_len = sizeof(SSID_NAME);
    }

    osi_at24c08_ReadData(SSID_ADDR, (uint8_t *)SSID_NAME, read_len, 0); //Read Wifi SSID
    ESP_LOGI(TAG, "%d,SSID_NAME=%s", __LINE__, SSID_NAME);

    start_user_wifi();

    if (esp_wifi_scan_start(NULL, true) != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_scan_start FAIL");
        scan_flag = false;
        return ERROR_CODE;
    }
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_info));
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ScanAP));
    ESP_LOGI(TAG, "Total APs scanned = %u", ScanAP);
    // for (int i = 0; (i < DEFAULT_SCAN_LIST_SIZE) && (i < ap_count); i++)
    // {
    //     printf("{\"SSID\":\"%s\",\"rssi\":%d}\n\r", ap_info[i].ssid, ap_info[i].rssi);
    // }
    if (uart_printf)
    {
        //  xSemaphoreTake(xMutex4, -1); //UART Semaphore Take
        xSemaphoreTake(xMutex4, -1);

        printf("{\"WiFi_List_Sum\":%d}\r\n", ScanAP);

        //  xSemaphoreGive(xMutex4); //UART Semaphore Give
        xSemaphoreGive(xMutex4);
    }

    if (ScanAP > 0)
    {
        if (rssi_val != NULL && rssi_ssid != NULL)
            *rssi_val = -127;

        for (i = 0; i < ScanAP; i++)
        {
            if (uart_printf)
            {
                memset(scan_buf, 0, sizeof(scan_buf));

                Web_Wifi_Set(scan_buf, sizeof(scan_buf), (char *)ap_info[i].ssid, ap_info[i].rssi, ap_info[i].authmode, (char *)ap_info[i].bssid, 1);

                //  xSemaphoreTake(xMutex4, -1); //UART Semaphore Take
                xSemaphoreTake(xMutex4, -1);
                printf("%s\r\n", scan_buf);
                //        printf("{\"SSID\":\"%s\",\"rssi\":%d,\"type\":%d}\r\n",ap_info[i].ssid,ap_info[i].rssi,ap_info[i].sec_type);
                xSemaphoreGive(xMutex4);
                //  xSemaphoreGive(xMutex4); //UART Semaphore Give
            }

            if (i < 5)
            {
                snprintf(bssid_buf[i], 18, "%02x:%02x:%02x:%02x:%02x:%02x", ap_info[i].bssid[0], ap_info[i].bssid[1], ap_info[i].bssid[2], ap_info[i].bssid[3], ap_info[i].bssid[4], ap_info[i].bssid[5]);

                bssid_rssi_val[i] = ap_info[i].rssi;
            }

            if (!strcmp((char const *)ap_info[i].ssid, (char const *)SSID_NAME))
            {
                memset(SEC_TYPE, 0, sizeof(SEC_TYPE));

                if (ap_info[i].authmode == WIFI_AUTH_OPEN)
                {
                    memcpy(SEC_TYPE, "OPEN", strlen("OPEN")); //WiFi sec type
                }
                else if (ap_info[i].authmode == WIFI_AUTH_WEP)
                {
                    memcpy(SEC_TYPE, "WEP", strlen("WEP")); //WiFi sec type
                }
                else
                {
                    memcpy(SEC_TYPE, "WPA", strlen("WPA")); //WiFi sec type
                }

                if (Wifi_List_Val < ap_info[i].rssi)
                {
                    Wifi_List_Val = ap_info[i].rssi;
                }
            }
            if (rssi_val != NULL && rssi_ssid != NULL)
            {
                if (*rssi_val < ap_info[i].rssi)
                {
                    *rssi_val = ap_info[i].rssi;

                    memset(rssi_ssid, 0, 32);

                    memcpy(rssi_ssid, ap_info[i].ssid, strlen((char *)ap_info[i].ssid));
                }
            }
        }
        osi_Read_UTCtime(utctime, sizeof(utctime)); //read time

        memset(wifi_buf, 0, sizeof(wifi_buf));

        snprintf(wifi_buf, sizeof(wifi_buf), ",{\"created_at\":\"%s\",\"wifi\":\"%s,%d;%s,%d;%s,%d;%s,%d;%s,%d\"}", utctime, bssid_buf[0], bssid_rssi_val[0], bssid_buf[1], bssid_rssi_val[1], bssid_buf[2], bssid_rssi_val[2], bssid_buf[3], bssid_rssi_val[3], bssid_buf[4], bssid_rssi_val[4]);

        ESP_LOGI(TAG, "SCAN WIFI OK :\r\n%s", wifi_buf);
    }
    else
    {
        osi_Read_UTCtime(utctime, sizeof(utctime)); //read time
        memset(wifi_buf, 0, sizeof(wifi_buf));
        snprintf(wifi_buf, sizeof(wifi_buf), ",{\"created_at\":\"%s\",\"wifi\":\"\"}", utctime);

        ESP_LOGI(TAG, "SCAN WIFI FAIL :\r\n%s", wifi_buf);
    }

    stop_user_wifi();
    scan_flag = false;

    if ((Wifi_List_Val < 0) && (Wifi_List_Val > -100))
    {
        return Wifi_List_Val;
    }
    else
    {
        osi_System_ERROR_Save(SCN_WF_FLR_ERR); //save ERROR data

        return ERROR_CODE;
    }
}

/*******************************************************************************
//Wlan Connect To The Accesspoint
*******************************************************************************/
int WlanConnect(void)
{
    //osi_at24c08_ReadData(SECTYPE_ADDR,(uint8_t*)SEC_TYPE,sizeof(SEC_TYPE),1);  //Read Wifi Sectype

    start_user_wifi();
    //等网络连接
    if ((xEventGroupWaitBits(Net_sta_group, CONNECTED_BIT, false, true, 30000 / portTICK_RATE_MS) & CONNECTED_BIT) == CONNECTED_BIT)
    {
        return SUCCESS;
    }
    else
    {
        return FAILURE;
    }
}

/*******************************************************************************
//Scan WIFI LIST with locked
*******************************************************************************/
void osi_Scan_Wifi_List(char *rssi_ssid, int *rssi_val, bool uart_printf)
{
    //  xSemaphoreTake(xMutex2, -1); //SimpleLink Semaphore Take
    xSemaphoreTake(xMutex2, -1);

    Scan_Wifi_List(rssi_ssid, rssi_val, uart_printf);

    //  xSemaphoreGive(xMutex2); //SimpleLink Semaphore Give
    xSemaphoreGive(xMutex2);
}

/*******************************************************************************
//WiFi connect test
*******************************************************************************/
short WiFi_Connect_Test(void)
{
    short test_val = -1;

    //  Scan_Wifi_List(NULL,NULL,0);
    //  ConfigureSimpleLinkToDefaultState();  //configure the device to default state

    test_val = WlanConnect(); //wlan connect to the accesspoint
    // if (test_val >= 0)
    // {
    //     osi_at24c08_WriteData(SECTYPE_ADDR, (uint8_t *)SEC_TYPE, strlen(SEC_TYPE), 1); //save type in at24c08
    // }

    Wlan_Disconnect_AP(); //wlan disconnect form the ap

    // MAP_UtilsDelay(80000); //delay about 6ms

    return test_val;
}

/*******************************************************************************
//WiFi connect test with locked
*******************************************************************************/
short osi_WiFi_Connect_Test(void)
{
    short test_status = -1;

    //  xSemaphoreTake(xMutex2, -1); //SimpleLink Semaphore Take
    xSemaphoreTake(xMutex2, -1);

    test_status = WiFi_Connect_Test();

    //  xSemaphoreGive(xMutex2); //SimpleLink Semaphore Give
    xSemaphoreGive(xMutex2);

    return test_status;
}

/*******************************************************************************
//start wlan AP mode
*******************************************************************************/
void WlanAPMode(void *pvParameters)
{
    ESP_LOGI(TAG, "%d,WlanAPMode", __LINE__);
    xEventGroupClearBits(Task_Group, AP_MODE_END_BIT);
    char Readflag[16];
    int lRetVal = -1;
    int sock;
    char addr_str[128];
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;
    struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
    dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr_ip4->sin_family = AF_INET;
    dest_addr_ip4->sin_port = htons(TCP_PORT);
    ip_protocol = IPPROTO_IP;
    int listen_sock;

    ap_mode_status = 1;
    AP_MODE_END_FLAG = 1;

    // osi_SyncObjSignalFromISR(&xBinary13); //Start Tasks End Check
    if (xBinary13 != NULL)
        vTaskNotifyGiveFromISR(xBinary13, NULL);

    my_xTaskCreate(Green_Red_LedFlashed_Task, "Green_Red_LedFlashed_Task", 256, NULL, 7, &GR_LED_TaskHandle); //Create Green and Red Led Flash Task

    //  xSemaphoreTake(xMutex2, -1); //SimpleLink Semaphore Take
    xSemaphoreTake(xMutex2, -1);

    start_softap();

    listen_sock = socket(AF_INET, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0)
    {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        goto CLEAN_UP;
    }

    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0)
    {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(TAG, "IPPROTO: %d", AF_INET);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", TCP_PORT);

    err = listen(listen_sock, 1);
    if (err != 0)
    {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    // for (;;)
    // {
    //等待AP下游设备接入，获取到设备IP
    // while (!IS_IP_LEASED(g_ulStatus)) //wating for the client to connect
    // {
    //     osi_Sleep(5); //delay 5ms

    //     sys_run_time = 0; //clear system time out
    // }

    for (;;)
    {
        ESP_LOGI(TAG, "Socket listening");
        struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
        uint addr_len = sizeof(source_addr);
        ESP_LOGI(TAG, "%d", __LINE__);
        sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        ESP_LOGI(TAG, "%d", __LINE__);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        // Convert ip address to string
        if (source_addr.sin6_family == PF_INET)
        {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
            ESP_LOGI(TAG, "%d", __LINE__);
        }
        else if (source_addr.sin6_family == PF_INET6)
        {
            inet6_ntoa_r(source_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
            ESP_LOGI(TAG, "%d", __LINE__);
        }
        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

        for (;;)
        {

            memset(UartGet, 0, sizeof(UartGet));
            int len = recv(sock, UartGet, sizeof(UartGet) - 1, 0);
            if (len < 0)
            {
                ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
            }
            else if (len == 0)
            {
                ESP_LOGW(TAG, "Connection closed");
            }
            else
            {
                UartGet[len] = '\0';

                ESP_LOGI(TAG, "TCP_Recivebuffer:%s", UartGet);

                lRetVal = ParseTcpUartCmd(UartGet);
                if (lRetVal < 0)
                {
                    Tcp_Send(sock, FAILURED_CODE, strlen(FAILURED_CODE), 0);
                }
                else if (lRetVal == 0)
                {
                    Tcp_Send(sock, SUCCESSED_CODE, strlen(SUCCESSED_CODE), 0);
                }
                else if (lRetVal == 1) //Command:SetupWifi
                {
                    Tcp_Send(sock, SUCCESSED_CODE, strlen(SUCCESSED_CODE), 0);
                    break;
                }
                else if (lRetVal == 2) //{command: ReadProduct}
                {
                    memset(UartGet, 0, sizeof(UartGet));
                    Read_Product_Set(UartGet, sizeof(UartGet));
                    Tcp_Send(sock, UartGet, strlen(UartGet), 0);
                }
                else if (lRetVal == 3) //Command:ReadWifi
                {
                    memset(UartGet, 0, sizeof(UartGet));
                    Read_Wifi_Set(UartGet, sizeof(UartGet));
                    Tcp_Send(sock, UartGet, strlen(UartGet), 0);
                }
                else if (lRetVal == 4) //Command:GetLastError
                {
                    memset(UartGet, 0, sizeof(UartGet));
                    Read_System_ERROR_Code(UartGet, sizeof(UartGet));
                    Tcp_Send(sock, UartGet, strlen(UartGet), 0);
                }
                else if (lRetVal == 5) //command:BreakOut
                {
                    break;
                }
                else if (lRetVal == 6) //Command:ReadMetaData
                {
                    memset(UartGet, 0, sizeof(UartGet));
                    Cmd_Read_MetaData(UartGet, sizeof(UartGet));
                    Tcp_Send(sock, UartGet, strlen(UartGet), 0);
                }
                else if (lRetVal == 9) //Command:ReadData
                {
                    char data_len_buf[25];
                    unsigned long read_addr;
                    unsigned long save_data_num;

                    osi_at24c08_read_addr(); //Read Post Data Amount/Write Data/Post Data/Delete Data Address

                    if (POST_NUM)
                    {
                        save_data_num = POST_NUM;
                        read_addr = POST_ADDR;

                        snprintf(data_len_buf, sizeof(data_len_buf), "{\"save_data_sum\":%ld}", save_data_num);
                        Tcp_Send(sock, data_len_buf, strlen(data_len_buf), 0);
                        Tcp_Send(sock, "\r\n", strlen("\r\n"), 0);

                        while (save_data_num--)
                        {
                            memset(UartGet, 0, sizeof(UartGet));
                            osi_w25q_ReadData(read_addr, UartGet, 0xff, NULL);
                            Tcp_Send(sock, UartGet, strlen(UartGet), 0);
                            Tcp_Send(sock, "\r\n", strlen("\r\n"), 0);
                            read_addr += 1 + strlen(UartGet); //end whit '!'
                            sys_run_time = 0;                 //clear system time out
                        }
                    }
                }
                else if (lRetVal == 10) //Command:ClearData
                {
                    Tcp_Send(sock, SUCCESSED_CODE, strlen(SUCCESSED_CODE), 0);
                    vTaskDelete(GR_LED_TaskHandle);                                                                           //delete Green and Red Led Flash Task
                    SET_GREEN_LED_OFF();                                                                                      //Set Green Led Off
                    osi_Save_Data_Reset();                                                                                    //Nor Flash Memory Chip Reset
                    my_xTaskCreate(Green_Red_LedFlashed_Task, "Green_Red_LedFlashed_Task", 256, NULL, 7, &GR_LED_TaskHandle); //Create Green and Red Led Flash Task
                }
            }
            sys_run_time = 0; //clear system time out
        }
        sys_run_time = 0; //clear system time out
        if ((lRetVal == 1) || (lRetVal == 5))
        {
            break;
        }
    }
    //等待TCP发送
    vTaskDelay(1000 / portTICK_RATE_MS);

    // }

CLEAN_UP:
    // esp_restart();
    close(listen_sock);
    stop_user_wifi();

    vTaskDelete(GR_LED_TaskHandle); //delete Green and Red Led Flash Task
    SET_GREEN_LED_OFF();            //Set Green Led Off
    SET_RED_LED_OFF();              //Set Red Led Off

    if ((lRetVal == 1))
    {
        osi_bell_makeSound(200);
        my_xTaskCreate(Green_Red_Led_FastFlashed_Task, "Green_Red_Led_FastFlashed_Task", 256, NULL, 7, &GR_LED_FAST_TaskHandle); //Create Green and Red Led Fast Flash Task
        lRetVal = WiFi_Connect_Test();                                                                                           //wlan connect test
        vTaskDelete(GR_LED_FAST_TaskHandle);                                                                                     //delete Green and Red Led Fast Flash Task
        SET_GREEN_LED_OFF();                                                                                                     //Set Green Led Off
        SET_RED_LED_OFF();                                                                                                       //Set Red Led Off
        osi_bell_makeSound(200);

        if (lRetVal < 0)
        {
            Red_Led_Flashed(3, 3);
        }
        else
        {
            Green_Led_Flashed(3, 3);
        }
    }

    osi_at24c08_ReadData(DATAURI_FLAG_ADDR, (uint8_t *)Readflag, sizeof(Readflag), 1); //read datapost addr
    if (!strcmp((char const *)Readflag, DATA_URI))                                     //check datapost addr
    {
        my_xTaskCreate(UpdateTimeTask, "UpdateTimeTask", 1024, NULL, 7, NULL); //Create UpdataTime Task
    }
    else
    {
        my_xTaskCreate(ApikeyGetTask, "ApikeyGetTask", 1024, NULL, 7, NULL); //Create ApiKeyGetTask
    }

    xSemaphoreGive(xMutex2); //SimpleLink Semaphore Give
    ap_mode_status = 0;
    AP_MODE_END_FLAG = 0;
    // MAP_UtilsDelay(80000); //delay about 6ms
    xEventGroupSetBits(Task_Group, AP_MODE_END_BIT);
    vTaskDelete(NULL);
}
