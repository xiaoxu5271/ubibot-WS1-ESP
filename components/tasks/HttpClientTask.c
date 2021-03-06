/*******************************************************************************
  * @file       HTTP Client Application Task
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
#include "stdlib.h"
#include "stdbool.h"
#include "math.h"
// #include "osi.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
// #include "hw_types.h"
// #include "hw_memmap.h"
// #include "common.h"
// #include "utils.h"f
// #include "rom_map.h"
// #include "gpio.h"
// #include "uart_if.h"
// #include "simplelink.h"
// #include <http/client/httpcli.h>
// #include <http/client/common.h>

#include "MsgType.h"
#include "at24c08.h"
#include "w25q128.h"
#include "JsonParse.h"
#include "PeripheralDriver.h"
#include "iic.h"
#include "base64.h"
#include "app_config.h"
#include "esp_http_client.h"

#include "HttpClientTask.h"

#define TAG "HttpClient"

extern uint8_t Host_Flag;
extern uint8_t HOST_IP[4];
extern bool OTA_FLAG;

extern QueueHandle_t xQueue0; //Used for cjson and memory save

extern TaskHandle_t xBinary0;  //For DataPostTask interrupt
extern TaskHandle_t xBinary11; //For Memory Delete Task
extern TaskHandle_t xBinary13; //Task end check
extern TaskHandle_t xBinary16; //WiFi List Scan Task
extern TaskHandle_t xBinary17; //Internet Application

extern SemaphoreHandle_t xMutex2; //Used for SimpleLink Lock
extern SemaphoreHandle_t xMutex3; //Used for cJSON Lock
extern SemaphoreHandle_t xMutex4; //Used for UART Lock
extern SemaphoreHandle_t xMutex5; //Used for Post_Data_Buffer Lock
extern SemaphoreHandle_t xMutex7;

extern char SEC_TYPE[8];
extern char SSID_NAME[32];
extern char HOST_NAME[64];
extern char POST_REQUEST_URI[250];
extern char Post_Data_Buffer[4096];
extern char Read_Response_Buffer[RESP_BUF_LEN];

extern char wifi_buf[180];
char calidata_buf[512] = {0};

extern volatile uint16_t sys_run_time;
extern volatile bool PostSetData; //post url data
extern volatile bool update_time; //update time
extern volatile bool POST_TASK_END_FLAG;
extern volatile uint8_t cg_data_led; //Led ON or OFF when data post
extern volatile uint8_t no_net_fn;   //change fn_dp when no net
extern volatile uint8_t wifi_mode;   //1:connect wifi immediately,0:connect wifi when scaned
extern volatile uint8_t save_addr_flag;
extern volatile unsigned long fn_dp; //data post frequence
extern volatile bool usb_status_val; //usb status

//extern unsigned char            g_buff[MAX_BUFF_SIZE+1];
extern float f5_a, f5_b;
extern volatile unsigned long POST_NUM;
extern volatile unsigned long DELETE_ADDR, POST_ADDR, WRITE_ADDR;

#define ERROR_CODE 0xffff

bool dns_fault = 0;

esp_http_client_handle_t Http_Init_Fun(void);

/*******************************************************************************
//This function read respose from server and dump on console
*******************************************************************************/
static int readResponse(esp_http_client_handle_t httpClient)
{
  long lRetVal = 0;
  // int bytesRead = 0;

  int content_length = esp_http_client_fetch_headers(httpClient);
  if (content_length < 0)
  {
    ESP_LOGE(TAG, "HTTP client fetch headers failed");
  }
  else
  {
    int data_read = esp_http_client_read_response(httpClient, Read_Response_Buffer, RESP_BUF_LEN);
    if (data_read >= 0)
    {
      lRetVal = esp_http_client_get_status_code(httpClient);
      ESP_LOGI(TAG, "HTTP GET Status = %ld, data_read = %d",
               lRetVal,
               data_read);
      ESP_LOGI(TAG, "GET Request READ:\n%s", Read_Response_Buffer);
    }
    else
    {
      ESP_LOGE(TAG, "Failed to read response");
    }
  }

  // Read_Response_Buffer[bytesRead] = '\0';

  //  xSemaphoreTake(xMutex3, -1); //cJSON Semaphore Take
  xSemaphoreTake(xMutex3, -1);

  switch (lRetVal)
  {
  case 200:
  {
    if (PostSetData)
    {
      lRetVal = ParseSetJSONData(Read_Response_Buffer); //Api Key
    }
    else if (update_time)
    {
      lRetVal = ParseTimeData(Read_Response_Buffer); //Update Time
    }
    else
    {
      lRetVal = ParseJSONData(Read_Response_Buffer); //Sensor Data
    }
  }
  break;
    /*
        case 400:
        {
          ParseTimeData(Read_Response_Buffer);  //Update Time
        }
        break;
        case 401:
        {
          ParseTimeData(Read_Response_Buffer);  //Update Time
        }
        break;
        case 402:
        {
          ParseTimeData(Read_Response_Buffer);  //Update Time
        }
        break;
        case 410:
        {
          ParseTimeData(Read_Response_Buffer);  //Update Time
        }
        break;    
  */
  default:
  {
    ParseTimeData(Read_Response_Buffer); //Update Time
  }
  break;
  }

  //  xSemaphoreGive(xMutex3); //cJSON Semaphore Give
  xSemaphoreGive(xMutex3);

  return lRetVal;
}

/*******************************************************************************
  HTTP GET METHOD
*******************************************************************************/
int HTTPGetMethod(esp_http_client_handle_t httpClient)
{
  int lRetVal = 0;

  //osi_at24c08_ReadData(HOST_ADDR,(uint8_t*)HOST_NAME,sizeof(HOST_NAME),1);  //read host

  memset(POST_REQUEST_URI, 0, sizeof(POST_REQUEST_URI));

  if (PostSetData)
  {
    read_product_url(POST_REQUEST_URI);
  }
  else if (update_time)
  {
    memcpy(POST_REQUEST_URI, "/utilities/time", strlen("/utilities/time"));
  }

  esp_http_client_set_url(httpClient, POST_REQUEST_URI);
  esp_http_client_set_method(httpClient, HTTP_METHOD_GET);

  lRetVal = esp_http_client_open(httpClient, 0);
  if (lRetVal != ESP_OK)
  {
    ESP_LOGE(TAG, "esp_http_client_open FAIL:%s", esp_err_to_name(lRetVal));
    if (Host_Flag == 0)
    {
      Host_Flag = 0XFF;
      if (no_net_fn == 1)
      {
        if (fn_dp < NO_NET_FN_DP)
        {
          unsigned long fn_dp_x = fn_dp;

          fn_dp = 2 * fn_dp_x;

          if (fn_dp > NO_NET_FN_DP)
          {
            fn_dp = NO_NET_FN_DP;
          }

          osi_at24c08_write(FN_DP_ADDR, fn_dp); //fn_dp
        }
      }
    }
    else
    {
      Host_Flag = 0;
    }
    osi_at24c08_write_byte(HOST_NAME_IP_ADDR, Host_Flag);
    return FAILURE;
  }
  else
  {
    ESP_LOGI(TAG, "Connection to server successfully\r\n");

    //????????????IP??????????????????????????????IP??????
    if (dns_fault == 1)
    {
      Host_Flag = 0XFF;
      osi_at24c08_write_byte(HOST_NAME_IP_ADDR, Host_Flag);
    }
  }

  lRetVal = readResponse(httpClient);

  return lRetVal;
}

/*******************************************************************************
  HTTP POST Demonstration
*******************************************************************************/
int HTTPPostMethod(esp_http_client_handle_t httpClient, unsigned long DataLen, uint16_t Post_Amount)
{
  uint8_t url_len = 0;
  // char tmpBuf[8] = {0};
  long lRetVal = 0;
  bool EndFlag = 0;
  uint16_t post_data_sum;
  unsigned long MemoryAddr;
  char status_buf[128];
  char mac_buf[18] = {0};
  uint8_t mac_addr[8] = {0};
  char base64_ssid[48] = {0};

  memset(status_buf, 0, sizeof(status_buf));

  // osi_at24c08_ReadData(MAC_ADDR, mac_addr, SL_MAC_ADDR_LEN, 0);
  esp_read_mac(mac_addr, 0); //??????????????????????????????MAC

  snprintf(mac_buf, sizeof(mac_buf), "%02x:%02x:%02x:%02x:%02x:%02x", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

  memset(POST_REQUEST_URI, 0, sizeof(POST_REQUEST_URI));
  //osi_at24c08_ReadData(HOST_ADDR,(uint8_t*)HOST_NAME,sizeof(HOST_NAME),1);  //read host name

  osi_at24c08_ReadData(DATAURI_ADDR, (uint8_t *)POST_REQUEST_URI, sizeof(POST_REQUEST_URI), 1); //read DATA-URI
  // ESP_LOGI(TAG, "%d,%s", __LINE__, POST_REQUEST_URI);

  snprintf(POST_REQUEST_URI + strlen(POST_REQUEST_URI), sizeof(POST_REQUEST_URI) - strlen(POST_REQUEST_URI), "&firmware=%s", FIRMWARENUM);
  base64_encode(SSID_NAME, strlen(SSID_NAME), base64_ssid, sizeof(base64_ssid));
  snprintf(POST_REQUEST_URI + strlen(POST_REQUEST_URI), sizeof(POST_REQUEST_URI) - strlen(POST_REQUEST_URI), "&ssid_base64=%s", base64_ssid);

  POST_REQUEST_URI[strlen(POST_REQUEST_URI)] = '\0';

  snprintf(status_buf, sizeof(status_buf), ",\"status\":\"mac=%s,usb=%d\",\"ssid_base64\":\"%s\"}",
           mac_buf,
           usb_status_val,
           base64_ssid); //MAC SSID USB

  ESP_LOGI(TAG, "%d,%s", __LINE__, POST_REQUEST_URI);

  esp_http_client_set_url(httpClient, POST_REQUEST_URI);
  esp_http_client_set_method(httpClient, HTTP_METHOD_POST);
  esp_http_client_set_header(httpClient, "Content-Type", "application/json");

  lRetVal = esp_http_client_open(httpClient, (DataLen + strlen(status_buf) + strlen(wifi_buf) - 1 + strlen(calidata_buf) + 8));
  if (lRetVal != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(lRetVal));
  }

  MemoryAddr = POST_ADDR;

  // osi_UartPrint("{\"feeds\":[");

  lRetVal = esp_http_client_write(httpClient, "{\"feeds\":[", strlen("{\"feeds\":["));
  if (lRetVal < 0)
  {
    ESP_LOGE(TAG, "Write failed");
  }

  while (Post_Amount)
  {
    post_data_sum = Post_Amount > SEND_DATA_NUMBER ? SEND_DATA_NUMBER : Post_Amount; //post data max once

    Post_Amount -= post_data_sum;
    if (Post_Amount == 0)
    {
      EndFlag = 1;
    }

    //  xSemaphoreTake(xMutex5, -1); //Post_Data_Buffer Semaphore Take
    xSemaphoreTake(xMutex5, -1);

    MemoryAddr = Read_PostDataBuffer(MemoryAddr, Post_Data_Buffer, post_data_sum, EndFlag); //read post data

    // ESP_LOGI(TAG, "%s", Post_Data_Buffer);

    lRetVal = esp_http_client_write(httpClient, (const char *)Post_Data_Buffer, (strlen(Post_Data_Buffer))); //Send POST data/body
    //  xSemaphoreGive(xMutex5); //Post_Data_Buffer Semaphore Give
    xSemaphoreGive(xMutex5);
    if (lRetVal < 0)
    {
      ESP_LOGE(TAG, "Write failed %d", __LINE__);
      return lRetVal;
    }
  }

  ESP_LOGI(TAG, "%s", wifi_buf);

  lRetVal = esp_http_client_write(httpClient, wifi_buf, strlen(wifi_buf)); //wifi_buf
  if (lRetVal < 0)
  {
    ESP_LOGE(TAG, "Write failed %d", __LINE__);
    return lRetVal;
  }

  lRetVal = esp_http_client_write(httpClient, "],\"cali\":", strlen("],\"cali\":")); //"],\"cali\":"
  if (lRetVal < 0)
  {
    ESP_LOGE(TAG, "Write failed %d", __LINE__);
    return lRetVal;
  }

  ESP_LOGI(TAG, "%s", calidata_buf);
  // osi_UartPrint(calidata_buf);

  lRetVal = esp_http_client_write(httpClient, calidata_buf, strlen(calidata_buf)); //Send calidata_buf
  if (lRetVal < 0)
  {
    ESP_LOGE(TAG, "Write failed %d", __LINE__);
    return lRetVal;
  }

  // osi_UartPrint(status_buf);
  ESP_LOGI(TAG, "%s", status_buf);

  lRetVal = esp_http_client_write(httpClient, status_buf, strlen(status_buf)); //Send status data
  if (lRetVal < 0)
  {
    ESP_LOGE(TAG, "Write failed %d", __LINE__);
    return lRetVal;
  }

  ESP_LOGI(TAG, "Successed to send HTTP POST request body.\r\n");

  lRetVal = readResponse(httpClient); //read respose from server

  return lRetVal;
}
/******************************************************************************
  HTTP INIT
******************************************************************************/
esp_http_client_handle_t Http_Init_Fun(void)
{
  esp_http_client_config_t config = {
      // .url = "http://httpbin.org/post",
      // .host = "api.ubibot.cn",
      .path = "/",
  };

  Host_Flag = osi_at24c08_read_byte(HOST_NAME_IP_ADDR);
  ESP_LOGI(TAG, "%d,Host_Flag=%d", __LINE__, Host_Flag);
  if (Host_Flag == 0)
  {
    osi_at24c08_ReadData(HOST_ADDR, (uint8_t *)HOST_NAME, sizeof(HOST_NAME), 1); //read the host name
    ESP_LOGI(TAG, "%d,HOST_NAME:%s", __LINE__, HOST_NAME);
    dns_fault = 0;
    config.host = (char *)HOST_NAME;
  }
  else
  {
    dns_fault = 1;
    osi_at24c08_ReadData(HOST_IP_ADDR, HOST_IP, sizeof(HOST_IP), 1);
    snprintf(HOST_NAME, sizeof(HOST_NAME), "%d.%d.%d.%d", HOST_IP[0], HOST_IP[1], HOST_IP[2], HOST_IP[3]);
    ESP_LOGI(TAG, "%d,HOST_NAME:%s", __LINE__, HOST_NAME);
    config.host = (char *)HOST_NAME;
  }

  config.port = osi_at24c08_read(HOST_PORT_ADDR);

  esp_http_client_handle_t client = esp_http_client_init(&config);
  return client;
}

/******************************************************************************
  HTTP Get Form Server
******************************************************************************/
short Http_Get_Function(void)
{
  long lRetVal = -1;

  esp_http_client_handle_t client = Http_Init_Fun();
  if (client == NULL)
  {
    return lRetVal;
  }

  lRetVal = HTTPGetMethod(client);
  esp_http_client_cleanup(client);

  return lRetVal;
}

/*******************************************************************************
  deleted the post data
*******************************************************************************/
void PostAddrChage(unsigned long data_num, unsigned long end_addr)
{
  unsigned long d_addr, p_addr, w_addr;

  d_addr = DELETE_ADDR; //delete pointer variable value

  p_addr = POST_ADDR; //post pointer variable value

  w_addr = WRITE_ADDR; //write pointer variable value

  if (((w_addr >= p_addr) && (w_addr >= d_addr) && (p_addr >= d_addr)) || ((d_addr >= w_addr) && (d_addr >= p_addr) && (w_addr >= p_addr))) //w_addr > p_addr
  {
    POST_ADDR = end_addr;

    if (w_addr <= end_addr)
    {
      // osi_SyncObjSignalFromISR(&xBinary11); //start delete task
      if (xBinary11 != NULL)
        vTaskNotifyGiveFromISR(xBinary11, NULL);
    }
  }
  else if ((p_addr >= d_addr) && (p_addr >= w_addr) && (d_addr >= w_addr)) //p_addr>w_addr
  {
    if ((end_addr >= p_addr) && (end_addr < Memory_Max_Addr))
    {
      POST_ADDR = end_addr;
    }
    else if (end_addr <= w_addr)
    {
      POST_ADDR = end_addr;
    }
    else
    {
      POST_ADDR = end_addr;

      // osi_SyncObjSignalFromISR(&xBinary11); //start delete task
      if (xBinary11 != NULL)
        vTaskNotifyGiveFromISR(xBinary11, NULL);
    }
  }
  else
  {
    // osi_SyncObjSignalFromISR(&xBinary11); //start delete task
    if (xBinary11 != NULL)
      vTaskNotifyGiveFromISR(xBinary11, NULL);
  }

  if (POST_NUM >= data_num)
  {
    // portENTER_CRITICAL(0); //enter critical
    xSemaphoreTake(xMutex7, -1);
    POST_NUM -= data_num;
    xSemaphoreGive(xMutex7);
  }
  else
  {
    POST_NUM = 0;
  }

  if (POST_NUM == 0)
  {
    POST_ADDR = WRITE_ADDR;
  }

  if (save_addr_flag == 0)
  {
    osi_at24c08_write(DATA_AMOUNT_ADDR1, POST_NUM);

    osi_at24c08_write(DATA_POST_ADDR1, POST_ADDR);
  }
  else
  {
    osi_at24c08_write(DATA_AMOUNT_ADDR2, POST_NUM);

    osi_at24c08_write(DATA_POST_ADDR2, POST_ADDR);
  }

#ifdef DEBUG

  osi_UartPrint_Val("POST_NUM:", POST_NUM);

  osi_UartPrint_Val("POST_ADDR:", POST_ADDR);

#endif
}

/*******************************************************************************
// data post task
*******************************************************************************/
void DataPostTask(void *pvParameters)
{
  uint8_t err_code_num;
  int Rssi_val;
  bool Err_Status;
  short lRetVal = -1;
  uint16_t read_data_num;
  uint16_t post_data_num;
  unsigned long post_data_len;
  unsigned long read_data_end_addr;
  SensorMessage Rssi_Msg;
  wifi_ap_record_t wifidata_t;

  for (;;)
  {
    // osi_SyncObjWait(&xBinary0,-1);  //Wait For The Operation Message
    xEventGroupSetBits(Task_Group, POST_TASK_BIT);
    ulTaskNotifyTake(pdTRUE, -1);
    xEventGroupClearBits(Task_Group, POST_TASK_BIT);
    ESP_LOGI(TAG, "%d", __LINE__);

    Err_Status = 1;

    //  xSemaphoreTake(xMutex2, -1); //SimpleLink Semaphore Take
    xSemaphoreTake(xMutex2, -1);

    sys_run_time = 0; //clear wifi post time

    POST_TASK_END_FLAG = 1;

    // osi_SyncObjSignalFromISR(&xBinary13); //Start Tasks End Check

    // osi_SyncObjSignalFromISR(&xBinary17);

    //Start LED Blink
    if (xBinary17 != NULL)
      vTaskNotifyGiveFromISR(xBinary17, NULL);

    if (wifi_mode != 1)
    {
      Rssi_val = Scan_Wifi_List(NULL, NULL, 0);
    }
    else
    {
      Rssi_val = 1;
    }

    if ((Rssi_val != ERROR_CODE) || (wifi_mode == 1))
    {
      // if (Rssi_val == ERROR_CODE)
      // {
      //   osi_at24c08_ReadData(SECTYPE_ADDR, (uint8_t *)SEC_TYPE, sizeof(SEC_TYPE), 1); //Read Wifi Sectype
      // }
      // else
      // {
      //   Rssi_Msg.sensornum = RSSI_NUM;               //Message Number
      //   Rssi_Msg.sensorval = f5_a * Rssi_val + f5_b; //Message Value
      //                                                // osi_MsgQWrite(&xQueue0, &Rssi_Msg, OSI_NO_WAIT); //Rssi Value Data Message
      //   xQueueSend(xQueue0, &Rssi_Msg, 0);
      // }

      for (err_code_num = 0; err_code_num < RETRY_TIME_OUT; err_code_num++)
      {
        lRetVal = WlanConnect(); //Wlan Connect To The Accesspoint 10s timeout
        if (lRetVal >= 0)
        {
          esp_wifi_sta_get_ap_info(&wifidata_t);
          Rssi_Msg.sensornum = RSSI_NUM;                      //Message Number
          Rssi_Msg.sensorval = f5_a * wifidata_t.rssi + f5_b; //Message Value
                                                              // osi_MsgQWrite(&xQueue0, &Rssi_Msg, OSI_NO_WAIT); //Rssi Value Data Message
          xQueueSend(xQueue0, &Rssi_Msg, 0);
          // vTaskDelay(10 / portTICK_RATE_MS);
          ESP_LOGI(TAG, "%d", __LINE__);
          break;
        }
        else
        {
          ESP_LOGI(TAG, "%d", __LINE__);
          Wlan_Disconnect_AP(); //wlan disconnect form the ap
          vTaskDelay(500 / portTICK_RATE_MS);
        }
      }

      err_code_num = 0;

      if (lRetVal >= 0)
      {
        memset(calidata_buf, 0, sizeof(calidata_buf));
        Read_cali(calidata_buf, sizeof(calidata_buf)); // cali data

        ESP_LOGI(TAG, "%d", __LINE__);
        //?????????????????????????????????????????????????????????
        xEventGroupWaitBits(Task_Group, SAVE_TASK_BIT, false, false, -1);
        ESP_LOGI(TAG, "%d", __LINE__);
        while (POST_NUM)
        {
          sys_run_time = 0; //clear wifi post time
          xSemaphoreTake(xMutex7, -1);
          read_data_num = POST_NUM > POST_DATA_NUMBER ? POST_DATA_NUMBER : POST_NUM; //read post data len
          lRetVal = Read_PostDataLen(POST_ADDR, &read_data_end_addr, read_data_num, &post_data_num, &post_data_len);
          xSemaphoreGive(xMutex7);
          if (lRetVal < 0)
          {
            ESP_LOGE(TAG, "%d", __LINE__);
            osi_System_ERROR_Save(MEMORY_SAVE_DATA_ERR); //Save ERROR Data

            PostAddrChage(post_data_num, read_data_end_addr); //change the point
          }
          else
          {
            esp_http_client_handle_t client = Http_Init_Fun(); //HTTP INIT
            if (client == NULL)
            {
              ESP_LOGE(TAG, "%d", __LINE__);
              osi_System_ERROR_Save(CNT_SERVER_FLR_ERR); //save ERROR data
              break;
              // return lRetVal;
            }

            ESP_LOGI(TAG, "%d", __LINE__);
            lRetVal = HTTPPostMethod(client, post_data_len, post_data_num);
            esp_http_client_cleanup(client);

            // HTTPCli_disconnect(&client); //disconnect to http server

            if (lRetVal == SUCCESS)
            {
              ESP_LOGI(TAG, "%d", __LINE__);
              Err_Status = 0;

              PostAddrChage(post_data_num, read_data_end_addr); //change the point
            }
            else if (lRetVal == 400)
            {

              ESP_LOGE(TAG, "%d", __LINE__);
              osi_System_ERROR_Save(POST_DATA_JSON_FAULT); //save ERROR data

              PostAddrChage(post_data_num, read_data_end_addr); //change the point

              err_code_num += 1;
            }
            else if (lRetVal == 410)
            {
              ESP_LOGE(TAG, "%d", __LINE__);
              PostSetData = 1;

              lRetVal = Http_Get_Function(); //HTTP Get Form Server

              PostSetData = 0;

              if (lRetVal != SUCCESS)
              {
                break;
              }
            }
            else
            {

              ESP_LOGE(TAG, "%d", __LINE__);
              ESP_LOGI(TAG, "HTTP Post failed.\r\n");

              osi_System_ERROR_Save(POST_DATA_FLR_ERR); //save ERROR data

              err_code_num += 1;
              // MAP_UtilsDelay(40000000); //delay about 3s
            }

            if (err_code_num >= RETRY_TIME_OUT)
            {
              break;
            }
          }
        }
        if (OTA_FLAG == false)
        {
          Wlan_Disconnect_AP(); //wlan disconnect form the ap
        }

        // sl_Stop(SL_STOP_TIMEOUT); //stop the simple link
        // MAP_UtilsDelay(80000); //Delay About 6ms
      }
    }
    else
    {
      ESP_LOGE(TAG, "%d,Rssi_val=%d,wifi_mode=%d", __LINE__, Rssi_val, wifi_mode);
    }

    POST_TASK_END_FLAG = 0;

    //  xSemaphoreGive(xMutex2); //SimpleLink Semaphore Give
    xSemaphoreGive(xMutex2);

    // osi_SyncObjSignalFromISR(&xBinary11); //start delete task
    if (xBinary11 != NULL)
      vTaskNotifyGiveFromISR(xBinary11, NULL);

    if (Err_Status)
    {
      Red_Led_On_time(10); //RED LED ON 1.5s
    }
  }
}

/*******************************************************************************
                                      END         
*******************************************************************************/
