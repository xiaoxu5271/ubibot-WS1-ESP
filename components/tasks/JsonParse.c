/*******************************************************************************
  * @file       cJson Application Task    
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
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "PCF8563.h"
#include "at24c08.h"
#include "MsgType.h"
#include "AcceSensorTask.h"
#include "PeripheralDriver.h"
#include "PowerMeasureTask.h"
#include "HttpClientTask.h"
#include "MagTask.h"
#include "w25q128.h"
#include "app_config.h"
#include "ota.h"

#define TAG "JsonParse"

extern uint8_t HOST_IP[4];
extern char SEC_TYPE[8];
extern char SSID_NAME[32];
extern char PASS_WORD[64];
extern char HOST_NAME[64];
extern char mqtt_ota_url[128];
char ProductURI[128];

extern QueueHandle_t xQueue1;     //Used for LED control task
extern QueueHandle_t xQueue2;     //Used for bell conctrol task
extern SemaphoreHandle_t xMutex1; //Used for SPI Lock
extern SemaphoreHandle_t xMutex2; //Used for SimpleLink Lock
extern SemaphoreHandle_t xMutex3; //Used for cJSON Lock
extern TaskHandle_t xBinary9;

#ifdef MAG_SENSOR
extern volatile uint8_t fn_mag_int;             //0:no data save,1:save data,2:save data and post
extern volatile unsigned long fn_mag, fn_mag_t; //Magnetic sensor frequence
#endif
extern volatile uint8_t fn_acc_tap1;   //0:closed single tap,1:single tap interrupt,2:single tap interrupt and post
extern volatile uint8_t fn_acc_tap2;   //0:closed double tap,1:double tap interrupt,2:double tap interrupt and post
extern volatile uint8_t fn_acc_act;    //0:cloused act interrupt,1:act interrupt
extern volatile uint8_t thres_acc_min; //min value for acceleration sensor act interrupt
extern volatile uint8_t cg_data_led;   //Led ON or OFF when data post
extern volatile uint8_t no_net_fn;     //change fn_dp when no net
extern volatile uint8_t wifi_mode;     //1:connect wifi immediately,0:connect wifi when scaned
extern short flash_set_val;
extern unsigned long now_unix_t;                        //now unix time
extern volatile unsigned long fn_th, fn_th_t;           //Temp&Humi sensor frequence
extern volatile unsigned long fn_light, fn_light_t;     //Light sensor frequence
extern volatile unsigned long fn_bt, fn_bt_t;           //Body Temperature Measure frequence
extern volatile unsigned long fn_ext, fn_ext_t;         //Noise Measure frequence
extern volatile unsigned long fn_battery, fn_battery_t; //PowerMeasure frequence
extern volatile unsigned long fn_dp, fn_dp_t;           //data post frequence

extern float f1_a, f1_b, f2_a, f2_b, f3_a, f3_b, f4_a, f4_b, f5_a, f5_b, f6_a, f6_b, f7_a, f7_b, f8_a, f8_b, f9_a, f9_b, f10_a, f10_b;

extern volatile unsigned long POST_NUM;

void Parse_IP_MASK_GW_DNS(char *parse_buf, uint8_t *res_buf);

/*******************************************************************************
  update task frequence
*******************************************************************************/
static unsigned long fn_x_t_update(unsigned long new_fn)
{
  osi_Read_UnixTime(); //update system unix time

  return now_unix_t + new_fn;
}

/*******************************************************************************
  parse metedata
*******************************************************************************/
static short Parse_metadata(char *ptrptr)
{
  bool fn_flag = 0;
  bool acc_flag = 0;

  if (NULL == ptrptr)
  {
    return FAILURE;
  }

  cJSON *pJsonJson = cJSON_Parse(ptrptr);
  if (NULL == pJsonJson)
  {
    cJSON_Delete(pJsonJson); //delete pJson

    return FAILURE;
  }

  cJSON *pSubSubSub = cJSON_GetObjectItem(pJsonJson, "fn_th"); //"fn_th"
  if (NULL != pSubSubSub)
  {
#ifdef DEBUG_RESPONSE

    osi_UartPrint_Val("\"fn_th\":", pSubSubSub->valueint);

#endif

    if ((unsigned long)pSubSubSub->valueint != fn_th)
    {
      fn_flag = 1;

      fn_th = (unsigned long)pSubSubSub->valueint;

      fn_th_t = fn_x_t_update((unsigned long)pSubSubSub->valueint);

      osi_at24c08_write(FN_TH_ADDR, fn_th); //fn_th

      osi_at24c08_write(FN_TH_T_ADDR, fn_th_t); //fn_th_t
    }
  }

  pSubSubSub = cJSON_GetObjectItem(pJsonJson, "fn_light"); //"fn_light"
  if (NULL != pSubSubSub)
  {
#ifdef DEBUG_RESPONSE

    osi_UartPrint_Val("\"fn_light\":", pSubSubSub->valueint);

#endif

    if ((unsigned long)pSubSubSub->valueint != fn_light)
    {
      fn_flag = 1;

      fn_light = (unsigned long)pSubSubSub->valueint;

      fn_light_t = fn_x_t_update((unsigned long)pSubSubSub->valueint);

      osi_at24c08_write(FN_LIGHT_ADDR, fn_light); //fn_light

      osi_at24c08_write(FN_LIGHT_T_ADDR, fn_light_t); //fn_light_t
    }
  }

#ifdef MAG_SENSOR
  pSubSubSub = cJSON_GetObjectItem(pJsonJson, "fn_mag"); //"fn_mag"
  if (NULL != pSubSubSub)
  {
#ifdef DEBUG_RESPONSE

    osi_UartPrint_Val("\"fn_mag\":", pSubSubSub->valueint);

#endif

    if ((unsigned long)pSubSubSub->valueint != fn_mag)
    {
      fn_flag = 1;

      fn_mag = (unsigned long)pSubSubSub->valueint;

      fn_mag_t = fn_x_t_update((unsigned long)pSubSubSub->valueint);

      osi_at24c08_write(FN_MAG_ADDR, fn_mag); //fn_mag

      osi_at24c08_write(FN_MAG_T_ADDR, fn_mag_t); //fn_mag_t
    }
  }

  pSubSubSub = cJSON_GetObjectItem(pJsonJson, "fn_mag_int"); //"fn_mag_int"
  if (NULL != pSubSubSub)
  {
#ifdef DEBUG_RESPONSE

    osi_UartPrint_Val("\"fn_mag_int\":", pSubSubSub->valueint);

#endif

    if ((uint8_t)pSubSubSub->valueint != fn_mag_int)
    {
      fn_mag_int = (uint8_t)pSubSubSub->valueint;

      osi_at24c08_write_byte(FN_MAG_INT_ADDR, fn_mag_int);
    }
  }
#endif

  pSubSubSub = cJSON_GetObjectItem(pJsonJson, "fn_acc_tap1"); //"fn_acc_tap1"
  if (NULL != pSubSubSub)
  {
#ifdef DEBUG_RESPONSE

    osi_UartPrint_Val("\"fn_acc_tap1\":", pSubSubSub->valueint);

#endif

    if ((uint8_t)pSubSubSub->valueint != fn_acc_tap1)
    {
      acc_flag = 1;

      fn_acc_tap1 = (uint8_t)pSubSubSub->valueint;

      osi_at24c08_write_byte(FN_ACC_TAP1_ADDR, fn_acc_tap1);
    }
  }

  pSubSubSub = cJSON_GetObjectItem(pJsonJson, "fn_acc_tap2"); //"fn_acc_tap2"
  if (NULL != pSubSubSub)
  {
#ifdef DEBUG_RESPONSE

    osi_UartPrint_Val("\"fn_acc_tap2\":", pSubSubSub->valueint);

#endif

    if ((uint8_t)pSubSubSub->valueint != fn_acc_tap2)
    {
      acc_flag = 1;

      fn_acc_tap2 = (uint8_t)pSubSubSub->valueint;

      osi_at24c08_write_byte(FN_ACC_TAP2_ADDR, fn_acc_tap2);
    }
  }

  pSubSubSub = cJSON_GetObjectItem(pJsonJson, "fn_acc_act"); //"fn_acc_act"
  if (NULL != pSubSubSub)
  {
#ifdef DEBUG_RESPONSE

    osi_UartPrint_Val("\"fn_acc_act\":", pSubSubSub->valueint);

#endif

    if ((uint8_t)pSubSubSub->valueint != fn_acc_act)
    {
      acc_flag = 1;

      fn_acc_act = (uint8_t)pSubSubSub->valueint;

      osi_at24c08_write_byte(FN_ACC_ACT_ADDR, fn_acc_act);
    }
  }

  pSubSubSub = cJSON_GetObjectItem(pJsonJson, "thres_acc_min"); //"thres_acc_min"
  if (NULL != pSubSubSub)
  {
#ifdef DEBUG_RESPONSE

    osi_UartPrint_Val("\"thres_acc_min\":", pSubSubSub->valueint);

#endif

    if ((uint8_t)pSubSubSub->valueint != thres_acc_min)
    {
      acc_flag = 1;

      thres_acc_min = (uint8_t)pSubSubSub->valueint > DEFAULT_THRES_ACC_MIN ? (uint8_t)pSubSubSub->valueint : DEFAULT_THRES_ACC_MIN;

      osi_at24c08_write_byte(THRES_ACC_MIN_ADDR, thres_acc_min);
    }
  }

  pSubSubSub = cJSON_GetObjectItem(pJsonJson, "fn_bt"); //"fn_bt"
  if (NULL != pSubSubSub)
  {
#ifdef DEBUG_RESPONSE

    osi_UartPrint_Val("\"fn_bt\":", pSubSubSub->valueint);

#endif

    if ((unsigned long)pSubSubSub->valueint != fn_bt)
    {
      fn_flag = 1;

      fn_bt = (unsigned long)pSubSubSub->valueint;

      fn_bt_t = fn_x_t_update((unsigned long)pSubSubSub->valueint);

      osi_at24c08_write(FN_BT_ADDR, fn_bt); //fn_bt

      osi_at24c08_write(FN_BT_T_ADDR, fn_bt_t); //fn_bt_t
    }
  }

  pSubSubSub = cJSON_GetObjectItem(pJsonJson, "fn_ext_t"); //"fn_ext_t"
  if (NULL != pSubSubSub)
  {
#ifdef DEBUG_RESPONSE

    osi_UartPrint_Val("\"fn_ext_t\":", pSubSubSub->valueint);

#endif

    if ((unsigned long)pSubSubSub->valueint != fn_ext)
    {
      fn_flag = 1;

      fn_ext = (unsigned long)pSubSubSub->valueint;

      fn_ext_t = fn_x_t_update((unsigned long)pSubSubSub->valueint);

      osi_at24c08_write(FN_EXT_ADDR, fn_ext); //fn_ext

      osi_at24c08_write(FN_EXT_T_ADDR, fn_ext_t); //fn_ext_t
    }
  }

  pSubSubSub = cJSON_GetObjectItem(pJsonJson, "fn_battery"); //"fn_battery"
  if (NULL != pSubSubSub)
  {
#ifdef DEBUG_RESPONSE

    osi_UartPrint_Val("\"fn_battery\":", pSubSubSub->valueint);

#endif

    if ((unsigned long)pSubSubSub->valueint != fn_battery)
    {
      fn_flag = 1;

      fn_battery = (unsigned long)pSubSubSub->valueint;

      fn_battery_t = fn_x_t_update((unsigned long)pSubSubSub->valueint);

      osi_at24c08_write(FN_BATTERY_ADDR, fn_battery);

      osi_at24c08_write(FN_BATTERY_T_ADDR, fn_battery_t); //fn_battery_t
    }
  }

  pSubSubSub = cJSON_GetObjectItem(pJsonJson, "fn_dp"); //"fn_dp"
  if (NULL != pSubSubSub)
  {
#ifdef DEBUG_RESPONSE

    osi_UartPrint_Val("\"fn_dp\":", pSubSubSub->valueint);

#endif

    if ((unsigned long)pSubSubSub->valueint != fn_dp)
    {
      fn_flag = 1;

      fn_dp = (unsigned long)pSubSubSub->valueint;

      fn_dp_t = fn_x_t_update((unsigned long)pSubSubSub->valueint);

      osi_at24c08_write(FN_DP_ADDR, fn_dp); //fn_dp

      osi_at24c08_write(FN_DP_T_ADDR, fn_dp_t); //fn_dp_t
    }
  }

  pSubSubSub = cJSON_GetObjectItem(pJsonJson, "cg_data_led"); //"cg_data_led"
  if (NULL != pSubSubSub)
  {
#ifdef DEBUG_RESPONSE

    osi_UartPrint_Val("\"cg_data_led\":", pSubSubSub->valueint);

#endif

    if ((uint8_t)pSubSubSub->valueint != cg_data_led)
    {
      cg_data_led = (uint8_t)pSubSubSub->valueint;

      osi_at24c08_write_byte(CG_LED_DATA_ADDR, cg_data_led);
    }
  }

  pSubSubSub = cJSON_GetObjectItem(pJsonJson, "no_net_fn"); //"no_net_fn"
  if (NULL != pSubSubSub)
  {
#ifdef DEBUG_RESPONSE

    osi_UartPrint_Val("\"no_net_fn\":", pSubSubSub->valueint);

#endif

    if ((uint8_t)pSubSubSub->valueint != no_net_fn)
    {
      no_net_fn = (uint8_t)pSubSubSub->valueint;

      osi_at24c08_write_byte(NO_NET_FN_ADDR, no_net_fn);
    }
  }

  pSubSubSub = cJSON_GetObjectItem(pJsonJson, "wifi_mode"); //"wifi_mode"
  if (NULL != pSubSubSub)
  {
#ifdef DEBUG_RESPONSE

    osi_UartPrint_Val("\"wifi_mode\":", pSubSubSub->valueint);

#endif

    if ((uint8_t)pSubSubSub->valueint != wifi_mode)
    {
      wifi_mode = (uint8_t)pSubSubSub->valueint;

      osi_at24c08_write_byte(WIFI_MODE_ADDR, wifi_mode);
    }
  }

  if (acc_flag > 0)
  {
    acce_sensor_reset(); //reset the acce sensor
  }
  if (fn_flag > 0)
  {
    // osi_SyncObjSignalFromISR(&xBinary9);  //check Task start time
    if (xBinary9 != NULL)
      vTaskNotifyGiveFromISR(xBinary9, NULL);
  }

  cJSON_Delete(pJsonJson);

  return SUCCESS;
}

/*******************************************************************************
// parse sensors fields num
*******************************************************************************/
static short Parse_cali(char *ptr)
{
  f_cali resp_cali;

  if (NULL == ptr)
  {
    return FAILURE;
  }
  cJSON *pJson = cJSON_Parse(ptr);
  if (NULL == pJson)
  {
    cJSON_Delete(pJson); //delete pJson
    return FAILURE;
  }

  cJSON *pSub = cJSON_GetObjectItem(pJson, "f1_a"); //"f1_a"
  if (NULL != pSub)
  {
#ifdef DEBUG_RESPONSE
    printf("f1_a:%f\r\n", pSub->valuedouble);
#endif
    if (f1_a != (float)pSub->valuedouble)
    {
      f1_a = (float)pSub->valuedouble;
      resp_cali.cali_val = (float)pSub->valuedouble;
      osi_at24c08_write(F1_A_ADDR, resp_cali.cali_buf); //f1_a
    }
  }
  pSub = cJSON_GetObjectItem(pJson, "f1_b"); //"f1_b"
  if (NULL != pSub)
  {
#ifdef DEBUG_RESPONSE
    printf("f1_b:%f\r\n", pSub->valuedouble);
#endif
    if (f1_b != (float)pSub->valuedouble)
    {
      f1_b = (float)pSub->valuedouble;
      resp_cali.cali_val = (float)pSub->valuedouble;
      osi_at24c08_write(F1_B_ADDR, resp_cali.cali_buf); //f1_b
    }
  }

  pSub = cJSON_GetObjectItem(pJson, "f2_a"); //"f2_a"
  if (NULL != pSub)
  {
#ifdef DEBUG_RESPONSE
    printf("f2_a:%f\r\n", pSub->valuedouble);
#endif
    if (f2_a != (float)pSub->valuedouble)
    {
      f2_a = (float)pSub->valuedouble;
      resp_cali.cali_val = (float)pSub->valuedouble;
      osi_at24c08_write(F2_A_ADDR, resp_cali.cali_buf); //f2_a
    }
  }
  pSub = cJSON_GetObjectItem(pJson, "f2_b"); //"f2_b"
  if (NULL != pSub)
  {
#ifdef DEBUG_RESPONSE
    printf("f2_b:%f\r\n", pSub->valuedouble);
#endif
    if (f2_b != (float)pSub->valuedouble)
    {
      f2_b = (float)pSub->valuedouble;
      resp_cali.cali_val = (float)pSub->valuedouble;
      osi_at24c08_write(F2_B_ADDR, resp_cali.cali_buf); //f2_b
    }
  }

  pSub = cJSON_GetObjectItem(pJson, "f3_a"); //"f3_a"
  if (NULL != pSub)
  {
#ifdef DEBUG_RESPONSE
    printf("f3_a:%f\r\n", pSub->valuedouble);
#endif
    if (f3_a != (float)pSub->valuedouble)
    {
      f3_a = (float)pSub->valuedouble;
      resp_cali.cali_val = (float)pSub->valuedouble;
      osi_at24c08_write(F3_A_ADDR, resp_cali.cali_buf); //f3_a
    }
  }
  pSub = cJSON_GetObjectItem(pJson, "f3_b"); //"f3_b"
  if (NULL != pSub)
  {
#ifdef DEBUG_RESPONSE
    printf("f3_b:%f\r\n", pSub->valuedouble);
#endif
    if (f3_b != (float)pSub->valuedouble)
    {
      f3_b = (float)pSub->valuedouble;
      resp_cali.cali_val = (float)pSub->valuedouble;
      osi_at24c08_write(F3_B_ADDR, resp_cali.cali_buf); //f3_b
    }
  }

  pSub = cJSON_GetObjectItem(pJson, "f4_a"); //"f4_a"
  if (NULL != pSub)
  {
#ifdef DEBUG_RESPONSE
    printf("f4_a:%f\r\n", pSub->valuedouble);
#endif
    if (f4_a != (float)pSub->valuedouble)
    {
      f4_a = (float)pSub->valuedouble;
      resp_cali.cali_val = (float)pSub->valuedouble;
      osi_at24c08_write(F4_A_ADDR, resp_cali.cali_buf); //f4_a
    }
  }
  pSub = cJSON_GetObjectItem(pJson, "f4_b"); //"f4_b"
  if (NULL != pSub)
  {
#ifdef DEBUG_RESPONSE
    printf("f4_b:%f\r\n", pSub->valuedouble);
#endif
    if (f4_b != (float)pSub->valuedouble)
    {
      f4_b = (float)pSub->valuedouble;
      resp_cali.cali_val = (float)pSub->valuedouble;
      osi_at24c08_write(F4_B_ADDR, resp_cali.cali_buf); //f4_b
    }
  }

  pSub = cJSON_GetObjectItem(pJson, "f5_a"); //"f5_a"
  if (NULL != pSub)
  {
#ifdef DEBUG_RESPONSE
    printf("f5_a:%f\r\n", pSub->valuedouble);
#endif
    if (f5_a != (float)pSub->valuedouble)
    {
      f5_a = (float)pSub->valuedouble;
      resp_cali.cali_val = (float)pSub->valuedouble;
      osi_at24c08_write(F5_A_ADDR, resp_cali.cali_buf); //f5_a
    }
  }
  pSub = cJSON_GetObjectItem(pJson, "f5_b"); //"f5_b"
  if (NULL != pSub)
  {
#ifdef DEBUG_RESPONSE
    printf("f5_b:%f\r\n", pSub->valuedouble);
#endif
    if (f5_b != (float)pSub->valuedouble)
    {
      f5_b = (float)pSub->valuedouble;
      resp_cali.cali_val = (float)pSub->valuedouble;
      osi_at24c08_write(F5_B_ADDR, resp_cali.cali_buf); //f5_b
    }
  }

  pSub = cJSON_GetObjectItem(pJson, "f6_a"); //"f6_a"
  if (NULL != pSub)
  {
#ifdef DEBUG_RESPONSE
    printf("f6_a:%f\r\n", pSub->valuedouble);
#endif
    if (f6_a != (float)pSub->valuedouble)
    {
      f6_a = (float)pSub->valuedouble;
      resp_cali.cali_val = (float)pSub->valuedouble;
      osi_at24c08_write(F6_A_ADDR, resp_cali.cali_buf); //f6_a
    }
  }
  pSub = cJSON_GetObjectItem(pJson, "f6_b"); //"f6_b"
  if (NULL != pSub)
  {
#ifdef DEBUG_RESPONSE
    printf("f6_b:%f\r\n", pSub->valuedouble);
#endif
    if (f6_b != (float)pSub->valuedouble)
    {
      f6_b = (float)pSub->valuedouble;
      resp_cali.cali_val = (float)pSub->valuedouble;
      osi_at24c08_write(F6_B_ADDR, resp_cali.cali_buf); //f6_b
    }
  }

  pSub = cJSON_GetObjectItem(pJson, "f7_a"); //"f7_a"
  if (NULL != pSub)
  {
#ifdef DEBUG_RESPONSE
    printf("f7_a:%f\r\n", pSub->valuedouble);
#endif
    if (f7_a != (float)pSub->valuedouble)
    {
      f7_a = (float)pSub->valuedouble;
      resp_cali.cali_val = (float)pSub->valuedouble;
      osi_at24c08_write(F7_A_ADDR, resp_cali.cali_buf); //f7_a
    }
  }
  pSub = cJSON_GetObjectItem(pJson, "f7_b"); //"f7_b"
  if (NULL != pSub)
  {
#ifdef DEBUG_RESPONSE
    printf("f7_b:%f\r\n", pSub->valuedouble);
#endif
    if (f7_b != (float)pSub->valuedouble)
    {
      f7_b = (float)pSub->valuedouble;
      resp_cali.cali_val = (float)pSub->valuedouble;
      osi_at24c08_write(F7_B_ADDR, resp_cali.cali_buf); //f7_b
    }
  }

  pSub = cJSON_GetObjectItem(pJson, "f8_a"); //"f8_a"
  if (NULL != pSub)
  {
#ifdef DEBUG_RESPONSE
    printf("f8_a:%f\r\n", pSub->valuedouble);
#endif
    if (f8_a != (float)pSub->valuedouble)
    {
      f8_a = (float)pSub->valuedouble;
      resp_cali.cali_val = (float)pSub->valuedouble;
      osi_at24c08_write(F8_A_ADDR, resp_cali.cali_buf); //f8_a
    }
  }
  pSub = cJSON_GetObjectItem(pJson, "f8_b"); //"f8_b"
  if (NULL != pSub)
  {
#ifdef DEBUG_RESPONSE
    printf("f8_b:%f\r\n", pSub->valuedouble);
#endif
    if (f8_b != (float)pSub->valuedouble)
    {
      f8_b = (float)pSub->valuedouble;
      resp_cali.cali_val = (float)pSub->valuedouble;
      osi_at24c08_write(F8_B_ADDR, resp_cali.cali_buf); //f8_b
    }
  }

  pSub = cJSON_GetObjectItem(pJson, "f9_a"); //"f9_a"
  if (NULL != pSub)
  {
#ifdef DEBUG_RESPONSE
    printf("f9_a:%f\r\n", pSub->valuedouble);
#endif
    if (f9_a != (float)pSub->valuedouble)
    {
      f9_a = (float)pSub->valuedouble;
      resp_cali.cali_val = (float)pSub->valuedouble;
      osi_at24c08_write(F9_A_ADDR, resp_cali.cali_buf); //f9_a
    }
  }
  pSub = cJSON_GetObjectItem(pJson, "f9_b"); //"f9_b"
  if (NULL != pSub)
  {
#ifdef DEBUG_RESPONSE
    printf("f9_b:%f\r\n", pSub->valuedouble);
#endif
    if (f9_b != (float)pSub->valuedouble)
    {
      f9_b = (float)pSub->valuedouble;
      resp_cali.cali_val = (float)pSub->valuedouble;
      osi_at24c08_write(F9_B_ADDR, resp_cali.cali_buf); //f9_b
    }
  }

  pSub = cJSON_GetObjectItem(pJson, "f10_a"); //"f10_a"
  if (NULL != pSub)
  {
#ifdef DEBUG_RESPONSE
    printf("f10_a:%f\r\n", pSub->valuedouble);
#endif
    if (f10_a != (float)pSub->valuedouble)
    {
      f10_a = (float)pSub->valuedouble;
      resp_cali.cali_val = (float)pSub->valuedouble;
      osi_at24c08_write(F10_A_ADDR, resp_cali.cali_buf); //f10_a
    }
  }
  pSub = cJSON_GetObjectItem(pJson, "f10_b"); //"f10_b"
  if (NULL != pSub)
  {
#ifdef DEBUG_RESPONSE
    printf("f10_b:%f\r\n", pSub->valuedouble);
#endif
    if (f10_b != (float)pSub->valuedouble)
    {
      f10_b = (float)pSub->valuedouble;
      resp_cali.cali_val = (float)pSub->valuedouble;
      osi_at24c08_write(F10_B_ADDR, resp_cali.cali_buf); //f10_b
    }
  }

  cJSON_Delete(pJson);

  return SUCCESS;
}

/*******************************************************************************
// parse response commands
*******************************************************************************/
int Parse_commands(char *ptr)
{
  if (NULL == ptr)
  {
    return FAILURE;
  }

  cJSON *pJson = cJSON_Parse(ptr);
  if (NULL == pJson)
  {
    cJSON_Delete(pJson); //delete pJson

    return FAILURE;
  }

  cJSON *pSub = cJSON_GetObjectItem(pJson, "cm_led"); //"cm_led"
  if (NULL != pSub)
  {
#ifdef DEBUG_RESPONSE
    osi_UartPrint_Val("\"cm_led\":", pSub->valueint);
#endif

    xQueueSend(xQueue1, &pSub->valueint, 0); //send led control message
  }

  pSub = cJSON_GetObjectItem(pJson, "cm_buzzer"); //"cm_buzzer"
  if (NULL != pSub)
  {
#ifdef DEBUG_RESPONSE

    osi_UartPrint_Val("\"cm_buzzer\":", pSub->valueint);

#endif

    xQueueSend(xQueue2, &pSub->valueint, 0); //send bell control message
  }

  pSub = cJSON_GetObjectItem(pJson, "cm_power_off"); //"cm_power_off"
  if (NULL != pSub)
  {
#ifdef DEBUG_RESPONSE

    osi_UartPrint_Val("\"cm_power_off\":", pSub->valueint);

#endif

    if (pSub->valueint == 1)
    {
      osi_at24c08_WriteData(SYSTEM_STATUS_ADDR, (uint8_t *)SYSTEM_OFF, strlen(SYSTEM_OFF), 1); //Restor The System Status-OFF
    }
  }

  pSub = cJSON_GetObjectItem(pJson, "Command"); //"Command"
  if (NULL != pSub)
  {
    if (!strcmp((char const *)pSub->valuestring, "SetupWifi")) //Command:SetupWifi
    {
      pSub = cJSON_GetObjectItem(pJson, "SSID"); //"SSID"
      if (NULL != pSub)
      {
#ifdef DEBUG
        osi_UartPrint_Mul("\"SSID\":", pSub->valuestring);
#endif

        if (sizeof(SSID_NAME) >= strlen(pSub->valuestring))
        {
          osi_at24c08_WriteData(SSID_FLAG_ADDR, (uint8_t *)"SSID", strlen("SSID"), 1); //save ssid flag to at24c08
          osi_at24c08_write_byte(SSID_LEN_ADDR, strlen(pSub->valuestring));
          osi_at24c08_WriteData(SSID_ADDR, (uint8_t *)pSub->valuestring, strlen(pSub->valuestring), 0); //save ssid
        }
      }

      pSub = cJSON_GetObjectItem(pJson, "password"); //"password"
      if (NULL != pSub)
      {
#ifdef DEBUG
        osi_UartPrint_Mul("\"password\":", pSub->valuestring);
#endif

        if (sizeof(PASS_WORD) >= strlen(pSub->valuestring))
        {
          osi_at24c08_write_byte(PASSWORD_LEN_ADDR, strlen(pSub->valuestring));
          osi_at24c08_WriteData(PASSWORD_ADDR, (uint8_t *)pSub->valuestring, strlen(pSub->valuestring), 0); //save password
        }
      }

      pSub = cJSON_GetObjectItem(pJson, "type"); //"type"
      if (NULL != pSub)
      {
#ifdef DEBUG
        osi_UartPrint_Mul("\"type\":", pSub->valuestring);
#endif

        osi_at24c08_WriteData(SECTYPE_ADDR, (uint8_t *)pSub->valuestring, strlen(pSub->valuestring), 1); //save type
      }

      pSub = cJSON_GetObjectItem(pJson, "backup_ip"); //"backup_ip"
      if (NULL != pSub)
      {
#ifdef DEBUG
        osi_UartPrint_Mul("\"backup_ip\":", pSub->valuestring);
#endif

        char *InpString;
        InpString = strtok(pSub->valuestring, ".");
        HOST_IP[0] = (uint8_t)strtoul(InpString, 0, 10);
        InpString = strtok(NULL, ".");
        HOST_IP[1] = (uint8_t)strtoul(InpString, 0, 10);
        InpString = strtok(NULL, ".");
        HOST_IP[2] = (uint8_t)strtoul(InpString, 0, 10);
        InpString = strtok(NULL, ".");
        HOST_IP[3] = (uint8_t)strtoul(InpString, 0, 10);

        Parse_IP_MASK_GW_DNS(pSub->valuestring, HOST_IP);
        osi_at24c08_WriteData(HOST_IP_ADDR, HOST_IP, sizeof(HOST_IP), 1); //Save Host IP
      }

      uint8_t set_buf[4];
      //      char *InpString;
      pSub = cJSON_GetObjectItem(pJson, "dhcp"); //"dhcp"
      if (NULL != pSub)
      {
#ifdef DEBUG_RESPONSE
        osi_UartPrint_Val("\"dhcp\":", pSub->valueint);
#endif
        osi_at24c08_write_byte(DHCP_MODE_ADDR, ((uint8_t)pSub->valueint));
      }
      pSub = cJSON_GetObjectItem(pJson, "ip"); //"ip"
      if (NULL != pSub)
      {
#ifdef DEBUG
        osi_UartPrint_Mul("\"ip\":", pSub->valuestring);
#endif
        //        InpString = strtok(pSub->valuestring, ".");
        //        set_buf[0]=(uint8_t)strtoul(InpString, 0, 10);
        //        InpString = strtok(NULL, ".");
        //        set_buf[1]=(uint8_t)strtoul(InpString, 0, 10);
        //        InpString = strtok(NULL, ".");
        //        set_buf[2]=(uint8_t)strtoul(InpString, 0, 10);
        //        InpString = strtok(NULL, ".");
        //        set_buf[3]=(uint8_t)strtoul(InpString, 0, 10);

        memset(set_buf, 0, sizeof(set_buf));
        Parse_IP_MASK_GW_DNS(pSub->valuestring, set_buf);
        osi_at24c08_WriteData(STATIC_IP_ADDR, set_buf, sizeof(set_buf), 1); //Save STATIC IP
      }

      pSub = cJSON_GetObjectItem(pJson, "mask"); //"mask"
      if (NULL != pSub)
      {
#ifdef DEBUG
        osi_UartPrint_Mul("\"mask\":", pSub->valuestring);
#endif
        //        InpString = strtok(pSub->valuestring, ".");
        //        set_buf[0]=(uint8_t)strtoul(InpString, 0, 10);
        //        InpString = strtok(NULL, ".");
        //        set_buf[1]=(uint8_t)strtoul(InpString, 0, 10);
        //        InpString = strtok(NULL, ".");
        //        set_buf[2]=(uint8_t)strtoul(InpString, 0, 10);
        //        InpString = strtok(NULL, ".");
        //        set_buf[3]=(uint8_t)strtoul(InpString, 0, 10);

        memset(set_buf, 0, sizeof(set_buf));
        Parse_IP_MASK_GW_DNS(pSub->valuestring, set_buf);
        osi_at24c08_WriteData(STATIC_SN_ADDR, set_buf, sizeof(set_buf), 1); //Save STATIC SN
      }

      pSub = cJSON_GetObjectItem(pJson, "gw"); //"gw"
      if (NULL != pSub)
      {
#ifdef DEBUG
        osi_UartPrint_Mul("\"gw\":", pSub->valuestring);
#endif
        //        InpString = strtok(pSub->valuestring, ".");
        //        set_buf[0]=(uint8_t)strtoul(InpString, 0, 10);
        //        InpString = strtok(NULL, ".");
        //        set_buf[1]=(uint8_t)strtoul(InpString, 0, 10);
        //        InpString = strtok(NULL, ".");
        //        set_buf[2]=(uint8_t)strtoul(InpString, 0, 10);
        //        InpString = strtok(NULL, ".");
        //        set_buf[3]=(uint8_t)strtoul(InpString, 0, 10);

        memset(set_buf, 0, sizeof(set_buf));
        Parse_IP_MASK_GW_DNS(pSub->valuestring, set_buf);
        osi_at24c08_WriteData(STATIC_GW_ADDR, set_buf, sizeof(set_buf), 1); //Save STATIC GW
      }

      pSub = cJSON_GetObjectItem(pJson, "dns1"); //"dns1"
      if (NULL != pSub)
      {
#ifdef DEBUG
        osi_UartPrint_Mul("\"dns1\":", pSub->valuestring);
#endif
        //        InpString = strtok(pSub->valuestring, ".");
        //        set_buf[0]=(uint8_t)strtoul(InpString, 0, 10);
        //        InpString = strtok(NULL, ".");
        //        set_buf[1]=(uint8_t)strtoul(InpString, 0, 10);
        //        InpString = strtok(NULL, ".");
        //        set_buf[2]=(uint8_t)strtoul(InpString, 0, 10);
        //        InpString = strtok(NULL, ".");
        //        set_buf[3]=(uint8_t)strtoul(InpString, 0, 10);

        memset(set_buf, 0, sizeof(set_buf));
        Parse_IP_MASK_GW_DNS(pSub->valuestring, set_buf);
        osi_at24c08_WriteData(STATIC_DNS_ADDR, set_buf, sizeof(set_buf), 1); //Save STATIC DNS
      }
      /*
      uint8_t dhcp_mode;    //0:static ;1:dhcp
      dhcp_mode = osi_at24c08_read_byte(DHCP_MODE_ADDR); 
      if(dhcp_mode)
      {
        unsigned char ucVal = 1;
        sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&ucVal);  //Enable DHCP client
      }
      else
      {
        SlNetCfgIpV4Args_t ipV4;
        uint8_t ip[4]={0};    //< Source IP Address
        uint8_t sn[4]={0};    //< Subnet Mask
        uint8_t gw[4]={0};    //< Gateway IP Address
        uint8_t dns[4]={0};   //< DNS server IP Address
        osi_at24c08_ReadData(STATIC_IP_ADDR,ip,sizeof(ip),1);
        osi_at24c08_ReadData(STATIC_SN_ADDR,sn,sizeof(sn),1);
        osi_at24c08_ReadData(STATIC_GW_ADDR,gw,sizeof(gw),1);
        osi_at24c08_ReadData(STATIC_DNS_ADDR,dns,sizeof(dns),1);
        
        ipV4.ipV4 = (_u32)SL_IPV4_VAL(ip[0],ip[1],ip[2],ip[3]); // _u32 IP address
        ipV4.ipV4Mask = (_u32)SL_IPV4_VAL(sn[0],sn[1],sn[2],sn[3]); // _u32 Subnet mask for this STA/P2P
        ipV4.ipV4Gateway = (_u32)SL_IPV4_VAL(gw[0],gw[1],gw[2],gw[3]); // _u32 Default gateway address
        ipV4.ipV4DnsServer = (_u32)SL_IPV4_VAL(dns[0],dns[1],dns[2],dns[3]); // _u32 DNS server address
        
        sl_NetCfgSet(SL_IPV4_STA_P2P_CL_STATIC_ENABLE,IPCONFIG_MODE_ENABLE_IPV4,sizeof(SlNetCfgIpV4Args_t),(_u8 *)&ipV4);
      }
*/
    }
    else if (!strcmp((char const *)pSub->valuestring, "SetupHost")) //Command:SetupHost
    {
      pSub = cJSON_GetObjectItem(pJson, "Host"); //"Host"
      if (NULL != pSub)
      {
#ifdef DEBUG
        osi_UartPrint_Mul("\"Host\":", pSub->valuestring);
#endif

        osi_at24c08_WriteData(HOST_ADDR, (uint8_t *)pSub->valuestring, strlen(pSub->valuestring), 1); //save host
      }
      pSub = cJSON_GetObjectItem(pJson, "Port"); //"Port"
      if (NULL != pSub)
      {
#ifdef DEBUG
        osi_UartPrint_Val("\"Port\":", pSub->valueint);
#endif

        osi_at24c08_write(HOST_PORT_ADDR, pSub->valueint); //Port
      }
    }
  }

  pSub = cJSON_GetObjectItem(pJson, "action"); //
  if (pSub != NULL)
  {
    if (strcmp(pSub->valuestring, "ota") == 0)
    {
      pSub = cJSON_GetObjectItem(pJson, "url"); //
      if (pSub != NULL)
      {
        strcpy(mqtt_ota_url, pSub->valuestring);
        ESP_LOGI(TAG, "OTA_URL=%s", mqtt_ota_url);

        // pSub = cJSON_GetObjectItem(json_data_parse_1, "size"); //
        // if (pSub != NULL)
        // {
        // mqtt_json_s.mqtt_file_size = (uint32_t)pSub->valuedouble;
        // ESP_LOGI(TAG, "OTA_SIZE=%d", mqtt_json_s.mqtt_file_size);

        pSub = cJSON_GetObjectItem(pJson, "version"); //
        if (pSub != NULL)
        {
          // if (strcmp(pSub->valuestring, FIRMWARE) != 0) //与当前 版本号 对比
          // {
          ESP_LOGI(TAG, "OTA_VERSION=%s", pSub->valuestring);
          ota_start(); //启动OTA
          // }
        }
        // }
      }
    }
  }

  cJSON_Delete(pJson);

  return SUCCESS;
}

/*******************************************************************************
// parse post response data
*******************************************************************************/
int ParseJSONData(char *ptr)
{
  cJSON *pSub;

  if (NULL == ptr)
  {
    return FAILURE;
  }

  cJSON *pJsonJson = cJSON_Parse(ptr);
  if (pJsonJson == NULL)
  {
    cJSON_Delete(pJsonJson); //delete pJson

    return FAILURE;
  }
  /* 
  pSub = cJSON_GetObjectItem(pJsonJson, "result");  //"result"
  if(NULL!=pSub)
  {
    #ifdef DEBUG_RESPONSE
      osi_UartPrint("\"result\":");
      osi_UartPrint(pSub->valuestring);
      osi_UartPrint("\r\n");
    #endif
    
    if(!strcmp((char const*)pSub->valuestring,"error"))
    {
      pSub = cJSON_GetObjectItem(pJsonJson, "errorCode");  //errorCode
      if(NULL != pSub)
      {
        #ifdef DEBUG_RESPONSE
          osi_UartPrint("\"errorCode\":");
          osi_UartPrint(pSub->valuestring);
          osi_UartPrint("\r\n");
        #endif
        if((!strcmp((char const*)pSub->valuestring,"invalid_format"))||(!strcmp((char const*)pSub->valuestring,"invalid_json_format")))
        {
          cJSON_Delete(pJsonJson);  //delete pJson
  
          return 400;
        }
      }
      cJSON_Delete(pJsonJson);  //delete pJson
          
      return FAILURE;
    }
  }
*/
  pSub = cJSON_GetObjectItem(pJsonJson, "server_time"); //"server_time"
  if (NULL != pSub)
  {
#ifdef DEBUG_RESPONSE

    osi_UartPrint_Mul("\"server_time\":", pSub->valuestring);

#endif

    osi_Update_UTCtime(pSub->valuestring); //update time
  }

#ifdef DEBUG_RESPONSE

  pSub = cJSON_GetObjectItem(pJsonJson, "channel_id"); //"channel_id"
  if (NULL != pSub)
  {
    osi_UartPrint_Mul("\"channel_id\":", pSub->valuestring);
  }

#endif

#ifdef DEBUG_RESPONSE

  pSub = cJSON_GetObjectItem(pJsonJson, "last_entry_id"); //"last_entry_id"
  if (NULL != pSub)
  {
    osi_UartPrint_Val("\"last_entry_id\":", pSub->valueint);
  }

#endif

#ifdef DEBUG_RESPONSE

  pSub = cJSON_GetObjectItem(pJsonJson, "num_records_updated"); //"num_records_updated"
  if (NULL != pSub)
  {
    osi_UartPrint_Val("\"num_records_updated\":", pSub->valueint);
  }

#endif

#ifdef DEBUG_RESPONSE

  pSub = cJSON_GetObjectItem(pJsonJson, "num_records_matched"); //"num_records_matched"
  if (NULL != pSub)
  {
    osi_UartPrint_Val("\"num_records_updated\":", pSub->valueint);
  }

#endif

#ifdef DEBUG_RESPONSE

  pSub = cJSON_GetObjectItem(pJsonJson, "user_id"); //user_id
  if (NULL != pSub)
  {
    osi_UartPrint_Mul("\"user_id\":", pSub->valuestring);
  }

#endif

  pSub = cJSON_GetObjectItem(pJsonJson, "command"); //"command"
  if (NULL != pSub)
  {

#ifdef DEBUG_RESPONSE

    osi_UartPrint("\"command\":");

#endif

    cJSON *pSubSub;

#ifdef DEBUG_RESPONSE

    pSubSub = cJSON_GetObjectItem(pSub, "command_id"); //"command_id"
    if (NULL != pSubSub)
    {
      osi_UartPrint_Mul("\"command_id\":", pSubSub->valuestring);
    }

#endif

    pSubSub = cJSON_GetObjectItem(pSub, "command_string"); //"command_string"
    if (NULL != pSubSub)
    {
#ifdef DEBUG_RESPONSE

      osi_UartPrint_Mul("\"command_string\":", pSubSub->valuestring);

#endif

      Parse_commands(pSubSub->valuestring);
    }

#ifdef DEBUG_RESPONSE

    pSubSub = cJSON_GetObjectItem(pSub, "position"); //"position"
    if (NULL != pSubSub)
    {
      osi_UartPrint_Mul("\"position\":", pSubSub->valuestring);
    }

#endif

#ifdef DEBUG_RESPONSE

    pSubSub = cJSON_GetObjectItem(pSub, "executed_at"); //"executed_at"
    if (NULL != pSubSub)
    {
      osi_UartPrint_Val("\"executed_at\":", pSubSub->valueint);
    }

#endif

#ifdef DEBUG_RESPONSE

    pSubSub = cJSON_GetObjectItem(pSub, "created_at"); //"created_at"
    if (NULL != pSubSub)
    {
      osi_UartPrint_Mul("\"created_at\":", pSubSub->valuestring);
    }

#endif
  }

  pSub = cJSON_GetObjectItem(pJsonJson, "metadata"); //"metadata"
  if (NULL != pSub)
  {
#ifdef DEBUG_RESPONSE

    osi_UartPrint_Mul("\"metadata\":", pSub->valuestring);

#endif

    Parse_metadata(pSub->valuestring);
  }
  pSub = cJSON_GetObjectItem(pJsonJson, "cali"); //"cali"
  if (NULL != pSub)
  {
#ifdef DEBUG_RESPONSE
    osi_UartPrint("\"cali\":");
    osi_UartPrint(pSub->valuestring);
    osi_UartPrint("\r\n");
#endif

    Parse_cali(pSub->valuestring);
  }
  cJSON_Delete(pJsonJson); //delete pJson

  return SUCCESS;
}

/*******************************************************************************
// parse update time data
*******************************************************************************/
char ParseTimeData(char *ptr)
{
  if (NULL == ptr)
  {
    return FAILURE;
  }

  cJSON *pJson = cJSON_Parse(ptr);
  if (NULL == pJson)
  {
    cJSON_Delete(pJson); //delete pJson

    return FAILURE;
  }

  cJSON *pSub;

#ifdef DEBUG_RESPONSE

  pSub = cJSON_GetObjectItem(pJson, "result"); //"result"
  if (NULL != pSub)
  {
    osi_UartPrint_Mul("\"result\":", pSub->valuestring);
  }

#endif

  pSub = cJSON_GetObjectItem(pJson, "server_time"); //"server_time"
  if (NULL != pSub)
  {
#ifdef DEBUG_RESPONSE

    osi_UartPrint_Mul("\"server_time\":", pSub->valuestring);

#endif

    osi_Update_UTCtime(pSub->valuestring); //update time
  }

  cJSON_Delete(pJson); //delete pJson

  return SUCCESS;
}

/*******************************************************************************
  parse get set data
*******************************************************************************/
int ParseSetJSONData(char *ptr)
{
  uint8_t i = 0;

  if (NULL == ptr)
  {
    return FAILURE;
  }

  cJSON *pJson = cJSON_Parse(ptr);
  if (NULL == pJson)
  {
    cJSON_Delete(pJson); //delete pJson

    return FAILURE;
  }

  cJSON *pSub;
  /*
  pSub = cJSON_GetObjectItem(pJson, "result");  //result
  if(NULL!=pSub)
  {
    #ifdef DEBUG_RESPONSE
      osi_UartPrint("\"result\":");
      osi_UartPrint(pSub->valuestring);
      osi_UartPrint("\r\n"); 
    #endif
    
    if(!strcmp((char const*)pSub->valuestring,"error"))
    {
      cJSON_Delete(pJson);  //delete pJson
      
      return FAILURE;
    }
  }
*/
  pSub = cJSON_GetObjectItem(pJson, "server_time"); //server_time
  if (NULL != pSub)
  {

#ifdef DEBUG_RESPONSE

    osi_UartPrint_Mul("\"server_time\":", pSub->valuestring);

#endif

    osi_Update_UTCtime(pSub->valuestring); //update time
  }

  pSub = cJSON_GetObjectItem(pJson, "channel"); //channel
  if (NULL != pSub)
  {
#ifdef DEBUG_RESPONSE

    osi_UartPrint("\"channel\":");

#endif

    cJSON *pSubSub;

    pSubSub = cJSON_GetObjectItem(pSub, "channel_id"); //channel_id
    if (NULL != pSubSub)
    {
#ifdef DEBUG_RESPONSE

      osi_UartPrint_Mul("\"channel_id\":", pSubSub->valuestring);

#endif

      osi_at24c08_WriteData(CHANNEL_ID_ADDR, (uint8_t *)pSubSub->valuestring, strlen(pSubSub->valuestring), 1);
    }

#ifdef DEBUG_RESPONSE

    pSubSub = cJSON_GetObjectItem(pSub, "name"); //name
    if (NULL != pSubSub)
    {
      osi_UartPrint_Mul("\"name\":", pSubSub->valuestring);
    }

#endif

    pSubSub = cJSON_GetObjectItem(pSub, "metadata"); //metadata
    if (NULL != pSubSub)
    {
#ifdef DEBUG_RESPONSE

      osi_UartPrint_Mul("\"metadata\":", pSubSub->valuestring);

#endif

      Parse_metadata(pSubSub->valuestring); //parse metadata
    }
    pSubSub = cJSON_GetObjectItem(pSub, "cali"); //"cali"
    if (NULL != pSubSub)
    {
#ifdef DEBUG_RESPONSE
      osi_UartPrint("\"cali\":");
      osi_UartPrint(pSubSub->valuestring);
      osi_UartPrint("\r\n");
#endif

      Parse_cali(pSubSub->valuestring);
    }

    pSubSub = cJSON_GetObjectItem(pSub, "user_id"); //user_id
    if (NULL != pSubSub)
    {
#ifdef DEBUG_RESPONSE

      osi_UartPrint_Mul("\"user_id\":", pSubSub->valuestring);

#endif

      osi_at24c08_WriteData(USER_ID_ADDR, (uint8_t *)pSubSub->valuestring, strlen(pSubSub->valuestring), 1);
    }

    pSubSub = cJSON_GetObjectItem(pSub, "write_key"); //write_key
    if (NULL != pSubSub)
    {
#ifdef DEBUG_RESPONSE

      osi_UartPrint_Mul("\"write_key\":", pSubSub->valuestring);

#endif

      memcpy(ProductURI, DataURI1, strlen(DataURI1) + 1);

      i += strlen(DataURI1);

      memcpy(ProductURI + i, pSubSub->valuestring, strlen(pSubSub->valuestring) + 1);

      i += strlen(pSubSub->valuestring);

      // memcpy(ProductURI + i, FIRMWAREVIEW, strlen(FIRMWAREVIEW) + 1);

      // i += strlen(FIRMWAREVIEW);

      ProductURI[i] = '\0';
      ESP_LOGI(TAG, "%d,%s", __LINE__, ProductURI);

      osi_at24c08_WriteData(DATAURI_FLAG_ADDR, (uint8_t *)DATA_URI, strlen(DATA_URI), 1); //save datauri flag to at24c08

      osi_at24c08_WriteData(DATAURI_ADDR, (uint8_t *)ProductURI, strlen(ProductURI), 1); //save datauri to at24c08
    }
  }

  cJSON_Delete(pJson); //delete pJson

  return SUCCESS;
}

/*******************************************************************************
//make product set data json
*******************************************************************************/
void Read_Product_Set(char *read_buf, uint16_t data_len)
{
  char *out_buf;
  cJSON *pJsonRoot;
  char product_id[32] = {0};
  char series_number[16] = {0};
  char channel_id[8] = {0};
  char user_id[40] = {0};
  char useage[5] = {0};
  char host_ip_buf[16] = {0};
  char mac_buf[18] = {0};
  uint8_t mac_addr[8] = {0};
  uint8_t use_age = 0;
  uint32_t host_port_num = osi_at24c08_read(HOST_PORT_ADDR);
  osi_at24c08_ReadData(PRODUCT_ID_ADDR, (uint8_t *)product_id, sizeof(product_id), 1);

  osi_at24c08_ReadData(SERISE_NUM_ADDR, (uint8_t *)series_number, sizeof(series_number), 1);

  osi_at24c08_ReadData(HOST_ADDR, (uint8_t *)HOST_NAME, sizeof(HOST_NAME), 1);

  osi_at24c08_ReadData(HOST_IP_ADDR, HOST_IP, sizeof(HOST_IP), 1);

  snprintf(host_ip_buf, sizeof(host_ip_buf), "%d.%d.%d.%d", HOST_IP[0], HOST_IP[1], HOST_IP[2], HOST_IP[3]);

  osi_at24c08_ReadData(CHANNEL_ID_ADDR, (uint8_t *)channel_id, sizeof(channel_id), 1);

  osi_at24c08_ReadData(USER_ID_ADDR, (uint8_t *)user_id, sizeof(user_id), 1);

  xSemaphoreTake(xMutex3, -1); //cJSON Semaphore Take

  pJsonRoot = cJSON_CreateObject();

  cJSON_AddStringToObject(pJsonRoot, "ProductID", product_id);

  cJSON_AddStringToObject(pJsonRoot, "SeriesNumber", series_number);

  cJSON_AddStringToObject(pJsonRoot, "Host", HOST_NAME);
  cJSON_AddNumberToObject(pJsonRoot, "Port", host_port_num);
  cJSON_AddStringToObject(pJsonRoot, "backup_ip", host_ip_buf);

  cJSON_AddStringToObject(pJsonRoot, "CHANNEL_ID", channel_id);

  cJSON_AddStringToObject(pJsonRoot, "USER_ID", user_id);

  use_age = Get_Usage_Val();

  snprintf(useage, sizeof(useage), "%d%%", use_age);

  cJSON_AddStringToObject(pJsonRoot, "USAGE", useage);

  // osi_at24c08_ReadData(MAC_ADDR, mac_addr, SL_MAC_ADDR_LEN, 0);
  esp_read_mac(mac_addr, 0); //获取芯片内部默认出厂MAC
  snprintf(mac_buf, sizeof(mac_buf), "%02x:%02x:%02x:%02x:%02x:%02x", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

  cJSON_AddStringToObject(pJsonRoot, "MAC", mac_buf);

  cJSON_AddStringToObject(pJsonRoot, "firmware", FIRMWARENUM);

  out_buf = cJSON_PrintUnformatted(pJsonRoot); //cJSON_Print(Root)

  cJSON_Delete(pJsonRoot); //delete cjson root

  memcpy(read_buf, out_buf, data_len);

  cJSON_free(out_buf);

  xSemaphoreGive(xMutex3); //cJSON Semaphore Give
}

/*******************************************************************************
  make wifi set data json
*******************************************************************************/
void Read_Wifi_Set(char *read_buf, uint16_t data_len)
{
  uint8_t read_len;
  char *out_buf;
  cJSON *pJsonRoot;

  memset(SSID_NAME, 0, sizeof(SSID_NAME));

  memset(PASS_WORD, 0, sizeof(PASS_WORD));

  read_len = osi_at24c08_read_byte(SSID_LEN_ADDR);

  if (read_len > sizeof(SSID_NAME))
  {
    read_len = sizeof(SSID_NAME);
  }

  osi_at24c08_ReadData(SSID_ADDR, (uint8_t *)SSID_NAME, read_len, 0); //Read Wifi SSID

  read_len = osi_at24c08_read_byte(PASSWORD_LEN_ADDR);
  if (read_len > sizeof(PASS_WORD))
  {
    read_len = sizeof(PASS_WORD);
  }

  osi_at24c08_ReadData(PASSWORD_ADDR, (uint8_t *)PASS_WORD, read_len, 0); //Read Wifi Password

  osi_at24c08_ReadData(SECTYPE_ADDR, (uint8_t *)SEC_TYPE, sizeof(SEC_TYPE), 1);

  xSemaphoreTake(xMutex3, -1); //cJSON Semaphore Take

  pJsonRoot = cJSON_CreateObject();

  cJSON_AddStringToObject(pJsonRoot, "SSID", SSID_NAME);

  cJSON_AddStringToObject(pJsonRoot, "password", PASS_WORD);

  cJSON_AddStringToObject(pJsonRoot, "type", SEC_TYPE);

  uint8_t dhcp_mode;
  uint8_t ip[4] = {0};  //< Source IP Address
  uint8_t sn[4] = {0};  //< Subnet Mask
  uint8_t gw[4] = {0};  //< Gateway IP Address
  uint8_t dns[4] = {0}; //< DNS server IP Address
                        //  uint8_t sdns[4]={0};  //< DNS server IP Address
  uint8_t set_buf[16] = {0};

  dhcp_mode = osi_at24c08_read_byte(DHCP_MODE_ADDR);
  osi_at24c08_ReadData(STATIC_IP_ADDR, ip, sizeof(ip), 1);
  osi_at24c08_ReadData(STATIC_SN_ADDR, sn, sizeof(sn), 1);
  osi_at24c08_ReadData(STATIC_GW_ADDR, gw, sizeof(gw), 1);
  osi_at24c08_ReadData(STATIC_DNS_ADDR, dns, sizeof(dns), 1);
  //  osi_at24c08_ReadData(STATIC_SDNS_ADDR,sdns,sizeof(sdns),1);

  cJSON_AddNumberToObject(pJsonRoot, "dhcp", dhcp_mode);

  memset(set_buf, 0, sizeof(set_buf));
  snprintf((char *)set_buf, sizeof(set_buf), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
  cJSON_AddStringToObject(pJsonRoot, "ip", (char *)set_buf);

  memset(set_buf, 0, sizeof(set_buf));
  snprintf((char *)set_buf, sizeof(set_buf), "%d.%d.%d.%d", sn[0], sn[1], sn[2], sn[3]);
  cJSON_AddStringToObject(pJsonRoot, "mask", (char *)set_buf);

  memset(set_buf, 0, sizeof(set_buf));
  snprintf((char *)set_buf, sizeof(set_buf), "%d.%d.%d.%d", gw[0], gw[1], gw[2], gw[3]);
  cJSON_AddStringToObject(pJsonRoot, "gw", (char *)set_buf);

  memset(set_buf, 0, sizeof(set_buf));
  snprintf((char *)set_buf, sizeof(set_buf), "%d.%d.%d.%d", dns[0], dns[1], dns[2], dns[3]);
  cJSON_AddStringToObject(pJsonRoot, "dns1", (char *)set_buf);

  //  memset(set_buf,0,sizeof(set_buf));
  //  snprintf((char *)set_buf,sizeof(set_buf),"%d.%d.%d.%d",sdns[0],sdns[1],sdns[2],sdns[3]);
  //  cJSON_AddStringToObject(pJsonRoot,"dns2",(char *)set_buf);

  out_buf = cJSON_PrintUnformatted(pJsonRoot); //cJSON_Print(Root)

  cJSON_Delete(pJsonRoot); //delete cjson root

  memcpy(read_buf, out_buf, data_len);

  cJSON_free(out_buf);

  xSemaphoreGive(xMutex3); //cJSON Semaphore Give
}

/*******************************************************************************
  make meta data json
*******************************************************************************/
void Cmd_Read_MetaData(char *read_buf, uint16_t data_len)
{
  char *out_buf;
  cJSON *pJsonRoot;

  osi_MetaData_Read();

  xSemaphoreTake(xMutex3, -1); //cJSON Semaphore Take

  pJsonRoot = cJSON_CreateObject();

  cJSON_AddNumberToObject(pJsonRoot, "fn_th", fn_th);

  cJSON_AddNumberToObject(pJsonRoot, "fn_light", fn_light);
#ifdef MAG_SENSOR
  cJSON_AddNumberToObject(pJsonRoot, "fn_mag", fn_mag);

  cJSON_AddNumberToObject(pJsonRoot, "fn_mag_int", fn_mag_int);
#endif
  cJSON_AddNumberToObject(pJsonRoot, "fn_acc_tap1", fn_acc_tap1);

  cJSON_AddNumberToObject(pJsonRoot, "fn_acc_tap2", fn_acc_tap2);

  cJSON_AddNumberToObject(pJsonRoot, "fn_acc_act", fn_acc_act);

  cJSON_AddNumberToObject(pJsonRoot, "thres_acc_min", thres_acc_min);

  cJSON_AddNumberToObject(pJsonRoot, "fn_bt", fn_bt);

  cJSON_AddNumberToObject(pJsonRoot, "fn_ext", fn_ext);

  cJSON_AddNumberToObject(pJsonRoot, "fn_battery", fn_battery);

  cJSON_AddNumberToObject(pJsonRoot, "fn_dp", fn_dp);

  cJSON_AddNumberToObject(pJsonRoot, "cg_data_led", cg_data_led);

  cJSON_AddNumberToObject(pJsonRoot, "no_net_fn", no_net_fn);

  cJSON_AddNumberToObject(pJsonRoot, "wifi_mode", wifi_mode);

  out_buf = cJSON_PrintUnformatted(pJsonRoot); //cJSON_Print(Root)

  cJSON_Delete(pJsonRoot); //delete cjson root

  memcpy(read_buf, out_buf, data_len);

  cJSON_free(out_buf);

  xSemaphoreGive(xMutex3); //cJSON Semaphore Give
}

/*******************************************************************************
  make system test data json
*******************************************************************************/
void Cmd_System_TestData(char *read_buf, uint16_t data_len)
{
  char *out_buf;
  cJSON *pJsonRoot;
  char utc_time[21];
  char rssi_ssid[32];
  char mac_buf[18] = {0};
  uint8_t mac_addr[8] = {0};
  int rssi_val = 0;
  float sensor_val1, sensor_val2;
  short acce_xval, acce_yval, acce_zval;

  if (flash_set_val != SUCCESS)
  {
    xSemaphoreTake(xMutex1, -1); //SPI Semaphore Take

    flash_set_val = w25q_Init();

    xSemaphoreGive(xMutex1); //SPI Semaphore Give
  }

  osi_Scan_Wifi_List(rssi_ssid, &rssi_val, 0); //Scan WIFI LIST with locked

  xSemaphoreTake(xMutex3, -1); //cJSON Semaphore Take

  pJsonRoot = cJSON_CreateObject();

  // osi_at24c08_ReadData(MAC_ADDR, mac_addr, SL_MAC_ADDR_LEN, 0);
  esp_read_mac(mac_addr, 0); //获取芯片内部默认出厂MAC
  snprintf(mac_buf, sizeof(mac_buf), "%02x:%02x:%02x:%02x:%02x:%02x", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

  cJSON_AddStringToObject(pJsonRoot, "MAC", mac_buf);

  osi_Read_UTCtime(utc_time, sizeof(utc_time)); //read time

  cJSON_AddStringToObject(pJsonRoot, "created_at", utc_time);

  osi_sht30_SingleShotMeasure(&sensor_val1, &sensor_val2); //read temperature humility data with locked

  cJSON_AddNumberToObject(pJsonRoot, "temp_val", (float)sensor_val1);

  cJSON_AddNumberToObject(pJsonRoot, "humi_val", (float)sensor_val2);

  osi_OPT3001_value(&sensor_val1); //Read Light Value with locked

  cJSON_AddNumberToObject(pJsonRoot, "light_val", (float)sensor_val1);

  sensor_val1 = power_adcValue(); //power voltage value

  cJSON_AddNumberToObject(pJsonRoot, "power_vol_val", (float)sensor_val1);

  cJSON_AddStringToObject(pJsonRoot, "ssid", rssi_ssid);

  cJSON_AddNumberToObject(pJsonRoot, "rssi", (int)rssi_val);

  osi_ADXL345_RD_xyz(&acce_xval, &acce_yval, &acce_zval); //read three Axis data with locked

  cJSON_AddNumberToObject(pJsonRoot, "acce_xval", (int)acce_xval);

  cJSON_AddNumberToObject(pJsonRoot, "acce_yval", (int)acce_yval);

  cJSON_AddNumberToObject(pJsonRoot, "acce_zval", (int)acce_zval);

  cJSON_AddNumberToObject(pJsonRoot, "flash", (int)(flash_set_val + 1));

#ifdef MAG_SENSOR
  rssi_val = (short)MagSensor_Status(); //mag sensor status

  cJSON_AddNumberToObject(pJsonRoot, "mag_val", (int)rssi_val);
#endif

  sensor_val1 = osi_ds18b20_get_temp(); //measure the EXT temperature

  cJSON_AddNumberToObject(pJsonRoot, "ext_temp_val", (float)sensor_val1);

  out_buf = cJSON_PrintUnformatted(pJsonRoot); //cJSON_Print(Root)

  cJSON_Delete(pJsonRoot); //delete cjson root

  memcpy(read_buf, out_buf, data_len);

  cJSON_free(out_buf);

  xSemaphoreGive(xMutex3); //cJSON Semaphore Give
}

/******************************************************************************
//parse IP/MASK/GW/DNS 
******************************************************************************/
void Parse_IP_MASK_GW_DNS(char *parse_buf, uint8_t *res_buf)
{
  char *InpString;
  InpString = strtok(parse_buf, ".");
  res_buf[0] = (uint8_t)strtoul(InpString, 0, 10);
  InpString = strtok(NULL, ".");
  res_buf[1] = (uint8_t)strtoul(InpString, 0, 10);
  InpString = strtok(NULL, ".");
  res_buf[2] = (uint8_t)strtoul(InpString, 0, 10);
  InpString = strtok(NULL, ".");
  res_buf[3] = (uint8_t)strtoul(InpString, 0, 10);
}

/******************************************************************************
//parse Mac address 
******************************************************************************/
void Parse_Mac_addr(char *parse_buf, uint8_t *res_buf)
{
  char *InpString;
  InpString = strtok(parse_buf, ":");
  res_buf[0] = (uint8_t)strtoul(InpString, 0, 16);
  InpString = strtok(NULL, ":");
  res_buf[1] = (uint8_t)strtoul(InpString, 0, 16);
  InpString = strtok(NULL, ":");
  res_buf[2] = (uint8_t)strtoul(InpString, 0, 16);
  InpString = strtok(NULL, ":");
  res_buf[3] = (uint8_t)strtoul(InpString, 0, 16);
  InpString = strtok(NULL, ":");
  res_buf[4] = (uint8_t)strtoul(InpString, 0, 16);
  InpString = strtok(NULL, ":");
  res_buf[5] = (uint8_t)strtoul(InpString, 0, 16);
}

extern volatile bool AP_MODE_END_FLAG;
extern volatile bool APIGET_TASK_END_FLAG;
extern volatile bool UPDATETIME_TASK_END_FLAG;
extern volatile bool POST_TASK_END_FLAG;
/******************************************************************************
  parse tcp and uart command
******************************************************************************/
int ParseTcpUartCmd(char *pcCmdBuffer)
{
  if (NULL == pcCmdBuffer) //null
  {
    return FAILURE;
  }

  cJSON *pJson = cJSON_Parse(pcCmdBuffer); //parse json data
  if (NULL == pJson)
  {
    cJSON_Delete(pJson); //delete pJson

    return FAILURE;
  }

  cJSON *pSub = cJSON_GetObjectItem(pJson, "Command"); //"Command"
  if (NULL != pSub)
  {
#ifdef DEBUG

    osi_UartPrint_Mul("\"Command\":", pSub->valuestring);

#endif

    if (!strcmp((char const *)pSub->valuestring, "SetupProduct")) //Command:SetupProduct
    {
      pSub = cJSON_GetObjectItem(pJson, "Password"); //"Password"
      if (NULL != pSub)
      {
#ifdef DEBUG

        osi_UartPrint_Mul("\"Password\":", pSub->valuestring);

#endif

        if (!strcmp((char const *)pSub->valuestring, "CloudForce"))
        {
          osi_at24c08_WriteData(PRODUCTURI_FLAG_ADDR, (uint8_t *)PRODUCT_URI, strlen(PRODUCT_URI), 1); //save product-uri flag

          pSub = cJSON_GetObjectItem(pJson, "ProductID"); //"ProductID"
          if (NULL != pSub)
          {
#ifdef DEBUG
            osi_UartPrint_Mul("\"ProductID\":", pSub->valuestring);
#endif

            osi_at24c08_WriteData(PRODUCT_ID_ADDR, (uint8_t *)pSub->valuestring, strlen(pSub->valuestring), 1); //save ProductID
          }

          pSub = cJSON_GetObjectItem(pJson, "SeriesNumber"); //"SeriesNumber"
          if (NULL != pSub)
          {
#ifdef DEBUG
            osi_UartPrint_Mul("\"SeriesNumber\":", pSub->valuestring);
#endif

            osi_at24c08_WriteData(SERISE_NUM_ADDR, (uint8_t *)pSub->valuestring, strlen(pSub->valuestring), 1); //save SeriesNumber
          }

          pSub = cJSON_GetObjectItem(pJson, "Host"); //"Host"
          if (NULL != pSub)
          {
#ifdef DEBUG
            osi_UartPrint_Mul("\"Host\":", pSub->valuestring);
#endif

            osi_at24c08_WriteData(HOST_ADDR, (uint8_t *)pSub->valuestring, strlen(pSub->valuestring), 1); //save host in at24c08
          }

          cJSON_Delete(pJson); //delete pJson

          return SUCCESS;
        }
      }
    }
    else if (!strcmp((char const *)pSub->valuestring, "SetupHost")) //Command:SetupHost
    {
      pSub = cJSON_GetObjectItem(pJson, "Host"); //"Host"
      if (NULL != pSub)
      {
#ifdef DEBUG

        osi_UartPrint_Mul("\"Host\":", pSub->valuestring);

#endif

        osi_at24c08_WriteData(HOST_ADDR, (uint8_t *)pSub->valuestring, strlen(pSub->valuestring), 1); //save host
      }
      pSub = cJSON_GetObjectItem(pJson, "backup_ip"); //"backup_ip"
      if (NULL != pSub)
      {
#ifdef DEBUG
        osi_UartPrint_Mul("\"backup_ip\":", pSub->valuestring);
#endif
        Parse_IP_MASK_GW_DNS(pSub->valuestring, HOST_IP);
        osi_at24c08_WriteData(HOST_IP_ADDR, HOST_IP, sizeof(HOST_IP), 1); //Save Host IP
      }
      pSub = cJSON_GetObjectItem(pJson, "Port"); //"Port"
      if (NULL != pSub)
      {
#ifdef DEBUG
        osi_UartPrint_Val("\"Port\":", pSub->valueint);
#endif

        osi_at24c08_write(HOST_PORT_ADDR, pSub->valueint); //Port
      }

      cJSON_Delete(pJson); //delete pJson

      return SUCCESS;
    }
    //     else if (!strcmp((char const *)pSub->valuestring, "SetMac")) //Command:SetMac
    //     {
    //       pSub = cJSON_GetObjectItem(pJson, "mac"); //"mac"
    //       if (NULL != pSub)
    //       {
    // #ifdef DEBUG
    //         osi_UartPrint_Mul("\"mac\":", pSub->valuestring);
    // #endif

    //         uint8_t mac_addr[6] = {0};
    //         Parse_Mac_addr(pSub->valuestring, mac_addr);

    //         if (AP_MODE_END_FLAG)
    //         {
    //           sl_NetCfgSet(SL_MAC_ADDRESS_SET, 1, SL_MAC_ADDR_LEN, mac_addr);
    //         }
    //         else if ((!APIGET_TASK_END_FLAG) && (!UPDATETIME_TASK_END_FLAG) && (!POST_TASK_END_FLAG))
    //         {
    //           xSemaphoreTake(xMutex2, -1); //SimpleLink Semaphore Take
    //           sl_Start(0, 0, 0);           //start the simple link
    //           sl_NetCfgSet(SL_MAC_ADDRESS_SET, 1, SL_MAC_ADDR_LEN, mac_addr);
    //           sl_Stop(SL_STOP_TIMEOUT); //stop the simple link
    //           MAP_UtilsDelay(80000);    //delay about 6ms
    //           xSemaphoreGive(xMutex2);  //SimpleLink Semaphore Give
    //         }
    //         else
    //         {
    //           cJSON_Delete(pJson); //delete pJson
    //           return FAILURE;
    //         }
    //         osi_at24c08_WriteData(MAC_ADDR, (uint8_t *)mac_addr, SL_MAC_ADDR_LEN, 0); //save type
    //       }

    //       cJSON_Delete(pJson); //delete pJson
    //       return SUCCESS;
    //     }
    else if (!strcmp((char const *)pSub->valuestring, "SetupWifi")) //Command:SetupWifi
    {
      pSub = cJSON_GetObjectItem(pJson, "SSID"); //"SSID"
      if (NULL != pSub)
      {
#ifdef DEBUG

        osi_UartPrint_Mul("\"SSID\":", pSub->valuestring);

#endif

        if (sizeof(SSID_NAME) >= strlen(pSub->valuestring))
        {
          osi_at24c08_WriteData(SSID_FLAG_ADDR, (uint8_t *)"SSID", strlen("SSID"), 1); //save ssid flag to at24c08

          osi_at24c08_write_byte(SSID_LEN_ADDR, strlen(pSub->valuestring));

          osi_at24c08_WriteData(SSID_ADDR, (uint8_t *)pSub->valuestring, strlen(pSub->valuestring), 0); //save ssid
        }
      }

      pSub = cJSON_GetObjectItem(pJson, "password"); //"password"
      if (NULL != pSub)
      {
#ifdef DEBUG

        osi_UartPrint_Mul("\"password\":", pSub->valuestring);

#endif

        if (sizeof(PASS_WORD) >= strlen(pSub->valuestring))
        {
          osi_at24c08_write_byte(PASSWORD_LEN_ADDR, strlen(pSub->valuestring));

          osi_at24c08_WriteData(PASSWORD_ADDR, (uint8_t *)pSub->valuestring, strlen(pSub->valuestring), 0); //save password
        }
      }

      pSub = cJSON_GetObjectItem(pJson, "type"); //"type"
      if (NULL != pSub)
      {
#ifdef DEBUG

        osi_UartPrint_Mul("\"type\":", pSub->valuestring);

#endif

        osi_at24c08_WriteData(SECTYPE_ADDR, (uint8_t *)pSub->valuestring, strlen(pSub->valuestring), 1); //save type
      }

      pSub = cJSON_GetObjectItem(pJson, "backup_ip"); //"backup_ip"
      if (NULL != pSub)
      {
#ifdef DEBUG
        osi_UartPrint_Mul("\"backup_ip\":", pSub->valuestring);
#endif

        //        char *InpString;
        //        InpString = strtok(pSub->valuestring, ".");
        //        HOST_IP[0]=(uint8_t)strtoul(InpString, 0, 10);
        //        InpString = strtok(NULL, ".");
        //        HOST_IP[1]=(uint8_t)strtoul(InpString, 0, 10);
        //        InpString = strtok(NULL, ".");
        //        HOST_IP[2]=(uint8_t)strtoul(InpString, 0, 10);
        //        InpString = strtok(NULL, ".");
        //        HOST_IP[3]=(uint8_t)strtoul(InpString, 0, 10);

        Parse_IP_MASK_GW_DNS(pSub->valuestring, HOST_IP);
        osi_at24c08_WriteData(HOST_IP_ADDR, HOST_IP, sizeof(HOST_IP), 1); //Save Host IP
      }

      uint8_t set_buf[4];
      //      char *InpString;
      pSub = cJSON_GetObjectItem(pJson, "dhcp"); //"dhcp"
      if (NULL != pSub)
      {
        osi_at24c08_write_byte(DHCP_MODE_ADDR, ((uint8_t)pSub->valueint));
      }
      pSub = cJSON_GetObjectItem(pJson, "ip"); //"ip"
      if (NULL != pSub)
      {
        //        InpString = strtok(pSub->valuestring, ".");
        //        set_buf[0]=(uint8_t)strtoul(InpString, 0, 10);
        //        InpString = strtok(NULL, ".");
        //        set_buf[1]=(uint8_t)strtoul(InpString, 0, 10);
        //        InpString = strtok(NULL, ".");
        //        set_buf[2]=(uint8_t)strtoul(InpString, 0, 10);
        //        InpString = strtok(NULL, ".");
        //        set_buf[3]=(uint8_t)strtoul(InpString, 0, 10);

        memset(set_buf, 0, sizeof(set_buf));
        Parse_IP_MASK_GW_DNS(pSub->valuestring, set_buf);
        osi_at24c08_WriteData(STATIC_IP_ADDR, set_buf, sizeof(set_buf), 1); //Save STATIC IP
      }

      pSub = cJSON_GetObjectItem(pJson, "mask"); //"mask"
      if (NULL != pSub)
      {
        //        InpString = strtok(pSub->valuestring, ".");
        //        set_buf[0]=(uint8_t)strtoul(InpString, 0, 10);
        //        InpString = strtok(NULL, ".");
        //        set_buf[1]=(uint8_t)strtoul(InpString, 0, 10);
        //        InpString = strtok(NULL, ".");
        //        set_buf[2]=(uint8_t)strtoul(InpString, 0, 10);
        //        InpString = strtok(NULL, ".");
        //        set_buf[3]=(uint8_t)strtoul(InpString, 0, 10);

        memset(set_buf, 0, sizeof(set_buf));
        Parse_IP_MASK_GW_DNS(pSub->valuestring, set_buf);
        osi_at24c08_WriteData(STATIC_SN_ADDR, set_buf, sizeof(set_buf), 1); //Save STATIC SN
      }

      pSub = cJSON_GetObjectItem(pJson, "gw"); //"gw"
      if (NULL != pSub)
      {
        //        InpString = strtok(pSub->valuestring, ".");
        //        set_buf[0]=(uint8_t)strtoul(InpString, 0, 10);
        //        InpString = strtok(NULL, ".");
        //        set_buf[1]=(uint8_t)strtoul(InpString, 0, 10);
        //        InpString = strtok(NULL, ".");
        //        set_buf[2]=(uint8_t)strtoul(InpString, 0, 10);
        //        InpString = strtok(NULL, ".");
        //        set_buf[3]=(uint8_t)strtoul(InpString, 0, 10);

        memset(set_buf, 0, sizeof(set_buf));
        Parse_IP_MASK_GW_DNS(pSub->valuestring, set_buf);
        osi_at24c08_WriteData(STATIC_GW_ADDR, set_buf, sizeof(set_buf), 1); //Save STATIC GW
      }

      pSub = cJSON_GetObjectItem(pJson, "dns1"); //"dns1"
      if (NULL != pSub)
      {
        //        InpString = strtok(pSub->valuestring, ".");
        //        set_buf[0]=(uint8_t)strtoul(InpString, 0, 10);
        //        InpString = strtok(NULL, ".");
        //        set_buf[1]=(uint8_t)strtoul(InpString, 0, 10);
        //        InpString = strtok(NULL, ".");
        //        set_buf[2]=(uint8_t)strtoul(InpString, 0, 10);
        //        InpString = strtok(NULL, ".");
        //        set_buf[3]=(uint8_t)strtoul(InpString, 0, 10);

        memset(set_buf, 0, sizeof(set_buf));
        Parse_IP_MASK_GW_DNS(pSub->valuestring, set_buf);
        osi_at24c08_WriteData(STATIC_DNS_ADDR, set_buf, sizeof(set_buf), 1); //Save STATIC DNS
      }
      /*      
      pSub = cJSON_GetObjectItem(pJson, "dns2");  //"dns2"
      if(NULL!=pSub)
      {
//        InpString = strtok(pSub->valuestring, ".");
//        set_buf[0]=(uint8_t)strtoul(InpString, 0, 10);
//        InpString = strtok(NULL, ".");
//        set_buf[1]=(uint8_t)strtoul(InpString, 0, 10);
//        InpString = strtok(NULL, ".");
//        set_buf[2]=(uint8_t)strtoul(InpString, 0, 10);
//        InpString = strtok(NULL, ".");
//        set_buf[3]=(uint8_t)strtoul(InpString, 0, 10);
        
        memset(set_buf,0,sizeof(set_buf));
        Parse_IP_MASK_GW_DNS(pSub->valuestring,set_buf);
        osi_at24c08_WriteData(STATIC_SDNS_ADDR,set_buf,sizeof(set_buf),1);  //Save STATIC SDNS
      }
*/
      cJSON_Delete(pJson); //delete pJson

      return 1;
    }
    else if (!strcmp((char const *)pSub->valuestring, "ReadProduct")) //Command:ReadProduct
    {

      cJSON_Delete(pJson); //delete pJson

      return 2;
    }
    else if (!strcmp((char const *)pSub->valuestring, "ReadWifi")) //Command:ReadWifi
    {

      cJSON_Delete(pJson); //delete pJson

      return 3;
    }
    else if (!strcmp((char const *)pSub->valuestring, "GetLastError")) //Command:GetLastError
    {
      cJSON_Delete(pJson); //delete pJson

      return 4;
    }
    else if (!strcmp((char const *)pSub->valuestring, "BreakOut")) //Command:BreakOut
    {
      cJSON_Delete(pJson); //delete pJson

      return 5;
    }
    else if (!strcmp((char const *)pSub->valuestring, "SetMetaData")) //Command:SetMetaData
    {
      pSub = cJSON_GetObjectItem(pJson, "metadata"); //"metadata"
      if (NULL != pSub)
      {
#ifdef DEBUG

        osi_UartPrint_Mul("\"metadata\":", pSub->valuestring);

#endif

        Parse_metadata(pSub->valuestring);
      }

      cJSON_Delete(pJson); //delete pJson

      return SUCCESS;
    }
    else if (!strcmp((char const *)pSub->valuestring, "ReadMetaData")) //Command:ReadMetaData
    {
      cJSON_Delete(pJson); //delete pJson

      return 6;
    }
    else if (!strcmp((char const *)pSub->valuestring, "SetCali")) //Command:SetCali
    {
      pSub = cJSON_GetObjectItem(pJson, "cali"); //"cali"
      if (NULL != pSub)
      {
#ifdef DEBUG
        osi_UartPrint("\"cali\":");
        osi_UartPrint(pSub->valuestring);
        osi_UartPrint("\r\n");
#endif

        Parse_cali(pSub->valuestring);
      }

      cJSON_Delete(pJson); //delete pJson

      return SUCCESS;
    }
    else if (!strcmp((char const *)pSub->valuestring, "ReadCali")) //Command:ReadCali
    {
      cJSON_Delete(pJson); //delete pJson

      return 12;
    }
    else if (!strcmp((char const *)pSub->valuestring, "ScanWifiList")) //Command:ScanWifiList
    {
      cJSON_Delete(pJson); //delete pJson

      return 7;
    }
    else if (!strcmp((char const *)pSub->valuestring, "CheckSensors")) //Command:CheckSensors
    {
      cJSON_Delete(pJson); //delete pJson

      return 8;
    }
    else if (!strcmp((char const *)pSub->valuestring, "ReadData")) //Command:ReadData
    {

      cJSON_Delete(pJson); //delete pJson

      return 9;
    }
    else if (!strcmp((char const *)pSub->valuestring, "ClearData")) //Command:ClearData
    {
      cJSON_Delete(pJson); //delete pJson

      return 10;
    }
    else if (!strcmp((char const *)pSub->valuestring, "DeviceActivate")) //DeviceActivate
    {
      pSub = cJSON_GetObjectItem(pJson, "ActivateData"); //"ActivateData"
      if (NULL != pSub)
      {
#ifdef DEBUG

        osi_UartPrint_Mul("\"ActivateData\":", pSub->valuestring);

#endif

        ParseSetJSONData(pSub->valuestring);
      }

      cJSON_Delete(pJson); //delete pJson

      return SUCCESS;
    }
  }

  cJSON_Delete(pJson); //delete pJson

  return FAILURE;
}

/*******************************************************************************
// read product url
*******************************************************************************/
void read_product_url(char *url_buf)
{
  uint8_t i = 0;

  char product_id[32] = {0};
  char series_number[16] = {0};

  osi_at24c08_ReadData(PRODUCT_ID_ADDR, (uint8_t *)product_id, sizeof(product_id), 1);

  osi_at24c08_ReadData(SERISE_NUM_ADDR, (uint8_t *)series_number, sizeof(series_number), 1);

  memcpy(url_buf, ProductURI1, strlen(ProductURI1));

  i += strlen(ProductURI1); //ProductURI1

  memcpy(url_buf + i, product_id, strlen(product_id));

  i += strlen(product_id); //ProductID

  memcpy(url_buf + i, ProductURI2, strlen(ProductURI2));

  i += strlen(ProductURI2); //ProductURI2

  memcpy(url_buf + i, series_number, strlen(series_number));

  i += strlen(series_number); //SeriesNumber

  memcpy(url_buf + i, ProductURI3, strlen(ProductURI3));

  i += strlen(ProductURI3); //"/activate"

  url_buf[i] = '\0';

#ifdef DEBUG
  osi_UartPrint("ProductURI:");
  osi_UartPrint(url_buf);
  osi_UartPrint("\r\n");
#endif
}

/*******************************************************************************
  Save ERROR cjson data
*******************************************************************************/
void System_ERROR_Save(uint16_t save_addr)
{
  now_unix_t = Read_UnixTime(); //read unix time

  at24c08_write(save_addr, now_unix_t);
}

/*******************************************************************************
  Save ERROR cjson data with locked
*******************************************************************************/
void osi_System_ERROR_Save(uint16_t save_addr)
{
  osi_Read_UnixTime(); //read unix time

  osi_at24c08_write(save_addr, now_unix_t);
}

/*******************************************************************************
  Read System Error Code data
*******************************************************************************/
void Read_System_ERROR_Code(char *read_buf, uint16_t data_len)
{
  bool err_flag = 0;
  char *out_buf;
  cJSON *pJsonRoot;
  unsigned long err_time;

  xSemaphoreTake(xMutex3, -1); //cJSON Semaphore Take

  pJsonRoot = cJSON_CreateObject();

  //  err_time=osi_at24c08_read(MALLOC_FLR_ERR);
  //  if(err_time)
  //  {
  //    err_flag=1;
  //
  //    cJSON_AddNumberToObject(pJsonRoot,"mlc_flr",err_time);  //"mlc_flr"
  //  }
  //
  //  err_time=osi_at24c08_read(STATCK_OVER_FLOW);
  //  if(err_time)
  //  {
  //    err_flag=1;
  //
  //    cJSON_AddNumberToObject(pJsonRoot,"stc_ov_fw",err_time);  //"stc_ov_fw"
  //  }

  err_time = osi_at24c08_read(SCN_WF_FLR_ERR);
  if (err_time)
  {
    err_flag = 1;

    cJSON_AddNumberToObject(pJsonRoot, "scn_wf_flr", err_time); //"scn_wf_flr"
  }

  err_time = osi_at24c08_read(WF_PWD_WR_ERR);
  if (err_time)
  {
    err_flag = 1;

    cJSON_AddNumberToObject(pJsonRoot, "wf_pwd_wr", err_time); //"wf_pwd_wr"
  }

  err_time = osi_at24c08_read(WD_RESET_ERR);
  if (err_time)
  {
    err_flag = 1;

    cJSON_AddNumberToObject(pJsonRoot, "wd_rst", err_time); //"wd_rst"
  }

  err_time = osi_at24c08_read(CNT_WIFI_FLR_ERR);
  if (err_time)
  {
    err_flag = 1;

    cJSON_AddNumberToObject(pJsonRoot, "c_wf_flr", err_time); //"c_wf_flr"
  }

  err_time = osi_at24c08_read(RSL_HOST_FLR_ERR);
  if (err_time)
  {
    err_flag = 1;

    cJSON_AddNumberToObject(pJsonRoot, "rsl_h_flr", err_time); //"rsl_h_flr"
  }

  err_time = osi_at24c08_read(CNT_SERVER_FLR_ERR);
  if (err_time)
  {
    err_flag = 1;

    cJSON_AddNumberToObject(pJsonRoot, "c_srv_flr", err_time); //"c_srv_flr"
  }

  err_time = osi_at24c08_read(API_GET_FLR_ERR);
  if (err_time)
  {
    err_flag = 1;

    cJSON_AddNumberToObject(pJsonRoot, "api_gt_flr", err_time); //"api_gt_flr"
  }

  err_time = osi_at24c08_read(UPDATE_TIME_FLR_ERR);
  if (err_time)
  {
    err_flag = 1;

    cJSON_AddNumberToObject(pJsonRoot, "udt_tm_flr", err_time); //"udt_tm_flr"
  }

  err_time = osi_at24c08_read(MEMORY_SAVE_DATA_ERR);
  if (err_time)
  {
    err_flag = 1;

    cJSON_AddNumberToObject(pJsonRoot, "mry_dt_err", err_time); //"mry_dt_err"
  }

  err_time = osi_at24c08_read(POST_DATA_JSON_FAULT);
  if (err_time)
  {
    err_flag = 1;

    cJSON_AddNumberToObject(pJsonRoot, "pt_dt_js_err", err_time); //"pt_dt_js_err"
  }

  err_time = osi_at24c08_read(POST_DATA_FLR_ERR);
  if (err_time)
  {
    err_flag = 1;

    cJSON_AddNumberToObject(pJsonRoot, "pt_dt_flr", err_time); //"pt_dt_flr"
  }

  if (err_flag == 0)
  {
    cJSON_AddStringToObject(pJsonRoot, "Result", "NoneErrorCode"); //no error
  }

  out_buf = cJSON_PrintUnformatted(pJsonRoot); //cJSON_Print(Root)

  cJSON_Delete(pJsonRoot); //delete cjson root

  memcpy(read_buf, out_buf, data_len);

  cJSON_free(out_buf);

  xSemaphoreGive(xMutex3); //cJSON Semaphore Give
}

/*******************************************************************************
// make wifi data json
*******************************************************************************/
void Web_Wifi_Set(char *read_buf, uint8_t data_len, char *WiFi_ssid, short WiFi_rssi, uint8_t WiFi_type, char *WiFi_bssid, bool bssid_flag)
{
  char *out_buf;
  cJSON *pJsonRoot;
  char mac_buf[18] = {0};

  xSemaphoreTake(xMutex3, -1); //cJSON Semaphore Take

  pJsonRoot = cJSON_CreateObject();

  cJSON_AddStringToObject(pJsonRoot, "SSID", WiFi_ssid);

  cJSON_AddNumberToObject(pJsonRoot, "rssi", WiFi_rssi);

  cJSON_AddNumberToObject(pJsonRoot, "type", WiFi_type);

  if (bssid_flag)
  {
    snprintf(mac_buf, sizeof(mac_buf), "%02x:%02x:%02x:%02x:%02x:%02x", WiFi_bssid[0], WiFi_bssid[1], WiFi_bssid[2], WiFi_bssid[3], WiFi_bssid[4], WiFi_bssid[5]);

    cJSON_AddStringToObject(pJsonRoot, "BSSID", mac_buf);
  }

  out_buf = cJSON_PrintUnformatted(pJsonRoot); //cJSON_Print(Root)

  cJSON_Delete(pJsonRoot); //delete cjson root

  memcpy(read_buf, out_buf, data_len);

  cJSON_free(out_buf);

  xSemaphoreGive(xMutex3); //cJSON Semaphore Give
}

/*******************************************************************************
// Read cali set
*******************************************************************************/
void Read_cali(char *read_buf, uint16_t data_len)
{
  char *out_buf;
  cJSON *pJsonRoot;
  xSemaphoreTake(xMutex3, -1); //cJSON Semaphore Take
  pJsonRoot = cJSON_CreateObject();

  cJSON_AddNumberToObject(pJsonRoot, "f1_a", f1_a); //f1_a
  cJSON_AddNumberToObject(pJsonRoot, "f1_b", f1_b); //f1_b

  cJSON_AddNumberToObject(pJsonRoot, "f2_a", f2_a); //f2_a
  cJSON_AddNumberToObject(pJsonRoot, "f2_b", f2_b); //f2_b

  cJSON_AddNumberToObject(pJsonRoot, "f3_a", f3_a); //f3_a
  cJSON_AddNumberToObject(pJsonRoot, "f3_b", f3_b); //f3_b

  cJSON_AddNumberToObject(pJsonRoot, "f4_a", f4_a); //f4_a
  cJSON_AddNumberToObject(pJsonRoot, "f4_b", f4_b); //f4_b

  cJSON_AddNumberToObject(pJsonRoot, "f5_a", f5_a); //f5_a
  cJSON_AddNumberToObject(pJsonRoot, "f5_b", f5_b); //f5_b

  cJSON_AddNumberToObject(pJsonRoot, "f6_a", f6_a); //f6_a
  cJSON_AddNumberToObject(pJsonRoot, "f6_b", f6_b); //f6_b

  cJSON_AddNumberToObject(pJsonRoot, "f7_a", f7_a); //f7_a
  cJSON_AddNumberToObject(pJsonRoot, "f7_b", f7_b); //f7_b

  cJSON_AddNumberToObject(pJsonRoot, "f8_a", f8_a); //f8_a
  cJSON_AddNumberToObject(pJsonRoot, "f8_b", f8_b); //f8_b

  cJSON_AddNumberToObject(pJsonRoot, "f9_a", f9_a); //f9_a
  cJSON_AddNumberToObject(pJsonRoot, "f9_b", f9_b); //f9_b

  cJSON_AddNumberToObject(pJsonRoot, "f10_a", f10_a); //f10_a
  cJSON_AddNumberToObject(pJsonRoot, "f10_b", f10_b); //f10_b

  out_buf = cJSON_PrintUnformatted(pJsonRoot); //cJSON_Print(Root)
  cJSON_Delete(pJsonRoot);                     //delete cjson root
  memcpy(read_buf, out_buf, data_len);
  cJSON_free(out_buf);
  xSemaphoreGive(xMutex3); //cJSON Semaphore Give
}

/*******************************************************************************
                                      END         
*******************************************************************************/
