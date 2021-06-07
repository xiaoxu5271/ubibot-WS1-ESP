/*******************************************************************************
  * @file       Peripheral Driver Application Task
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
// #include "stdbool.h"
// #include "stdlib.h"
// #include "osi.h"
// #include "string.h"
// #include "rom_map.h"
// #include "utils.h"
// #include "uart_if.h"
// #include "common.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "iic.h"
#include "at24c08.h"
#include "sht30dis.h"
#include "opt3001.h"
#include "ds18b20.h"
#include "PCF8563.h"
#include "w25q128.h"
#include "adxl345.h"
#include "MsgType.h"
#include "MagTask.h"
#include "PeripheralDriver.h"
// #include "ht1621.h"

#define TAG "PeripheralDriver"

extern QueueHandle_t xQueue1;     //Used for LED control task
extern QueueHandle_t xQueue2;     //Used for bell conctrol task
extern SemaphoreHandle_t xMutex1; //Used for SPI Lock
extern SemaphoreHandle_t xMutex4; //Used for UART Lock
extern TaskHandle_t xBinary17;    //Internet Application

extern volatile bool POST_TASK_END_FLAG;
extern volatile bool UPDATETIME_TASK_END_FLAG;
extern volatile bool APIGET_TASK_END_FLAG;
extern volatile uint16_t sys_run_time;
extern unsigned long now_unix_t;    //now unix time
extern volatile bool r_led_flashed; //red led flashed

#ifdef MAG_SENSOR
extern volatile uint8_t door_status;
extern volatile uint8_t fn_mag_int;             //0:no data save,1:save data,2:save data and post
extern volatile unsigned long fn_mag, fn_mag_t; //Magnetic sensor frequence
#endif

extern volatile uint8_t save_addr_flag;
extern short flash_set_val;

extern volatile uint8_t fn_acc_tap1;   //0:closed single tap,1:single tap interrupt,2:single tap interrupt and post
extern volatile uint8_t fn_acc_tap2;   //0:closed double tap,1:double tap interrupt,2:double tap interrupt and post
extern volatile uint8_t fn_acc_act;    //0:cloused act interrupt,1:act interrupt
extern volatile uint8_t thres_acc_min; //min value for acceleration sensor act interrupt
extern volatile uint8_t cg_data_led;   //Led ON or OFF when data post
extern volatile uint8_t no_net_fn;     //change fn_dp when no net
extern volatile uint8_t wifi_mode;     //1:connect wifi immediately,0:connect wifi when scaned

extern volatile unsigned long fn_th, fn_th_t;           //Temp&Humi sensor frequence
extern volatile unsigned long fn_light, fn_light_t;     //Light sensor frequence
extern volatile unsigned long fn_bt, fn_bt_t;           //Body Temperature Measure frequence
extern volatile unsigned long fn_ext, fn_ext_t;         //Noise Measure frequence
extern volatile unsigned long fn_battery, fn_battery_t; //PowerMeasure frequence
extern volatile unsigned long fn_dp, fn_dp_t;           //data post frequence

extern float f1_a, f1_b, f2_a, f2_b, f3_a, f3_b, f4_a, f4_b, f5_a, f5_b, f6_a, f6_b, f7_a, f7_b, f8_a, f8_b, f9_a, f9_b, f10_a, f10_b;

extern volatile unsigned long POST_NUM;
extern volatile unsigned long DELETE_ADDR, POST_ADDR, WRITE_ADDR;

//新增ESP用任务延时
void osi_Sleep(uint16_t ms)
{
  vTaskDelay(ms / portTICK_RATE_MS);
}
/*******************************************************************************
//Green led flashed on_time*150ms
*******************************************************************************/
void Green_Led_Flashed(uint8_t n_time, uint8_t on_time)
{
  uint8_t i;

  for (i = 0; i < n_time; i++)
  {
    SET_GREEN_LED_ON();

    // MAP_UtilsDelay(on_time * 2000000); //n*150ms
    vTaskDelay(on_time * 150 / portTICK_RATE_MS);

    SET_GREEN_LED_OFF();

    // MAP_UtilsDelay(on_time * 2000000); //n*150ms
    vTaskDelay(on_time * 150 / portTICK_RATE_MS);
  }
}

/*******************************************************************************
//Red led flashed on_time*150ms
*******************************************************************************/
void Red_Led_Flashed(uint8_t n_time, uint8_t on_time)
{
  uint8_t i;

  for (i = 0; i < n_time; i++)
  {
    SET_RED_LED_ON();

    // MAP_UtilsDelay(on_time * 2000000); //n*150ms
    vTaskDelay(on_time * 150 / portTICK_RATE_MS);

    SET_RED_LED_OFF();

    // MAP_UtilsDelay(on_time * 2000000); //n*150ms
    vTaskDelay(on_time * 150 / portTICK_RATE_MS);
  }
}

/*******************************************************************************
  Red led on time on_time*150ms
*******************************************************************************/
void Red_Led_On_time(uint8_t on_time)
{
  SET_RED_LED_ON();

  vTaskDelay(on_time * 150 / portTICK_RATE_MS);

  SET_RED_LED_OFF();
}

/*******************************************************************************
//Green and Red led flashed on_time*150ms
*******************************************************************************/
void Green_Red_Led_Flashed(uint8_t n_time, uint8_t on_time)
{
  uint8_t i;

  for (i = 0; i < n_time; i++)
  {
    SET_GREEN_LED_ON();

    SET_RED_LED_OFF();

    vTaskDelay(on_time * 150 / portTICK_RATE_MS);

    SET_RED_LED_ON();

    SET_GREEN_LED_OFF();

    vTaskDelay(on_time * 150 / portTICK_RATE_MS);
  }
}

/*******************************************************************************
//Green and Red led flashed at the same time on_time*150ms
*******************************************************************************/
static void st_Green_Red_Led_Flashed(uint8_t n_time, uint8_t on_time)
{
  uint8_t i;

  for (i = 0; i < n_time; i++)
  {
    SET_GREEN_LED_ON(); //Set Green Led On

    SET_RED_LED_ON(); //Set Red Led On

    vTaskDelay(on_time * 150 / portTICK_RATE_MS);

    SET_GREEN_LED_OFF(); //Set Green Led Off

    SET_RED_LED_OFF(); //Set Red Led Off

    vTaskDelay(on_time * 150 / portTICK_RATE_MS);
  }
}

/*******************************************************************************
//GREEN and RED LED Flashed at the same time
*******************************************************************************/
void st_Green_Red_LedFlashed_Task(void *pvParameters)
{
  for (;;)
  {
    st_Green_Red_Led_Flashed(1, 10);
  }
}

/*******************************************************************************
//osi Green and Red led flashed on_time*150ms
*******************************************************************************/
void osi_Green_Red_Led_Flashed(uint8_t n_time, uint8_t on_time)
{
  uint8_t i;

  for (i = 0; i < n_time; i++)
  {
    SET_GREEN_LED_ON();

    SET_RED_LED_OFF();

    osi_Sleep(on_time * 150); //n*150ms

    SET_RED_LED_ON();

    SET_GREEN_LED_OFF();

    osi_Sleep(on_time * 150); //n*150ms
  }
}

/*******************************************************************************
//GREEN and RED LED Flashed when AP Task 
*******************************************************************************/
void Green_Red_LedFlashed_Task(void *pvParameters)
{
  for (;;)
  {
    osi_Green_Red_Led_Flashed(1, 6);

    //    sys_run_time = 0;  //clear system time out
  }
}

/*******************************************************************************
//GREEN and RED LED Fast Flashed when AP Task WIFI Connect
*******************************************************************************/
void Green_Red_Led_FastFlashed_Task(void *pvParameters)
{
  for (;;)
  {
    osi_Green_Red_Led_Flashed(1, 2);

    //    sys_run_time = 0;  //clear system time out
  }
}

/*******************************************************************************
//GREEN LED Flashed when Post Task 
*******************************************************************************/
void Green_LedFlashed_Task(void *pvParameters)
{
  for (;;)
  {
    // osi_SyncObjWait(&xBinary17, -1); //Wait Internet Application Task Start Message
    ulTaskNotifyTake(pdTRUE, -1);

    while (POST_TASK_END_FLAG || UPDATETIME_TASK_END_FLAG || APIGET_TASK_END_FLAG)
    {
      if (cg_data_led)
      {
        SET_GREEN_LED_ON();

        osi_Sleep(500); //delay 500ms

        SET_GREEN_LED_OFF();

        osi_Sleep(500); //delay 500ms
      }
      else
      {
        osi_Sleep(1000); //delay 1s
      }
    }
  }
}

/*******************************************************************************
//LED control task
*******************************************************************************/
void Green_LedControl_Task(void *pvParameters)
{
  uint8_t led_t;

  for (;;)
  {
    // osi_MsgQRead(&xQueue1, &led_t, -1); //Wait LED Value Message
    xQueueReceive(xQueue1, &led_t, portMAX_DELAY);

    //vTaskSuspendAll(); //disable the tasks

    SET_GREEN_LED_ON(); //LED ON

    // MAP_UtilsDelay(led_t * 13300000); //delay about led_t s
    vTaskDelay(led_t * 1000 / portTICK_RATE_MS);

    SET_GREEN_LED_OFF(); //LED OFF

    //xTaskResumeAll(); //enable all tasks
  }
}

/*******************************************************************************
//bell make sound
*******************************************************************************/
void bell_makeSound(uint32_t n_bel)
{
  uint16_t bel;

  for (bel = 0; bel < n_bel; bel++)
  {
    SET_BELL_ON();

    MAP_UtilsDelay(2500);

    SET_BELL_OFF();

    MAP_UtilsDelay(2500);
  }
}

/*******************************************************************************
//Green Led ON and Bell Sound
*******************************************************************************/
void Green_Led_Bell_Sound(uint16_t n_bel)
{
  SET_GREEN_LED_ON(); //Green LED ON

  bell_makeSound(n_bel);

  SET_GREEN_LED_OFF(); //LED OFF
}

/*******************************************************************************
//bell make sound with locked
*******************************************************************************/
void osi_bell_makeSound(uint16_t n_bel)
{
  //vTaskSuspendAll(); //disable the tasks

  bell_makeSound(n_bel); //Bell make a sound,n_bel times

  //xTaskResumeAll(); //enable all tasks
}

/*******************************************************************************
//bell control task
*******************************************************************************/
void Bell_Control_Task(void *pvParameters)
{
  uint16_t bell_t;

  for (;;)
  {
    // osi_MsgQRead(&xQueue2, &bell_t, -1); //Wait BELL Value Message
    xQueueReceive(xQueue2, &bell_t, portMAX_DELAY);

    osi_bell_makeSound(bell_t); //Bell make a sound,bell_t times
  }
}

/*******************************************************************************
//Uart Print String
*******************************************************************************/
void osi_UartPrint(char *buffer)
{
  xSemaphoreTake(xMutex4, -1); //UART Semaphore Take

  printf("%s", buffer);

  xSemaphoreGive(xMutex4); //UART Semaphore Give
}

/*******************************************************************************
//Uart Print Two String//
*******************************************************************************/
void osi_UartPrint_Mul(char *buffer1, char *buffer2)
{
  xSemaphoreTake(xMutex4, -1); //UART Semaphore Take

  printf("%s", buffer1);

  printf("%s", buffer2);

  xSemaphoreGive(xMutex4); //UART Semaphore Give
}

/*******************************************************************************
//Uart Print Value
*******************************************************************************/
void osi_UartPrint_Val(char *buffer, unsigned long value)
{
  xSemaphoreTake(xMutex4, -1); //UART Semaphore Take

  printf("%s", buffer);

  printf("%ld", value);

  xSemaphoreGive(xMutex4); //UART Semaphore Give
}

/*******************************************************************************
//IIC read register with locked
*******************************************************************************/
uint8_t osi_IIC_ReadReg(uint8_t sla_addr, uint8_t reg_addr)
{
  uint8_t sec_n;

  //vTaskSuspendAll(); //disable the tasks

  MulTry_IIC_RD_Reg(sla_addr, reg_addr, &sec_n); //read time sec

  //xTaskResumeAll(); //enable all tasks

  return sec_n;
}

/*******************************************************************************
//write a byte in at24c08 with locked
*******************************************************************************/
void osi_at24c08_write_byte(uint16_t reg_addr, uint8_t num)
{
  //vTaskSuspendAll(); //disable the tasks

  at24c08_write_byte(reg_addr, num);

  //xTaskResumeAll(); //enable all tasks
}

/*******************************************************************************
//read a byte in at24c08 with locked//
*******************************************************************************/
uint8_t osi_at24c08_read_byte(uint16_t reg_addr)
{
  uint8_t read_val;

  //vTaskSuspendAll(); //disable the tasks

  read_val = at24c08_read_byte(reg_addr);

  //xTaskResumeAll(); //enable all tasks

  return read_val;
}

/*******************************************************************************
//write unsigned long data with locked
*******************************************************************************/
void osi_at24c08_write(uint16_t reg_addr, unsigned long num)
{
  //vTaskSuspendAll(); //disable the tasks

  at24c08_write(reg_addr, num);

  //xTaskResumeAll(); //enable all tasks
}

/*******************************************************************************
//read unsigned long data 
*******************************************************************************/
unsigned long osi_at24c08_read(uint16_t reg_addr)
{
  unsigned long read_val;

  //vTaskSuspendAll(); //disable the tasks

  read_val = at24c08_read(reg_addr);

  //xTaskResumeAll(); //enable all tasks

  return read_val;
}

/*******************************************************************************
//at24c08 write data with locked
*******************************************************************************/
void osi_at24c08_WriteData(uint16_t addr, uint8_t *buffer, uint8_t Size, bool end_flag)
{
  //vTaskSuspendAll(); //disable the tasks

  at24c08_WriteData(addr, buffer, Size, end_flag);

  //xTaskResumeAll(); //enable all tasks
}

/*******************************************************************************
//at24c08 read data with locked
*******************************************************************************/
void osi_at24c08_ReadData(uint16_t addr, uint8_t *buffer, uint8_t size, bool end_flag)
{
  //vTaskSuspendAll(); //disable the tasks

  at24c08_ReadData(addr, buffer, size, end_flag);

  //xTaskResumeAll(); //enable all tasks
}

//******************************************************************************
//update utc time with locked//
//******************************************************************************
void osi_Update_UTCtime(char *time_buff)
{
  //vTaskSuspendAll(); //disable the tasks

  Update_UTCtime(time_buff); //update time

  //xTaskResumeAll(); //enable all tasks
}

/*******************************************************************************
//Read UTC time with locked
*******************************************************************************/
void osi_Read_UTCtime(char *buffer, uint8_t buf_size)
{
  //vTaskSuspendAll(); //disable the tasks

  Read_UTCtime(buffer, buf_size);

  //xTaskResumeAll(); //enable all tasks
}

/*******************************************************************************
//read unix time with locked
*******************************************************************************/
void osi_Read_UnixTime(void)
{
  //vTaskSuspendAll(); //disable the tasks

  now_unix_t = Read_UnixTime(); //read unix time

  //xTaskResumeAll(); //enable all tasks
}

/*******************************************************************************
 Save Post Data Amount/Write Data/Post Data/Delete Data Address
*******************************************************************************/
void at24c08_save_addr(void)
{
  if (save_addr_flag == 0)
  {
    at24c08_write(DATA_AMOUNT_ADDR1, POST_NUM); //save post data amount

    at24c08_write(DATA_WRITE_ADDR1, WRITE_ADDR); //save data write address

    at24c08_write(DATA_POST_ADDR1, POST_ADDR); //save data post address

    at24c08_write(DATA_DELETE_ADDR1, DELETE_ADDR); //save delete data address
  }
  else
  {
    at24c08_write(DATA_AMOUNT_ADDR2, POST_NUM); //save post data amount

    at24c08_write(DATA_WRITE_ADDR2, WRITE_ADDR); //save data write address

    at24c08_write(DATA_POST_ADDR2, POST_ADDR); //save data post address

    at24c08_write(DATA_DELETE_ADDR2, DELETE_ADDR); //save delete data address
  }
}

/*******************************************************************************
  Save Post Data Amount/Write Data/Post Data/Delete Data Address with locked
*******************************************************************************/
void osi_at24c08_save_addr(void)
{
  //vTaskSuspendAll(); //disable the tasks

  at24c08_save_addr(); //Save Post Data Amount/Write Data/Post Data/Delete Data Address

  //xTaskResumeAll(); //enable all tasks
}

/*******************************************************************************
  Read Post Data Amount/Write Data/Post Data/Delete Data Address
*******************************************************************************/
void at24c08_read_addr(void)
{
  if (save_addr_flag == 0)
  {
    POST_NUM = at24c08_read(DATA_AMOUNT_ADDR1); //read data post amount

    WRITE_ADDR = at24c08_read(DATA_WRITE_ADDR1); //read data write address

    POST_ADDR = at24c08_read(DATA_POST_ADDR1); //read data post address

    DELETE_ADDR = at24c08_read(DATA_DELETE_ADDR1); //read data delete address
  }
  else
  {
    POST_NUM = at24c08_read(DATA_AMOUNT_ADDR2); //read data post amount

    WRITE_ADDR = at24c08_read(DATA_WRITE_ADDR2); //read data write address

    POST_ADDR = at24c08_read(DATA_POST_ADDR2); //read data post address

    DELETE_ADDR = at24c08_read(DATA_DELETE_ADDR2); //read data delete address
  }
}

/*******************************************************************************
  Read Post Data Amount/Write Data/Post Data/Delete Data Address whit locked
*******************************************************************************/
void osi_at24c08_read_addr(void)
{
  //vTaskSuspendAll(); //disable the tasks

  at24c08_read_addr();

  //xTaskResumeAll(); //enable all tasks
}

/*******************************************************************************
  Metadata init
*******************************************************************************/
void MetaData_Init(void)
{
  fn_th = DEFAULT_FN_TH; //reset temperature and humility sensor frequency

  fn_light = DEFAULT_FN_LIGHT; //reset light sensor frequence

#ifdef MAG_SENSOR
  fn_mag = DEFAULT_FN_MAG; //mag sensor frequence

  fn_mag_int = DEFAULT_FN_MAG_INT; //mag sensor int set
#endif

  fn_acc_tap1 = DEFAULT_FN_ACC_TAP1; //acce sensor tap1 set

  fn_acc_tap2 = DEFAULT_FN_ACC_TAP2; //acce sensor tap2 set

  fn_acc_act = DEFAULT_FN_ACC_ACT; //acce sensor act set

  thres_acc_min = DEFAULT_THRES_ACC_MIN; //acce sensor act min value set

  fn_bt = DEFAULT_FN_BT; //body temperature frequence

  fn_ext = DEFAULT_FN_EXT; //exit tempeture measure frequence

  fn_battery = DEFAULT_FN_BATTERY; //battery measure frequence

  fn_dp = DEFAULT_FN_DP; //data post frequence

  cg_data_led = DEFAULT_CG_DATA_LED; //led on or off when post data

  no_net_fn = DEFAULT_NO_NET_FN; //change fn_dp when no net

  wifi_mode = DEFAULT_WIFI_MODE; //connect wifi immediately
}

/*******************************************************************************
  At24c08 Save Metadata
*******************************************************************************/
void MetaData_Save(void)
{
  at24c08_write(FN_TH_ADDR, fn_th); //fn_th

  at24c08_write(FN_LIGHT_ADDR, fn_light); //fn_light

#ifdef MAG_SENSOR
  at24c08_write(FN_MAG_ADDR, fn_mag); //fn_mag

  at24c08_write_byte(FN_MAG_INT_ADDR, fn_mag_int); //fn_mag_int
#endif

  at24c08_write_byte(FN_ACC_TAP1_ADDR, fn_acc_tap1); //fn_acc_tap1

  at24c08_write_byte(FN_ACC_TAP2_ADDR, fn_acc_tap2); //fn_acc_tap2

  at24c08_write_byte(FN_ACC_ACT_ADDR, fn_acc_act); //fn_acc_act

  at24c08_write_byte(THRES_ACC_MIN_ADDR, thres_acc_min); //thres_acc_min

  at24c08_write(FN_BT_ADDR, fn_bt); //fn_bt

  at24c08_write(FN_EXT_ADDR, fn_ext); //fn_ext

  at24c08_write(FN_BATTERY_ADDR, fn_battery); //fn_battery

  at24c08_write(FN_DP_ADDR, fn_dp); //fn_dp

  at24c08_write_byte(CG_LED_DATA_ADDR, cg_data_led); //cg_data_led

  at24c08_write_byte(NO_NET_FN_ADDR, no_net_fn); //no_net_fn

  at24c08_write_byte(WIFI_MODE_ADDR, wifi_mode); //wifi_mode
}

/*******************************************************************************
  At24c08 Save Metadata with locked
*******************************************************************************/
void osi_MetaData_Save(void)
{
  //vTaskSuspendAll(); //disable the tasks

  MetaData_Save(); //At24c08 Save Metadata

  //xTaskResumeAll(); //enable all tasks
}

/*******************************************************************************
//Read Metadata in At24c08
*******************************************************************************/
void MetaData_Read(void)
{
  fn_th = at24c08_read(FN_TH_ADDR); //fn_th

  fn_light = at24c08_read(FN_LIGHT_ADDR); //fn_light

#ifdef MAG_SENSOR
  fn_mag = at24c08_read(FN_MAG_ADDR); //fn_mag

  fn_mag_int = at24c08_read_byte(FN_MAG_INT_ADDR); //fn_mag_int
#endif

  fn_acc_tap1 = at24c08_read_byte(FN_ACC_TAP1_ADDR); //fn_acc_tap1

  fn_acc_tap2 = at24c08_read_byte(FN_ACC_TAP2_ADDR); //fn_acc_tap2

  fn_acc_act = at24c08_read_byte(FN_ACC_ACT_ADDR); //fn_acc_act

  thres_acc_min = at24c08_read_byte(THRES_ACC_MIN_ADDR); //thres_acc_min

  thres_acc_min = thres_acc_min > DEFAULT_THRES_ACC_MIN ? thres_acc_min : DEFAULT_THRES_ACC_MIN;

  fn_bt = at24c08_read(FN_BT_ADDR); //fn_bt

  fn_ext = at24c08_read(FN_EXT_ADDR); //fn_ext

  fn_battery = at24c08_read(FN_BATTERY_ADDR); //fn_battery

  fn_dp = at24c08_read(FN_DP_ADDR); //fn_dp

  cg_data_led = at24c08_read_byte(CG_LED_DATA_ADDR); //cg_data_led

  no_net_fn = at24c08_read_byte(NO_NET_FN_ADDR); //no_net_fn

  wifi_mode = at24c08_read_byte(WIFI_MODE_ADDR); //wifi_mode
}

/*******************************************************************************
//Read Metadata in At24c08 with locked
*******************************************************************************/
void osi_MetaData_Read(void)
{
  //vTaskSuspendAll(); //disable the tasks

  MetaData_Read();

  //xTaskResumeAll(); //enable all tasks
}

/*******************************************************************************
//  Cali data init
*******************************************************************************/
void CaliData_Init(void)
{
  f1_a = DEFAULT_CALI_A; //reset
  f1_b = DEFAULT_CALI_B; //reset

  f2_a = DEFAULT_CALI_A; //reset
  f2_b = DEFAULT_CALI_B; //reset

  f3_a = DEFAULT_CALI_A; //reset
  f3_b = DEFAULT_CALI_B; //reset

  f4_a = DEFAULT_CALI_A; //reset
  f4_b = DEFAULT_CALI_B; //reset

  f5_a = DEFAULT_CALI_A; //reset
  f5_b = DEFAULT_CALI_B; //reset

  f6_a = DEFAULT_CALI_A; //reset
  f6_b = DEFAULT_CALI_B; //reset

  f7_a = DEFAULT_CALI_A; //reset
  f7_b = DEFAULT_CALI_B; //reset

  f8_a = DEFAULT_CALI_A; //reset
  f8_b = DEFAULT_CALI_B; //reset

  f9_a = DEFAULT_CALI_A; //reset
  f9_b = DEFAULT_CALI_B; //reset

  f10_a = DEFAULT_CALI_A; //reset
  f10_b = DEFAULT_CALI_B; //reset
}

/*******************************************************************************
//  At24c08 Save Cali data
*******************************************************************************/
void CaliData_Save(void)
{
  f_cali resp_cali;

  resp_cali.cali_val = f1_a;
  at24c08_write(F1_A_ADDR, resp_cali.cali_buf); //f1_a
  resp_cali.cali_val = f1_b;
  at24c08_write(F1_B_ADDR, resp_cali.cali_buf); //f1_b

  resp_cali.cali_val = f2_a;
  at24c08_write(F2_A_ADDR, resp_cali.cali_buf); //f2_a
  resp_cali.cali_val = f2_b;
  at24c08_write(F2_B_ADDR, resp_cali.cali_buf); //f2_b

  resp_cali.cali_val = f3_a;
  at24c08_write(F3_A_ADDR, resp_cali.cali_buf); //f3_a
  resp_cali.cali_val = f3_b;
  at24c08_write(F3_B_ADDR, resp_cali.cali_buf); //f3_b

  resp_cali.cali_val = f4_a;
  at24c08_write(F4_A_ADDR, resp_cali.cali_buf); //f4_a
  resp_cali.cali_val = f4_b;
  at24c08_write(F4_B_ADDR, resp_cali.cali_buf); //f4_b

  resp_cali.cali_val = f5_a;
  at24c08_write(F5_A_ADDR, resp_cali.cali_buf); //f5_a
  resp_cali.cali_val = f5_b;
  at24c08_write(F5_B_ADDR, resp_cali.cali_buf); //f5_b

  resp_cali.cali_val = f6_a;
  at24c08_write(F6_A_ADDR, resp_cali.cali_buf); //f6_a
  resp_cali.cali_val = f6_b;
  at24c08_write(F6_B_ADDR, resp_cali.cali_buf); //f6_b

  resp_cali.cali_val = f7_a;
  at24c08_write(F7_A_ADDR, resp_cali.cali_buf); //f7_a
  resp_cali.cali_val = f7_b;
  at24c08_write(F7_B_ADDR, resp_cali.cali_buf); //f7_b

  resp_cali.cali_val = f8_a;
  at24c08_write(F8_A_ADDR, resp_cali.cali_buf); //f8_a
  resp_cali.cali_val = f8_b;
  at24c08_write(F8_B_ADDR, resp_cali.cali_buf); //f8_b

  resp_cali.cali_val = f9_a;
  at24c08_write(F9_A_ADDR, resp_cali.cali_buf); //f9_a
  resp_cali.cali_val = f9_b;
  at24c08_write(F9_B_ADDR, resp_cali.cali_buf); //f9_b

  resp_cali.cali_val = f10_a;
  at24c08_write(F10_A_ADDR, resp_cali.cali_buf); //f10_a
  resp_cali.cali_val = f10_b;
  at24c08_write(F10_B_ADDR, resp_cali.cali_buf); //f10_b
}

/*******************************************************************************
//  At24c08 Save Calidata with locked
*******************************************************************************/
void osi_CaliData_Save(void)
{
  //vTaskSuspendAll(); //disable the tasks

  CaliData_Save(); //At24c08 Save Calidata

  //xTaskResumeAll(); //enable all tasks
}

/*******************************************************************************
//Read Calidata in At24c08
*******************************************************************************/
void CaliData_Read(void)
{
  f_cali resp_cali;

  resp_cali.cali_buf = at24c08_read(F1_A_ADDR); //f1_a
  f1_a = resp_cali.cali_val;
  resp_cali.cali_buf = at24c08_read(F1_B_ADDR); //f1_a
  f1_b = resp_cali.cali_val;

  resp_cali.cali_buf = at24c08_read(F2_A_ADDR); //f2_a
  f2_a = resp_cali.cali_val;
  resp_cali.cali_buf = at24c08_read(F2_B_ADDR); //f2_a
  f2_b = resp_cali.cali_val;

  resp_cali.cali_buf = at24c08_read(F3_A_ADDR); //f3_a
  f3_a = resp_cali.cali_val;
  resp_cali.cali_buf = at24c08_read(F3_B_ADDR); //f3_a
  f3_b = resp_cali.cali_val;

  resp_cali.cali_buf = at24c08_read(F4_A_ADDR); //f4_a
  f4_a = resp_cali.cali_val;
  resp_cali.cali_buf = at24c08_read(F4_B_ADDR); //f4_a
  f4_b = resp_cali.cali_val;

  resp_cali.cali_buf = at24c08_read(F5_A_ADDR); //f5_a
  f5_a = resp_cali.cali_val;
  resp_cali.cali_buf = at24c08_read(F5_B_ADDR); //f5_a
  f5_b = resp_cali.cali_val;

  resp_cali.cali_buf = at24c08_read(F6_A_ADDR); //f6_a
  f6_a = resp_cali.cali_val;
  resp_cali.cali_buf = at24c08_read(F6_B_ADDR); //f6_a
  f6_b = resp_cali.cali_val;

  resp_cali.cali_buf = at24c08_read(F7_A_ADDR); //f7_a
  f7_a = resp_cali.cali_val;
  resp_cali.cali_buf = at24c08_read(F7_B_ADDR); //f7_a
  f7_b = resp_cali.cali_val;

  resp_cali.cali_buf = at24c08_read(F8_A_ADDR); //f8_a
  f8_a = resp_cali.cali_val;
  resp_cali.cali_buf = at24c08_read(F8_B_ADDR); //f8_a
  f8_b = resp_cali.cali_val;

  resp_cali.cali_buf = at24c08_read(F9_A_ADDR); //f9_a
  f9_a = resp_cali.cali_val;
  resp_cali.cali_buf = at24c08_read(F9_B_ADDR); //f9_a
  f9_b = resp_cali.cali_val;

  resp_cali.cali_buf = at24c08_read(F10_A_ADDR); //f10_a
  f10_a = resp_cali.cali_val;
  resp_cali.cali_buf = at24c08_read(F10_B_ADDR); //f10_a
  f10_b = resp_cali.cali_val;
}

/*******************************************************************************
//Read Calidata in At24c08 with locked
*******************************************************************************/
void osi_CaliData_Read(void)
{
  //vTaskSuspendAll(); //disable the tasks

  CaliData_Read();

  //xTaskResumeAll(); //enable all tasks
}

/*******************************************************************************
//Save Operate Data in At24c08
*******************************************************************************/
void OperateData_Save(void)
{
  at24c08_write(FN_TH_T_ADDR, fn_th_t); //fn_th_t

  at24c08_write(FN_LIGHT_T_ADDR, fn_light_t); //fn_light_t

#ifdef MAG_SENSOR
  at24c08_write(FN_MAG_T_ADDR, fn_mag_t); //fn_mag_t
#endif

  at24c08_write(FN_BT_T_ADDR, fn_bt_t); //fn_bt_t

  at24c08_write(FN_EXT_T_ADDR, fn_ext_t); //fn_ext_t

  at24c08_write(FN_BATTERY_T_ADDR, fn_battery_t); //fn_battery_t

  at24c08_write(FN_DP_T_ADDR, fn_dp_t); //fn_dp_t
}

/*******************************************************************************
//Save Operate Data in At24c08 with locked
*******************************************************************************/
void osi_OperateData_Save(void)
{
  //vTaskSuspendAll(); //disable the tasks

  OperateData_Save();

  //xTaskResumeAll(); //enable all tasks
}

/*******************************************************************************
  Error Code Init Save in At24c08
*******************************************************************************/
void Error_Code_Init(void)
{
  //  at24c08_write(MALLOC_FLR_ERR,0);      //Malloc Failed
  //
  //  at24c08_write(STATCK_OVER_FLOW,0);    //Stack Over flow

  at24c08_write(SCN_WF_FLR_ERR, 0); //scan wifi failured

  at24c08_write(WF_PWD_WR_ERR, 0); //wifi password wrong

  at24c08_write(WD_RESET_ERR, 0); //watch dog reset

  at24c08_write(CNT_WIFI_FLR_ERR, 0); //wifi connect failure

  at24c08_write(RSL_HOST_FLR_ERR, 0); //Resolve HOST NAME/IP failued

  at24c08_write(CNT_SERVER_FLR_ERR, 0); //connect to the http server failured

  at24c08_write(API_GET_FLR_ERR, 0); //api get failure

  at24c08_write(UPDATE_TIME_FLR_ERR, 0); //get time form server failured

  at24c08_write(MEMORY_SAVE_DATA_ERR, 0); //read save data fault

  at24c08_write(POST_DATA_JSON_FAULT, 0); //post data json fault

  at24c08_write(POST_DATA_FLR_ERR, 0); //post data failured
}

/*******************************************************************************
  Error Code Init Save in At24c08 whit locked
*******************************************************************************/
void osi_Error_Code_Init(void)
{
  //vTaskSuspendAll(); //disable the tasks

  Error_Code_Init();

  //xTaskResumeAll(); //enable all tasks
}

/*******************************************************************************
  Read Operate Data in At24c08
*******************************************************************************/
void OperateData_Read(void)
{
  fn_th_t = at24c08_read(FN_TH_T_ADDR); //fn_th_t

  fn_light_t = at24c08_read(FN_LIGHT_T_ADDR); //fn_light_t

#ifdef MAG_SENSOR
  fn_mag_t = at24c08_read(FN_MAG_T_ADDR); //fn_mag_t
#endif

  fn_bt_t = at24c08_read(FN_BT_T_ADDR); //fn_bt_t

  fn_ext_t = at24c08_read(FN_EXT_T_ADDR); //fn_ext_t

  fn_battery_t = at24c08_read(FN_BATTERY_T_ADDR); //fn_battery_t

  fn_dp_t = at24c08_read(FN_DP_T_ADDR); //fn_dp_t

#ifdef MAG_SENSOR
  door_status = at24c08_read_byte(DOOR_STATUS_ADDR); //door status
#endif

  save_addr_flag = at24c08_read_byte(DATA_SAVE_FLAG_ADDR);
}

/*******************************************************************************
  Read Operate Data in At24c08 with locked
*******************************************************************************/
void osi_OperateData_Read(void)
{
  //vTaskSuspendAll(); //disable the tasks

  OperateData_Read();

  //xTaskResumeAll(); //enable all tasks
}

/*******************************************************************************
  Operate Data Init
*******************************************************************************/
void OperateData_Init(void)
{
  fn_th_t = now_unix_t + fn_th;

  fn_light_t = now_unix_t + fn_light;

#ifdef MAG_SENSOR
  fn_mag_t = now_unix_t + fn_mag;
#endif

  fn_bt_t = now_unix_t + fn_bt;

  fn_ext_t = now_unix_t + fn_ext;

  fn_battery_t = now_unix_t + fn_battery;

  fn_dp_t = now_unix_t + fn_dp;
}

/*******************************************************************************
  Nor Flash Memory Chip Reset
*******************************************************************************/
void Save_Data_Reset(void)
{
  POST_NUM = 0;    //reset data post amount
  WRITE_ADDR = 0;  //reset data write address
  POST_ADDR = 0;   //reset data post address
  DELETE_ADDR = 0; //reset data delete address
}

/*******************************************************************************
 save address test
*******************************************************************************/
void save_addr_test(void)
{
  save_addr_flag = 0;

  POST_NUM = TEST_NUM;
  WRITE_ADDR = TEST_NUM;
  POST_ADDR = TEST_NUM;
  DELETE_ADDR = TEST_NUM;

  osi_at24c08_save_addr(); //save

  osi_at24c08_read_addr(); //read

  if ((POST_NUM != TEST_NUM) || (WRITE_ADDR != TEST_NUM) || (POST_ADDR != TEST_NUM) || (DELETE_ADDR != TEST_NUM))
  {
    save_addr_flag = 0xFF;
  }

  osi_at24c08_write_byte(DATA_SAVE_FLAG_ADDR, save_addr_flag); //save save addr flag
}

/*******************************************************************************
  SPI Locked Nor Flash Memory Chip Reset
*******************************************************************************/
void osi_Save_Data_Reset(void)
{
  uint8_t ret_ry = 0;
  uint8_t reg_val = 1;

  save_addr_test();

  Save_Data_Reset();

  osi_at24c08_save_addr(); //save post data amount/write data address/post data address/delete data address

  xSemaphoreTake(xMutex1, -1); //SPI Semaphore Take

  if (flash_set_val != SUCCESS)
  {
    flash_set_val = w25q_Init();
  }

  w25q_WriteReg(WRITE_STATUS_REGISTER, 0x00); //write status register,no protected

  w25q_WriteCommand(WRITE_ENABLE); //write enable

  w25q_WriteCommand(CHIP_ERASE); //chip eaase code

  xSemaphoreGive(xMutex1); //SPI Semaphore Give

  while (reg_val & 0x01)
  {
    if (ret_ry++ > 200) //time out 1min
    {
      break;
    }

    SET_RED_LED_ON(); //Set Red Led ON

    // MAP_UtilsDelay(6000000); //delay about 450ms
    vTaskDelay(450 / portTICK_RATE_MS);

    SET_RED_LED_OFF(); //Set Red Led Off

    // MAP_UtilsDelay(6000000); //delay about 450ms
    vTaskDelay(450 / portTICK_RATE_MS);

    xSemaphoreTake(xMutex1, -1); //SPI Semaphore Take

    reg_val = w25q_ReadReg(READ_STATUS_REGISTER);

    xSemaphoreGive(xMutex1); //SPI Semaphore Give

    sys_run_time = 0; //clear system time out
  }
}

/*******************************************************************************
//Read data in w25q128 memory chip whit spi locked
*******************************************************************************/
short osi_w25q_ReadData(uint32_t addr, char *buffer, uint8_t size, uint8_t *read_size)
{
  short retVal;

  xSemaphoreTake(xMutex1, -1); //SPI Semaphore Take

  retVal = w25q_ReadData(addr, buffer, size, read_size);

  xSemaphoreGive(xMutex1); //SPI Semaphore Give

  return retVal;
}

/*******************************************************************************
//read post data length
*******************************************************************************/
short Read_PostDataLen(unsigned long start_addr, unsigned long *end_addr, uint16_t read_num, uint16_t *post_num, unsigned long *post_data_len)
{
  uint16_t i_read;
  short read_flag = -1;
  uint8_t read_try;
  uint8_t read_data_size;
  unsigned long data_len = 0;
  char read_buf[SAVE_DATA_SIZE];

  data_len = strlen("{\"feeds\":[");

  ESP_LOGI(TAG, "%d,start_addr=%ld,end_addr=%ld,read_num=%d,post_num=%d,post_data_len=%ld", __LINE__,
           start_addr,
           *end_addr,
           read_num,
           *post_num,
           *post_data_len);

  for (i_read = 0; i_read < read_num; i_read++)
  {
    if ((start_addr + SAVE_DATA_SIZE) > Memory_Max_Addr) //when write WRITE_ADDR+sizeof(buffer)<=Memory_Max_Addr,WRITE_ADDR=0
    {
      start_addr = 0;
    }

    for (read_try = 0; read_try < RETRY_TIME_OUT; read_try++)
    {
      read_flag = osi_w25q_ReadData(start_addr, read_buf, SAVE_DATA_SIZE, &read_data_size); //read data
      ESP_LOGI(TAG, "%d,read_flag=%d,read_buf=%s", __LINE__, read_flag, read_buf);

      if (read_flag >= 0)
      {
        break;
      }
    }
    if (read_flag >= 0)
    {
      data_len += strlen(read_buf) + 1; //json between whit ','

      start_addr += strlen(read_buf) + 1; //save end whit '!'
    }
    else
    {
      if (i_read > 0) //data need to post
      {
        goto end;
      }
      else //fist data error
      {
        *post_num = 1;

        *end_addr = start_addr + read_data_size;

        return FAILURE;
      }
    }
  }

end:

  data_len -= 1; //delete end ','

  data_len += strlen("]}");

  *end_addr = start_addr; //read end address

  *post_num = i_read; //post data number

  *post_data_len = data_len; //post data length

  return SUCCESS;
}

/*******************************************************************************
//read post data
*******************************************************************************/
unsigned long Read_PostDataBuffer(unsigned long Address, char *buffer, uint16_t Amount, bool flag)
{
  uint16_t i_read;
  short read_flag;
  uint8_t read_try;
  uint8_t read_data_size;
  uint16_t DataBit = 0;
  char read_buf[SAVE_DATA_SIZE];

  for (i_read = 0; i_read < Amount; i_read++)
  {
    if ((Address + SAVE_DATA_SIZE) >= Memory_Max_Addr)
    {
      Address = 0;
    }

    for (read_try = 0; read_try < RETRY_TIME_OUT; read_try++)
    {
      read_flag = osi_w25q_ReadData(Address, read_buf, SAVE_DATA_SIZE, &read_data_size);

      if (read_flag >= 0)
      {
        break;
      }
    }

    memcpy(buffer + DataBit, read_buf, strlen(read_buf));

    DataBit += strlen(read_buf);

    memcpy(buffer + DataBit, ",", 1);

    DataBit += 1; //end with ','

    Address += strlen(read_buf) + 1; //end whit '!'
  }
  if (flag) //read post data end
  {
    buffer[DataBit - 1] = '\0';
  }
  else
  {
    buffer[DataBit] = '\0';
  }

  return Address;
}

/*******************************************************************************
//read temperature humility data with locked
*******************************************************************************/
void osi_sht30_SingleShotMeasure(float *temp, float *humi)
{
  //vTaskSuspendAll(); //disable the tasks

  // sht30_SingleShotMeasure(temp, humi); //read temperature humility data
  sht30_SS_get_value(temp, humi);

  //xTaskResumeAll(); //enable all tasks
}

/*******************************************************************************
//Read Light Value with locked
*******************************************************************************/
void osi_OPT3001_value(float *lightvalue)
{
  //vTaskSuspendAll(); //disable the tasks

  OPT3001_value(lightvalue); //Read Light Value

  //xTaskResumeAll(); //enable all tasks
}

/*******************************************************************************
//read three Axis data with locked
*******************************************************************************/
void osi_ADXL345_RD_xyz(short *x_val, short *y_val, short *z_val)
{
#ifdef ADXL345_SPI
  xSemaphoreTake(xMutex1, -1); //SPI Semaphore Take
  if (Adxl345_Check_Id() < 0)
  {
    *x_val = ERROR_CODE;
    *y_val = ERROR_CODE;
    *z_val = ERROR_CODE;
  }
  else
  {
    ADXL345_RD_xyz(x_val, y_val, z_val); //read three Axis data
  }
  xSemaphoreGive(xMutex1); //SPI Semaphore Give
#endif
#ifdef ADXL345_IIC
  //vTaskSuspendAll(); //disable the tasks
  if (Adxl345_Check_Id() < 0)
  {
    *x_val = ERROR_CODE;
    *y_val = ERROR_CODE;
    *z_val = ERROR_CODE;
  }
  else
  {
    ADXL345_RD_xyz(x_val, y_val, z_val); //read three Axis data
  }
  //xTaskResumeAll(); //enable all tasks
#endif
}

/******************************************************************************
  measure the temperature  with locked
******************************************************************************/
float osi_ds18b20_get_temp(void)
{
  float temp_val;

  //vTaskSuspendAll(); //disable the tasks

  temp_val = ds18b20_get_temp(); //measure the temperature

  if (temp_val == 85) //the fist time read temp val is 85
  {
    temp_val = ds18b20_get_temp(); //measure the temperature
  }

  //xTaskResumeAll(); //enable all tasks

  return temp_val;
}

/*******************************************************************************
//Get the Usage value
*******************************************************************************/
uint8_t Get_Usage_Val(void)
{
  uint8_t usg_val;
  unsigned long post_addr_val;
  unsigned long write_addr_val;

  post_addr_val = POST_ADDR;

  write_addr_val = WRITE_ADDR;

  if (write_addr_val >= post_addr_val)
  {
    usg_val = (uint8_t)((100 * (write_addr_val - post_addr_val)) / Memory_Max_Addr);
  }
  else
  {
    usg_val = (uint8_t)((100 * (Memory_Max_Addr - (post_addr_val - write_addr_val))) / Memory_Max_Addr);
  }

  return usg_val;
}

/*******************************************************************************
                                      END         
*******************************************************************************/
