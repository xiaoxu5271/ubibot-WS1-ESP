/*******************************************************************************
  * @file       Acce Sensor Application Task    
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
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "MsgType.h"
#include "adxl345.h"
#include "LIS2DH12.h"
#include "PCF8563.h"
#include "AcceSensorTask.h"
#include "PeripheralDriver.h"

#define TAG "AcceSensor_Task"

#define ERROR_CODE 0xffff

extern TaskHandle_t xBinary5;  //For Acceleration SensorTask//
extern TaskHandle_t xBinary12; //For Acce Sensor Interrupt Task//
extern QueueHandle_t xQueue0;  //Used for cjson and memory save//
extern TaskHandle_t xMutex1;   //Used for SPI Lock//

extern volatile bool data_post; //need post data immediately//
extern volatile bool acce_act;  //acceleration sensor active//
extern volatile bool acc_sle;
extern volatile uint16_t sys_run_time;

extern volatile uint8_t fn_acc_tap1;   //0:closed single tap,1:single tap interrupt,2:single tap interrupt and post//
extern volatile uint8_t fn_acc_tap2;   //0:closed double tap,1:double tap interrupt,2:double tap interrupt and post//
extern volatile uint8_t fn_acc_act;    //0:cloused act interrupt,1:act interrupt//
extern volatile uint8_t thres_acc_min; //min value for acceleration sensor act interrupt//

extern float f6_a, f6_b, f7_a, f7_b;

/*******************************************************************************
adx345 read register with locked
*******************************************************************************/
uint8_t osi_adx345_readReg(uint8_t addr)
{
  uint8_t read_val;

#ifdef ADXL345_SPI
  xSemaphoreTake(xMutex1, -1); //SPI Semaphore Take
  read_val = adx345_readReg(addr);
  xSemaphoreGive(xMutex1); //SPI Semaphore Give
#endif
#ifdef ADXL345_IIC
  //vTaskSuspendAll(); //disable the tasks
  read_val = adx345_readReg(addr);
  //xTaskResumeAll(); //enable all tasks
#endif

  return read_val;
}

/*******************************************************************************
  Reset the acce sensor
*******************************************************************************/
void acce_sensor_reset(void)
{
  if (acc_sle)
  {
    /* code */
  }
  else
  {
    uint8_t set_val = 0x87; //7:DATA_READY/1:WATERMARK/0:OVERRUN Interrupt Enable

    if (fn_acc_act)
    {
      set_val |= 0x18; //4:ACTIVITY/3:INACTIVITY Interrupt Enable
    }
    if ((fn_acc_tap1) || (fn_acc_tap2))
    {
      set_val |= 0x60; //6:SINGLE_TAP/DOUBLE_TAP Interrupt Enable
    }
#ifdef ADXL345_SPI
    xSemaphoreTake(xMutex1, -1);           //SPI Semaphore Take
    if (adx345_readReg(DEVICE_ID) == 0xE5) //read the sensor device id
    {
      if (fn_acc_tap1 || fn_acc_tap2 || fn_acc_act)
      {
        adx345_writeReg(POWER_CTL, 0x20);             //standby mode
        adx345_writeReg(INT_ENABLE, 0x00);            //disable all interrupt
        adx345_writeReg(THRESH_ACT, thres_acc_min);   //thres_acc_min*62.5mg
        adx345_writeReg(THRESH_INACT, thres_acc_min); //thres_acc_min*62.5mg
        adx345_writeReg(INT_ENABLE, set_val);         //enable acce sensor interrupt
        adx345_writeReg(POWER_CTL, 0x28);             //link mode,measure mode
      }
      else
      {
        adx345_writeReg(POWER_CTL, 0x24); //link mode,standby mode,deep sleep mode
      }
    }
    xSemaphoreGive(xMutex1); //SPI Semaphore Give
#endif
#ifdef ADXL345_IIC
    //vTaskSuspendAll();                     //disable the tasks
    if (adx345_readReg(DEVICE_ID) == 0xE5) //read the sensor device id
    {
      if (fn_acc_tap1 || fn_acc_tap2 || fn_acc_act)
      {
        adx345_writeReg(POWER_CTL, 0x20);             //standby mode
        adx345_writeReg(INT_ENABLE, 0x00);            //disable all interrupt
        adx345_writeReg(THRESH_ACT, thres_acc_min);   //thres_acc_min*62.5mg
        adx345_writeReg(THRESH_INACT, thres_acc_min); //thres_acc_min*62.5mg
        adx345_writeReg(INT_ENABLE, set_val);         //enable acce sensor interrupt
        adx345_writeReg(POWER_CTL, 0x28);             //link mode,measure mode
      }
      else
      {
        adx345_writeReg(POWER_CTL, 0x24); //link mode,standby mode,deep sleep mode
      }
    }
    //xTaskResumeAll(); //enable all tasks
#endif
  }
}

/*******************************************************************************
  adxl345 read x y z axis value whit locked
*******************************************************************************/
static void osi_adxl_read(short *x_val, short *y_val, short *z_val)
{
#ifdef ADXL345_SPI
  xSemaphoreTake(xMutex1, -1); //SPI Semaphore Take
  ADXL345_RD_xyz(x_val, y_val, z_val);
  xSemaphoreGive(xMutex1); //SPI Semaphore Give
#endif
#ifdef ADXL345_IIC
  //vTaskSuspendAll();                   //disable the tasks
  ADXL345_RD_xyz(x_val, y_val, z_val); //read three Axis data
  //xTaskResumeAll();                    //enable all tasks
#endif
}

/*******************************************************************************
  get acceleration sensor value
*******************************************************************************/
static void AccelerationValue(float *accevalue)
{
  float SumAcce = 0;
  uint8_t i_time = 0;
  uint8_t m_time, n_time;
  short x1_val, y1_val, z1_val;
  short x2_val, y2_val, z2_val;
  uint8_t f_sec_val, s_sec_val;
  long xAxis_val, yAxis_val, zAxis_val;

  f_sec_val = osi_IIC_ReadReg(PCF8563_ADDR, VL_seconds); //read time sec
  s_sec_val = f_sec_val;
  osi_adxl_read(&x1_val, &y1_val, &z1_val); //adxl345 read x y z axis value whit locked
  while (f_sec_val == s_sec_val)
  {
    m_time = osi_adx345_readReg(FIFO_STATUS);
    for (n_time = 0; n_time < m_time; n_time++)
    {
      i_time++;
      osi_adxl_read(&x2_val, &y2_val, &z2_val); //adxl345 read x y z axis value whit locked

      xAxis_val = (x2_val - x1_val) * (x2_val - x1_val);
      yAxis_val = (y2_val - y1_val) * (y2_val - y1_val);
      zAxis_val = (z2_val - z1_val) * (z2_val - z1_val);

      x1_val = x2_val;
      y1_val = y2_val;
      z1_val = z2_val;

      SumAcce += (float)sqrt(xAxis_val + yAxis_val + zAxis_val);
    }

    if (i_time > 200)
    {
      break; //time out
    }

    // MAP_UtilsDelay(200000);                                //delay 15ms
    vTaskDelay(15 / portTICK_RATE_MS);
    s_sec_val = osi_IIC_ReadReg(PCF8563_ADDR, VL_seconds); //read time sec
  }

#ifdef DEBUG
  osi_UartPrint_Val("ACCE:", i_time);
#endif

  *accevalue = i_time <= 0 ? ERROR_CODE : SumAcce / i_time;
}

/*******************************************************************************
  Acceleration Interrupt Application Task
*******************************************************************************/
void AcceSensor_Int_Task(void *pvParameters)
{
  uint8_t acce_status;
  // SensorMessage xMsg;

  for (;;)
  {
    // osi_SyncObjWait(&xBinary12, -1); //Waite GPIO Interrupt Message
    xEventGroupSetBits(Task_Group, SENTASK_12);
    ulTaskNotifyTake(pdTRUE, -1);
    xEventGroupClearBits(Task_Group, SENTASK_12);

    ESP_LOGI(TAG, "%d,ACCE_SRC_WKUP:%d", __LINE__, gpio_get_level(ACCE_SRC_WKUP));
    if (gpio_get_level(ACCE_SRC_WKUP))
    {
      if (acc_sle == 0) //ADXL
      {
        acce_status = osi_adx345_readReg(INT_SOURCE);
        ESP_LOGI(TAG, "%d,acce_status:%d", __LINE__, acce_status);

        if (acce_status & 0x10) //ACCE Sensor ACT Interrupt
        {
          if (fn_acc_act)
          {
            acce_act = 1;
            // osi_SyncObjSignalFromISR(&xBinary5);  //Start Acce Sensor Task
            if (xBinary5 != NULL)
              vTaskNotifyGiveFromISR(xBinary5, NULL);
          }
        }
        acce_status = osi_adx345_readReg(INT_SOURCE);
        if (acce_status & 0x08) //ACCE Sensor INACT Interrupt
        {
          acce_act = 0;
        }
      }
      else //LIS
      {
        acce_status = lis2dh12_clear_int();
        ESP_LOGI(TAG, "%d,acce_status:%d,fn_acc_act", __LINE__, acce_status);
        if (acce_status & 0x6a)
        {
          if (fn_acc_act)
          {
            acce_act = 1;
            if (xBinary5 != NULL)
              vTaskNotifyGiveFromISR(xBinary5, NULL);

            // acce_act = 0;
          }
        }
      }
    }
  }
}

/*******************************************************************************
//ACCELERATION SENSOR VALUE TASK
*******************************************************************************/
void AccelerationSensorTask(void *pvParameters)
{
  uint8_t acce_read;
  float accevalue;
  SensorMessage aMsg;

  for (;;)
  {
    // osi_SyncObjWait(&xBinary5, -1); //Wait AcceSensor Interrupt Message
    xEventGroupSetBits(Task_Group, SENTASK_5);
    ulTaskNotifyTake(pdTRUE, -1);
    xEventGroupClearBits(Task_Group, SENTASK_5);

    if (acc_sle)
    {
      accevalue = lis2dh12_data();
      aMsg.sensornum = ACCE_NUM;                //Message Number
      aMsg.sensorval = f6_a * accevalue + f6_b; //Message Value
      ESP_LOGI(TAG, "%d,sensorval=%.4f", __LINE__, aMsg.sensorval);
      xQueueSend(xQueue0, &aMsg, 0); //send acceleration value
    }
    else
    {
      acce_read = 0;
      while (acce_act) //acceleration sensor active
      {
        acce_read += 1;
        if (acce_read > 60)
        {
          acce_act = 0;
          // osi_SyncObjSignalFromISR(&xBinary12); //Start Acce Sensor interrupt Task
          if (xBinary12 != NULL)
            vTaskNotifyGiveFromISR(xBinary12, NULL);
          break;
        }

        AccelerationValue(&accevalue); //read acceleration value
        if (accevalue != ERROR_CODE)
        {
          aMsg.sensornum = ACCE_NUM;                //Message Number
          aMsg.sensorval = f6_a * accevalue + f6_b; //Message Value
          ESP_LOGI(TAG, "%d,sensorval=%.4f", __LINE__, aMsg.sensorval);
          xQueueSend(xQueue0, &aMsg, 0); //send acceleration value
        }
        sys_run_time = 0; //clear system time out
      }
    }
  }
}

/*******************************************************************************
                                      END         
*******************************************************************************/
