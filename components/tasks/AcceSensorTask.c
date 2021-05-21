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
#include "stdlib.h"
#include "stdbool.h"
#include "math.h"
#include "osi.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "rom_map.h"
#include "utils.h"
#include "gpio.h"
#include "cc_types.h"
#include "MsgType.h"
#include "adxl345.h"
#include "PCF8563.h"
#include "AcceSensorTask.h"
#include "PeripheralDriver.h"

#define ERROR_CODE      0xffff

extern OsiSyncObj_t             xBinary5;       //For Acceleration SensorTask//
extern OsiSyncObj_t             xBinary12;      //For Acce Sensor Interrupt Task//
extern OsiMsgQ_t                xQueue0;        //Used for cjson and memory save//
extern OsiSyncObj_t             xMutex1;        //Used for SPI Lock//

extern volatile bool            data_post;      //need post data immediately//
extern volatile bool            acce_act;       //acceleration sensor active//
extern volatile uint16_t        sys_run_time;

extern volatile uint8_t         fn_acc_tap1;    //0:closed single tap,1:single tap interrupt,2:single tap interrupt and post//
extern volatile uint8_t         fn_acc_tap2;    //0:closed double tap,1:double tap interrupt,2:double tap interrupt and post//
extern volatile uint8_t         fn_acc_act;     //0:cloused act interrupt,1:act interrupt//
extern volatile uint8_t         thres_acc_min;  //min value for acceleration sensor act interrupt//

extern float f6_a,f6_b,f7_a,f7_b;

/*******************************************************************************
adx345 read register with locked
*******************************************************************************/
uint8_t osi_adx345_readReg(uint8_t addr)
{
  uint8_t read_val;

#ifdef ADXL345_SPI
  osi_SyncObjWait(&xMutex1,OSI_WAIT_FOREVER);  //SPI Semaphore Take   
  read_val=adx345_readReg(addr);
  osi_SyncObjSignal(&xMutex1);  //SPI Semaphore Give
#endif
#ifdef ADXL345_IIC
  osi_TaskDisable();  //disable the tasks
  read_val=adx345_readReg(addr);
  osi_TaskEnable(0);  //enable all tasks
#endif
  
  return read_val;
}

/*******************************************************************************
  Reset the acce sensor
*******************************************************************************/
void acce_sensor_reset(void)
{
  uint8_t set_val=0x87;  //7:DATA_READY/1:WATERMARK/0:OVERRUN Interrupt Enable
   
  if(fn_acc_act)
  {
    set_val|=0x18;  //4:ACTIVITY/3:INACTIVITY Interrupt Enable
  }
  if((fn_acc_tap1)||(fn_acc_tap2))
  {
    set_val|=0x60;  //6:SINGLE_TAP/DOUBLE_TAP Interrupt Enable
  }
#ifdef ADXL345_SPI
  osi_SyncObjWait(&xMutex1,OSI_WAIT_FOREVER);  //SPI Semaphore Take
  if(adx345_readReg(DEVICE_ID)==0xE5)  //read the sensor device id
  {
    if(fn_acc_tap1||fn_acc_tap2||fn_acc_act)
    {
      adx345_writeReg(POWER_CTL,0x20);  //standby mode
      adx345_writeReg(INT_ENABLE,0x00);  //disable all interrupt
      adx345_writeReg(THRESH_ACT,thres_acc_min);  //thres_acc_min*62.5mg
      adx345_writeReg(THRESH_INACT,thres_acc_min);  //thres_acc_min*62.5mg
      adx345_writeReg(INT_ENABLE,set_val);  //enable acce sensor interrupt
      adx345_writeReg(POWER_CTL,0x28);  //link mode,measure mode
    }
    else
    {
      adx345_writeReg(POWER_CTL,0x24);  //link mode,standby mode,deep sleep mode
    }
  }
  osi_SyncObjSignal(&xMutex1);  //SPI Semaphore Give
#endif
#ifdef ADXL345_IIC
  osi_TaskDisable();  //disable the tasks
  if(adx345_readReg(DEVICE_ID)==0xE5)  //read the sensor device id
  {
    if(fn_acc_tap1||fn_acc_tap2||fn_acc_act)
    {
      adx345_writeReg(POWER_CTL,0x20);  //standby mode
      adx345_writeReg(INT_ENABLE,0x00);  //disable all interrupt
      adx345_writeReg(THRESH_ACT,thres_acc_min);  //thres_acc_min*62.5mg
      adx345_writeReg(THRESH_INACT,thres_acc_min);  //thres_acc_min*62.5mg
      adx345_writeReg(INT_ENABLE,set_val);  //enable acce sensor interrupt
      adx345_writeReg(POWER_CTL,0x28);  //link mode,measure mode
    }
    else
    {
      adx345_writeReg(POWER_CTL,0x24);  //link mode,standby mode,deep sleep mode
    }
  }
  osi_TaskEnable(0);  //enable all tasks
#endif
}

/*******************************************************************************
  adxl345 read x y z axis value whit locked
*******************************************************************************/
static void osi_adxl_read(short* x_val,short* y_val,short* z_val)
{
#ifdef ADXL345_SPI
  osi_SyncObjWait(&xMutex1,OSI_WAIT_FOREVER);   //SPI Semaphore Take   
  ADXL345_RD_xyz(x_val,y_val,z_val);
  osi_SyncObjSignal(&xMutex1);                  //SPI Semaphore Give
#endif
#ifdef ADXL345_IIC
  osi_TaskDisable();  //disable the tasks
  ADXL345_RD_xyz(x_val,y_val,z_val);  //read three Axis data 
  osi_TaskEnable(0);  //enable all tasks
#endif
}

/*******************************************************************************
  get acceleration sensor value
*******************************************************************************/
static void AccelerationValue(float *accevalue)
{
  float SumAcce;
  uint8_t i_time=0;
  uint8_t m_time,n_time;
  short x1_val,y1_val,z1_val;
  short x2_val,y2_val,z2_val;
  uint8_t f_sec_val,s_sec_val;
  long xAxis_val,yAxis_val,zAxis_val;
  
  f_sec_val=osi_IIC_ReadReg(PCF8563_ADDR,VL_seconds);  //read time sec
  s_sec_val=f_sec_val;
  osi_adxl_read(&x1_val,&y1_val,&z1_val);               //adxl345 read x y z axis value whit locked
  while(f_sec_val==s_sec_val)
  {
    m_time=osi_adx345_readReg(FIFO_STATUS);
    for(n_time=0;n_time<m_time;n_time++)
    { 
      i_time++;
      osi_adxl_read(&x2_val,&y2_val,&z2_val);           //adxl345 read x y z axis value whit locked
      
      xAxis_val = (x2_val-x1_val)*(x2_val-x1_val);
      yAxis_val = (y2_val-y1_val)*(y2_val-y1_val);
      zAxis_val = (z2_val-z1_val)*(z2_val-z1_val);
      
      x1_val=x2_val;
      y1_val=y2_val;
      z1_val=z2_val;
      
      SumAcce+=(float)sqrt(xAxis_val + yAxis_val + zAxis_val);
    }

    if(i_time>200)
    {
      break;    //time out
    }
    
    MAP_UtilsDelay(200000);     //delay 15ms
    s_sec_val = osi_IIC_ReadReg(PCF8563_ADDR,VL_seconds);       //read time sec
  }
  
#ifdef DEBUG
  osi_UartPrint_Val("ACCE:",i_time);
#endif
  
  *accevalue = i_time<=0?ERROR_CODE:SumAcce/i_time;
}

/*******************************************************************************
  Acceleration Interrupt Application Task
*******************************************************************************/
void AcceSensor_Int_Task(void *pvParameters)
{
  uint8_t acce_status;
  SensorMessage xMsg;
  
  for(;;)
  {
    osi_SyncObjWait(&xBinary12,OSI_WAIT_FOREVER);  //Waite GPIO Interrupt Message 
    
    if(!GPIOPinRead(ACCE_PORT,ACCE_PIN))  
    {
      acce_status=osi_adx345_readReg(INT_SOURCE);
      #ifdef DEBUG
        osi_UartPrint_Val("INT_SOURCE:",acce_status);
      #endif
  
      if(acce_status&0x10)  //ACCE Sensor ACT Interrupt
      {
        if(fn_acc_act)
        {
          acce_act=1;
          osi_SyncObjSignalFromISR(&xBinary5);  //Start Acce Sensor Task 
        }
      }
      if(acce_status&0x08)  //ACCE Sensor INACT Interrupt
      {
        acce_act=0;
      }
      
      if(acce_status&0x20)  //ACCE Sensor DOUBLE_TAP Interrupt
      {
        if(fn_acc_tap2)
        {
          if(fn_acc_tap2==2)
          {
            data_post=1;  //Need Post Data Immediately
          }
          xMsg.sensornum=TAP_NUM;  //Message Number
          xMsg.sensorval = f7_a*2 + f7_b;  //Message Value
          osi_MsgQWrite(&xQueue0,&xMsg,OSI_NO_WAIT);  //Send Acceleration Value
          MAP_UtilsDelay(4000000);  //300ms
        }
      }
      else if(acce_status&0x40) //ACCE Sensor SINGLE_TAP Interrupt
      {
        if(fn_acc_tap1)
        {
          if(fn_acc_tap1==2)
          {
            data_post=1;  //Need Post Data Immediately
          }
          xMsg.sensornum=TAP_NUM;  //Message Number
          xMsg.sensorval= f7_a*1 + f7_b;  //Message Value
          osi_MsgQWrite(&xQueue0,&xMsg,OSI_NO_WAIT);  //Send Acceleration Value
          MAP_UtilsDelay(4000000);  //300ms
        }
      }
      
      acce_status=osi_adx345_readReg(INT_SOURCE);
      if(acce_status&0x08)              //ACCE Sensor INACT Interrupt
      {
        acce_act=0;
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
  
  for(;;)
  {
    osi_SyncObjWait(&xBinary5,OSI_WAIT_FOREVER);  //Wait AcceSensor Interrupt Message
    
    acce_read=0;  
    while(acce_act)  //acceleration sensor active
    {
      acce_read+=1; 
      if(acce_read>60)
      {
        acce_act=0;
        osi_SyncObjSignalFromISR(&xBinary12);  //Start Acce Sensor interrupt Task 
        break;
      }
      
      AccelerationValue(&accevalue);  //read acceleration value
      if(accevalue!=ERROR_CODE)
      {
        aMsg.sensornum=ACCE_NUM;  //Message Number
        aMsg.sensorval=f6_a*accevalue + f6_b;  //Message Value
        osi_MsgQWrite(&xQueue0,&aMsg,OSI_NO_WAIT);  //send acceleration value
      }
      sys_run_time = 0;  //clear system time out
    }
  }
}


/*******************************************************************************
                                      END         
*******************************************************************************/




