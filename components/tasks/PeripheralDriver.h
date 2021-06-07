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
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"

#define USB_PIN 35

#define G_LED_PIN 25
#define R_LED_PIN 26
#define BELL_PIN 22
#define BUTTON_PIN 4
#define USB_SRC_WKUP 35
// #define BUTTON_SRC_WKUP 4
#define ACCE_SRC_WKUP 13          //INT1->13 ,INT2->15
#define SLOW_CLK_FREQ (32 * 1024) //slow clock frequence

#define ERROR_CODE 0xffff

#define SET_GREEN_LED_ON() gpio_set_level(G_LED_PIN, 1);  // MAP_GPIOPinWrite(GPIOA0_BASE, GPIO_PIN_0, GPIO_PIN_0); //SET GREEN LED ON//
#define SET_GREEN_LED_OFF() gpio_set_level(G_LED_PIN, 0); // MAP_GPIOPinWrite(GPIOA0_BASE, GPIO_PIN_0, 0x00); //SET GREEN LED OFF//

#define SET_RED_LED_ON() gpio_set_level(R_LED_PIN, 1);  // MAP_GPIOPinWrite(GPIOA3_BASE, GPIO_PIN_6, GPIO_PIN_6); //SET RED LED ON//
#define SET_RED_LED_OFF() gpio_set_level(R_LED_PIN, 0); // MAP_GPIOPinWrite(GPIOA3_BASE, GPIO_PIN_6, 0x00); //SET RED LED OFF//

#define SET_BELL_ON() gpio_set_level(BELL_PIN, 1);  // MAP_GPIOPinWrite(GPIOA1_BASE, GPIO_PIN_0, GPIO_PIN_0); //SET BELL ON//
#define SET_BELL_OFF() gpio_set_level(BELL_PIN, 0); //MAP_GPIOPinWrite(GPIOA1_BASE, GPIO_PIN_0, 0x00);      //SET BELL OFF//

/*******************************************************************************
 FUNCTION PROTOTYPES
*******************************************************************************/
extern void osi_Sleep(uint16_t ms);

extern void PinMuxConfig(void);

extern void osi_Ht1621_Display_Err_Val(uint8_t err_val); //Display the error code//

extern void Green_Led_Flashed(uint8_t n_time, uint8_t on_time); //Green led flashed on_time*150ms//

extern void Red_Led_Flashed(uint8_t n_time, uint8_t on_time); //Red led flashed on_time*150ms//

extern void Red_Led_On_time(uint8_t on_time); //Red led on time on_time*150ms//

extern void Green_Red_Led_Flashed(uint8_t n_time, uint8_t on_time); //Green and Red led flashed on_time*150ms//

extern void Green_Red_LedFlashed_Task(void *pvParameters); //GREEN LED Flashed when AP Task //

extern void st_Green_Red_LedFlashed_Task(void *pvParameters); //GREEN and RED LED Flashed at the same time//

extern void Green_Red_Led_FastFlashed_Task(void *pvParameters); //GREEN and RED LED Fast Flashed when AP Task WIFI Connect//

extern void Green_LedFlashed_Task(void *pvParameters); //GREEN LED Flashed when Post Task

extern void Green_LedControl_Task(void *pvParameters); //Green LED control task//

extern void bell_makeSound(uint32_t n_bel); //bell make sound//

extern void Green_Led_Bell_Sound(uint16_t n_bel); //Green Led ON and Bell Sound//

extern void osi_bell_makeSound(uint16_t n_bel); //bell make sound with locked//

extern void Bell_Control_Task(void *pvParameters); //bell control task//

extern void osi_UartPrint(char *buffer); //Uart Print String//

extern void osi_UartPrint_Mul(char *buffer1, char *buffer2); //Uart Print Two String//

extern void osi_UartPrint_Val(char *buffer, unsigned long value); //Uart Print Value//

extern uint8_t osi_IIC_ReadReg(uint8_t sla_addr, uint8_t reg_addr); //IIC read register with locked//

extern void osi_at24c08_write_byte(uint16_t reg_addr, uint8_t num); //write a byte in at24c08 with locked//

extern uint8_t osi_at24c08_read_byte(uint16_t reg_addr); //read a byte in at24c08 with locked//

extern void osi_at24c08_write(uint16_t reg_addr, unsigned long num); //write unsigned long data with locked//

extern unsigned long osi_at24c08_read(uint16_t reg_addr); //read unsigned long data //

extern void osi_at24c08_WriteData(uint16_t addr, uint8_t *buffer, uint8_t Size, bool end_flag); //at24c08 write data with locked//

extern void osi_at24c08_ReadData(uint16_t addr, uint8_t *buffer, uint8_t size, bool end_flag); //at24c08 read data with locked//

extern void osi_Update_UTCtime(char *time_buff); //update utc time with locked//

extern void osi_Read_UTCtime(char *buffer, uint8_t buf_size); //Read UTC time with locked//

extern void osi_Read_UnixTime(void); //read unix time with locked//

extern void at24c08_save_addr(void); //Save Post Data Amount/Write Data/Post Data/Delete Data Address//

extern void osi_at24c08_save_addr(void); //Save Post Data Amount/Write Data/Post Data/Delete Data Address with locked//

extern void at24c08_read_addr(void); //Save Post Data Amount/Write Data/Post Data/Delete Data Address

extern void osi_at24c08_read_addr(void); //Read Post Data Amount/Write Data/Post Data/Delete Data Address whit locked

extern void MetaData_Init(void);     //Metadata init//
extern void MetaData_Save(void);     //At24c08 Save Metadata//
extern void osi_MetaData_Save(void); //At24c08 Save Metadata with locked//
extern void MetaData_Read(void);     //Read Metadata in At24c08//
extern void osi_MetaData_Read(void); //Read Metadata in At24c08 with locked//

extern void CaliData_Init(void);     //Calidata init//
extern void CaliData_Save(void);     //At24c08 Save Calidata//
extern void osi_CaliData_Save(void); //At24c08 Save Calidata with locked//
extern void CaliData_Read(void);     //Read Calidata in At24c08//
extern void osi_CaliData_Read(void); //Read Calidata in At24c08 with locked//

extern void OperateData_Save(void); //Save Operate Data in At24c08//

extern void osi_OperateData_Save(void); //Save Operate Data in At24c08 with locked//

extern void Error_Code_Init(void); //Save Operate Data in At24c08//

extern void osi_Error_Code_Init(void); //Error Code Init Save in At24c08 whit locked//

extern void OperateData_Read(void); //Read Operate Data in At24c08//

extern void osi_OperateData_Read(void); //Read Operate Data in At24c08 with locked//

extern void OperateData_Init(void); //Operate Data Init//

extern void Save_Data_Reset(void); //Nor Flash Memory Chip Reset//

extern void osi_Save_Data_Reset(void); //SPI Locked Nor Flash Memory Chip Reset//

extern short osi_w25q_ReadData(uint32_t addr, char *buffer, uint8_t size, uint8_t *read_size); //Read data in w25q128 memory chip whit spi locked//

extern short Read_PostDataLen(unsigned long start_addr, unsigned long *end_addr, uint16_t read_num, uint16_t *post_num, unsigned long *post_data_len); //read post data length//

extern unsigned long Read_PostDataBuffer(unsigned long Address, char *buffer, uint16_t Amount, bool flag); //read post data//

extern void osi_sht30_SingleShotMeasure(float *temp, float *humi); //read temperature humility data with locked//

extern void osi_OPT3001_value(float *lightvalue); //Read Light Value//

extern void osi_ADXL345_RD_xyz(short *x_val, short *y_val, short *z_val); //read three Axis data with locked//

extern float osi_ds18b20_get_temp(void); //measure the temperature  with locked//

extern uint8_t Get_Usage_Val(void); //Get the Usage value//

/*******************************************************************************
                                      END         
*******************************************************************************/
