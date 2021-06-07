/*******************************************************************************
  * @file       W25Q128 NOR FLASH CHIP DRIVER APPLICATION      
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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include "iic.h"
#include "MsgType.h"
#include "user_spi.h"

#include "w25q128.h"

#define TAG "w25q128"

extern SemaphoreHandle_t xMutex1; //Used for SPI Lock
extern SemaphoreHandle_t xMutex5; //Used for Post_Data_Buffer Lock

extern char Post_Data_Buffer[4096];

static void w25q_Read_Data(uint32_t addr, char *buffer, uint16_t size);

/*******************************************************************************
  nor flash spi start
*******************************************************************************/
static void Flash_Spi_Start(void)
{
  SET_SPI2_CS_OFF(); //w25q128 spi cs enable

  // MAP_SPIEnable(GSPI_BASE); //enable the spi channel
}

/*******************************************************************************
  nor flash spi stop
*******************************************************************************/
static void Flash_Spi_Stop(void)
{
  // MAP_SPIDisable(GSPI_BASE); //disable the spi channel

  SET_SPI2_CS_ON(); //w25q128 spi cs disable
}

/*******************************************************************************
  w25q128 read register
*******************************************************************************/
uint8_t w25q_ReadReg(uint8_t com_val)
{
  uint8_t value;

  Flash_Spi_Start(); //nor flash spi start

  SPI_Write(com_val); //send the read command

  value = SPI_Read();

  Flash_Spi_Stop(); //nor flash spi stop

  return value;
}

/*******************************************************************************
  w25q128 write register
*******************************************************************************/
void w25q_WriteCommand(uint8_t com_val)
{
  Flash_Spi_Start(); //nor flash spi start/

  SPI_Write(com_val); //send the write command

  Flash_Spi_Stop(); //nor flash spi stop
}

/*******************************************************************************
  waite write completed
*******************************************************************************/
static void w25q_WaitCompleted(void)
{
  uint16_t retry = 0;

  while (w25q_ReadReg(READ_STATUS_REGISTER) & 0x01) //read status register,bit0=0:Ready,bit0=1:Busy
  {
    if (retry++ > 60000) //time out 1.5min
    {
      break;
    }
    // MAP_UtilsDelay(20000); //delay about 1.5ms
    vTaskDelay(1.5 / portTICK_RATE_MS);
  }
}

/*******************************************************************************
  w25q128 write register
*******************************************************************************/
void w25q_WriteReg(uint8_t com_val, uint8_t value)
{
  w25q_WriteCommand(WRITE_ENABLE); //write enable

  Flash_Spi_Start(); //nor flash spi start

  SPI_Write(com_val); //send the write register command

  SPI_Write(value); //write register value

  Flash_Spi_Stop(); //nor flash spi stop

  w25q_WaitCompleted(); //waite command completed
}

/*******************************************************************************
  w25q128 read id
*******************************************************************************/
static uint16_t w25q_ReadId(void)
{
  uint16_t w25qid = 0;

  Flash_Spi_Start(); //nor flash spi start

  SPI_Write(W25Q_DEVICE_ID); //send the read manufacturer id

  SPI_Write(0x00); //recive data

  SPI_Write(0x00); //recive data

  SPI_Write(0x00); //recive data

  // w25qid = SPI_Read();

  // w25qid = w25qid << 8;

  // w25qid = SPI_Read();

  // w25qid += SPI_Read();

  w25qid |= SPI_Read() << 8;
  w25qid |= SPI_Read();

  Flash_Spi_Stop(); //nor flash spi stop

  ESP_LOGI(TAG, "%d,%04X", __LINE__, w25qid);
  return w25qid;
}

/*******************************************************************************
  init memory ic
*******************************************************************************/
short w25q_Init(void)
{
  if (w25q_ReadId() == W25Q128)
  {
    return SUCCESS;
  }
  return FAILURE;
}

/*******************************************************************************
  w25q128 read data
*******************************************************************************/
short w25q_ReadData(uint32_t addr, char *buffer, uint8_t size, uint8_t *read_size)
{
  uint8_t i;

  Flash_Spi_Start(); //nor flash spi start

  SPI_Write(READ_DATA); //read operations code

  SPI_Write((uint8_t)((addr) >> 16)); //24bit address first 8 bit address

  SPI_Write((uint8_t)((addr) >> 8));

  SPI_Write((uint8_t)(addr)); //24bit address last 8 bit address

  for (i = 0; i < size; i++)
  {
    buffer[i] = SPI_Read(); //read one byte

    if (buffer[i] == '!') //end with '!'
    {
      buffer[i] = '\0'; //Read completed

      *read_size = i + 1;

      Flash_Spi_Stop(); //nor flash spi stop

      if (buffer[0] == '{')
      {
        return SUCCESS;
      }
      else
      {
        return FAILURE;
      }
    }
  }

  Flash_Spi_Stop(); //nor flash spi stop

  *read_size = size;

  return FAILURE;
}

/*******************************************************************************
  w25q128 write data
*******************************************************************************/
static void w25q_WritePage(uint32_t addr, char *buffer, uint8_t Size, uint8_t end_flag)
{
  uint8_t i;

  w25q_WriteCommand(WRITE_ENABLE); //write enable

  Flash_Spi_Start(); //nor flash spi start

  SPI_Write(PAGE_PROGRAM); //page program code

  SPI_Write((uint8_t)(addr >> 16)); //24bit address first 8 bit address

  SPI_Write((uint8_t)(addr >> 8));

  SPI_Write((uint8_t)addr); //24bit address last 8 bit address

  for (i = 0; i < Size; i++)
  {
    SPI_Write(buffer[i]); //write data
  }

  if (end_flag)
  {
    SPI_Write((uint8_t)'!'); //write End Flag
    ESP_LOGI(TAG, "%d", __LINE__);
  }

  Flash_Spi_Stop(); //nor flash spi stop

  w25q_WaitCompleted(); //w25q128 wait command completed
}

/*******************************************************************************
  w25q128 write data
*******************************************************************************/
void w25q_WriteData(uint32_t addr, char *buffer, uint8_t Size)
{
  uint8_t remain;

  remain = 256 - addr % 256; //page remain byte number

  if ((Size + 1) <= remain) //can write completed in current page
  {
    w25q_WritePage(addr, buffer, Size, 1); //write completed in current page
  }
  else
  {
    w25q_WritePage(addr, buffer, remain, 0); //can not write completed in current page

    w25q_WritePage(addr + remain, buffer + remain, Size - remain, 1); //write completed in current page
  }
}

/*******************************************************************************
  w25q128 erase subsector-4k
*******************************************************************************/
void w25q_EraseSubsector(uint32_t addr)
{
  w25q_WriteCommand(WRITE_ENABLE); //write enable

  Flash_Spi_Start(); //nor flash spi start

  SPI_Write(SECTOR_ERASE); //subsector eaase code

  SPI_Write((uint8_t)((addr) >> 16)); //24bit address first 8 bit address

  SPI_Write((uint8_t)((addr) >> 8));

  SPI_Write((uint8_t)(addr)); //24bit address last 8 bit address

  Flash_Spi_Stop(); //nor flash spi stop

  w25q_WaitCompleted(); //wait command completed
}

/*******************************************************************************
  w25q128 erase chip
*******************************************************************************/
void w25q_EraseChip(void)
{
  w25q_WriteCommand(WRITE_ENABLE); //write enable

  w25q_WriteCommand(CHIP_ERASE); //chip eaase code

  w25q_WaitCompleted(); //wait command completed
}

/*******************************************************************************
  w25q128 power down mode
*******************************************************************************/
void w25q_PowerDown(void)
{
  w25q_WriteCommand(POWER_DOWN); //power down code

  MAP_UtilsDelay(400); //delay about 30us
}

/*******************************************************************************
  w25q128 wake up
*******************************************************************************/
void w25q_WakeUp(void)
{
  w25q_WriteCommand(RELEASE_POWER_DOWN); //release power down code

  MAP_UtilsDelay(400); //delay about 30us
}

/*******************************************************************************
  w25q128 read data
*******************************************************************************/
static void w25q_Read_Data(uint32_t addr, char *buffer, uint16_t size)
{
  uint16_t i;

  Flash_Spi_Start(); //nor flash spi start

  SPI_Write(READ_DATA); //read operations code

  SPI_Write((uint8_t)((addr) >> 16)); //24bit address first 8 bit address

  SPI_Write((uint8_t)((addr) >> 8));

  SPI_Write((uint8_t)(addr)); //24bit address last 8 bit address

  for (i = 0; i < size; i++)
  {
    buffer[i] = SPI_Read(); //read one byte
  }

  Flash_Spi_Stop(); //nor flash spi stop
}

/*******************************************************************************
  w25q128 write data no check
*******************************************************************************/
static void w25q_Write_Data(uint32_t addr, char *buffer, uint16_t Size)
{
  uint8_t i, n_i;
  uint16_t remain;

  n_i = Size / 256 + 2;

  remain = 256 - addr % 256; //page remain byte number

  remain = remain > Size ? Size : remain;

  for (i = 0; i <= n_i; i++)
  {
    if (remain > 0)
    {
      w25q_WritePage(addr, buffer, remain, 0); //no end flag
    }

    if (remain == Size)
    {
      break;
    }

    buffer += remain;

    addr += remain;

    Size -= remain;

    remain = Size > 256 ? 256 : Size;
  }
}

/*******************************************************************************
  w25q128 check write address with locked
*******************************************************************************/
void osi_w25q_Write_Addr_Check(unsigned long w_Addr)
{
  uint8_t i;
  uint16_t sec_data;
  uint16_t sec_remain;
  unsigned long sec_addr;
  char read_buf[SAVE_DATA_SIZE];

  sec_addr = (w_Addr / 4096) * 4096;

  sec_data = w_Addr % 4096;

  sec_remain = 4096 - sec_data;

  xSemaphoreTake(xMutex1, -1); //SPI Semaphore Take

  w25q_Read_Data(w_Addr, read_buf, SAVE_DATA_SIZE); //read the data write area

  xSemaphoreGive(xMutex1); //SPI Semaphore Give

  for (i = 0; i < SAVE_DATA_SIZE; i++)
  {
    if (read_buf[i] != 0xff) //need to delete
    {
      if (sec_data > 0)
      {
        xSemaphoreTake(xMutex5, -1); //Post_Data_Buffer Semaphore Take

        memset(Post_Data_Buffer, '\0', sizeof(Post_Data_Buffer)); //clear the data buffer

        xSemaphoreTake(xMutex1, -1); //SPI Semaphore Take

        w25q_Read_Data(sec_addr, Post_Data_Buffer, sec_data);

        w25q_EraseSubsector(sec_addr); //erase the subsector

        w25q_Write_Data(sec_addr, Post_Data_Buffer, sec_data);

        if (sec_remain < SAVE_DATA_SIZE)
        {
          w25q_EraseSubsector(sec_addr + 4096); //erase the subsector
        }

        xSemaphoreGive(xMutex1); //SPI Semaphore Give

        xSemaphoreGive(xMutex5); //Post_Data_Buffer Semaphore Give
      }
      else
      {
        xSemaphoreTake(xMutex1, -1); //SPI Semaphore Take

        w25q_EraseSubsector(sec_addr); //erase the subsector

        xSemaphoreGive(xMutex1); //SPI Semaphore Give
      }

      break;
    }
  }
}

/*******************************************************************************
                                      END         
*******************************************************************************/
