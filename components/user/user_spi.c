/*******************************************************************************
  * @file       SPI BUS DRIVER APPLICATION       
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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"

#include "user_spi.h"

#define PIN_NUM_CS 5
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK 18

static spi_device_handle_t g_spi;

void SET_SPI2_CS_ON(void)
{
  gpio_set_level(PIN_NUM_CS, 1);
}

void SET_SPI2_CS_OFF(void)
{
  gpio_set_level(PIN_NUM_CS, 0);
}

//SPI int
int VprocHALInit(void)
{
  /*if the customer platform requires any init
    * then implement such init here.
    * Otherwise the implementation of this function is complete
    */
  esp_err_t ret = ESP_OK;

  spi_bus_config_t buscfg = {
      .miso_io_num = PIN_NUM_MISO,
      .mosi_io_num = PIN_NUM_MOSI,
      .sclk_io_num = PIN_NUM_CLK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1};

  spi_device_interface_config_t devcfg = {
      .clock_speed_hz = 20 * 1000 * 1000, // Clock out at 10 MHz
      .mode = 0,                          // SPI mode 0
      .spics_io_num = -1,                 //GPIO_NUM_15,             // CS pin
      .queue_size = 6,                    //queue 7 transactions at a time
  };
  //Initialize the SPI bus
  if (g_spi)
  {
    return ret;
  }
  ret = spi_bus_initialize(HSPI_HOST, &buscfg, 0);
  assert(ret == ESP_OK);
  ret = spi_bus_add_device(HSPI_HOST, &devcfg, &g_spi);
  assert(ret == ESP_OK);
  gpio_set_pull_mode(PIN_NUM_CS, GPIO_FLOATING);
  return ret;
}

/*******************************************************************************
//spi interface init
*******************************************************************************/
void UserSpiInit(void)
{
  // MAP_PRCMPeripheralReset(PRCM_GSPI);  //Reset the peripheral

  // MAP_SPIReset(GSPI_BASE);  //Reset SPI

  // //Configure SPI interface
  // MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
  //                  SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_3,
  //                  (SPI_HW_CTRL_CS |
  //                  SPI_3PIN_MODE |
  //                  SPI_TURBO_OFF |
  //                  SPI_CS_ACTIVELOW |
  //                  SPI_WL_8));

  // SET_SPI1_CS_ON();  //adx345 spi cs high

  // SET_SPI2_CS_ON();  //n25q512a spi cs high

  // MAP_SPIEnable(GSPI_BASE);  //Enable SPI for communication

  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = (1ULL << PIN_NUM_CS);
  io_conf.pull_down_en = 0;
  io_conf.pull_up_en = 0;
  gpio_config(&io_conf);

  VprocHALInit();
}

/* This is the platform dependent low level spi
 * function to write 16-bit data to the ZL380xx device
 */
int SPI_Write(uint8_t val)
{
  /*Note: Implement this as per your platform*/
  esp_err_t ret;
  spi_transaction_t t;
  // unsigned short data = 0;
  memset(&t, 0, sizeof(t));       //Zero out the transaction
  t.length = sizeof(uint8_t) * 8; //Len is in bytes, transaction length is in bits.

  t.tx_buffer = &val;

  ret = spi_device_transmit(g_spi, &t); //Transmit
  assert(ret == ESP_OK);

  return 0;
}

/* This is the platform dependent low level spi
 * function to read 16-bit data from the ZL380xx device
 */
uint8_t SPI_Read(void)
{
  /*Note: Implement this as per your platform*/
  esp_err_t ret;
  spi_transaction_t t;
  // unsigned short data = 0xFFFF;
  uint8_t data1 = 0xFF;

  memset(&t, 0, sizeof(t)); //Zero out the transaction
  /*t.length = sizeof(unsigned short) * 8;
    t.rxlength = sizeof(unsigned short) * 8; //The unit of len is byte, and the unit of length is bit.
    t.rx_buffer = &data;*/
  t.length = sizeof(uint8_t) * 8;
  t.rxlength = sizeof(uint8_t) * 8; //The unit of len is byte, and the unit of length is bit.
  t.rx_buffer = &data1;
  ret = spi_device_transmit(g_spi, &t); //Transmit!
  assert(ret == ESP_OK);

  //*pVal = ntohs(data);

  return data1;
}

/*******************************************************************************
                                      END         
*******************************************************************************/
