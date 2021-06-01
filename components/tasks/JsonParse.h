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
#include "freertos/FreeRTOS.h"

/*******************************************************************************
 FUNCTION PROTOTYPES
*******************************************************************************/
extern char ParseTimeData(char *ptr); //parse update time response data

extern int ParseJSONData(char *ptr); //parse post data response data

extern int ParseSetJSONData(char *ptr); //parse get set response data

extern void Read_Product_Set(char *read_buf, uint16_t data_len); //make product set data json

extern void Read_Wifi_Set(char *read_buf, uint16_t data_len); //make wifi set data json

extern void Cmd_Read_MetaData(char *read_buf, uint16_t data_len); //make meta data json

extern void Cmd_System_TestData(char *read_buf, uint16_t data_len); //make system test data json

extern int ParseTcpUartCmd(char *pcCmdBuffer); //parse tcp usrt get data

extern void read_product_url(char *url_buf);

extern void System_ERROR_Save(uint16_t save_addr); //save ERROR cjson data

extern void osi_System_ERROR_Save(uint16_t save_addr); //Save ERROR cjson data with locked

extern void Read_System_ERROR_Code(char *read_buf, uint16_t data_len); //Read System Error Code data

extern void Web_Wifi_Set(char *read_buf, uint8_t data_len, char *WiFi_ssid, short WiFi_rssi, uint8_t WiFi_type, char *WiFi_bssid, bool bssid_flag);

extern short Parse_HttpMsg(char *ptr);

extern void Read_cali(char *read_buf, uint16_t data_len);

/*******************************************************************************
                                      END         
*******************************************************************************/
