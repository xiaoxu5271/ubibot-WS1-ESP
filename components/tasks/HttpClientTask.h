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
#include <http/client/httpcli.h>


/*******************************************************************************
 FUNCTION PROTOTYPES
*******************************************************************************/
extern void InitializeAppVariables(void);  //Initialize App Variables//

extern void Wlan_Disconnect_AP(void);  //wlan disconnect form the ap//

extern int ConfigureSimpleLinkToDefaultState(void);  //set the wifi unit in default state//

extern int ConfigureMode(int iMode);  //config the wifi Unit in AP mode//

extern int Scan_Wifi_List(char *rssi_ssid,int *rssi_val,bool uart_printf);  //Scan WIFI LIST//

extern short WlanConnect(void);  //Wlan Connect To The Accesspoint//

extern int ConnectToHTTPServer(HTTPCli_Handle httpClient);  //Connect to Http Server//

extern int HTTPGetMethod(HTTPCli_Handle httpClient);  //HTTP GET METHOD//

extern int HTTPPostMethod(HTTPCli_Handle httpClient,unsigned long DataLen,uint16_t Amount);  //HTTP POST Demonstration//

extern short Http_Get_Function(void);  //HTTP Get Form Server

extern void PostAddrChage(unsigned long data_num,unsigned long end_addr);  //deleted the post data//

extern void DataPostTask(void *pvParameters);  //data post task//

extern void osi_Scan_Wifi_List(char *rssi_ssid,int *rssi_val,bool uart_printf);  //Scan WIFI LIST with locked//

extern short WiFi_Connect_Test(void);  //WiFi connect test

extern short osi_WiFi_Connect_Test(void);  //WiFi connect test with locked


/*******************************************************************************
                                      END         
*******************************************************************************/




