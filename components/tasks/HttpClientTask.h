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
#include "esp_http_client.h"
/*******************************************************************************
 FUNCTION PROTOTYPES
*******************************************************************************/
// extern void InitializeAppVariables(void); //Initialize App Variables//

extern int HTTPGetMethod(esp_http_client_handle_t httpClient); //HTTP GET METHOD//

extern int HTTPPostMethod(esp_http_client_handle_t httpClient, unsigned long DataLen, uint16_t Amount); //HTTP POST Demonstration//

extern short Http_Get_Function(void); //HTTP Get Form Server

extern void PostAddrChage(unsigned long data_num, unsigned long end_addr); //deleted the post data//

extern void DataPostTask(void *pvParameters); //data post task//

/*******************************************************************************
                                      END         
*******************************************************************************/
