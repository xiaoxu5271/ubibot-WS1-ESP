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
#include "stdlib.h"
#include "stdbool.h"
#include "math.h"
#include "osi.h"
#include "string.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "common.h"
#include "utils.h"
#include "rom_map.h"
#include "gpio.h"
#include "uart_if.h"
#include "simplelink.h"
#include <http/client/httpcli.h>
#include <http/client/common.h>
#include "MsgType.h"
#include "at24c08.h"
#include "w25q128.h"
#include "JsonParse.h"
#include "PeripheralDriver.h"
#include "iic.h"
#include "base64.h"

extern  uint8_t Host_Flag;
extern uint8_t HOST_IP[4];
Sl_WlanNetworkEntry_t netEntries[20];   // Wlan Network Entry

extern OsiMsgQ_t        xQueue0;        //Used for cjson and memory save

extern OsiSyncObj_t     xBinary0;       //For DataPostTask interrupt
extern OsiSyncObj_t     xBinary11;      //For Memory Delete Task
extern OsiSyncObj_t     xBinary13;      //Task end check
extern OsiSyncObj_t     xBinary16;      //WiFi List Scan Task
extern OsiSyncObj_t     xBinary17;      //Internet Application

extern OsiSyncObj_t     xMutex2;        //Used for SimpleLink Lock
extern OsiSyncObj_t     xMutex3;        //Used for cJSON Lock
extern OsiSyncObj_t     xMutex4;        //Used for UART Lock
extern OsiSyncObj_t     xMutex5;        //Used for Post_Data_Buffer Lock

extern char SEC_TYPE[8];
extern char SSID_NAME[32];
extern char PASS_WORD[64];
extern char HOST_NAME[64];
extern char POST_REQUEST_URI[250];
extern char Post_Data_Buffer[4096];
extern char Read_Response_Buffer[RESP_BUF_LEN];
char wifi_buf[180];
char calidata_buf[512]={0};

extern volatile uint16_t sys_run_time;  
extern volatile bool PostSetData;      //post url data
extern volatile bool update_time;      //update time
extern volatile bool POST_TASK_END_FLAG;
extern volatile uint8_t cg_data_led;    //Led ON or OFF when data post
extern volatile uint8_t no_net_fn;      //change fn_dp when no net
extern volatile uint8_t wifi_mode;      //1:connect wifi immediately,0:connect wifi when scaned
extern volatile uint8_t save_addr_flag;
extern volatile unsigned long fn_dp;  //data post frequence
extern volatile bool usb_status_val;  //usb status

extern volatile unsigned long   g_ulStatus;                             //SimpleLink Status
extern unsigned long            g_ulStaIp;
extern unsigned long            g_ulGatewayIP;                          //Network Gateway IP address
extern unsigned long            g_ulDestinationIP;                      //IP address of destination server
extern unsigned char            g_ucConnectionSSID[SSID_LEN_MAX+1];     //Connection SSID
extern unsigned char            g_ucConnectionBSSID[BSSID_LEN_MAX];     //Connection BSSID
//extern unsigned char            g_buff[MAX_BUFF_SIZE+1];
extern float f5_a,f5_b;
extern volatile unsigned long POST_NUM;
extern volatile unsigned long DELETE_ADDR,POST_ADDR,WRITE_ADDR;

#define WLAN_SCAN_COUNT         20
#define ERROR_CODE              0xffff

/*******************************************************************************
//Initialize App Variables
*******************************************************************************/
void InitializeAppVariables(void)
{
  g_ulStatus = 0;
  g_ulStaIp = 0;
  g_ulGatewayIP = 0;
  memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
  memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
}

/*******************************************************************************
//check ssid include "
*******************************************************************************/
//static bool ssid_schar_check(void)
//{
//  char *rst_val;
//  rst_val=strstr((const char*)SSID_NAME,"\"");  //include "
//  if(rst_val==NULL)
//  {
//    return 0;
//  }
//  else
//  {
//    return 1;
//  }
//}

/*******************************************************************************
//wlan disconnect form the ap
*******************************************************************************/
void Wlan_Disconnect_AP(void)
{
  uint16_t time_out=0;
    
  if(sl_WlanDisconnect()==0)
  {
    while(IS_CONNECTED(g_ulStatus))  // Wait
    {
      if(time_out++ > 2000)
      {
        break;
      }
      MAP_UtilsDelay(20000);  //delay about 1.5ms
    }
  } 
}

/*******************************************************************************
//set the wifi unit in default state 
*******************************************************************************/
int ConfigureSimpleLinkToDefaultState(void)
{
  long lRetVal = -1;
  unsigned char ucPower = 0;
  unsigned char ucConfigOpt = 0;
  _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};
  
  lRetVal = sl_Start(0, 0, 0);  //start the simple link
  if (ROLE_STA != lRetVal)  //If the device is not in station-mode, try configuring it in station-mode
  {
    if (ROLE_AP == lRetVal)
    {
      while(!IS_IP_ACQUIRED(g_ulStatus))  //wait for AP mode end
      {
        if(lRetVal++>2000)  //3s time out
        {
          break;
        }
        MAP_UtilsDelay(20000);  //delay about 1.5ms
      }
    }
    sl_WlanSetMode(ROLE_STA);  //Switch to STA role and restart
 
    sl_Stop(SL_STOP_TIMEOUT);  //stop the simple link
    
    lRetVal = sl_Start(0, 0, 0);  //start the simple link
    
    if (ROLE_STA != lRetVal)  //Check if the device is in station again
    {
      sl_Stop(SL_STOP_TIMEOUT);  //stop the simple link
      
      return FAILURE;
    }
  } 
  
  #ifdef DEBUG
  
    SlVersionFull ver = {0};
    unsigned char ucConfigLen = 0;
    ucConfigOpt = SL_DEVICE_GENERAL_VERSION;  //Get the device's version-information
    ucConfigLen = sizeof(ver);
    
    sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &ucConfigOpt, 
                                &ucConfigLen, (unsigned char *)(&ver));
    
    osi_SyncObjWait(&xMutex4,OSI_WAIT_FOREVER);  //UART Semaphore Take
    
    UART_PRINT("Host Driver Version: %s\r\n",SL_DRIVER_VERSION);
    
    UART_PRINT("Build Version %d.%d.%d.%d.31.%d.%d.%d.%d.%d.%d.%d.%d\r\n",
    ver.NwpVersion[0],ver.NwpVersion[1],ver.NwpVersion[2],ver.NwpVersion[3],
    ver.ChipFwAndPhyVersion.FwVersion[0],ver.ChipFwAndPhyVersion.FwVersion[1],
    ver.ChipFwAndPhyVersion.FwVersion[2],ver.ChipFwAndPhyVersion.FwVersion[3],
    ver.ChipFwAndPhyVersion.PhyVersion[0],ver.ChipFwAndPhyVersion.PhyVersion[1],
    ver.ChipFwAndPhyVersion.PhyVersion[2],ver.ChipFwAndPhyVersion.PhyVersion[3]);
    
    osi_SyncObjSignal(&xMutex4);  //UART Semaphore Give
    
  #endif
    
/*
  sl_WlanPolicySet(SL_POLICY_CONNECTION, 
                              SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);  //Device's default connection policy,Set connection policy to Auto + SmartConfig//
*/
    
  sl_WlanProfileDel(0xFF);  //Remove all profiles

  Wlan_Disconnect_AP();  //wlan disconnect form the ap

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
  
  
  ucConfigOpt = SL_SCAN_POLICY(0);  //Disable scan
  sl_WlanPolicySet(SL_POLICY_SCAN , ucConfigOpt, NULL, 0);

  ucPower = 0;  //Set Tx power level for station mode
  sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, 
          WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&ucPower);
  
  sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY , NULL, 0);  //Set PM policy to normal
  
  sl_NetAppMDNSUnRegisterService(0, 0);  //Unregister mDNS services

  memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);  
  sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                     sizeof(_WlanRxFilterOperationCommandBuff_t));  //Remove  all 64 filters (8*8)

  sl_Stop(SL_STOP_TIMEOUT);  //stop the simple link
  
  MAP_UtilsDelay(80000);  //delay about 6ms
  
  InitializeAppVariables();  //init staus and ssid password
  
  return SUCCESS;
}

/*******************************************************************************
//config the wifi Unit in AP mode
*******************************************************************************/
int ConfigureMode(int iMode)
{
  int lRetVal = -1;
  char api_key_byte[5];
  char pcSsidName[16]="Ubibot-";
  //char  password[64]="12345678";
  unsigned char channel;
  SlNetCfgIpV4Args_t ipV4;
  
  //
  //Set AP IP params//
  //
  ipV4.ipV4 = SL_IPV4_VAL(192,168,1,1);
  ipV4.ipV4Mask = SL_IPV4_VAL(255,255,255,0);
  ipV4.ipV4Gateway = SL_IPV4_VAL(192,168,1,1);
  ipV4.ipV4DnsServer = SL_IPV4_VAL(192,168,1,1);
  
  sl_NetCfgSet(SL_IPV4_AP_P2P_GO_STATIC_ENABLE,IPCONFIG_MODE_ENABLE_IPV4,sizeof(SlNetCfgIpV4Args_t),(unsigned char *)&ipV4);  
  
  osi_at24c08_ReadData(SERISE_NUM_ADDR,(uint8_t*)api_key_byte,sizeof(api_key_byte),0);  //read wifi password
  
  strncpy(pcSsidName+strlen(pcSsidName),api_key_byte,sizeof(api_key_byte));
  
  lRetVal = sl_WlanSetMode(ROLE_AP);
  if(lRetVal<0)
  {
    #ifdef DEBUG
    
      osi_UartPrint("Device can not configured in AP mode\r\n");
      
    #endif
      
    return FAILURE;
  }
  
  sl_WlanSet(SL_WLAN_CFG_AP_ID, WLAN_AP_OPT_SSID, strlen(pcSsidName),(unsigned char*)pcSsidName);  //set the ssid name

  _u8 val = SL_SEC_TYPE_OPEN;  //SL_SEC_TYPE_WPA
  sl_WlanSet(SL_WLAN_CFG_AP_ID, WLAN_AP_OPT_SECURITY_TYPE, 1, (_u8 *)&val);  //set SEC type
  
  //sl_WlanSet(SL_WLAN_CFG_AP_ID, WLAN_AP_OPT_PASSWORD, strlen(password),(unsigned char*)password);
  
  channel=6;
  sl_WlanSet(SL_WLAN_CFG_AP_ID, WLAN_AP_OPT_CHANNEL, 1, (unsigned char*)&channel);  //Set AP channel

  lRetVal = sl_Stop(SL_STOP_TIMEOUT);   //Restart Network processor
  
  CLR_STATUS_BIT_ALL(g_ulStatus);  //reset status bits
  
  MAP_UtilsDelay(80000);  //delay about 100ms

  return sl_Start(NULL,NULL,NULL);  //start simple link
}

/*******************************************************************************
//Check the device mode and switch to STATION(STA) mode
*******************************************************************************/
static void SwitchToStaMode(int iMode)
{
  uint8_t err_code;
  
  if(iMode != ROLE_STA)
  {
    for(err_code=0;err_code>=RETRY_TIME_OUT;err_code++)
    {
      sl_WlanSetMode(ROLE_STA);
      
      MAP_UtilsDelay(80000);
      
      sl_Stop(SL_STOP_TIMEOUT);  //stop the simple link
      
      MAP_UtilsDelay(80000);
      
      if(sl_Start(0,0,0) == ROLE_STA)
      {
        break;
      }
    }
  }
}

/*******************************************************************************
//Scan WIFI LIST
//return:rssi value
*******************************************************************************/
int Scan_Wifi_List(char *rssi_ssid,int *rssi_val,bool uart_printf)
{
  uint8_t i;
  uint8_t read_len;
  int lRetVal = -1;
  int Wifi_List_Val=-127;
  char scan_buf[96];
  unsigned short ucIndex;
  unsigned char ucpolicyOpt;
  char utctime[21]={0};
  char bssid_buf[5][18]={0};
  signed char bssid_rssi_val[5]={0};
  
  union
  {
    unsigned char ucPolicy[4];
    unsigned int uiPolicyLen;
  }policyVal;

  mem_set(SSID_NAME,0,sizeof(SSID_NAME));
  
  read_len = osi_at24c08_read_byte(SSID_LEN_ADDR);
  
  if(read_len>sizeof(SSID_NAME))
  {
    read_len =sizeof(SSID_NAME);
  }
  
  osi_at24c08_ReadData(SSID_ADDR,(uint8_t*)SSID_NAME,read_len,0);  //Read Wifi SSID
  
//  InitializeAppVariables();  //Initializes the variables
  
//  ConfigureSimpleLinkToDefaultState();
  
  lRetVal=sl_Start(0, 0, 0);  //start the simple link
  if(lRetVal<0)
  {
    #ifdef DEBUG 
    
      osi_UartPrint("Failed to start the device \r\n"); 
      
    #endif 
  }
  
  SwitchToStaMode(lRetVal);  //Check Device In STA Mode or Not
  
  sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID,WLAN_GENERAL_PARAM_OPT_COUNTRY_CODE, 2,(unsigned char *)"EU");
    
  //make sure the connection policy is not set,no scan is run in the background
  ucpolicyOpt = SL_CONNECTION_POLICY(0, 0, 0, 0,0);  
  lRetVal = sl_WlanPolicySet(SL_POLICY_CONNECTION , ucpolicyOpt, NULL, 0);
  if(lRetVal != 0)
  {
    #ifdef DEBUG
    
      osi_UartPrint("Unable to clear the Connection Policy\r\n");
      
    #endif
  }
  
  ucpolicyOpt = SL_SCAN_POLICY(1);  //enable scan
  
  policyVal.uiPolicyLen = 10;  //set scan cycle to 10 seconds
  
  //set scan policy - this starts the scan
  lRetVal = sl_WlanPolicySet(SL_POLICY_SCAN , ucpolicyOpt,(unsigned char*)(policyVal.ucPolicy), sizeof(policyVal));
  if(lRetVal!=0)
  {
    #ifdef DEBUG
    
      osi_UartPrint("Unable to set the Scan Policy\r\n");
      
    #endif
  }
  
  MAP_UtilsDelay(20000000);  //delay 1.5s
  
  ucIndex = 0;// get scan results - all 20 entries in one transaction
  
  memset(netEntries, 0, sizeof(netEntries));
  
  //retVal indicates the valid number of entries,The scan results are occupied in netEntries[]
  lRetVal = sl_WlanGetNetworkList(ucIndex, (unsigned char)WLAN_SCAN_COUNT,&netEntries[ucIndex]);
  if(uart_printf)
  {
    osi_SyncObjWait(&xMutex4,OSI_WAIT_FOREVER);  //UART Semaphore Take
    
    UART_PRINT("{\"WiFi_List_Sum\":%d}\r\n",lRetVal);
    
    osi_SyncObjSignal(&xMutex4);  //UART Semaphore Give
  }
  
  if(lRetVal>0)
  {
    *rssi_val=-127;
    
    for(i=0;i<lRetVal;i++)
    {
      if(uart_printf)
      {
        memset(scan_buf,0,sizeof(scan_buf));
        
        Web_Wifi_Set(scan_buf,sizeof(scan_buf),(char *)netEntries[i].ssid,netEntries[i].rssi,netEntries[i].sec_type,(char *)netEntries[i].bssid,1);
        
        osi_SyncObjWait(&xMutex4,OSI_WAIT_FOREVER);  //UART Semaphore Take
        
        UART_PRINT("%s\r\n",scan_buf);
//        UART_PRINT("{\"SSID\":\"%s\",\"rssi\":%d,\"type\":%d}\r\n",netEntries[i].ssid,netEntries[i].rssi,netEntries[i].sec_type);
        
        osi_SyncObjSignal(&xMutex4);  //UART Semaphore Give
      }
      
      if(i<5)
      {
        snprintf(bssid_buf[i],18,"%02x:%02x:%02x:%02x:%02x:%02x",netEntries[i].bssid[0],netEntries[i].bssid[1],netEntries[i].bssid[2],netEntries[i].bssid[3],netEntries[i].bssid[4],netEntries[i].bssid[5]);
        
        bssid_rssi_val[i] = netEntries[i].rssi;
      }
      
      if(!strcmp((char const*)netEntries[i].ssid,(char const*)SSID_NAME))
      { 
        mem_set(SEC_TYPE,0,sizeof(SEC_TYPE));
        
        if(netEntries[i].sec_type==0)
        {
          mem_copy(SEC_TYPE,"OPEN",strlen("OPEN"));  //WiFi sec type
        }
        else if(netEntries[i].sec_type==1)
        {
          mem_copy(SEC_TYPE,"WEP",strlen("WEP"));  //WiFi sec type
        }
        else
        {
          mem_copy(SEC_TYPE,"WPA",strlen("WPA"));  //WiFi sec type
        }
        
        if(Wifi_List_Val < netEntries[i].rssi)
        {
          Wifi_List_Val=netEntries[i].rssi;
        }
      }
      if(*rssi_val<netEntries[i].rssi)
      {
        *rssi_val=netEntries[i].rssi;
        
        mem_set(rssi_ssid,0,32);
        
        mem_copy(rssi_ssid,netEntries[i].ssid,strlen((char *)netEntries[i].ssid));
      }
    }
    osi_Read_UTCtime(utctime,sizeof(utctime));  //read time
    
    memset(wifi_buf,0,sizeof(wifi_buf));
    
    snprintf(wifi_buf,sizeof(wifi_buf),",{\"created_at\":\"%s\",\"wifi\":\"%s,%d;%s,%d;%s,%d;%s,%d;%s,%d\"}",utctime,bssid_buf[0],bssid_rssi_val[0],bssid_buf[1],bssid_rssi_val[1],bssid_buf[2],bssid_rssi_val[2],bssid_buf[3],bssid_rssi_val[3],bssid_buf[4],bssid_rssi_val[4]);
    
    #ifdef DEBUG
      osi_UartPrint(wifi_buf);
      osi_UartPrint("\r\n"); 
    #endif
  }
  else
  {
    osi_Read_UTCtime(utctime,sizeof(utctime));  //read time
    
    memset(wifi_buf,0,sizeof(wifi_buf));
    
    snprintf(wifi_buf,sizeof(wifi_buf),",{\"created_at\":\"%s\",\"wifi\":\"\"}",utctime);
    
    #ifdef DEBUG
      osi_UartPrint(wifi_buf);
      osi_UartPrint("\r\n"); 
    #endif
      
    #ifdef DEBUG
    
      osi_UartPrint("Unable to retreive the network list\r\n");
      
    #endif
  }
  
  ucpolicyOpt = SL_SCAN_POLICY(0);  //disable scan
  
  lRetVal = sl_WlanPolicySet(SL_POLICY_SCAN , ucpolicyOpt, NULL, 0);
  if(lRetVal != 0)
  {
    #ifdef DEBUG
    
      osi_UartPrint("Unable to Clear the Scan Policy\r\n");
      
    #endif
  }
  
  sl_Stop(SL_STOP_TIMEOUT);  //stop the simple link
  
  MAP_UtilsDelay(80000);  //delay about 6ms
  
  if((Wifi_List_Val<0)&&(Wifi_List_Val>-100))
  {
    return Wifi_List_Val;
  }
  else
  {
    osi_System_ERROR_Save(SCN_WF_FLR_ERR);  //save ERROR data
    
    return ERROR_CODE;
  }
}

/*******************************************************************************
//Wlan Connect To The Accesspoint
*******************************************************************************/
int WlanConnect(void)
{
  uint8_t read_len;
  int lRetVal=-1;
  unsigned char sectype;
  SlSecParams_t secParams = {0};

  mem_set(SSID_NAME,0,sizeof(SSID_NAME));
  
  mem_set(PASS_WORD,0,sizeof(PASS_WORD));
  
  read_len = osi_at24c08_read_byte(SSID_LEN_ADDR);
    
  if(read_len>sizeof(SSID_NAME))
  {
    read_len =sizeof(SSID_NAME);
  }
  
  osi_at24c08_ReadData(SSID_ADDR,(uint8_t*)SSID_NAME,read_len,0);  //Read Wifi SSID
  
  read_len = osi_at24c08_read_byte(PASSWORD_LEN_ADDR);
  
  if(read_len>sizeof(PASS_WORD))
  {
    read_len = sizeof(PASS_WORD);
  }
  
  osi_at24c08_ReadData(PASSWORD_ADDR,(uint8_t*)PASS_WORD,read_len,0);  //Read Wifi Password
  
  //osi_at24c08_ReadData(SECTYPE_ADDR,(uint8_t*)SEC_TYPE,sizeof(SEC_TYPE),1);  //Read Wifi Sectype

  if(!strcmp((char const*)SEC_TYPE,"OPEN"))
  {
    sectype=SL_SEC_TYPE_OPEN;
    secParams.Key = NULL;
    secParams.KeyLen =0;
    secParams.Type = sectype;
  }
  else if(!strcmp((char const*)SEC_TYPE,"WEP"))
  {
    sectype=SL_SEC_TYPE_WEP;
    secParams.Key = (signed char *)PASS_WORD;
    secParams.KeyLen =strlen(PASS_WORD);
    secParams.Type = sectype;
  }
  else 
  {
    if(strcmp((char const*)SEC_TYPE,"WPA"))
    {
      mem_copy(SEC_TYPE,"WPA",strlen("WPA"));  //WiFi sec type
    }
    
    sectype=SL_SEC_TYPE_WPA;
    secParams.Key = (signed char *)PASS_WORD;
    secParams.KeyLen =strlen(PASS_WORD);
    secParams.Type = sectype;
  }
  
//  InitializeAppVariables();  //Initializes the variables
  
  ConfigureSimpleLinkToDefaultState();
  
  lRetVal=sl_Start(0, 0, 0);  //start the simple link
  if(lRetVal<0)
  {
    #ifdef DEBUG
    
      osi_UartPrint("Failed to start the device \r\n");
      
    #endif
  }
  
  SwitchToStaMode(lRetVal);  //Check Device In STA Mode or Not
  
  sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID,WLAN_GENERAL_PARAM_OPT_COUNTRY_CODE, 2,(unsigned char *)"EU");
  
  sl_WlanConnect((signed char *)SSID_NAME,
                         strlen((const char *)SSID_NAME), 0, &secParams, 0);
  
  //sl_WlanProfileAdd((signed char*)SSID_NAME, strlen(SSID_NAME), 0, &secParams, 0,7,NULL);  //Add the wifi connect profile
  
  //sl_WlanPolicySet(SL_POLICY_CONNECTION,SL_CONNECTION_POLICY(1,0,0,0,0),NULL,1);  //reset to default AUTO policy
  
  lRetVal=0;
  
  while((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus)))   //Wait for WLAN Event
  {
    lRetVal++;
    
    if(lRetVal >= 6000)  //18s time out
    {
      #ifdef USE_LCD      

        osi_Ht1621_Display_Err_Val(0x02);
     
      #endif
      
      osi_System_ERROR_Save(CNT_WIFI_FLR_ERR);  //save ERROR data
        
      return FAILURE;
    } 
    
    MAP_UtilsDelay(40000);  //Delay About 3ms
  } 
  
  return SUCCESS;
}

/*******************************************************************************
//This function read respose from server and dump on console
*******************************************************************************/
static int readResponse(HTTPCli_Handle httpClient)
{
  long lRetVal = 0;
  int bytesRead = 0;
  bool moreFlags = 1;
  const char *ids[4] = {
                          HTTPCli_FIELD_NAME_CONTENT_LENGTH,
                          HTTPCli_FIELD_NAME_CONNECTION,
                          HTTPCli_FIELD_NAME_CONTENT_TYPE,
                          NULL
                       };
  
  lRetVal = HTTPCli_getResponseStatus(httpClient);  //read HTTP POST request status code
  
#ifdef DEBUG
        
  osi_UartPrint_Val("HTTP Status:",lRetVal);
          
#endif
          
  if(lRetVal > 0)
  {
    HTTPCli_setResponseFields(httpClient, (const char **)ids);      //Set response header fields to filter response headers

//    while(HTTPCli_getResponseField(httpClient, (char *)g_buff, sizeof(g_buff), &moreFlags)!= HTTPCli_FIELD_ID_END)  //Read filter response header and take appropriate action
    while(HTTPCli_getResponseField(httpClient, (char *)Read_Response_Buffer, sizeof(Read_Response_Buffer), &moreFlags)!= HTTPCli_FIELD_ID_END)  //Read filter response header and take appropriate action
    {
      if(bytesRead++ >= RESP_BUF_LEN)
      {
        break;
      }
    }

//    mem_copy(Read_Response_Buffer,g_buff,sizeof(Read_Response_Buffer));
//#ifdef DEBUG 
//    uint16_t i;
//    osi_UartPrint("Read_Response_Buffer:");
//    for(i=0;i<RESP_BUF_LEN;i++)
//    {
//      UART_PRINT("%c",Read_Response_Buffer[i]);
//      MAP_UtilsDelay(200);  //Delay About 15us
//    }
//    osi_UartPrint("\r\n");
//#endif
#ifdef DEBUG 
    UART_PRINT("state:%d\r\n",httpClient->state);
    UART_PRINT("clen:%d\r\n",httpClient->clen);
    UART_PRINT("buf:%s\r\n",httpClient->buf);
    UART_PRINT("buflen:%d\r\n",httpClient->buflen);
    UART_PRINT("bufptr:%s\r\n",httpClient->bufptr);
#endif
    bytesRead = HTTPCli_readResponseBody(httpClient, (char *)Read_Response_Buffer, sizeof(Read_Response_Buffer), &moreFlags);
//#ifdef DEBUG      
//    osi_UartPrint("Read_Response_Buffer:");
//    for(i=0;i<1024;i++)
//    {
//      UART_PRINT("%c",Read_Response_Buffer[i]);
//      MAP_UtilsDelay(200);  //Delay About 15us
//    }
//    osi_UartPrint("\r\n");
//#endif
#ifdef DEBUG    
    osi_UartPrint_Val("BytesRead",bytesRead); 
    if(bytesRead <0)
    { 
      osi_UartPrint("Failed to received response body\r\n");
    }     
#endif 
  
    Read_Response_Buffer[bytesRead] = '\0';
  
#ifdef DEBUG      
    osi_UartPrint(Read_Response_Buffer);      
#endif
          
    osi_SyncObjWait(&xMutex3,OSI_WAIT_FOREVER);             //cJSON Semaphore Take
    
    switch(lRetVal)
    {    
        case 200:
        {
          if(PostSetData)
          {
            lRetVal = ParseSetJSONData(Read_Response_Buffer);     //Api Key
          }
          else if(update_time)
          {
            lRetVal = ParseTimeData(Read_Response_Buffer);        //Update Time
          }
          else
          {
            lRetVal = ParseJSONData(Read_Response_Buffer);        //Sensor Data               
          }
        }
        break;    
  /*
        case 400:
        {
          ParseTimeData(Read_Response_Buffer);  //Update Time
        }
        break;
        case 401:
        {
          ParseTimeData(Read_Response_Buffer);  //Update Time
        }
        break;
        case 402:
        {
          ParseTimeData(Read_Response_Buffer);  //Update Time
        }
        break;
        case 410:
        {
          ParseTimeData(Read_Response_Buffer);  //Update Time
        }
        break;    
  */
        default:
        {
          ParseTimeData(Read_Response_Buffer);  //Update Time
        }
        break;
    }
    osi_SyncObjSignal(&xMutex3);  //cJSON Semaphore Give
  }
  else
  {
  #ifdef DEBUG

    osi_UartPrint("Failed to receive data from server!\r\n");
    
  #endif
  }
  
  return lRetVal;
}

/*******************************************************************************
 Connect to Http Server
*******************************************************************************/
int ConnectToHTTPServer(HTTPCli_Handle httpClient)
{
  bool dns_fault=0;
  long lRetVal = -1;
  struct sockaddr_in addr; 
  
  osi_at24c08_ReadData(HOST_ADDR,(uint8_t*)HOST_NAME,sizeof(HOST_NAME),1);  //read the host name 

  Host_Flag = osi_at24c08_read_byte(HOST_NAME_IP_ADDR);
  if(Host_Flag == 0)
  {
    lRetVal = sl_NetAppDnsGetHostByName((signed char *)HOST_NAME,strlen((const char *)HOST_NAME),&g_ulDestinationIP,SL_AF_INET);  //Resolve HOST NAME/IP
    if(lRetVal < 0)
    {
      #ifdef DEBUG
  
        osi_UartPrint("Resolve HOST NAME/IP failued\r\n");
  
      #endif
     
      osi_System_ERROR_Save(RSL_HOST_FLR_ERR);  //save ERROR data
      
      #ifdef USE_LCD   
  
        osi_Ht1621_Display_Err_Val(0x03);
  
      #endif

/*
      if(lRetVal == DNS_WRONG_NUM)
      {
        #ifdef DEBUG
      
          osi_UartPrint("DNS fault\r\n");
          
        #endif
      }
*/
      dns_fault = 1;
      
      osi_at24c08_ReadData(HOST_IP_ADDR,HOST_IP,sizeof(HOST_IP),1);
  
      g_ulDestinationIP = SL_IPV4_VAL(HOST_IP[0],HOST_IP[1],HOST_IP[2],HOST_IP[3]);
    }
  }
  else
  { 
    #ifdef DEBUG
    
        osi_UartPrint("Use Host IP\r\n");
        
    #endif
        
    osi_at24c08_ReadData(HOST_IP_ADDR,HOST_IP,sizeof(HOST_IP),1);
    
    g_ulDestinationIP = SL_IPV4_VAL(HOST_IP[0],HOST_IP[1],HOST_IP[2],HOST_IP[3]);
  }
    
  uint32_t host_port_num = osi_at24c08_read(HOST_PORT_ADDR); 
  addr.sin_family = AF_INET;
  addr.sin_port = htons(host_port_num);
  addr.sin_addr.s_addr = sl_Htonl(g_ulDestinationIP);
  
  HTTPCli_construct(httpClient);  //Set up the input parameters for HTTP Connection
  
  lRetVal = HTTPCli_connect(httpClient, (struct sockaddr *)&addr, 0, NULL);
  if (lRetVal < 0)
  {
    #ifdef DEBUG
    
      osi_UartPrint("Connection to server failued.\r\n");
      
    #endif
      
    #ifdef USE_LCD  
      
      osi_Ht1621_Display_Err_Val(0x04);
      
    #endif
      
    if(Host_Flag == 0)
    {
      Host_Flag = 0XFF;
      
      if(no_net_fn == 1)
      {
        if(fn_dp < NO_NET_FN_DP)
        {
          unsigned long fn_dp_x = fn_dp;
          
          fn_dp = 2*fn_dp_x;
          
          if(fn_dp > NO_NET_FN_DP)
          {
            fn_dp = NO_NET_FN_DP;
          }
          
          osi_at24c08_write(FN_DP_ADDR,fn_dp);  //fn_dp
        }
      }
    }
    else
    {
      Host_Flag = 0;
    }
    
    osi_at24c08_write_byte(HOST_NAME_IP_ADDR,Host_Flag);

    return FAILURE;
  }    
  else
  {
    #ifdef DEBUG
    
      osi_UartPrint("Connection to server successfully\r\n");
      
    #endif

    if(dns_fault== 1)
    {
      Host_Flag = 0XFF;
      
      osi_at24c08_write_byte(HOST_NAME_IP_ADDR,Host_Flag);
    }

    return SUCCESS;
  }
}

/*******************************************************************************
  HTTP GET METHOD
*******************************************************************************/
int HTTPGetMethod(HTTPCli_Handle httpClient)
{
  bool moreFlags;
  int lRetVal = 0;
  
  //osi_at24c08_ReadData(HOST_ADDR,(uint8_t*)HOST_NAME,sizeof(HOST_NAME),1);  //read host
  
  memset(POST_REQUEST_URI,0,sizeof(POST_REQUEST_URI));
  
  if(PostSetData)
  {
    read_product_url(POST_REQUEST_URI);
  }
  else if(update_time)
  {
    mem_copy(POST_REQUEST_URI,"/utilities/time",strlen("/utilities/time"));
  }
  
  HTTPCli_Field fields[4] = {
                              {HTTPCli_FIELD_NAME_HOST, HOST_NAME},
                              {HTTPCli_FIELD_NAME_ACCEPT, "*/*"},
                              {HTTPCli_FIELD_NAME_CONTENT_LENGTH, "0"},
                              {NULL, NULL}
                          };

  HTTPCli_setRequestFields(httpClient, fields);  //Set request header fields to be send for HTTP request

  moreFlags = 0;
  
  lRetVal = HTTPCli_sendRequest(httpClient, HTTPCli_METHOD_GET, POST_REQUEST_URI, moreFlags);  //Send GET method request
  if(lRetVal < 0)
  {
    #ifdef DEBUG
    
      osi_UartPrint("Failed to send HTTP GET request.\r\n");
      
    #endif
    
    return lRetVal;
  }
  
  lRetVal = readResponse(httpClient);
  
  return lRetVal;
}

/*******************************************************************************
  HTTP POST Demonstration
*******************************************************************************/
int HTTPPostMethod(HTTPCli_Handle httpClient,unsigned long DataLen,uint16_t Post_Amount)
{
  uint8_t url_len = 0;
  char tmpBuf[8]={0};
  long lRetVal = 0;
  bool EndFlag = 0;
  bool lastFlag = 1;
  bool moreFlags = 1;
  uint16_t post_data_sum;
  unsigned long MemoryAddr;
  char status_buf[128];
  char mac_buf[18]={0};
  uint8_t mac_addr[8]={0};
  char base64_ssid[48] = {0};
  
  memset(status_buf,0,sizeof(status_buf));
  
  osi_at24c08_ReadData(MAC_ADDR,mac_addr,SL_MAC_ADDR_LEN,0);
  
  snprintf(mac_buf,sizeof(mac_buf),"%02x:%02x:%02x:%02x:%02x:%02x",mac_addr[0],mac_addr[1],mac_addr[2],mac_addr[3],mac_addr[4],mac_addr[5]);
  
  memset(POST_REQUEST_URI,0,sizeof(POST_REQUEST_URI));
  //osi_at24c08_ReadData(HOST_ADDR,(uint8_t*)HOST_NAME,sizeof(HOST_NAME),1);  //read host name
  
  osi_at24c08_ReadData(DATAURI_ADDR,(uint8_t*)POST_REQUEST_URI,sizeof(POST_REQUEST_URI),1);  //read DATA-URI
  
  url_len = strlen(POST_REQUEST_URI);
  
//  if(ssid_schar_check())
//  {
//    snprintf(status_buf,sizeof(status_buf),"],\"status\":\"mac=%s,usb=%d\"}",mac_buf,usb_status_val);  //MAC USB 
//  }
//  else
//  {
//    snprintf(status_buf,sizeof(status_buf),"],\"status\":\"mac=%s,ssid=%s,usb=%d\"}",mac_buf,SSID_NAME,usb_status_val);  //MAC SSID USB
//    
//    mem_copy(POST_REQUEST_URI+url_len,"&ssid=",strlen("&ssid="));
//    
//    url_len += strlen("&ssid=");
//    
//    mem_copy(POST_REQUEST_URI+url_len,SSID_NAME,strlen(SSID_NAME));
//    
//    url_len += strlen(SSID_NAME);
//  }
  
  base64_encode(SSID_NAME, strlen(SSID_NAME), base64_ssid, sizeof(base64_ssid));
  
  mem_copy(POST_REQUEST_URI+url_len,"&ssid_base64=",strlen("&ssid_base64="));
  
  url_len += strlen("&ssid_base64=");
  
  mem_copy(POST_REQUEST_URI+url_len,base64_ssid,strlen(base64_ssid));
  
  url_len += strlen(base64_ssid);
  
  POST_REQUEST_URI[url_len]= '\0';
  
  snprintf(status_buf,sizeof(status_buf),",\"status\":\"mac=%s,usb=%d\",\"ssid_base64\":\"%s\"}",mac_buf,usb_status_val,base64_ssid);  //MAC SSID USB
  
  #ifdef DEBUG
      osi_UartPrint(POST_REQUEST_URI);
  #endif
    
  HTTPCli_Field fields[4] = {
                              {HTTPCli_FIELD_NAME_HOST, HOST_NAME},
                              {HTTPCli_FIELD_NAME_ACCEPT, "*/*"},
                              {HTTPCli_FIELD_NAME_CONTENT_TYPE, "application/json"},
                              {NULL, NULL}
                            };
  
  HTTPCli_setRequestFields(httpClient, fields);  //Set request header fields to be send for HTTP request 
  
  moreFlags = 1;  //some more header fields need to send
  
  lRetVal = HTTPCli_sendRequest(httpClient, HTTPCli_METHOD_POST, POST_REQUEST_URI, moreFlags);
  if(lRetVal < 0)
  {  
    #ifdef DEBUG
      osi_UartPrint("Failed to send HTTP POST request header.\r\n");
    #endif
    
    return lRetVal;
  }
  
  snprintf((char *)tmpBuf,sizeof(tmpBuf),"%d",(DataLen+strlen(status_buf)+strlen(wifi_buf)-1+strlen(calidata_buf)+8));  //"]}"
  
  #ifdef DEBUG
    osi_UartPrint(tmpBuf);  
  #endif
  
  lastFlag = 1;  //Here we are setting lastFlag = 1 as it is last header field
  
  lRetVal = HTTPCli_sendField(httpClient, HTTPCli_FIELD_NAME_CONTENT_LENGTH, (const char *)tmpBuf, lastFlag);  //Send field
  if(lRetVal < 0)
  { 
    #ifdef DEBUG
      osi_UartPrint("Failed to send HTTP POST request header.\r\n"); 
    #endif
    
    return lRetVal;
  }

  MemoryAddr=POST_ADDR;
  
#ifdef DEBUG
   osi_UartPrint("{\"feeds\":[");      
#endif
  lRetVal = HTTPCli_sendRequestBody(httpClient, "{\"feeds\":[", strlen("{\"feeds\":["));  //Send POST data/body header
  if(lRetVal < 0)
  {
    #ifdef DEBUG
      osi_UartPrint("Failed to send HTTP POST request body.\r\n");      
    #endif
    
    return lRetVal;
  }
  
  while(Post_Amount)
  {
    post_data_sum=Post_Amount>SEND_DATA_NUMBER?SEND_DATA_NUMBER:Post_Amount;  //post data max once
    
    Post_Amount-=post_data_sum;
    if(Post_Amount==0)
    {
      EndFlag=1;
    }
    
    osi_SyncObjWait(&xMutex5,OSI_WAIT_FOREVER);  //Post_Data_Buffer Semaphore Take
    
    MemoryAddr=Read_PostDataBuffer(MemoryAddr,Post_Data_Buffer,post_data_sum,EndFlag);  //read post data
    
    #ifdef DEBUG
      osi_UartPrint(Post_Data_Buffer);   
    #endif
        
    lRetVal = HTTPCli_sendRequestBody(httpClient, (const char *)Post_Data_Buffer, (strlen(Post_Data_Buffer)));  //Send POST data/body
    
    osi_SyncObjSignal(&xMutex5);  //Post_Data_Buffer Semaphore Give
    
    if(lRetVal < 0)
    {
      #ifdef DEBUG    
        osi_UartPrint("Failed to send HTTP POST request body.\r\n");      
      #endif
        
      return lRetVal;
    }     
  }
#ifdef DEBUG
   osi_UartPrint(wifi_buf);      
#endif 
  lRetVal = HTTPCli_sendRequestBody(httpClient,wifi_buf,strlen(wifi_buf));  //wifi_buf
  if(lRetVal < 0)
  {
    #ifdef DEBUG
      osi_UartPrint("Failed to send HTTP POST request body.\r\n"); 
    #endif

    return lRetVal;
  }
#ifdef DEBUG
   osi_UartPrint("],\"cali\":");      
#endif
  lRetVal = HTTPCli_sendRequestBody(httpClient,"],\"cali\":",strlen("],\"cali\":"));  //"],\"cali\":"
  if(lRetVal < 0)
  {
    #ifdef DEBUG
      osi_UartPrint("Failed to send HTTP POST request body.\r\n"); 
    #endif

    return lRetVal;
  }
#ifdef DEBUG
   osi_UartPrint(calidata_buf);      
#endif
  lRetVal = HTTPCli_sendRequestBody(httpClient,calidata_buf,strlen(calidata_buf));  //Send calidata_buf
  if(lRetVal < 0)
  {
    #ifdef DEBUG
      osi_UartPrint("Failed to send HTTP POST request body.\r\n"); 
    #endif

    return lRetVal;
  }
#ifdef DEBUG
   osi_UartPrint(status_buf);      
#endif
  lRetVal = HTTPCli_sendRequestBody(httpClient,status_buf,strlen(status_buf));  //Send status data
  if(lRetVal < 0)
  {
    #ifdef DEBUG
      osi_UartPrint("Failed to send HTTP POST request body.\r\n"); 
    #endif

    return lRetVal;
  }
  
  #ifdef DEBUG

    osi_UartPrint("Successed to send HTTP POST request body.\r\n");

  #endif
  
  lRetVal = readResponse(httpClient);   //read respose from server

  return lRetVal;
}

/******************************************************************************
  HTTP Get Form Server
******************************************************************************/
short Http_Get_Function(void)
{
  short lRetVal = -1;
  HTTPCli_Struct httpClient = {0};
  
  mem_set(&httpClient,0,sizeof(httpClient));
  
  lRetVal = ConnectToHTTPServer(&httpClient);
  if(lRetVal<0)
  {
    HTTPCli_disconnect(&httpClient);  //disconnect to http server
    
    lRetVal = ConnectToHTTPServer(&httpClient);
    
    if(lRetVal<0)
    {
      return lRetVal;
    }
  }
 
  lRetVal = HTTPGetMethod(&httpClient);
  
  HTTPCli_disconnect(&httpClient);  //disconnect to http server
  
  return lRetVal; 
}

/*******************************************************************************
  deleted the post data
*******************************************************************************/
void PostAddrChage(unsigned long data_num,unsigned long end_addr)
{
  unsigned long d_addr,p_addr,w_addr;
  
  d_addr=DELETE_ADDR;  //delete pointer variable value
  
  p_addr=POST_ADDR;  //post pointer variable value
  
  w_addr=WRITE_ADDR;  //write pointer variable value
  
  if(((w_addr >= p_addr)&&(w_addr >= d_addr)&&(p_addr >= d_addr))||((d_addr >= w_addr)&&(d_addr >= p_addr)&&(w_addr >= p_addr)))  //w_addr > p_addr
  {
    POST_ADDR=end_addr;
    
    if(w_addr <= end_addr)
    {
      osi_SyncObjSignalFromISR(&xBinary11);  //start delete task
    }
  }
  else if((p_addr >= d_addr)&&(p_addr >= w_addr)&&(d_addr >= w_addr))   //p_addr>w_addr
  {
    if((end_addr>=p_addr)&&(end_addr<Memory_Max_Addr))
    {
      POST_ADDR=end_addr;
    }
    else if(end_addr <= w_addr)
    {
      POST_ADDR=end_addr;
    }
    else
    {
      POST_ADDR=end_addr;
      
      osi_SyncObjSignalFromISR(&xBinary11);  //start delete task
    }
  }
  else  
  {
    osi_SyncObjSignalFromISR(&xBinary11);  //start delete task
  }
  
  if(POST_NUM >= data_num)
  {
    osi_EnterCritical();  //enter critical
    
    POST_NUM -= data_num;
    
    osi_ExitCritical(0);  //exit critical
  }
  else
  {
    POST_NUM=0;
  }
  
  if(POST_NUM==0)
  {
    POST_ADDR = WRITE_ADDR;
  }
  
  if(save_addr_flag == 0)
  {
    osi_at24c08_write(DATA_AMOUNT_ADDR1,POST_NUM);
    
    osi_at24c08_write(DATA_POST_ADDR1,POST_ADDR);
  }
  else
  {
    osi_at24c08_write(DATA_AMOUNT_ADDR2,POST_NUM);
    
    osi_at24c08_write(DATA_POST_ADDR2,POST_ADDR);
  }
  
  #ifdef DEBUG 
    
    osi_UartPrint_Val("POST_NUM:",POST_NUM);
    
    osi_UartPrint_Val("POST_ADDR:",POST_ADDR);
    
  #endif
}

/*******************************************************************************
// data post task
*******************************************************************************/
void DataPostTask(void *pvParameters)
{
  uint8_t err_code_num;
  int Rssi_val;
  bool Err_Status;
  short lRetVal=-1;
  uint16_t read_data_num;
  uint16_t post_data_num;
  unsigned long post_data_len;
  unsigned long read_data_end_addr;
  SensorMessage Rssi_Msg;
  HTTPCli_Struct httpClient = {0};

  for(;;)
  {
    osi_SyncObjWait(&xBinary0,OSI_WAIT_FOREVER);  //Wait For The Operation Message  
    
    Err_Status = 1;
      
    osi_SyncObjWait(&xMutex2,OSI_WAIT_FOREVER);  //SimpleLink Semaphore Take
    
    sys_run_time = 0;  //clear wifi post time
    
    POST_TASK_END_FLAG = 1;
        
    osi_SyncObjSignalFromISR(&xBinary13);  //Start Tasks End Check
   
    osi_SyncObjSignalFromISR(&xBinary17);  //Start LED Blink 
    
    Rssi_val=Scan_Wifi_List(NULL,NULL,0);
    
    if((Rssi_val!=ERROR_CODE)||(wifi_mode == 1))
    {
      if(Rssi_val==ERROR_CODE)
      {
        osi_at24c08_ReadData(SECTYPE_ADDR,(uint8_t*)SEC_TYPE,sizeof(SEC_TYPE),1);  //Read Wifi Sectype
      }
      else
      {
        Rssi_Msg.sensornum=RSSI_NUM;  //Message Number
        Rssi_Msg.sensorval = f5_a*Rssi_val + f5_b;  //Message Value
        osi_MsgQWrite(&xQueue0,&Rssi_Msg,OSI_NO_WAIT);  //Rssi Value Data Message
      }
      
      for(err_code_num = 0;err_code_num < RETRY_TIME_OUT;err_code_num++)
      {
        lRetVal = WlanConnect();  //Wlan Connect To The Accesspoint
        if(lRetVal >= 0) 
        {
          break;
        }
        else 
        {
          Wlan_Disconnect_AP();  //wlan disconnect form the ap
                  
          sl_Stop(SL_STOP_TIMEOUT);  //stop the simple link
          
          MAP_UtilsDelay(80000);  //Delay About 6ms
          
          if(Rssi_val == ERROR_CODE)
          {
            break;
          }
        }
      }
      
      err_code_num = 0;
      
      if(lRetVal >= 0)                                    
      {  
        memset(calidata_buf,0,sizeof(calidata_buf));       
        Read_cali(calidata_buf,sizeof(calidata_buf));  // cali data 
    
        while(POST_NUM)
        {
          sys_run_time = 0;  //clear wifi post time
          
          read_data_num=POST_NUM>POST_DATA_NUMBER?POST_DATA_NUMBER:POST_NUM;  //read post data len
          
          lRetVal=Read_PostDataLen(POST_ADDR,&read_data_end_addr,read_data_num,&post_data_num,&post_data_len);  
          if(lRetVal < 0)
          {
            #ifdef USE_LCD      
  
              osi_Ht1621_Display_Err_Val(0x07);  //Display the Error Code
      
            #endif
            
            osi_System_ERROR_Save(MEMORY_SAVE_DATA_ERR);  //Save ERROR Data
            
            PostAddrChage(post_data_num,read_data_end_addr);  //change the point
          }
          else
          {
            lRetVal = ConnectToHTTPServer(&httpClient);  //connect to http server
            if(lRetVal < 0)
            {
              HTTPCli_disconnect(&httpClient);  //disconnect to http server
              
              lRetVal = ConnectToHTTPServer(&httpClient);  //connect to http server
              if(lRetVal < 0)
              {
                HTTPCli_disconnect(&httpClient);  //disconnect to http server
                
                osi_System_ERROR_Save(CNT_SERVER_FLR_ERR);  //save ERROR data
                
                break;
              }
            }
            
            lRetVal = HTTPPostMethod(&httpClient,post_data_len,post_data_num);
            
            HTTPCli_disconnect(&httpClient);  //disconnect to http server

            if(lRetVal ==0)
            {
              Err_Status=0;
              
              PostAddrChage(post_data_num,read_data_end_addr);  //change the point
            }
            else if(lRetVal==400)
            {
              #ifdef USE_LCD      

                osi_Ht1621_Display_Err_Val(0x08);

              #endif
              
              osi_System_ERROR_Save(POST_DATA_JSON_FAULT);  //save ERROR data
              
              PostAddrChage(post_data_num,read_data_end_addr);  //change the point
              
              err_code_num += 1;
            }
            else if(lRetVal ==410)
            {
              PostSetData=1;
              
              lRetVal=Http_Get_Function();  //HTTP Get Form Server

              PostSetData=0;
              
              if(lRetVal!=SUCCESS)
              {
                break;
              }
            }
            else
            {
              #ifdef DEBUG
              
                osi_UartPrint("HTTP Post failed.\r\n");
              
              #endif 

              #ifdef USE_LCD      

                osi_Ht1621_Display_Err_Val(0x09);
    
              #endif
              
              osi_System_ERROR_Save(POST_DATA_FLR_ERR);  //save ERROR data
              
              err_code_num += 1;
              MAP_UtilsDelay(40000000);  //delay about 3s
            }
            
            if(err_code_num >= RETRY_TIME_OUT)
            {
              break;
            }
          }
        }
        Wlan_Disconnect_AP();  //wlan disconnect form the ap
                  
        sl_Stop(SL_STOP_TIMEOUT);  //stop the simple link
        
        MAP_UtilsDelay(80000);  //Delay About 6ms
      }
    }
    
    POST_TASK_END_FLAG=0;
      
    osi_SyncObjSignal(&xMutex2);  //SimpleLink Semaphore Give
    
    osi_SyncObjSignalFromISR(&xBinary11);  //start delete task
    
    if(Err_Status)
    {
      Red_Led_On_time(10);  //RED LED ON 1.5s
    }
  }
}

/*******************************************************************************
//Scan WIFI LIST with locked
*******************************************************************************/
void osi_Scan_Wifi_List(char *rssi_ssid,int *rssi_val,bool uart_printf)
{
  osi_SyncObjWait(&xMutex2,OSI_WAIT_FOREVER);  //SimpleLink Semaphore Take
    
  Scan_Wifi_List(rssi_ssid,rssi_val,uart_printf);
  
  osi_SyncObjSignal(&xMutex2);  //SimpleLink Semaphore Give
}

/*******************************************************************************
//WiFi connect test
*******************************************************************************/
short WiFi_Connect_Test(void)
{
  short test_val=-1;
  
//  Scan_Wifi_List(NULL,NULL,0);
//  ConfigureSimpleLinkToDefaultState();  //configure the device to default state
  
  test_val = WlanConnect();  //wlan connect to the accesspoint
  if(test_val >= 0)
  {
    osi_at24c08_WriteData(SECTYPE_ADDR,(uint8_t *)SEC_TYPE,strlen(SEC_TYPE),1);  //save type in at24c08
  }
  
  Wlan_Disconnect_AP();  //wlan disconnect form the ap
  
  sl_Stop(SL_STOP_TIMEOUT);  //stop the simple link
  
  MAP_UtilsDelay(80000);  //delay about 6ms
  
  return test_val;
}

/*******************************************************************************
//WiFi connect test with locked
*******************************************************************************/
short osi_WiFi_Connect_Test(void)
{
  short test_status=-1;
  
  osi_SyncObjWait(&xMutex2,OSI_WAIT_FOREVER);  //SimpleLink Semaphore Take
    
  test_status=WiFi_Connect_Test();
  
  osi_SyncObjSignal(&xMutex2);  //SimpleLink Semaphore Give
  
  return test_status;
}


/*******************************************************************************
                                      END         
*******************************************************************************/




