
#include "bcomdef.h"
#include "gatt.h"
#include "gattservapp.h"
#include "App_GATTConfig.h"
#include "Service_ECGMonitor.h"


//配置ECGMonitor服务
//appCBs: 应用层为此服务提供的回调函数
//如果有其他服务需要配置，请仿照此函数创建一个新函数。
extern void GATTConfig_SetECGMonitorService(ecgServiceCBs_t* appCBs)
{
  ECGMonitor_AddService( GATT_ALL_SERVICES );  // 加载服务  

  // 登记回调
  VOID ECGMonitor_RegisterAppCBs( appCBs );  
}



//配置电池电量服务
extern void GATTConfig_SetBatteryService(batteryServiceCBs_t* appCBs)
{
  Battery_AddService( GATT_ALL_SERVICES );  // 加载服务  

  // 登记回调
  VOID Battery_RegisterAppCBs( appCBs );   
}