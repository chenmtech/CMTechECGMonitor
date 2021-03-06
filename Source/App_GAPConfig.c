
#include "bcomdef.h"
#include "gap.h"
#include "gapgattserver.h"
#include "gapbondmgr.h"
#include "peripheral.h"
#include "App_GAPConfig.h"
#include "CMUtil.h"

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
//#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
//#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800

// Slave latency to use if automatic parameter update request is enabled
//#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
//#define DEFAULT_DESIRED_CONN_TIMEOUT          1000





// 设置与广播相关的参数
// 采用General Discoverable模式，并且是持续广播
// advInt: 广播间隔时间，单位ms
// 关于广播时间设置的问题，可以参阅 http://blog.csdn.net/zzfenglin/article/details/51166830
extern void GAPConfig_SetAdvParam(uint16 advInt, uint16 servUUID)
{
  // 设置广播间隔时间
  advInt = (advInt < 20) ? 20 : ( (advInt > 10240) ? 10240 : advInt  );
  uint16 advInt625 = ((uint32)advInt)*1000L/625L;

  //GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt625 );
  //GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt625 );
  GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt625 );
  GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt625 );  
  
  // 不停地广播
  GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, 0 ); 
  
  
  // 设置广播数据，最大31字节，为了节省电量，尽量缩短  
  uint8 advertData[] =
  {
    0x02,   // length of this data
    GAP_ADTYPE_FLAGS,
    GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,  
  };  
  GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );  
  
  //设置扫描响应数据
  // 扫描响应数据，最大31字节
  uint8 scanRspData[] =
  {
    // 设备短名
    0x06,   // length of this data
    GAP_ADTYPE_LOCAL_NAME_SHORT,
    'C',
    'M',
    '1',
    '.',
    '0',
  
    // 通知主机，本从机包含的服务UUID 
    0x11,   // length of this data
    GAP_ADTYPE_128BIT_MORE,      // some of the UUID's, but not all
    CM_UUID( servUUID ),  
  
    // 发射功率等级
    0x02,   // length of this data
    GAP_ADTYPE_POWER_LEVEL,
    0,       // 0dBm
  };  
  GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );  
 
}

//使能或停止广播
extern void GAPConfig_EnableAdv(uint8 enable)
{
  GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &enable );     
}

//设置与连接相关的参数
//连接间隔时间：范围7.5ms~4s之间1.25ms的整数倍。如果min和max不一样，芯片会选择一个靠近max的值
//slave latency: 从机的潜伏次数，范围0 - 499。这个数的连接间隔过后，下一个连接间隔，从机必须响应
//监控超时时间：范围100ms-32s。书上建议至少给从机6次侦听的机会
//minInt: 单位ms
//maxInt: 单位ms
//latency: 潜伏次数
//timeout: 监控超时时间，单位ms
//when: 连接建立后多少秒开始更新参数，单位s
extern void GAPConfig_SetConnParam(uint16 minInt, uint16 maxInt, uint16 latency, uint16 timeout, uint8 when)
{
  uint8 enable_update_request = TRUE;
  minInt = (uint16)( ((float)minInt)/1.25 );
  maxInt = (uint16)( ((float)maxInt)/1.25 );  
  timeout /= 10;  

  GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
  GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &minInt );
  GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &maxInt );
  GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &latency );
  GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &timeout );  

  //这个参数是指从连接建立后到从机更新连接参数之间需要延时的时间
  //如果主机不同意更新参数，从机可以选择断开连接或继续忍受现有参数
  GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, when );    
}

//终止连接。从机没有权利发起连接，但可以主动终止连接
extern bStatus_t GAPConfig_TerminateConn()
{
  return GAPRole_TerminateConnection();
}

//设置GGS参数，只有设备名称属性可以设置
//devName: 设备名属性值
extern void GAPConfig_SetGGSParam(uint8* devName)
{
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, devName );
}

//设置绑定相关参数
// passkey : 配对密码
extern void GAPConfig_SetBondingParam(uint32 passkey, uint8 pairmode)
{
  uint8 mitm = FALSE;
  uint8 ioCap = GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT; //要求手机输入配对密码
  uint8 bonding = TRUE;
  GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
  GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairmode );
  GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
  GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
  GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
}