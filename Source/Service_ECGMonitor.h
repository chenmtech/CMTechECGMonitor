
/*
* Service_ECGMonitor.h : ECG监视器服务头文件
Written by chenm, 2017-04-13
*/


#ifndef SERVICE_ECGMONITOR_H
#define SERVICE_ECGMONITOR_H


/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

// ECG Monitor Service Parameters，用来表示要操作的Characteristic
#define ECG_DATA                  0   //ECG数据
#define ECG_CONF                  1   //ECG测量控制点
#define ECG_1MV                   2   //1mV标定值
#define ECG_SAMPLERATE            3   //采样率
#define ECG_LEADTYPE              4   //导联类型
  
  
// ECG Signal Value在Attribute Table中的位置
#define ECG_DATA_POS                       2
 

// ECG Monitor Service UUID and Characteristic UUID，都是自定义的
#define ECG_SERV_UUID               0xAA40  //ECG Monitor service UUID
#define ECG_DATA_UUID               0xAA41  //ECG信号值特征UUID
#define ECG_CONF_UUID               0xAA42  //ECG采样控制点UUID
#define ECG_1MV_UUID                0xAA43  //1mV定标值特征UUID
#define ECG_SAMPLERATE_UUID         0xAA44  //采样频率UUID
#define ECG_LEADTYPE_UUID           0xAA45  //ECG导联类型UUID

//导联类型
#define LEADTYPE_I            0x00    //Lead I
#define LEADTYPE_II           0x01    //Lead II
#define LEADTYPE_III          0x02    //Lead III


  
// ECG Monitor Service bit fields
#define ECGMONITOR_SERVICE                 0x00000001

// Callback events，这些事件是用来通知应用层要处理的事件
#define ECGMONITOR_SIGNAL_NOTI_ENABLED          1
#define ECGMONITOR_SIGNAL_NOTI_DISABLED         2


/*********************************************************************
 * TYPEDEFS
 */

// ECGMonitor Service App callback function，应用层的回调函数
typedef void (*ECGServiceAppCB_t)(uint8 event);


  
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */


/*********************************************************************
 * API FUNCTIONS 
 */
//加载服务
extern bStatus_t CMTechECGMonitor_AddService( uint32 services );

//登记应用层回调函数
extern void CMTechECGMonitor_Register( ECGServiceAppCB_t pfnServiceCB );

//让应用层设置服务的特征参数
extern bStatus_t CMTechECGMonitor_SetParameter( uint8 param, uint8 len, void *value );

//让应用层获取指定的服务特征参数
extern bStatus_t CMTechECGMonitor_GetParameter( uint8 param, void *value );

//发送一个ECG信号值的Notification
extern bStatus_t CMTechECGMonitor_ECGSignalNotify( uint16 connHandle, attHandleValueNoti_t *pNoti );



/*********************************************************************
*********************************************************************/


#endif 
