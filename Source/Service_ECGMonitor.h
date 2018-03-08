
/*
* Service_ECGMonitor.h : ECG����������ͷ�ļ�
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

// ECG Monitor Service Parameters��������ʾҪ������Characteristic
#define ECG_DATA                  0   //ECG����
#define ECG_CONF                  1   //ECG�������Ƶ�
#define ECG_1MV                   2   //1mV�궨ֵ
#define ECG_SAMPLERATE            3   //������
#define ECG_LEADTYPE              4   //��������
  
  
// ECG Signal Value��Attribute Table�е�λ��
#define ECG_DATA_POS                       2
 

// ECG Monitor Service UUID and Characteristic UUID�������Զ����
#define ECG_SERV_UUID               0xAA40  //ECG Monitor service UUID
#define ECG_DATA_UUID               0xAA41  //ECG�ź�ֵ����UUID
#define ECG_CONF_UUID               0xAA42  //ECG�������Ƶ�UUID
#define ECG_1MV_UUID                0xAA43  //1mV����ֵ����UUID
#define ECG_SAMPLERATE_UUID         0xAA44  //����Ƶ��UUID
#define ECG_LEADTYPE_UUID           0xAA45  //ECG��������UUID

//��������
#define LEADTYPE_I            0x00    //Lead I
#define LEADTYPE_II           0x01    //Lead II
#define LEADTYPE_III          0x02    //Lead III


  
// ECG Monitor Service bit fields
#define ECGMONITOR_SERVICE                 0x00000001

// Callback events����Щ�¼�������֪ͨӦ�ò�Ҫ������¼�
#define ECGMONITOR_SIGNAL_NOTI_ENABLED          1
#define ECGMONITOR_SIGNAL_NOTI_DISABLED         2


/*********************************************************************
 * TYPEDEFS
 */

// ECGMonitor Service App callback function��Ӧ�ò�Ļص�����
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
//���ط���
extern bStatus_t CMTechECGMonitor_AddService( uint32 services );

//�Ǽ�Ӧ�ò�ص�����
extern void CMTechECGMonitor_Register( ECGServiceAppCB_t pfnServiceCB );

//��Ӧ�ò����÷������������
extern bStatus_t CMTechECGMonitor_SetParameter( uint8 param, uint8 len, void *value );

//��Ӧ�ò��ȡָ���ķ�����������
extern bStatus_t CMTechECGMonitor_GetParameter( uint8 param, void *value );

//����һ��ECG�ź�ֵ��Notification
extern bStatus_t CMTechECGMonitor_ECGSignalNotify( uint16 connHandle, attHandleValueNoti_t *pNoti );



/*********************************************************************
*********************************************************************/


#endif 
