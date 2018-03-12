
#include <iocc2541.h>
#include "hal_drivers.h"
#include "hal_mcu.h"
#include "osal.h"
#include "CMTechECGMonitor.h"
#include "App_ECGFunc.h"
#include "Dev_ADS1x9x.h"
#include "Service_ECGMonitor.h"
#include "DCand50HzFilter.h"


/*****************************************************
 * ����
 */

// ֹͣ״̬
#define STATE_STOP              0x00
// �����ɼ�ECG״̬
#define STATE_START_ECG         0x01
// �����ɼ�1mV�����ź�״̬
#define STATE_START_1MV         0x02


/*****************************************************
 * �ֲ�����
 */

// ��ǰ״̬
static uint8 state = STATE_STOP;

// ���ݰ�
static uint8 buf[ECG_PACKET_LEN] = {0};

//static uint8 * p = 0;

// ��ǰ���ݰ����Ѿ�����������ֽ���
static uint8 count = 0;




/*****************************************************
 * �ֲ�����
 */
static void ECGFunc_ProcessData(int data);

// ����ɼ�����һ������
static void ECGFunc_ProcessData(int data)
{
  if(state == STATE_STOP) return;
  
  buf[count++] = (uint8)(data & 0x00FF);  
  buf[count++] = (uint8)((data >> 8) & 0x00FF);

  if(count == ECG_PACKET_LEN)
  {
    count = 0;
    ECGMonitor_SetParameter( ECGMONITOR_DATA, ECG_PACKET_LEN, buf );  
  }  
  /*
  if(i < 512)
    buf[i++] = data;//DCAND50HZ_IntegerFilter(data);
  else
  {
    i = 0;
  }
  */
  //DATATYPE d = (DATATYPE)FPFIRLPF1_Filter( DCAND50HZ_IntegerFilter(data) );

  //d = d >> 2;   //�����������Ҫ����Ӧ�ֻ��˵���Ļ��ʾ�߶ȣ�ʵ���Ͽ��Բ������ƣ����ֻ��˵����߶�
  
  //CommProtocol_sendCommand(COMM_SEND_DATA_RQ, sizeof(DATATYPE), (uint8 *)&d);       //ֱ�ӷ���    
}




/*****************************************************
 * ��������
 */

extern void ECGFunc_Init()
{
  state = STATE_STOP;
  
  ADS1x9x_Init(ECGFunc_ProcessData);
  
}

extern void ECGFunc_StartEcg()
{
  if(state == STATE_START_ECG) return;
  
  if(state == STATE_START_1MV) {
    ADS1x9x_StopConvert();
    ADS1x9x_SetRegsAsNormalECGSignal();
  }
  
  count = 0;
  ADS1x9x_StartConvert();
  state = STATE_START_ECG;
}

extern void ECGFunc_Start1mV()
{
  if(state == STATE_START_1MV) return;
  
  if(state == STATE_START_ECG) {
    ADS1x9x_StopConvert();
    ADS1x9x_SetRegsAsTestSignal();
  }
  
  count = 0;
  ADS1x9x_StartConvert();
  state = STATE_START_1MV;
}

extern void ECGFunc_Stop()
{
  if(state != STATE_STOP)
  {
    ADS1x9x_StopConvert();
    state = STATE_STOP; 
  }
}


