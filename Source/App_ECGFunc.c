
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

// ���ݰ���ָ��Service_ECGMonitor�е�ecgData
static uint8 *pBuf;

// ��ǰ���ݰ����Ѿ�����������ֽ���
static uint8 byteCnt = 0;




/*****************************************************
 * �ֲ�����
 */
static void ECGFunc_ProcessDataCB(int data);

// ����ɼ�����һ������
static void ECGFunc_ProcessDataCB(int data)
{
  if(state == STATE_STOP) return;
  
  *pBuf++ = (uint8)(data & 0x00FF);  
  *pBuf++ = (uint8)((data >> 8) & 0x00FF);
  byteCnt += 2;

  // �ﵽ���ݰ�����
  if(byteCnt == ECG_PACKET_LEN)
  {
    byteCnt = 0;
    pBuf = ECGMonitor_GetECGDataPointer();
    ECGMonitor_SetParameter( ECGMONITOR_DATA, ECG_PACKET_LEN, pBuf );  
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

//��ʱus
static void Delay_us(uint16 us)
{
  while(us--)
  {
    /* 32 NOPs == 1 usecs */
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop");
  }
}


/*****************************************************
 * ��������
 */

extern void ECGFunc_Init()
{
  state = STATE_STOP;
  
  // ��ʼ��ADS1x9x���������ݴ���ص�����
  ADS1x9x_Init(ECGFunc_ProcessDataCB);
  
  // ��ȡECG DATA���ݿռ�ָ��
  pBuf = ECGMonitor_GetECGDataPointer();
  
  byteCnt = 0;
  
}

extern void ECGFunc_StartEcg()
{
  switch(state) {
  case STATE_START_ECG:
    return;
  case STATE_START_1MV:
    ADS1x9x_StopConvert();
    break;
  case STATE_STOP:
    ADS1x9x_WakeUp();
    break;
  default:
    return;
  }
  
  // ����һ��Ҫ��ʱ��������������
  Delay_us(100);
  
  ADS1x9x_SetRegsAsNormalECGSignal();
  byteCnt = 0;
  pBuf = ECGMonitor_GetECGDataPointer();
  
  ADS1x9x_StartConvert();
  state = STATE_START_ECG;
}

extern void ECGFunc_Start1mV()
{
  switch(state) {
  case STATE_START_1MV:
    return;
  case STATE_START_ECG:
    ADS1x9x_StopConvert();
    break;
  case STATE_STOP:
    ADS1x9x_WakeUp();
    break;
  default:
    return;
  }
  
  // ����һ��Ҫ��ʱ��������������
  Delay_us(100);

  ADS1x9x_SetRegsAsTestSignal();  
  
  byteCnt = 0;
  pBuf = ECGMonitor_GetECGDataPointer();
  
  ADS1x9x_StartConvert();
  state = STATE_START_1MV;
}

extern void ECGFunc_Stop()
{
  if(state != STATE_STOP)
  {
    ADS1x9x_StopConvert();
    Delay_us(1000);
    state = STATE_STOP; 
    ADS1x9x_StandBy();
    Delay_us(1000);
  }
}


