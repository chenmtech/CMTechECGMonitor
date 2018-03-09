
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


//ÿ���ź����ݰ��ĳ���
#define PACKET_LEN          ECG_DATA_LEN

/*****************************************************
 * �ֲ�����
 */

// ��ǰ״̬
static uint8 state = STATE_STOP;

// ���ݰ�
static int packet[PACKET_LEN] = {0};

static uint8 * p = 0;

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
  
  *p++ = (uint8)(data & 0x00FF);  
  count++;
  *p++ = (uint8)((data >> 8) & 0x00FF);
  count++;  

  if(count == PACKET_LEN)
  {
    count = 0;
    p = (uint8*)(&packet[0]);    
    ECGMonitor_SendECGSignals(p, PACKET_LEN*sizeof(int));    
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

extern void ECGFunc_Start()
{
  if(state == STATE_STOP)
  {
    count = 0;
    p = (uint8*)(&packet[0]);
    ADS1x9x_StartConvert();
    state = STATE_START;    
  }
}

extern void ECGFunc_Stop()
{
  if(state == STATE_START)
  {
    ADS1x9x_StopConvert();
    state = STATE_STOP; 
    if(count != 0)
    {
      ECGMonitor_SendECGSignals(p, num*sizeof(int)); 
    }
  }
}


