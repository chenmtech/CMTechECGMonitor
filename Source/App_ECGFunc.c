
#include <iocc2541.h>
#include "hal_drivers.h"
#include "hal_mcu.h"
#include "osal.h"
#include "cmtechECGMonitor.h"
#include "App_ECGFunc.h"
#include "Dev_ADS129x.h"
#include "DCand50HzFilter.h"

#define STATE_STOP          0
#define STATE_START         1

//每个信号数据包包含8个信号值
#define PACKET_LEN          8


static uint8 state = STATE_STOP;

static uint8 taskID = 0;

static int packet[PACKET_LEN] = {0};
static uint8 * p = 0;
static uint8 num = 0;


static void ECGFunc_ProcessData(int data);


extern void ECGFunc_Init(uint8 TaskID)
{
  state = STATE_STOP;
  taskID = TaskID;

  
  ADS1x9x_Init(ECGFunc_ProcessData);
  
  //ADS1x9x_StartConvert();
  //state = STATE_START;
}

extern void ECGFunc_Start()
{
  if(state == STATE_STOP)
  {
    num = 0;
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
    if(num != 0)
    {
      CMTechECGMonitor_SendECGSignals(p, num*sizeof(int)); 
    }
  }
}

extern bool ECGFunc_ProcessEvent()
{

  
  return TRUE;
}

static void ECGFunc_ProcessData(int data)
{
  if(state == STATE_STOP) return;
  
  *p++ = (uint8)(data & 0x00FF);  
  *p++ = (uint8)((data >> 8) & 0x00FF);
  num++;  

  if(num == PACKET_LEN)
  {
    num = 0;
    p = (uint8*)(&packet[0]);    
    CMTechECGMonitor_SendECGSignals(p, PACKET_LEN*sizeof(int));    
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

  //d = d >> 2;   //这里的右移主要是适应手机端的屏幕显示尺度，实际上可以不用右移，在手机端调整尺度
  
  //CommProtocol_sendCommand(COMM_SEND_DATA_RQ, sizeof(DATATYPE), (uint8 *)&d);       //直接发送    
}
