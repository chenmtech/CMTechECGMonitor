
#include "Dev_ADS129x.H"
#include "hal_mcu.h"

//#define ADS_IS_START()                  (ADS_START_PxOUT & ADS_START)



//静态常量

//当用内部测试信号时的寄存器值
const static uint8 testRegs[12] = {  
  /*使用内部测试信号配置*/
  0x52,
  //CONFIG1
  0x02,                     //contineus sample,500sps
  //CONFIG2
  0xA3,                     //1Hz方波
  //LOFF
  0x10,                     //
  //CH1SET 
  0x15,                     //PGA=1, 测试信号
  //CH2SET
  0x80,                     //关闭CH2
  //RLD_SENS (default)      
  0x00,                     //default
  //LOFF_SENS (default)
  0x00,                     //default
  //LOFF_STAT
  0x00,                     //default
  //RESP1
  0x02,                     //
  //RESP2
  0x07,                     //
  //GPIO
  0x0C                      //
};	

const static uint8 normalECGRegs[12] = {  
  //DEVID
  0x52,
  //CONFIG1
  0x01,                     //continuous sample,250sps
  //CONFIG2
  0xA0,                     //
  //LOFF
  0x10,                     //
  //CH1SET 
  0x60,                     //PGA=12
  //CH2SET
  0x80,                     //关闭CH2
  //RLD_SENS     
  0x23,                     //
  //LOFF_SENS (default)
  0x00,                     //default
  //LOFF_STAT
  0x00,                     //default
  //RESP1
  0x02,                     //
  //RESP2
  0x87,
  //GPIO
  0x0C                      //
};

//静态变量
static uint8 defaultRegs[12];    //存放重启后缺省的寄存器值

//采样数据后的回调函数变量
static ADS_DataCB_t ADS_DataCB;

//每次需要读取的数据字节，包括状态字节和通道数据
static uint8 data[NUM_PER_SAMPLE];

//保存一个通道的ECG数据
static uint8 bytes[4] = {0};

static long tmp = 0;

static int16 aaa = 0;

static uint8 tmp1 = 1;


/****************************************************************/
/* 静态函数*/
/****************************************************************/
static void Delay_us(uint16 us);

static void execute(uint8 cmd);

static void readOneSample(void);

//使用内部测试信号
static void ADS1x9x_SetRegsAsTestSignal();

//正常采集ECG信号
static void ADS1x9x_SetRegsAsNormalECGSignal();


extern void ADS1x9x_Init(ADS_DataCB_t pfnADS_DataCB_t)
{
  // 初始化SPI模块
  SPI_Init();  
  
  ADS1x9x_ReadAllRegister(defaultRegs); 
  
  //ADS1x9x_SetRegsAsNormalECGSignal();
  ADS1x9x_SetRegsAsTestSignal();
  
  //设置样本数据处理钩子
  ADS_DataCB = pfnADS_DataCB_t;
}

extern void ADS1x9x_WakeUp(void)
{
  execute(WAKEUP);
}

extern void ADS1x9x_StandBy(void)
{
  execute(STANDBY);
}

extern void ADS1x9x_Reset(void)
{
  P1 &= ~(1<<1);    //PWDN/RESET 低电平
  Delay_us(10);
  P1 |= (1<<1);    //PWDN/RESET 高电平
  Delay_us(50);
}

extern void ADS1x9x_StartConvert(void)
{
  //设置连续采样模式
  ADS_CS_LOW();  
  SPI_ADS_SendByte(SDATAC);
  SPI_ADS_SendByte(RDATAC);  
  Delay_us(10);
  ADS_CS_HIGH();   
  
  P1 |= (1<<0);    //START 高电平
  Delay_us(16000); 
}

extern void ADS1x9x_StopConvert(void)
{
  ADS_CS_LOW();  
  SPI_ADS_SendByte(SDATAC);
  Delay_us(10);
  ADS_CS_HIGH();   
  
  P1 &= ~(1<<0);    //START 低电平 
  Delay_us(16000); 
}

extern void ADS1x9x_ReadAllRegister(uint8 * pRegs)
{
  ADS1x9x_ReadMultipleRegister(0x00, pRegs, 12);
}

extern void ADS1x9x_ReadMultipleRegister(uint8 beginaddr, uint8 * pRegs, uint8 len)
{
  ADS_CS_LOW();
  
  SPI_ADS_SendByte(SDATAC);
  SPI_ADS_SendByte(beginaddr | 0x20);               //发送RREG命令
  SPI_ADS_SendByte(len-1);                          //长度-1
  
  for(uint8 i = 0; i < len; i++)
    *(pRegs+i) = SPI_ADS_SendByte(ADS_DUMMY_CHAR);

  Delay_us(10);
  ADS_CS_HIGH();
}

extern uint8 ADS1x9x_ReadRegister(uint8 address)
{
  uint8 ret = 0;

  ADS_CS_LOW();
  
  SPI_ADS_SendByte(SDATAC);  
  SPI_ADS_SendByte(address | 0x20);         //发送RREG命令
  SPI_ADS_SendByte(0);                      //长度为1
  ret = SPI_ADS_SendByte(ADS_DUMMY_CHAR);   //读寄存器
  
  Delay_us(10);
  ADS_CS_HIGH();
  return ret;
}

extern void ADS1x9x_WriteAllRegister(const uint8 * pRegs)
{
  ADS1x9x_WriteMultipleRegister(0x00, pRegs, 12);
}

extern void ADS1x9x_WriteMultipleRegister(uint8 beginaddr, const uint8 * pRegs, uint8 len)
{
  ADS_CS_LOW();
  
  SPI_ADS_SendByte(SDATAC);  
  SPI_ADS_SendByte(beginaddr | 0x40);
  SPI_ADS_SendByte(len-1);
  
  for(uint8 i = 0; i < len; i++)
    SPI_ADS_SendByte( *(pRegs+i) );
     
  Delay_us(10); 
  ADS_CS_HIGH();
} 

extern void ADS1x9x_WriteRegister(uint8 address, uint8 onebyte)
{
  ADS_CS_LOW();
  
  SPI_ADS_SendByte(SDATAC);  
  SPI_ADS_SendByte(address | 0x40);
  SPI_ADS_SendByte(0);  
  SPI_ADS_SendByte(onebyte);
  
  Delay_us(10);
  ADS_CS_HIGH();
}  


//下面为静态函数
/******************************************************************************
//读一个样本
******************************************************************************/
static void readOneSample(void)
{  
  ADS_CS_LOW();
  
  data[0] = SPI_ADS_SendByte(ADS_DUMMY_CHAR);
  data[1] = SPI_ADS_SendByte(ADS_DUMMY_CHAR);
  data[2] = SPI_ADS_SendByte(ADS_DUMMY_CHAR);  
  
  bytes[3] = SPI_ADS_SendByte(ADS_DUMMY_CHAR);   //MSB
  bytes[2] = SPI_ADS_SendByte(ADS_DUMMY_CHAR);
  bytes[1] = SPI_ADS_SendByte(ADS_DUMMY_CHAR);   //LSB
  
  ADS_CS_HIGH();
  
  tmp = (*((long *)bytes)) >> 12;       //本来右移8位就是实际的数值，但是可能会有一些低位的噪声位，所以需要选择一个合适的右移位数
  
  //tmp = (tmp > 32767) ? 32767 : (tmp < -32767) ? -32767 : tmp;

  int d = (int)(tmp);  
  
  aaa = d;
  
  ADS_DataCB(d);
}



/******************************************************************************
//执行一个命令
******************************************************************************/
static void execute(uint8 cmd)
{
  ADS_CS_LOW();
  
  SPI_ADS_SendByte(SDATAC);
  SPI_ADS_SendByte(cmd);
  
  Delay_us(10);
  ADS_CS_HIGH();
}

//使用内部测试信号
static void ADS1x9x_SetRegsAsTestSignal()
{
  ADS1x9x_WriteAllRegister(testRegs); 
}

static void ADS1x9x_SetRegsAsNormalECGSignal()
{
  ADS1x9x_WriteAllRegister(normalECGRegs);   
}

//延时us
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

#pragma vector = P0INT_VECTOR
__interrupt void PORT0_ISR(void)
{ 
  halIntState_t intState;
  HAL_ENTER_CRITICAL_SECTION( intState );  // Hold off interrupts.
  
  if(P0IFG & 0x02)  //P0.1中断
  {
    P0IFG &= ~(1<<1);   //clear P0IFG 1 
    P0IF = 0;   //clear P0 IFG
    
    readOneSample();
  }
  
  HAL_EXIT_CRITICAL_SECTION( intState );   // Re-enable interrupts.  
}
