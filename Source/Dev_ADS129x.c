
#include "Dev_ADS129x.H"
#include "hal_mcu.h"

//#define ADS_IS_START()                  (ADS_START_PxOUT & ADS_START)



//��̬����

//�����ڲ������ź�ʱ�ļĴ���ֵ
const static uint8 testRegs[12] = {  
  /*ʹ���ڲ������ź�����*/
  0x52,
  //CONFIG1
  0x02,                     //contineus sample,500sps
  //CONFIG2
  0xA3,                     //1Hz����
  //LOFF
  0x10,                     //
  //CH1SET 
  0x15,                     //PGA=1, �����ź�
  //CH2SET
  0x80,                     //�ر�CH2
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
  0x80,                     //�ر�CH2
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

//��̬����
static uint8 defaultRegs[12];    //���������ȱʡ�ļĴ���ֵ

//�������ݺ�Ļص���������
static ADS_DataCB_t ADS_DataCB;

//ÿ����Ҫ��ȡ�������ֽڣ�����״̬�ֽں�ͨ������
static uint8 data[NUM_PER_SAMPLE];

//����һ��ͨ����ECG����
static uint8 bytes[4] = {0};

static long tmp = 0;

static int16 aaa = 0;

static uint8 tmp1 = 1;


/****************************************************************/
/* ��̬����*/
/****************************************************************/
static void Delay_us(uint16 us);

static void execute(uint8 cmd);

static void readOneSample(void);

//ʹ���ڲ������ź�
static void ADS1x9x_SetRegsAsTestSignal();

//�����ɼ�ECG�ź�
static void ADS1x9x_SetRegsAsNormalECGSignal();


extern void ADS1x9x_Init(ADS_DataCB_t pfnADS_DataCB_t)
{
  // ��ʼ��SPIģ��
  SPI_Init();  
  
  ADS1x9x_ReadAllRegister(defaultRegs); 
  
  //ADS1x9x_SetRegsAsNormalECGSignal();
  ADS1x9x_SetRegsAsTestSignal();
  
  //�����������ݴ�����
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
  P1 &= ~(1<<1);    //PWDN/RESET �͵�ƽ
  Delay_us(10);
  P1 |= (1<<1);    //PWDN/RESET �ߵ�ƽ
  Delay_us(50);
}

extern void ADS1x9x_StartConvert(void)
{
  //������������ģʽ
  ADS_CS_LOW();  
  SPI_ADS_SendByte(SDATAC);
  SPI_ADS_SendByte(RDATAC);  
  Delay_us(10);
  ADS_CS_HIGH();   
  
  P1 |= (1<<0);    //START �ߵ�ƽ
  Delay_us(16000); 
}

extern void ADS1x9x_StopConvert(void)
{
  ADS_CS_LOW();  
  SPI_ADS_SendByte(SDATAC);
  Delay_us(10);
  ADS_CS_HIGH();   
  
  P1 &= ~(1<<0);    //START �͵�ƽ 
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
  SPI_ADS_SendByte(beginaddr | 0x20);               //����RREG����
  SPI_ADS_SendByte(len-1);                          //����-1
  
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
  SPI_ADS_SendByte(address | 0x20);         //����RREG����
  SPI_ADS_SendByte(0);                      //����Ϊ1
  ret = SPI_ADS_SendByte(ADS_DUMMY_CHAR);   //���Ĵ���
  
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


//����Ϊ��̬����
/******************************************************************************
//��һ������
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
  
  tmp = (*((long *)bytes)) >> 12;       //��������8λ����ʵ�ʵ���ֵ�����ǿ��ܻ���һЩ��λ������λ��������Ҫѡ��һ�����ʵ�����λ��
  
  //tmp = (tmp > 32767) ? 32767 : (tmp < -32767) ? -32767 : tmp;

  int d = (int)(tmp);  
  
  aaa = d;
  
  ADS_DataCB(d);
}



/******************************************************************************
//ִ��һ������
******************************************************************************/
static void execute(uint8 cmd)
{
  ADS_CS_LOW();
  
  SPI_ADS_SendByte(SDATAC);
  SPI_ADS_SendByte(cmd);
  
  Delay_us(10);
  ADS_CS_HIGH();
}

//ʹ���ڲ������ź�
static void ADS1x9x_SetRegsAsTestSignal()
{
  ADS1x9x_WriteAllRegister(testRegs); 
}

static void ADS1x9x_SetRegsAsNormalECGSignal()
{
  ADS1x9x_WriteAllRegister(normalECGRegs);   
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

#pragma vector = P0INT_VECTOR
__interrupt void PORT0_ISR(void)
{ 
  halIntState_t intState;
  HAL_ENTER_CRITICAL_SECTION( intState );  // Hold off interrupts.
  
  if(P0IFG & 0x02)  //P0.1�ж�
  {
    P0IFG &= ~(1<<1);   //clear P0IFG 1 
    P0IF = 0;   //clear P0 IFG
    
    readOneSample();
  }
  
  HAL_EXIT_CRITICAL_SECTION( intState );   // Re-enable interrupts.  
}
