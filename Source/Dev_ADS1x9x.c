
#include "Dev_ADS1x9x.H"
#include "hal_mcu.h"


/*
 * �ֲ�����
*/

#define TYPE_ADS1191    0
#define TYPE_ADS1192    1
#define TYPE_ADS1291    2
#define TYPE_ADS1292    3


//�����ڲ������ź�ʱ�ļĴ���ֵ
const static uint8 test1mVRegs[12] = {  
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

// ��������ʱ�Ĵ�����ֵ
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


/*
 * �ֲ�����
*/
// �������������ȱʡ�Ĵ���ֵ
static uint8 defaultRegs[12];

// оƬ����
static uint8 type;

// ���ڱ���������ݺ�Ļص�����
static ADS_DataCB_t ADS_DataCB;

// ���յ���3�ֽ�״̬
static uint8 status[3] = {0};

//��ȡ��ͨ�������ֽ�
static uint8 data[4];



/****************************************************************
 * �ֲ�����
****************************************************************/

// us��ʱ
static void Delay_us(uint16 us);

// ִ������
static void execute(uint8 cmd);

// ��һ������ֵ
static void ADS1291_ReadOneSample(void);



/******************************************************************************
//ִ��һ������
******************************************************************************/
static void execute(uint8 cmd)
{
  ADS_CS_LOW();
  
  // ����ֹͣ��������
  SPI_ADS_SendByte(SDATAC);
  
  // ���͵�ǰ����
  SPI_ADS_SendByte(cmd);
  
  Delay_us(10);
  
  ADS_CS_HIGH();
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





/*************************************************************
 * ��������
****************************************************************/


// ADS ��ʼ��
extern void ADS1x9x_Init(ADS_DataCB_t pfnADS_DataCB_t)
{
  // ��ʼ��SPIģ��
  SPI_ADS_Init();  
  
  // ��ȱʡ�Ĵ���
  ADS1x9x_ReadAllRegister(defaultRegs); 
  
  type = (defaultRegs[0] & 0x03);
  
  // ���������ɼ��Ĵ���ֵ
  //ADS1x9x_SetRegsAsNormalECGSignal();
  // ���òɼ��ڲ������ź�ʱ�ļĴ���ֵ
  ADS1x9x_SetRegsAsTestSignal();
  
  //���ò������ݺ�Ļص�����
  ADS_DataCB = pfnADS_DataCB_t;
}

// ����
extern void ADS1x9x_WakeUp(void)
{
  execute(WAKEUP);
}

// ����
extern void ADS1x9x_StandBy(void)
{
  execute(STANDBY);
}


extern void ADS1x9x_Reset(void)
{
  ADS_RST_LOW();     //PWDN/RESET �͵�ƽ
  Delay_us(10);
  ADS_RST_HIGH();    //PWDN/RESET �ߵ�ƽ
  Delay_us(50);
}

// ��������
extern void ADS1x9x_StartConvert(void)
{
  //������������ģʽ
  ADS_CS_LOW();  
  SPI_ADS_SendByte(SDATAC);
  SPI_ADS_SendByte(RDATAC);  
  Delay_us(10);
  ADS_CS_HIGH();   
  
  //START �ߵ�ƽ
  ADS_START_HIGH();    
  Delay_us(16000); 
}

// ֹͣ����
extern void ADS1x9x_StopConvert(void)
{
  ADS_CS_LOW();  
  SPI_ADS_SendByte(SDATAC);
  Delay_us(10);
  ADS_CS_HIGH();   
  
  //START �͵�ƽ
  ADS_START_LOW();
  Delay_us(16000); 
}

// ������12���Ĵ���ֵ
extern void ADS1x9x_ReadAllRegister(uint8 * pRegs)
{
  ADS1x9x_ReadMultipleRegister(0x00, pRegs, 12);
}

// ������Ĵ���ֵ
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

// ��һ���Ĵ���ֵ
extern uint8 ADS1x9x_ReadRegister(uint8 address)
{
  uint8 result = 0;

  ADS_CS_LOW();
  
  SPI_ADS_SendByte(SDATAC);  
  SPI_ADS_SendByte(address | 0x20);         //����RREG����
  SPI_ADS_SendByte(0);                      //����Ϊ1
  result = SPI_ADS_SendByte(ADS_DUMMY_CHAR);   //���Ĵ���
  
  Delay_us(10);
  ADS_CS_HIGH();
  return result;
}

// д����12���Ĵ���
extern void ADS1x9x_WriteAllRegister(const uint8 * pRegs)
{
  ADS1x9x_WriteMultipleRegister(0x00, pRegs, 12);
}

//ʹ���ڲ������ź�
extern void ADS1x9x_SetRegsAsTestSignal()
{
  ADS1x9x_WriteAllRegister(test1mVRegs); 
}

extern void ADS1x9x_SetRegsAsNormalECGSignal()
{
  ADS1x9x_WriteAllRegister(normalECGRegs);   
}

// д����Ĵ���ֵ
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

// дһ���Ĵ���
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

#pragma vector = P0INT_VECTOR
__interrupt void PORT0_ISR(void)
{ 
  halIntState_t intState;
  HAL_ENTER_CRITICAL_SECTION( intState );  // Hold off interrupts.
  
  if(P0IFG & 0x02)  //P0_1�ж�
  {
    P0IFG &= ~(1<<1);   //clear P0_1 IFG 
    P0IF = 0;   //clear P0 interrupt flag
    
    if(type == TYPE_ADS1291)
      ADS1291_ReadOneSample();
  }
  
  HAL_EXIT_CRITICAL_SECTION( intState );   // Re-enable interrupts.  
}


/******************************************************************************
 * ��һ������
 * ADS1291�Ǹ߾��ȣ�24bit������ͨ��оƬ
******************************************************************************/
static void ADS1291_ReadOneSample(void)
{  
  ADS_CS_LOW();
  
  status[0] = SPI_ADS_SendByte(ADS_DUMMY_CHAR);
  status[1] = SPI_ADS_SendByte(ADS_DUMMY_CHAR);
  status[2] = SPI_ADS_SendByte(ADS_DUMMY_CHAR);  
  
  data[3] = SPI_ADS_SendByte(ADS_DUMMY_CHAR);   //MSB
  data[2] = SPI_ADS_SendByte(ADS_DUMMY_CHAR);
  data[1] = SPI_ADS_SendByte(ADS_DUMMY_CHAR);   //LSB
  
  ADS_CS_HIGH();
  
  long value = *((long *)data);
  
  // ����������
  value = (value >> 12);       //��������8λ����ʵ�ʵ���ֵ�����ǿ��ܻ���һЩ��λ������λ��������Ҫѡ��һ�����ʵ�����λ��
  
  //tmp = (tmp > 32767) ? 32767 : (tmp < -32767) ? -32767 : tmp;
  
  if(ADS_DataCB != 0)
    ADS_DataCB((int)(value));
}