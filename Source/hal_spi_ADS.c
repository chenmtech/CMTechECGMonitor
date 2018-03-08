
#include "hal_spi_ADS.h"

///静态函数
//ADS的IO配置,如DRDY, START, CS, PWDN等
static void IOCfgforADS();

//SPI的IO配置，采用SPI 1, alt.2，即：MI:P17, MO:P16, SCLK:P15
static void IOCfgforSPI();

//ADS专门的SPI配置
static void SPICfgforADS();



//ADS的IO配置,如DRDY, START, CS, PWDN等
static void IOCfgforADS()
{
  //P0.1 DRDY管脚配置  
  //先关P0.1即DRDY中断
  P0IEN &= ~(1<<1);
  P0IFG &= ~(1<<1);
  P0IF = 0;   //clear P0 IFG  
  //配置P0.1即DRDY中断
  P0SEL &= ~(1<<1); //GPIO
  P0DIR &= ~(1<<1); //Input
  PICTL |= (1<<0);  //下降沿触发
  //////////////////////////
  
  //P1.0 START管脚配置
  P1SEL &= ~(1<<0); //GPIO
  P1 &= ~(1<<0);    //初始为低电平
  P1DIR |= (1<<0);  //Output
  
  //P0.4 ~RESET管脚配置
  P0SEL &= ~(1<<4); //GPIO
  P0 |= (1<<4);    //初始为高电平
  P0DIR |= (1<<4);  //Output
  
  //P0.5 ~PWDN管脚配置
  P0SEL &= ~(1<<5); //GPIO
  P0 |= (1<<5);    //初始为高电平
  P0DIR |= (1<<5);  //Output  

  //P1.2 CS管脚配置  
  P1SEL &= ~(1<<2); //GPIO
  P1 |= (1<<2);     //初始为高电平
  P1DIR |= (1<<2);  //Output
  
  //开P0.1 DRDY中断
  P0IEN |= (1<<1);
  P0IE = 1;
}

//SPI的IO配置，采用SPI 1, alt.2，即：MI:P17, MO:P16, SCLK:P15
static void IOCfgforSPI()
{    
  //SPI 1 管脚配置
  P1SEL |= ((1<<5)|(1<<6)|(1<<7)); //P1.5 SCLK、P1.6 DIN和P1.7 DOUT为外设
  P1DIR |= ((1<<5)|(1<<6)); //P1.5 P1.6 Output
  P1DIR &= ~(1<<7); //P1.7 Input  
  PERCFG |= (1<<1); //SPI 1 use Alt.2
  P2SEL |= (1<<6);  //USART 1 has priority
  P2SEL &= ~(1<<5); //USART 1 has priority
}

//为ADS专门配置的SPI
static void SPICfgforADS()
{  
  URX1IE = 0; //disable SPI 1 RX interrupt
  URX1IF = 0;
  
  IEN2 &= ~(1<<3);  //disable SPI 1 TX interrupt
  UTX1IF = 0;  

  U1CSR = 0x00; //SPI 1 master mode
  
  U1GCR = 0x6E; // CPOL = 0, CPHA = 1, order = MSB, BAUD_E = 14
  U1BAUD = 0;  //BAUD_M = 0, SCLK = 512KHz
  
  //U1DBUF = 0x00;
}
///静态函数结束






///外部函数
//ADS芯片的所有SPI相关设置
extern void SPI_ADS_Setup()
{
  IOCfgforADS();
  IOCfgforSPI();
  SPICfgforADS();
}


//发送单字节
extern unsigned char SPI_ADS_SendByte(const unsigned char data)
{
  SPI_SEND(data); 
  while (!SPITXDONE);
  U1TX_BYTE = 0;
  return (U1DBUF);
}


//读指定大小的帧
extern unsigned char SPI_ADS_ReadFrame(unsigned char* pBuffer, unsigned int size)
{
  unsigned long i = 0;
  for (i = 0; i < size; i++){
    SPI_SEND(ADS_DUMMY_CHAR);
    while (!SPITXDONE);
    U1TX_BYTE = 0;
    pBuffer[i] = U1DBUF;
  } 
 return 0; 
}

//发送指定大小的帧
extern unsigned char SPI_ADS_SendFrame(const unsigned char* pBuffer, unsigned int size)
{
  unsigned long i = 0;
  for (i = 0; i < size; i++){
    SPI_SEND(pBuffer[i]);
    while (!SPITXDONE);
    U1TX_BYTE = 0;
  }  
  return 0;
}
///外部函数结束