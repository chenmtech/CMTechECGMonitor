
#include "hal_spi_ADS.h"





/*
 * �ֲ�����
*/

//ADSоƬ����IO������, ��DRDY, START, CS, PWDN��
static void spi_setupADS();

//SPIͨ��������ã�����SPI 1, alt.2������MI:P17, MO:P16, SCLK:P15
static void spi_setupSPI();



//ADSоƬ����IO������, ��DRDY, START, CS, PWDN��
static void spi_setupADS()
{
  //P0.1 DRDY�ܽ�����  
  //�ȹ�P0.1��DRDY�ж�
  P0IEN &= ~(1<<1);
  P0IFG &= ~(1<<1);
  P0IF = 0;   //clear P0 IFG  
  
  //����P0.1��DRDY �ж�
  P0SEL &= ~(1<<1); //GPIO
  P0DIR &= ~(1<<1); //Input
  PICTL |= (1<<0);  //�½��ش���
  //////////////////////////
  
  //P1.0 START�ܽ�����
  P1SEL &= ~(1<<0); //GPIO
  P1 &= ~(1<<0);    //��ʼΪ�͵�ƽ
  P1DIR |= (1<<0);  //Output
  
  //P1.1 ~PWDN/RESET�ܽ�����
  P1SEL &= ~(1<<1); //GPIO
  P1 |= (1<<1);    //��ʼΪ�ߵ�ƽ
  P1DIR |= (1<<1);  //Output

  //P1.2 ~CS�ܽ�����  
  P1SEL &= ~(1<<2); //GPIO
  P1 |= (1<<2);     //��ʼΪ�ߵ�ƽ
  P1DIR |= (1<<2);  //Output
  
  //��P0.1 DRDY�ж�
  P0IEN |= (1<<1);
  P0IE = 1;
}

//SPIͨ��������ã�����SPI 1, alt.2������MI:P17, MO:P16, SCLK:P15
static void spi_setupSPI()
{    
  //SPI 1 �ܽ�����
  P1SEL |= ((1<<5)|(1<<6)|(1<<7)); //P1.5 SCLK��P1.6 MOSI��P1.7 MISOΪ����
  P1DIR |= ((1<<5)|(1<<6)); //P1.5 SCLK, P1.6 MOSI Output
  P1DIR &= ~(1<<7); //P1.7 MISO Input  
  
  PERCFG |= (1<<1); //SPI 1 ʹ�� Alt.2
  P2SEL |= (1<<6);  //USART 1 has priority than USART 0
  P2SEL &= ~(1<<5); //USART 1 has priority than TIMER 3
  
  URX1IE = 0; //disable SPI 1 RX interrupt
  URX1IF = 0;
  
  IEN2 &= ~(1<<3);  //disable SPI 1 TX interrupt
  UTX1IF = 0;  

  U1CSR = 0x00; //SPI 1 master mode
  
  // SPIʱ��Ƶ��Ϊ512KHz
  U1GCR = 0x6E; // CPOL = 0, CPHA = 1, order = MSB, BAUD_E = 14
  U1BAUD = 0;   //BAUD_M = 0, SCLK = 512KHz
  
  //U1DBUF = 0x00;
}











/*
 * ��������
*/

//SPI��ʼ��
extern void SPI_Init()
{
  spi_setupADS();
  spi_setupSPI();
}


//���͵��ֽ�
extern unsigned char SPI_ADS_SendByte(const unsigned char data)
{
  SPI_SEND(data); 
  while (!SPITXDONE);
  U1TX_BYTE = 0;
  return (U1DBUF);
}


//��ָ����С��֡
extern void SPI_ADS_ReadFrame(unsigned char* pBuffer, unsigned int size)
{
  unsigned int i = 0;
  for (i = 0; i < size; i++){
    SPI_SEND(ADS_DUMMY_CHAR);
    while (!SPITXDONE);
    U1TX_BYTE = 0;
    pBuffer[i] = U1DBUF;
  } 
 return; 
}

//����ָ����С��֡
extern void SPI_ADS_SendFrame(const unsigned char* pBuffer, unsigned int size)
{
  unsigned int i = 0;
  for (i = 0; i < size; i++){
    SPI_SEND(pBuffer[i]);
    while (!SPITXDONE);
    U1TX_BYTE = 0;
  }  
  return;
}