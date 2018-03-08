//��CC2541��֧��ADS129X��SPI�ӿڳ���

#ifndef SPI_ADS_H
#define SPI_ADS_H

#include <iocc2541.h>


//���ַ������ڶ�������ʱ���͵��޹��ֽ�
#define ADS_DUMMY_CHAR 0x00         //ע����ЩоƬҪ���ڶ�����ʱ���͵͵�ƽ�źţ���ЩҪ���͸ߵ�ƽ�ź�

#define SPI_SEND(x)           U1DBUF=x
#define SPITXDONE             (U1TX_BYTE == 1)        //SPI 1�������


#define ADS_CS_LOW()    P1 &= ~(1<<2)     //CS�͵�ƽ               
#define ADS_CS_HIGH()   P1 |= (1<<2)      //CS�ߵ�ƽ 



///�ⲿ����
//ADSоƬ������SPI�������
extern void SPI_ADS_Setup();

//���͵����ֽ�
extern unsigned char SPI_ADS_SendByte(const unsigned char data);

//��ȡ����ֽ�
extern unsigned char SPI_ADS_ReadFrame(unsigned char* pBuffer, unsigned int size);

//���Ͷ���ֽ�
extern unsigned char SPI_ADS_SendFrame(const unsigned char* pBuffer, unsigned int size);
///�ⲿ��������

#endif

