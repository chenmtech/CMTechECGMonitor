/*
* hal_spi_ADS.h : CC2541��֧��ADS129XоƬ��SPI�ӿڳ���ͷ�ļ�
*/

#ifndef SPI_ADS_H
#define SPI_ADS_H

#include <iocc2541.h>

/*
 * �������
*/

//���ַ������ڶ�������ʱ���͵��޹��ֽ�
#define ADS_DUMMY_CHAR 0x00         //ע����ЩоƬҪ���ڶ�����ʱ���͵͵�ƽ�źţ���ЩҪ���͸ߵ�ƽ�ź�

// ����x
#define SPI_SEND(x)           U1DBUF = x

// �������
#define SPITXDONE             (U1TX_BYTE == 1)        //SPI 1�������

// ƬѡCS PIN : P1_2
// ƬѡCS�͵�ƽ
#define ADS_CS_LOW()    P1 &= ~(1<<2)                    

// ƬѡCS�ߵ�ƽ
#define ADS_CS_HIGH()   P1 |= (1<<2) 



/*
 * ��������
*/

//SPI��ʼ��
extern void SPI_Init();

//���͵����ֽ�
extern unsigned char SPI_ADS_SendByte(const unsigned char data);

//��ȡ����ֽ�
extern void SPI_ADS_ReadFrame(unsigned char* pBuffer, unsigned int size);

//���Ͷ���ֽ�
extern void SPI_ADS_SendFrame(const unsigned char* pBuffer, unsigned int size);



#endif

