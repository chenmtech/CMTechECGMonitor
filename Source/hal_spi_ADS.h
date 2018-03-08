/*
* hal_spi_ADS.h : CC2541上支持ADS129X芯片的SPI接口程序头文件
*/

#ifndef SPI_ADS_H
#define SPI_ADS_H

#include <iocc2541.h>

/*
 * 常量或宏
*/

//哑字符，用于读出数据时发送的无关字节
#define ADS_DUMMY_CHAR 0x00         //注意有些芯片要求在读数据时发送低电平信号，有些要求发送高电平信号

// 发送x
#define SPI_SEND(x)           U1DBUF = x

// 发送完毕
#define SPITXDONE             (U1TX_BYTE == 1)        //SPI 1发送完毕

// 片选CS PIN : P1_2
// 片选CS低电平
#define ADS_CS_LOW()    P1 &= ~(1<<2)                    

// 片选CS高电平
#define ADS_CS_HIGH()   P1 |= (1<<2) 



/*
 * 公共函数
*/

//SPI初始化
extern void SPI_Init();

//发送单个字节
extern unsigned char SPI_ADS_SendByte(const unsigned char data);

//读取多个字节
extern void SPI_ADS_ReadFrame(unsigned char* pBuffer, unsigned int size);

//发送多个字节
extern void SPI_ADS_SendFrame(const unsigned char* pBuffer, unsigned int size);



#endif

