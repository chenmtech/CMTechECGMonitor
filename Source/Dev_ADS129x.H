/*
 * 实现ADS129x芯片的所有功能
 * 主要包括：1、读写寄存器；2、控制ADS129x; 3、读采样数据

*/

#ifndef DEV_ADS129X_H
#define DEV_ADS129X_H

#include "hal_spi_ADS.h"
#include <hal_types.h>

//System Commands
#define WAKEUP		0x02		//Wake-up from standby mode
#define STANDBY	        0x04	        //Enter standby mode
#define RESET		0x06		//Reset the device
#define START		0x08		//Start/restart (synchronize) conversions
#define STOP		0x0A		//Stop conversion
#define OFFSETCAL       0x1A            //Channel offset calibration

// Data Read Commands
#define RDATAC		0x10		//Enable Read Data Continuous mode.
                                        //This mode is the default mode at power-up.
#define SDATAC		0x11		//Stop Read Data Continuously mode
#define RDATA		0x12		//Read data by command; supports multiple read back.

// Register Read Commands
#define RREG		0x20		//Read n nnnn registers starting at address r rrrr
                                        //first byte 001r rrrr (2xh)(2) - second byte 000n nnnn(2)
#define WREG		0x40		//Write n nnnn registers starting at address r rrrr
                                        //first byte 010r rrrr (2xh)(2) - second byte 000n nnnn(2)


//设备寄存器地址
//Device Settings(Read-Only Registers)
#define ADS1x9x_REG_DEVID               (0x0000u)

//Global Settings Across Channels
#define ADS1x9x_REG_CONFIG1             (0x0001u)
#define ADS1x9x_REG_CONFIG2             (0x0002u)
#define ADS1x9x_REG_LOFF                (0x0003u)

//Channel-Specific Settings
#define ADS1x9x_REG_CH1SET              (0x0004u)
#define ADS1x9x_REG_CH2SET              (0x0005u)
#define ADS1x9x_REG_RLD_SENS            (0x0006u)
#define ADS1x9x_REG_LOFF_SENS           (0x0007u)
#define ADS1x9x_REG_LOFF_STAT           (0x0008u)

//GPIO and Other Registers
#define ADS1x9x_REG_RESP1               (0x0009u)
#define ADS1x9x_REG_RESP2               (0x000Au)
#define ADS1x9x_REG_GPIO                (0x000Bu)



#define NUM_PER_SAMPLE  6               //ADS1291每次接收数据长度为6个字节：状态3字节，通道1为3字节，通道2没有输出




//采样到数据后的回调处理函数类型
typedef void (*ADS_DataCB_t)(int oneData); 


/****************************************************************/
/* extern functions*/
/****************************************************************/
extern void ADS1x9x_Init(ADS_DataCB_t pfnADS_DataCB_t);
extern void ADS1x9x_WakeUp(void);
extern void ADS1x9x_StandBy(void);
extern void ADS1x9x_Reset(void);
extern void ADS1x9x_StartConvert(void);
extern void ADS1x9x_StopConvert(void);
extern uint8 ADS1x9x_ReadRegister(uint8 address);
extern void ADS1x9x_ReadMultipleRegister(uint8 beginaddr, uint8 * pRegs, uint8 len);
extern void ADS1x9x_ReadAllRegister(uint8 * pRegs);
extern void ADS1x9x_WriteRegister(uint8 address, uint8 onebyte);
extern void ADS1x9x_WriteMultipleRegister(uint8 beginaddr, const uint8 * pRegs, uint8 len);
extern void ADS1x9x_WriteAllRegister(const uint8 * pRegs);

#endif
