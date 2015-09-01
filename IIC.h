#ifndef _IIC_H
#define _IIC_H

typedef unsigned char  uchar;
typedef unsigned short ushort;
typedef unsigned int   uint;


void delayus(uint ms);
void I2C_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_SendACK(uchar ack);
unsigned char I2C_RecvACK(void);
void I2C_SendByte(uchar dat);
unsigned char I2C_RecvByte(void);

unsigned char I2CWrite(unsigned char addr, unsigned char reg, unsigned char len, unsigned char * data);
unsigned char I2CRead(unsigned char addr, unsigned char reg, unsigned char len, unsigned char *buf);


#endif



