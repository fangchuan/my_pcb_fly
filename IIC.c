#include  "r_cg_macrodriver.h"
#include  "IIC.h"
#include  "r_cg_port.h"
#include  "delay.h"

#define	SCL	P0.0
#define	SDA	P0.1

/* End user code. Do not edit comment generated here */

void delayus(uint ms)
{
    while(ms*50 > 0)
        ms--;
}

void I2C_Init(void)
{
    PM0.0 = 0;
    PM0.1 = 0;
    SCL = 1;
    delay_5us();
    SDA = 1;
    delay_5us();
}
void I2C_Start(void)
{
    PM0.0 = 0;
    PM0.1 = 0;
    SDA = 1;
    SCL = 1;
    delay_5us();
    SDA = 0;
    delay_5us();
    SCL = 0;
}

void I2C_Stop(void)
{

    PM0.1 = 0;
    SDA = 0;
    SCL = 1;
    delay_5us();
    SDA = 1;
    delay_5us();
}

void I2C_SendACK(uchar ack)
{
    PM0.1 = 0;
    SDA = ack;
    SCL = 1;
    delay_5us();
    SCL = 0;
    delay_5us();
}

unsigned char I2C_RecvACK(void)
{

    uchar err;
    SCL = 1;
    delay_5us();
    PM0.1 = 1;
    err = SDA;
    SCL = 0;
    delay_5us();
    return err;
}

void I2C_SendByte(uchar dat)
{
    uchar i;
    PM0.1 = 0;
    for (i = 0; i < 8; i++)
    {
        if(dat & 0x80)
            SDA = 1;
        else
            SDA = 0;
        dat <<= 1;
        SCL = 1;
        delay_5us();
        SCL = 0;
        delay_5us();
    }
    
}

uchar I2C_RecvByte(void)
{
    uchar i;
    uchar dat = 0;
    PM0.1 = 0;
    SDA = 1;
    PM0.1 = 1;
    for (i = 0; i < 8; i++)
    {
        dat <<= 1;
        SCL = 1;
        delay_5us();
        dat |= SDA;
        SCL = 0;
        delay_5us();
    }
    return dat;
}



unsigned char I2CWrite(unsigned char addr, unsigned char reg, unsigned char len, unsigned char * data)
{
    uchar ret=0;
    int i;
    I2C_Start();
    I2C_SendByte(addr << 1| 0);
    ret |= I2C_RecvACK();
    I2C_SendByte(reg);
    I2C_RecvACK();

    for (i = 0; i < len; i++)
    {
        I2C_SendByte(data[i]);
	ret |= I2C_RecvACK();
    }
    I2C_Stop();
    return ret;
}

unsigned char I2CRead(unsigned char addr, unsigned char reg, unsigned char len, unsigned char *buf)
{
    uchar  ret =0; 
    I2C_Start();
    I2C_SendByte(addr << 1| 0 );
    ret |= I2C_RecvACK();
    I2C_SendByte(reg);
    I2C_RecvACK();
    I2C_Start();
    I2C_SendByte(addr << 1| 0x01);
    ret |= I2C_RecvACK();

    while (len) {
        *buf = I2C_RecvByte();
        if (len == 1)
            I2C_SendACK(1);
        else
            I2C_SendACK(0);
        buf++;
        len--;
    }
    I2C_Stop();
    return ret;
}
