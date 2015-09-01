#include "delay.h"

void delay_1us(void)
{
  unsigned char n;
  n = 1;
  for(; n>0; n--);
}

void delay_5us(void)
{ 
  unsigned char n;
  n = 5;
  for(; n>0; n--);
}

void delay_nus(unsigned long n)
{
     do{
        delay_1us();
     }while(n--);
}

void delay_1ms(void)
{
     unsigned char i,j;
     for(j=14; j>0; j--)
     for(i=250; i>0; i--)
     ;
     for(i=4; i>0; i--);
}

void delay_nms(unsigned long n)
{
     do{
        delay_1ms();
     }while(n--);
}