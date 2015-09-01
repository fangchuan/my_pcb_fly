#ifndef _CCD_H
#define _CCD_H

#define WHITE 220
#define BLACK 0

extern int8_t blackline_width;
extern int in_lost;

void CCD_init(void);
void ImageCapture(unsigned char * ImageData);
void update_ccd(void);
void SamplingDelay();
void uart_putchar(uint8_t cha);
void SendHex(unsigned char hex);
void SendImageData(unsigned char * ImageData);



#endif