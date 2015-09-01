#include "r_cg_macrodriver.h"
#include "r_cg_adc.h"
#include "r_cg_port.h"
#include "ccd.h"
#include "r_cg_serial.h"



extern uint8_t ad_end;

uint16_t threshold=0;
uint8_t image_data[128];

void CCD_init(void)
{
	CCD_SI = 1;
	CCD_CLK = 1;
  	R_ADC_Set_OperationOn(); 
}

void SamplingDelay()
{
//	NOP();
//	NOP();
	NOP();
//	NOP();
}

uint8_t ad_get(void)
{
	uint8_t ad_data;
	R_ADC_Start();
	while(ad_end);
	ad_data=ADCRH;
//	if(ad_max<ad_data)
//		ad_max=ad_data;
//	threshold = ad_max *4/5;
	ad_end=1;
	return ad_data;
}

void ImageCapture(unsigned char * ImageData) 
{

    unsigned char i;

    CCD_SI=1;            /* SI  = 1 */
    SamplingDelay();
    CCD_CLK=1;           /* CLK = 1 */
    SamplingDelay();
    CCD_SI=0;            /* SI  = 0 */
    SamplingDelay();

    //Delay 10us for sample the first pixel
    /**/
    for(i = 0; i < 200; i++) {                  
      SamplingDelay() ;  //200ns         
    }

    //Sampling Pixel 1

    *ImageData =  ad_get();
    ImageData ++ ;
    CCD_CLK=0;;           /* CLK = 0 */

    for(i=0; i<127; i++) {
        SamplingDelay();
        SamplingDelay();
        CCD_CLK=1;       /* CLK = 1 */
        SamplingDelay();
        SamplingDelay();
        //Sampling Pixel 2~128

       *ImageData =  ad_get();	    
        ImageData ++ ;
        CCD_CLK=0;      /* CLK = 0 */
    }
    SamplingDelay();
    SamplingDelay();
    CCD_CLK=1;           /* CLK = 1 */
    SamplingDelay();
    SamplingDelay();
    CCD_CLK=0;           /* CLK = 0 */
}

void Binary_Image(uint8_t image[])
{    
     uint8_t i=0;
     uint8_t ad_max=0,ad_min=0xff;
     for (;i<128;i=i+4){
           if(image[i] > ad_max)
	       ad_max = image[i];
	   if(image[i] < ad_min)
	       ad_min = image[i];
     }
     if((ad_max - ad_min) < 51)
	     threshold = 150;
     else
     threshold = (uint16_t)(ad_max-ad_min)/3 + ad_min;
     for (i=0;i<128;i++){
          if(image[i] > threshold)
	      image[i] = WHITE;
	  else
	      image[i] = BLACK;
     }
}

uint16_t blackline_center=0;
int8_t blackline_width=0;
uint8_t start_position_flag=0;
uint8_t finish_position_flag=0;
uint8_t turn_left_position_flag=0;
uint8_t leftblack=0,rightblack=0;

int in_lost = 0;
int lost_pos = 0;

#define LEFT_BORDER_EXTREM 0
#define LEFT_BORDER 10
#define RIGHT_BORDER 118
#define RIGHT_BORDER_EXTREM 128
#define VALID_WIDTH (RIGHT_BORDER - LEFT_BORDER)

void CCD_Data_Process(unsigned char ImageData[])
{
  
  unsigned int i;
  //static  uint16_t old_center=0;
  //static  uint8_t only_left_find_times=0;
  //static  uint8_t only_right_find_times=0;
  //static  uint8_t left_line_lost_flag=0,right_line_lost_flag=0;
  int cur_pos = 0;
  
  for(i = LEFT_BORDER;i<RIGHT_BORDER;i++)
  {
        if((ImageData[i]==BLACK)&&(ImageData[i+1]==BLACK)&&(ImageData[i+2]==BLACK)/*&&(ImageData[i+3]==BLACK)*/) 
        {
	    leftblack = i;
            break;
        }
	else
	    leftblack =LEFT_BORDER;//all white
  }
    
    
  for(i = RIGHT_BORDER;i>=leftblack;i--)
  {
        if((ImageData[i]==BLACK)&&(ImageData[i-1]==BLACK)&&(ImageData[i-2]==BLACK)/*&&(ImageData[i-3]==BLACK)*/)    
        {
	    rightblack = i;
            break;
        }
	else
	    rightblack = RIGHT_BORDER_EXTREM;//all white
  }
  
  
  blackline_width = rightblack - leftblack;
  if( blackline_width <= VALID_WIDTH )
  {
      uint16_t tmp_center = (uint16_t)(leftblack + rightblack) / 2;
      
      if(in_lost)
      {
          cur_pos = tmp_center < LEFT_BORDER + VALID_WIDTH / 3 ? -1
                : ( tmp_center > RIGHT_BORDER - VALID_WIDTH / 3 ? 1 : 0 );
          if(lost_pos == cur_pos)
          {
              blackline_center = tmp_center;
              in_lost = 0;
          }
      }
      else
      {
          blackline_center = tmp_center;
      }
      
      
   }
   else
   {        
//      if((leftblack == 10)&&(rightblack == 128)){
        if(!in_lost)
        {
            in_lost = 1;
            lost_pos = blackline_center < LEFT_BORDER + VALID_WIDTH / 3 ? -1
                : ( blackline_center > RIGHT_BORDER - VALID_WIDTH / 3 ? 1 : 0 );
            if(lost_pos == 1)
            {
                blackline_center = RIGHT_BORDER_EXTREM;
            }
            else if(lost_pos == -1)
            {
                blackline_center = LEFT_BORDER_EXTREM;
            }
        }
   }
             
 }
 
void update_ccd(void)
{
     ImageCapture(image_data);
     Binary_Image(image_data);
//     SendImageData(image_data)£»
     CCD_Data_Process(image_data);
}

void SendImageData(unsigned char * ImageData) 
{

    unsigned char i;
    unsigned char crc = 0;

    /* Send Data */
    uart_putchar('*');
    uart_putchar('L');
    uart_putchar('D');

    SendHex(0);
    SendHex(0);
    SendHex(0);
    SendHex(0);

    for(i=0; i<128; i++) {
      SendHex(*ImageData++);
    }

    SendHex(crc);
    uart_putchar('#');
}


void SendHex(unsigned char hex) 
{
	  unsigned char temp;
	  temp = hex >> 4;
	  if(temp < 10) 
	  {
	    uart_putchar(temp + '0');
	  } 
	  else 
	  {
	    uart_putchar(temp - 10 + 'A');
	  }
	  temp = hex & 0x0F;
	  if(temp < 10) 
	  {
	    uart_putchar(temp + '0');
	  } 
	  else 
	  {
	   uart_putchar(temp - 10 + 'A');
	  }
}

void uart_putchar(uint8_t cha)
{	
	R_UART0_Send_Block(&cha,1);	
}