/*
 *       AP_OpticalFlow_ADNS3080.cpp - ADNS3080 OpticalFlow Library for
 *       Ardupilot Mega
 *       Code by Randy Mackay. DIYDrones.com
 *
 */
#include "r_cg_macrodriver.h"
#include "r_cg_port.h"
#include "r_cg_it.h"
#include "AP_OpticalFlow_ADNS3080.h"
#include "Vector3f.h"
#include "delay.h"


enum Rotation _orientation;
extern float vlon,vlat;
struct AP_OpticalFlow_Flags {
        uint8_t healthy;    // true if sensor is healthy
    } _flags;


//MISO==P1.5,MOSI==P1.4,CS==P1.0,SCK==P1.3 ,RESET==P7.1
void SPI_Init()
{
    AP_OpticalFlow_ADNS3080_init();
}
char SPI_RW(char data)
{
     char i;
     char temp=0;
	
     for(i=0;i<8;i++) // output 8-bit
    {
        OP_SCK_L();
	if((data & 0x80)==0x80)
	{
	    OP_MOSI_H();         // output 'uchar', MSB to MOSI
	}
	else
	{
	    OP_MOSI_L();
	}	
	data = (data << 1);            // shift next bit into MSB..
	temp = (temp << 1);
	OP_SCK_H();                // Set SCK high..
	if(OP_MISO == 1)temp++;         // capture current MISO bit
	OP_SCK_H();
   	}
		              // ..then set SCK low again
    return(temp);           		  // return read uchar  
}

char SPI_Read(char reg)
{
	char reg_val=0;
	OP_CS_L();           // CSN low, initialize SPI communication...
	SPI_RW(reg);            // Select register to read from..
	delay_nus(50);
	reg_val = SPI_RW(0);    // ..then read registervalue
	OP_CS_H();           // CSN high, terminate SPI communication
	return(reg_val);       // return register value
}

char SPI_RW_Reg(char reg, char value)
{
	char status1;
	reg=0x80|reg;
	OP_CS_L();                   // CSN low, init SPI transaction
	status1 = SPI_RW(reg);      // select register
	
	SPI_RW(value);             // ..and write value to it..
	OP_CS_H();                   // CSN high again
	return(status1);            // return nRF24L01 status uchar
}

char SPI_Read_Buf(char reg, char *pBuf, char chars)
{
	char status2,uchar_ctr;
	OP_CS_L();                    		// Set CSN low, init SPI tranaction
	status2 = SPI_RW(reg);       		// Select register to write to and read status uchar
	for(uchar_ctr=0;uchar_ctr<chars;uchar_ctr++)
        {
	    pBuf[uchar_ctr] = SPI_RW(0);    // 
        }
	OP_CS_H();                           
	return(status2);   
}

char SPI_Write_Buf(char reg, char *pBuf, char chars)
{
	char status1,uchar_ctr;
	OP_CS_L();               
	status1 = SPI_RW(reg);   
	for(uchar_ctr=0; uchar_ctr<chars; uchar_ctr++) //
        {
	    SPI_RW(*pBuf++);
        }
	OP_CS_H();           
	return(status1);    		 
}

void Reset_ADNS3080()
{
	OP_RESET_H();
	delay_nus(10);
	OP_RESET_L();
}
// Public Methods //////////////////////////////////////////////////////////////
// init - initialise sensor
// assumes SPI bus has been initialised but will attempt to initialise 
// nonstandard SPI3 bus if required
void AP_OpticalFlow_ADNS3080_init()
{
    int8_t retry = 0;
    _flags.healthy = false;

    Reset_ADNS3080();
    OP_CS_H();
    OP_SCK_L();
    delay_nms(10);
    // check 3 times for the sensor on standard SPI bus
     while (!_flags.healthy && retry < 3) {
        if (SPI_Read(ADNS3080_PRODUCT_ID) == 0x17) {
             _flags.healthy = true;
            }
            retry++;
        }
//     field_of_view = AP_OPTICALFLOW_ADNS3080_08_FOV;
				
    // configure the sensor
    if (_flags.healthy) {
        // set frame rate to manual
        uint8_t regVal = SPI_Read(ADNS3080_EXTENDED_CONFIG);
        delay_nus(50);
        regVal = (regVal & ~0x01) | 0x01;
        SPI_RW_Reg(ADNS3080_EXTENDED_CONFIG, regVal);
        delay_nus(50);
       
        // set frame period to 12000 (0x2e20)
        SPI_RW_Reg(ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER,0x20);
        delay_nus(50);
        SPI_RW_Reg(ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER,0x2E);
        delay_nus(50);

        // set 1600 resolution bit
        regVal = SPI_Read(ADNS3080_CONFIGURATION_BITS);
        delay_nus(50);
        regVal |= 0x10;
        SPI_RW_Reg(ADNS3080_CONFIGURATION_BITS, regVal);
        delay_nus(50);

        // update scalers
        update_conversion_factors();

        // register the global static read function to be called at 1khz
        //hal.scheduler->register_timer_process(AP_HAL_MEMBERPROC(&AP_OpticalFlow_ADNS3080::read));
    }else{
        // no connection available.
          BEEP_ON;
         }
				 
     _orientation=ROTATION_NONE;

}


// read latest values from sensor and fill in x,y and totals
void AP_OpticalFlow_ADNS3080_update(void)
{
    Vector3f rot_vector;
    uint8_t motion_reg;
    int16_t  raw_dx, raw_dy;    // raw sensor change in x and y position (i.e. unrotated)
    surface_quality = SPI_Read(ADNS3080_SQUAL);
    
    delay_nus(50);

    // check for movement, update x,y values
    motion_reg = SPI_Read(ADNS3080_MOTION);	  
	 
    if (((motion_reg & 0x80) != 0) ) {
        raw_dx = ((int8_t)SPI_Read(ADNS3080_DELTA_X));
			  
        delay_nus(50);
        raw_dy = ((int8_t)SPI_Read(ADNS3080_DELTA_Y));
			  
    }else{
        raw_dx = 0;
        raw_dy = 0;
    }

//    last_update = millisecond();

    vector3f_init(&rot_vector,raw_dx, raw_dy, 0);

    // rotate dx and dy
    rot_vector = Vector3_rotate(_orientation,rot_vector);
    dx = rot_vector.x;
    dy = rot_vector.y;

}

// clear_motion - will cause the Delta_X, Delta_Y, and internal motion
// registers to be cleared
void clear_motion()
{
    // writing anything to this register will clear the sensor's motion
    // registers
    SPI_RW_Reg(ADNS3080_MOTION_CLEAR,0xFF); 
    x_cm = 0;
    y_cm = 0;
    dx = 0;
    dy = 0;
    vlon = 0;
    vlat = 0;
}

// get_pixel_data - captures an image from the sensor and stores it to the
// pixe_data array
//void print_pixel_data()
//{
//    int16_t i,j;
//    char isFirstPixel = true;
//    uint8_t regValue;
//    uint8_t pixelValue;
//
//    // write to frame capture register to force capture of frame
//    SPI_RW_Reg(ADNS3080_FRAME_CAPTURE,0x83);
//
//    // wait 3 frame periods + 10 nanoseconds for frame to be captured
//    // min frame speed is 2000 frames/second so 1 frame = 500 nano seconds.
//    // so 500 x 3 + 10 = 1510
//    delay_nus(1510);
//
//    // display the pixel data
//    for (i=0; i<ADNS3080_PIXELS_Y; i++) {
//        for (j=0; j<ADNS3080_PIXELS_X; j++) {
//            regValue = SPI_Read(ADNS3080_FRAME_CAPTURE);
//            if (isFirstPixel && (regValue & 0x40) == 0) {
//
//            }
//            isFirstPixel = false;
//            pixelValue = ( regValue << 2 );
//            //hal.console->print(pixelValue,BASE_DEC);
//            if (j!= ADNS3080_PIXELS_X-1)
//              
//                delay_nus(50);
//        }
//      
//    }
//}

// updates conversion factors that are dependent upon field_of_view
void update_conversion_factors()
{
    // multiply this number by altitude and pixel change to get horizontal
    // move (in same units as altitude)
    //conv_factor = ((1.0f / (float)(ADNS3080_PIXELS_X * AP_OPTICALFLOW_ADNS3080_SCALER_1600))
    //               * 2.0f * tanf(field_of_view / 2.0f));
    conv_factor = 0.00615;
    //radians_to_pixels = (ADNS3080_PIXELS_X * AP_OPTICALFLOW_ADNS3080_SCALER_1600) / field_of_view;
    radians_to_pixels = 162.99;
}
