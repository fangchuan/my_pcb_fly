#ifndef _AHRS_H
#define _AHRS_H

#include "r_cg_macrodriver.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "ArduCopter.h"
#include "Vector3f.h"
#include "Matrix3f.h"
#include "AP_Math.h"
#include "r_cg_intc.h"
#include "r_cg_port.h"
#include "IIC.h"
#include "r_cg_serial.h"
#include "r_cg_it.h"

#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)

struct hal_s
{
    unsigned char sensors;
    unsigned char dmp_on;
    volatile unsigned char new_gyro;
    unsigned short report;
    unsigned short dmp_features;
};


typedef struct{
	Matrix3f dcm_matrixs;
	float X;
	float Y;
	float Z;
        float X_OFFSET;
	float Y_OFFSET;
        float Z_OFFSET;
	short gyro[3];
	long quat[4];
	short accel[3];
	short sensors;
	unsigned char more;
       }_Q_ANGLE;
		

#define Gyro_250_Scale_Factor   131.0f
#define Gyro_500_Scale_Factor   65.5f
#define Gyro_1000_Scale_Factor  32.8f
#define Gyro_2000_Scale_Factor  16.4f
#define Accel_2_Scale_Factor    16384.0f
#define Accel_4_Scale_Factor    8192.0f
#define Accel_8_Scale_Factor    4096.0f
#define Accel_16_Scale_Factor   2048.0f

/* Exported function prototypes ---------------------------------------------*/
extern Vector3f omega;


extern int  dmp_init(void);
extern void mpu_tool(void);
extern void gyro_data_ready_cb(void);
extern void optical_flow_update(void);


#endif