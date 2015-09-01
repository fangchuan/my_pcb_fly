#include "ahrs.h"

_Q_ANGLE Q_ANGLE;
struct hal_s hal;

Vector3f omega={0};
//Vector3f accel={0};

#define OMEGATRANSFORMATION 0.06103515625f
#define M_PI   3.14159265358979323846
#define q30  1073741824.0f

char data[50];

float dmpsafe_asin(float v)
{
	if(isnan(v)){
		return 0.0;
	}
	if (v >= 1.0) {
		return M_PI/2;
	}
	if (v <= -1.0) {
		return -M_PI/2;
	}
	return asin(v);
}
float invSqrt(float x) 
{	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
 void print_dmp(long *quat)
 {
     float q[4], norm;
     float roll, pitch, yaw;
     float t13, t21, t22, t23, t33;
     float costheta;
     
     q[0] = (float)quat[0];
     q[1] = (float)quat[1];
     q[2] = (float)quat[2];
     q[3] = (float)quat[3];
     
     norm = invSqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);	
     q[0] = q[0]*norm;	
     q[1] = q[1]*norm;	
     q[2] = q[2]*norm;	
     q[3] = q[3]*norm;
     
     t13 = 2 * (q[1] * q[3] - q[0] * q[2]);
     t21 = 2 * (q[1] * q[2] - q[0] * q[3]);
     t22 = q[0] * q[0] + q[2] * q[2] - q[1] * q[1] - q[3] * q[3];
     t23 = 2 * (q[2] * q[3] + q[0] * q[1]);
     t33 = q[0] * q[0] + q[3] * q[3] - q[1] * q[1] - q[2] * q[2];
     
     pitch = asin(t23);
     costheta = cos(pitch);
     if(costheta == 0)
     {
         costheta = 1;
     }
     yaw = atan2(-t21 / costheta, t22 / costheta);
     roll = atan2(-t13 / costheta, t33 / costheta);
     Q_ANGLE.Y =  pitch*180/M_PI;
     Q_ANGLE.X = roll*180/M_PI;
     Q_ANGLE.Z = yaw*180/M_PI;
}
void quaterion_to_euler(long *data)
{  
     double q[4];
//     float  R3[3];
     q[0] = (data[0])*1.0f/q30;
     q[1] = (data[1])*1.0f/q30;
     q[2] = (data[2])*1.0f/q30;
     q[3] = (data[3])*1.0f/q30;
     
     Q_ANGLE.X = - (atan2(2.0*(q[0]*q[1] + q[2]*q[3]),1-2*(q[1]*q[1] + q[2]*q[2])))* 180/M_PI;
	 // we let safe_asin() handle the singularities near 90/-90 in pitch
     Q_ANGLE.Y = - dmpsafe_asin(2.0*(q[0]*q[2] - q[3]*q[1]))* 180/M_PI;

     Q_ANGLE.Z = (atan2(2.0*(q[0]*q[3] + q[1]*q[2]),1-2*(q[2]*q[2] + q[3]*q[3])))* 180/M_PI;
     
//     R3[0]=2*(q[1]*q[3]-q[2]*q[0]);
//     R3[1]=2*(q[2]*q[3]+q[0]*q[1]);
//     R3[2]=q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3];
//       
//     accel.z=((R3[0]*Q_ANGLE.accel[0]+R3[1]*Q_ANGLE.accel[1]+R3[2]*Q_ANGLE.accel[2])/Accel_2_Scale_Factor-1.0f)*9.8f;
}
/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static  unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static  unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}
static signed char gyro_orientation[9] = 
{
    -1, 0, 0,
    0, -1, 0,
    0, 0, 1
};
void gyro_data_ready_cb(void)
{
    hal.new_gyro = 1;
}

int dmp_init(void){
    int result;
    struct int_param_s int_param;
    unsigned short gyro_rate;
    int_param.cb = gyro_data_ready_cb;
    int_param.int0=R_INTC0_Start;
    result = mpu_init(&int_param);
    if (result)
    {
        R_UART0_Send_Block("mpu init fail!!!!\n",strlen("mpu init fail!!!!\n"));
        return 1;
    }

    /* Wake up all sensors. */
    if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS))
    {
         R_UART0_Send_Block("mpu_set_sensors fail!!!!\n",strlen("mpu_set_sensors fail!!!!\n"));
         return -1;
    }
	 /* Push both gyro and accel data into the FIFO. */
    if (mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
    {
         R_UART0_Send_Block("mpu_configure_fifo fail!!!!\n",strlen("mpu_configure_fifo fail!!!!\n"));
         return -1;
    }
    if (mpu_set_sample_rate(100))
    {     
         R_UART0_Send_Block("mpu_set_sample_rate fail!!!!\n",strlen("mpu_set_sample_rate fail!!!!\n"));
         return -1;
    }

    /* Initialize HAL state variables. */
    memset(&hal, 0, sizeof(hal));
    hal.sensors = ACCEL_ON | GYRO_ON;
    hal.report = PRINT_QUAT;

    if (dmp_load_motion_driver_firmware())
    {
         R_UART0_Send_Block("dmp_load_motion_driver_firmware fail!!!!\n",strlen("dmp_load_motion_driver_firmware fail!!!!\n"));
         return -1;
    }
    if (dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
    {
	 R_UART0_Send_Block("dmp_set_orientation fail!!!!\n",strlen("dmp_set_orientation fail!!!!\n"));
         return -1;
    }
    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
                       DMP_FEATURE_GYRO_CAL;
    if (dmp_enable_feature(hal.dmp_features))
    {
	 R_UART0_Send_Block("dmp_enable_feature fail!!!!\n",strlen("dmp_enable_feature fail!!!!\n"));
         return -1;
    }
    if (dmp_set_fifo_rate(200))
    {
	 R_UART0_Send_Block("dmp_set_fifo_rate fail!!!!\n",strlen("dmp_set_fifo_rate fail!!!!\n"));
         return -1;
    }
    if (mpu_set_dmp_state(1))
    {
	 R_UART0_Send_Block("mpu_set_dmp_state fail!!!!\n",strlen("mpu_set_dmp_state fail!!!!\n"));
         return -1;
    }
    hal.dmp_on = 1;
    if (dmp_set_interrupt_mode(DMP_INT_CONTINUOUS))
    {
	 R_UART0_Send_Block("dmp_set_interrupt_mode fail!!!!\n",strlen("dmp_set_interrupt_mode fail!!!!\n"));
         return -1;
    }
//    R_UART0_Send_Block("mpu initialize success!\r\n",strlen("mpu initialize success!\r\n"));
    return 0;
}
void mpu_tool(void)
{
    float rate_x,rate_y,rate_z;
    unsigned long sensor_timestamp;
    if(hal.new_gyro == 1)
    {
       dmp_read_fifo(Q_ANGLE.gyro, Q_ANGLE.accel, Q_ANGLE.quat, &sensor_timestamp, &Q_ANGLE.sensors,&Q_ANGLE.more);
	
       quaterion_to_euler(Q_ANGLE.quat);
       rate_x = - Q_ANGLE.gyro[0]*OMEGATRANSFORMATION;
       rate_y = - Q_ANGLE.gyro[1]*OMEGATRANSFORMATION;
       rate_z =   Q_ANGLE.gyro[2]*OMEGATRANSFORMATION;
       
//       accel.x = Q_ANGLE.accel[0]/Accel_2_Scale_Factor;
//       accel.y = Q_ANGLE.accel[1]/Accel_2_Scale_Factor;
              
       omega.x=radians(rate_x);
       omega.y=radians(rate_y);
       omega.z=radians(rate_z);

    if (!Q_ANGLE.more)    hal.new_gyro = 0;
    }         

}
