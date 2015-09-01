#include "AP_Math.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

float safe_asin(float v)
{
	if (isnan(v)) {
		return 0.0;
	}
	if (v >= 1.0) {
		return (float)(PI/2);
	}
	if (v <= -1.0) {
		return (float)(-PI/2);
	}
	return (float)asin(v);
}

float safe_sqrt(float v)
{
	if(v < 0)
	{
		return 0;
	}
	else
	{
		return (float)sqrt(v);
	}
}
// a faster varient of atan.  accurate to 6 decimal places for values between -1 ~ 1 but then diverges quickly
float fast_atan(float v)
{
    float v2 = v*v;
    return (v*(1.6867629106f + v2*0.4378497304f)/(1.6867633134f + v2));
}
//constrain a value
float constrain_float(float amt, float low, float high) 
{
	// the check for NaN as a float prevents propogation of
	// floating point errors through any function that uses
	// constrain_float(). The normal float semantics already handle -Inf
	// and +Inf
	if (isnan(amt)) {
		return (low+high)*0.5f;
	}
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

long wrap_360(long error)
{
	if (error > 36000) error -= 36000;
	if (error < 0) error += 36000;
	return error;
}

long wrap_180(long error)
{
	if (error > 18000) error -= 36000;
	if (error < -18000) error += 36000;
	return error;
}

int isnan(float f)
{
	return 0;
}

int finite(float f)
{
	return 0;
}

//size: include \0
int float_to_str(float f, int precision, char *str, int size)
{
	int int_part;
	float float_part;
	int is_negative;

	char tmp_buf[50];
	int tmp_index;
	int tmp_value;
	int i;

	if(f < 0)
	{
		is_negative = 1;
		f = -f;
	}
	else
	{
		is_negative = 0;
	}

	//get integer
	int_part = (int)f;
	tmp_index = 0;
	do
	{
		tmp_value = int_part % 10;
		tmp_buf[tmp_index++] = (char)(tmp_value + '0');
		int_part /= 10;
	} while (tmp_index < 50 && int_part > 0);

	//save integer
	if(is_negative)
	{
		if(size > 0)
		{
			*str = '-';
			str++;
			size--;
		}
		else
		{
			return 0;
		}
	}
	

	if(size >= tmp_index)
	{
		for(i = tmp_index - 1; i >= 0; i--)
		{
			*str = tmp_buf[i];
			str++;
		}
		size -= tmp_index;
		tmp_index = 0;
	}
	else
	{
		return 0;
	}

	//get float
	float_part= f - (int)f;
	tmp_index = 0;
	while(tmp_index < 50 && tmp_index < precision)
	{
		tmp_value = (int)(float_part * 10);

		tmp_buf[tmp_index++] = (char)(tmp_value + '0');
		float_part *= 10;
		float_part -= (int)float_part;
	}
	for(i = tmp_index - 1; i >=0; i--)
	{
		if(tmp_buf[i] == '0')
		{
			tmp_buf[i] = '\0';
		}
		else
		{
			break;
		}
	}
	//save float
	if(tmp_index > 0)
	{
		if(size > tmp_index)
		{
			*str = '.';
			size--;
			str++;

			memcpy(str, tmp_buf, tmp_index);
			size -= tmp_index;
			str = str + tmp_index;
		}
		else
		{
			return 0;
		}
	}
	

	//write end
	if(size > 0)
	{
		*str = '\0';
	}
	else
	{
		return 0;
	}
}
