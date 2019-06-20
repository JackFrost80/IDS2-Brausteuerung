/*
 * PID.cpp
 *
 * Created: 19.06.2019 12:54:00
 *  Author: JackFrost
 */ 
#include "sam.h"
#include "PID.h"

void PID_calculations(p_PID_parameters_t PID,float *real_value, float *setpoint, uint16_t *output,bool reset_I,bool running)
{
	if(running)
	{
		static int32_t I_part = 0;
		static float old_real_value = 0;
		int32_t D_Part = 0;
		int32_t output_helper = 0;
		if(reset_I)
		I_part = 0;
		float error = *setpoint - *real_value;
		output_helper = error * PID->Kp;  // P Part
		I_part += error * (1/(PID->Tn * PID->cycle_time));
		if(I_part > 10000)
		I_part = 10000;
		if(I_part < 0)
		I_part = 0; // Antiwindup
		output_helper += I_part / 100;  // I_Part
		D_Part = PID->Tv * ((*real_value - old_real_value)/PID->cycle_time)*(1/PID->cycle_time); // D-part
		output_helper += D_Part;
		if(output_helper > 100)
		output_helper = 100;
		if(output_helper < 0)
		output_helper = 0;
		*output = output_helper ;
		old_real_value = *real_value;
	}
	
}