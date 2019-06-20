/*
 * PID.h
 *
 * Created: 19.06.2019 12:54:10
 *  Author: JackFrost
 */ 


#ifndef PID_H_
#define PID_H_

typedef struct PID_parameters {
	
	float Kp;
	float Tn;
	float Tv;
	float cycle_time;
	

} PID_parameters_t,*p_PID_parameters_t;

void PID_calculations(p_PID_parameters_t PID,float *real_value, float *setpoint, uint16_t *output,bool reset_I,bool running);

#endif /* PID_H_ */