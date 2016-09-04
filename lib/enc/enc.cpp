#include "enc.h"

Encoder_pid::Encoder_pid(PinName ChannelA,
						 PinName ChannelB,
						 PinName index = NC,
						 int pulse_per_revolution = 710,
						 QEI::Encoding encodingType =  QEI::X2_ENCODING): Encoder_obj(ChannelA, ChannelB, index, pulse_per_revolution, encodingType){
	this->k_pid[0] = 0.0f;
	this->k_pid[1] = 0.0f;
	this->k_pid[2] = 0.0f;

	this->encoder_velocity = 0;

	this->encoder_velocity_error[0] = 0.0f;
	this->encoder_velocity_error[1] = 0.0f;
	this->encoder_velocity_error[2] = 0.0f;

	this->encoder_val[0] = 0;
	this->encoder_val[1] = 0;

	this->motor_input = 0;

}

void Encoder_pid::set_K_pid(float Kp, float Ki, float Kd){
	this->k_pid[0] = Kp;
	this->k_pid[1] = Ki;
	this->k_pid[2] = Kd;
	return;
}

float Encoder_pid::get_K_pid(char pid){
	float result;
	switch(pid){
		case 'p':
		result = this->k_pid[0];
		break;

		case 'i':
		result = this->k_pid[1];
		break;

		case 'd':
		result = this->k_pid[2];
		break;

		default:
		result = -1;
		break;
	}
	return result;
	// -1 means error at input parameter
	// input parameter should be 'p' or 'i' or 'd'
}

void Encoder_pid::set_controlled_value(float target_velocity){
	int tmp = this->Encoder_obj.getPulses();
	this->encoder_val[0] = this->encoder_val[1];
	this->encoder_val[1] = tmp;
	this->encoder_velocity = this->encoder_val[1] - this->encoder_val[0];

	this->encoder_velocity_error[0] = this->encoder_velocity_error[1];
	this->encoder_velocity_error[1] = this->encoder_velocity_error[2];
	this->encoder_velocity_error[2] = target_velocity - this->encoder_velocity;
	this->controlled_err = this->encoder_velocity_error[2];
	return;
}

float Encoder_pid::get_controlled_value(){
	float motor_input_mod = this->k_pid[0]*(this->encoder_velocity_error[2] - this->encoder_velocity_error[1])
							+ this->k_pid[1]*(this->encoder_velocity_error[2])
							+ this->k_pid[2]*((this->encoder_velocity_error[2] - this->encoder_velocity_error[1])-(this->encoder_velocity_error[1] - this->encoder_velocity_error[0]));

	this->motor_input += motor_input_mod;
	return motor_input;
}

float Encoder_pid::get_err_value(){
	return this->controlled_err;
}

void Encoder_pid::reset_enc_value(){
	this->Encoder_obj.reset();
	return;
}
