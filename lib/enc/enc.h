#ifndef ENC_H
#define ENC_H

#include <mbed.h>
#include <QEI.h>


class Encoder_pid{
	public:
		Encoder_pid(PinName ChannelA, PinName channelB, PinName index, int pulse_per_revolution, QEI::Encoding encodingType);

		void set_K_pid(float Kp, float Ki, float Kd);
		float get_K_pid(char pid);

		void set_controlled_value(float target_velocity);
		float get_controlled_value();

		float get_err_value();
		void reset_enc_value();


		QEI Encoder_obj;



	private:
		float k_pid[3];
		//K_pid[0] = Kp, K_pid[1] = Ki, K_pid[2] = Kd

		float encoder_velocity;
		float encoder_velocity_error[3];
		float motor_input;
		float controlled_err;

		int encoder_val[2];
};


#endif
