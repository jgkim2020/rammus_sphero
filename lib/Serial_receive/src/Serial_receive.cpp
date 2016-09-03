#include "Serial_receive.h"
Serial debug(USBTX, USBRX);

Serial_Receive::Serial_Receive(PinName TX, PinName RX): bluetooth(TX,RX){
}

void Serial_Receive::baud(int baudrate = 115200){
		bluetooth.baud(baudrate);
		return;
}

void Serial_Receive::Receive_data(){
	//# is start character

	char getChar;
	while(bluetooth.readable()){
		getChar = (char)bluetooth.getc();
		if(getChar == '#'){
			this->data_set.clear();
		}
		this->data_set = this->data_set + getChar;
	}
return;
}


int Serial_Receive::readable(){
	if(bluetooth.readable()){
		return 1;
	}else{
		return 0;
	}
}

char Serial_Receive::Get_raw_char_data(int index){
	if(index <0){
		return 'e';
	}
	return this->data_set[index];
}

void Serial_Receive::Motor_reset(){
	this->data_set.clear();
	this->data_set = "#+0+0";
	return;
}


float Serial_Receive::Get_processed_motor_Value(char L_or_R){
	/* this function is just only for Rammus Serial communication protocol specified by Jaeyeon park.
	   From Serial Rx, mbed board should receive data in this form
		 "#<sign><integer>;<sign><ingeter>;"
		 Example:
		 "#+9:+2;"

		 Straight(a.k.a S) direction (+)
		 ㅅ
		 |
		 |
		 |
		 |
		 |
		 -------------->Turn(a.k.a T) direction (+)
		 //imagine X,Y coordinate

		 first value means Straight, second value means Turn. 9 is value of S, 2 is value of T. value : (0 ~ 9)
		 Left motor value = value of S + value of T
		 right motor value = value of S - value of T
		 it makes RAMMUS turn in place,

		 This function will return the motor value(0.0f ~ 1.0f)
		 if you want left, just input parameter 'L'

		 ////structure////
		 1. parsing raw_char_data into real value
		 2. make integer to float;
		 3. return the motor value
*/

	const float max = 81.0f;
	const int num_of_vect = 2;
	const int num_of_motor = 2;
	const int buffer_size = 256;
	float result;


	char will_be_processed[buffer_size];
	char sign[num_of_vect];
	int Extract_motor_value[num_of_motor];
	char* token;
	for(int i = 0; i<buffer_size;i++){
		will_be_processed[i] = this->data_set[i];
	}

	sign[0] = will_be_processed[1];
	sign[1] = will_be_processed[1+2];

	token = strtok(will_be_processed,"-+"); // extract START POINT CHAR #
	debug.printf("\ntoken : %s\n",token);
	token = strtok(NULL,"-+");
	Extract_motor_value[0] = atoi(token);
	Extract_motor_value[0] = Extract_motor_value[0]*Extract_motor_value[0];
	if(sign[0] == '-'){
		Extract_motor_value[0] = -1*Extract_motor_value[0];
	}
	debug.printf("token2 : %s\n",token);
	token = strtok(NULL,"-+");
	Extract_motor_value[1] = atoi(token);
	Extract_motor_value[1] = Extract_motor_value[1]*Extract_motor_value[1];
	if(sign[1] == '-'){
		Extract_motor_value[1] = -1*Extract_motor_value[1];
	}
	debug.printf("token3 : %s\n", token); // return NULL (it means End)

	token = strtok(NULL,"-+");
	debug.printf("value Straight : %d\n",Extract_motor_value[0]);
	debug.printf("value Turn : %d\n",Extract_motor_value[1]);
	debug.printf("endpoint %s\n",token);



	this->processed_motor_value[0] = 1.0*(Extract_motor_value[0]+Extract_motor_value[1])/max;
	this->processed_motor_value[1] = 1.0*(Extract_motor_value[0]-Extract_motor_value[1])/max;
	switch(L_or_R){
		case 'L':
		result = this->processed_motor_value[0];
		break;
		case 'R':
		result = this->processed_motor_value[1];
		break;
		default :
		result = 0.0f;
		break;
	}
return result;
};





/*DEPRECATED*/
/*
char* Serial_Receive::Get_data_set(){
	return this->data_set;
}
*/
