//////////////////////////////////////////////////////
/*
> 2016 Round Active Multi-Module Universal System  -part sphero-
> 2016. 07. 03. 21:16
> written by Jaeyeon park

<DESCRIPTION>
	#The purpose of this library
		: control the speed of each motor of sphero wireless

	#protocol
		:16byte Serial Communication

		1. Toss and Receive "string" as Serial communication
		2. String form
			"#<sign><integer><sign><integer>"
			   ------------- ---------------
			   Straight Var   Turn Var

			# 			: char to inform that THIS IS NEW COMMAND FROM CONTROLLER
			<sign>		: + or -
			<integer> 	: 0 ~ 9, the value of straight var or turn var

			Example
			:

		3. YOU SHOULD FOLLOW THESE STRING FORM

	#Explanation about Straight & Turn variable.

		Straight(a.k.a S) direction
		(+)
		^
		|
		|
		|
		|
		|
    ----|--------------->(+) Turn(a.k.a T) direction
    	|
    	|

    	This method is for 2-wheel robot.
    	speed of Left motor  = value of S + value of T
    	speed of Right motor = value of S - value of T

    #Sequence to communicate
    	1 : Make Serial instance with TX, RX pin
    	2 : Set baudrate. Default value is 115200.
    		however I recommend you WRITE BAUDRATE ALTHOUGH IT IS 115200 to increase the clarity of the code
    				------------------------------------------------------
		3 : Attach RX interrupt
			>>code recommendation
			: <Serial_Receive instance>.bluetooth.attach(&<Serial_Receive instance>, &Serial_Receive::Receive_data, Serial::RxIrq);

			It makes board get command instantly as interrupt method
		4 : DONE.
</DESCRIPTION>
*/
//////////////////////////////////////////////////////


#ifndef RECEIVE_H
#define RECEIVE_H

#include "mbed.h"
#include <string>

class Serial_Receive{
	public:
		Serial_Receive(PinName TX, PinName RX); // CONSTRUCTOR
		void baud(int baudrate); // SET BAUDRATE
		void Receive_data(); // RX INTERRUPT FUNCTIONS

		int readable(); // TO DEBUG. IT RETURNS 1 IF SERIAL BUFFER EXISTS, ELSE RETURNS 0
		char Get_raw_char_data(int index); // TO DEBUG. IT RETURNS CHAR DATA IN COMMAND STRING. FIND IT BY INDEX
		void Motor_reset();

		float Get_processed_motor_Value(char L_or_R); // TO DELIVER PWM RATIO. IT RETURNS FLOAT VALUE


		Serial bluetooth;

		/*DEPRECATED FUNCTIONS
		// char* Get_data_set();
		*/

	private:
		string data_set;
		float processed_motor_value[2];
};


#endif
