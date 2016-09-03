
/******************************************************************************
 * 2016 Round Active Multi-Module Universal System
 * -part sphero-
 *
 * p1 - project folder name
 * *FROM NOW ONLY USING PC CONTROL*
 * *PID CONTROL WILL BE ADDED*
 *
 * FILE = main.cpp
 *
 * CC-BY-SA (c) 2016 Yong-jae Yoo
 *
 ******************************************************************************/


#include "mbed.h"
#include "Serial_receive.h"

Serial pc_serial(USBTX,USBRX);
Serial_Receive blt(PC_12, PD_2);

PwmOut motor1_control_pin_forward(PB_10);
PwmOut motor1_control_pin_backward(PB_4);

PwmOut motor2_control_pin_forward(PB_3);
PwmOut motor2_control_pin_backward(PB_5);


float velocity_L = 0.0;
float velocity_R = 0.0;

int main_delay = 100;

float target_velocity = 0.0f;

void initialize(){
  pc_serial.baud(115200);
  blt.baud(115200);
  blt.bluetooth.attach(&blt, &Serial_Receive::Receive_data, Serial::RxIrq);
  motor1_control_pin_forward.period_us(400);
  motor1_control_pin_backward.period_us(400);
  motor2_control_pin_forward.period_us(400);
  motor2_control_pin_backward.period_us(400);
  return;
}

void motor_control(float velocity, int motor_num){
  if(motor_num==1){
    if(velocity>=0){
      motor1_control_pin_forward.write(velocity);
      motor1_control_pin_backward.write(0.0f);
    }
    else{
      motor1_control_pin_backward.write(-velocity);
      motor1_control_pin_forward.write(0.0f);
    }
  }
  else if(motor_num==2){
    if(velocity>=0){
      motor2_control_pin_forward.write(velocity);
      motor2_control_pin_backward.write(0.0f);
    }
    else{
      motor2_control_pin_backward.write(-velocity);
      motor2_control_pin_forward.write(0.0f);
    }
  }
  return;
}

int main(void){
  initialize();
  while(1) {
    //blt.Motor_reset();
    velocity_L = blt.Get_processed_motor_Value('L');
    pc_serial.printf("L : %f\n",velocity_L);
    velocity_R = blt.Get_processed_motor_Value('R');
    pc_serial.printf("R : %f\n",velocity_R);
    motor_control(velocity_L,1);
    motor_control(velocity_R,2);
    wait_ms(main_delay);
  }
}

//copy end

/*
//LED FOR DEBUG


//set pin number
Serial pc(USBTX, USBRX); // tx, rx

PwmOut motor_control_pin(D2);

int encoder_val[2] = {0,0};
float encoder_velocity;
float encoder_velocity_error[3] = {0,0,0};

char command = 'd';

float target_velocity = 0.0f;

float motor_input = 0.0f;

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////VARIABLES///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
float Kp = 0.0f;
float Ki = 0.0f;
float Kd = 0.0f;

int main_delay = 250;
////////////////////////////////////////////////////////////////////////////////

void pc_control(){
  if(pc.readable()){
    command = pc.getc();
  }
}

void encoder_read(){
  int tmp = encoder1.getCurrentState();
  encoder_val[0] = encoder_val[1];
  encoder_val[1] = tmp;
  encoder_velocity = encoder_val[1] - encoder_val[0];
  encoder_velocity_error[0] = encoder_velocity_error[1];
  encoder_velocity_error[1] = encoder_velocity_error[2];
  encoder_velocity_error[2] = target_velocity - encoder_velocity;
}

void motor_control(float velocity){
  float motor_input_mod = Kp*(encoder_velocity_error[2]-encoder_velocity_error[1])
                          + Ki*(encoder_velocity_error[2])
                          + Kd*((encoder_velocity_error[2]-encoder_velocity_error[1])-(encoder_velocity_error[1]-encoder_velocity_error[0]));
  motor_input += motor_input_mod;
  motor_control_pin.write(motor_input);

}

void initialize(){
  motor_control_pin.period_us(400);
}

int main(void){
  initialize();
  while(1) {
    encoder_read();
    pc_control();
    switch (command) {
      case 'w':
      target_velocity = forward_velocity;
        break;
      case 's':
      target_velocity = backward_velocity;
        break;
      default:
      target_velocity = 0.0f;
        break;
    }
    motor_control(target_velocity);
    encoder1.reset();
    wait_ms(main_delay);
  }
}



*/
