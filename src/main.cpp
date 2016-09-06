
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
#include "MadgwickAHRS.h"
#include "MPU6050.h"

Serial pc_serial(USBTX,USBRX);
Serial pc(SERIAL_TX, SERIAL_RX); // PA_2, PA_3
Serial_Receive blt(PC_12, PD_2);

Ticker timer_int;

PwmOut motor1_control_pin_forward(PB_10);
PwmOut motor1_control_pin_backward(PB_4);

PwmOut motor2_control_pin_forward(PB_3);
PwmOut motor2_control_pin_backward(PB_5);

Madgwick filter;
MPU6050 sensor;

float velocity_L = 0.0;
float velocity_R = 0.0;
float velocity_L_temp = 0.0;
float velocity_R_temp = 0.0;
float velocity_L_set = 0.0;
float velocity_R_set = 0.0;
float velocity_L_delta = 0.0;
float velocity_R_delta = 0.0;
float velocity_L_now = 0.0;
float velocity_R_now = 0.0;

int main_delay = 100;

float target_velocity = 0.0f;

bool newcommand = false;
int tick = 0;
int debug_cnt = 0;

void initialize(void);
void motor_control(float velocity, int motor_num);

void timer_int_handler(void)
{
  //pc_serial.printf("%d\n", debug_cnt);
  //debug_cnt++;
  if(newcommand == true){
    tick = 200;
    newcommand = false;
  }
  else if((newcommand == false) && (tick > 0)) {
    tick--;
    velocity_L_now += velocity_L_delta/200.0;
    velocity_R_now += velocity_R_delta/200.0;
    motor_control(velocity_L_now, 1);
    motor_control(velocity_R_now, 2);
  }
  if(tick == 0) {
    motor_control(velocity_L_set, 1);
    motor_control(velocity_R_set, 2);
  }
}

int main(void){
  initialize();
  while(1) {
    //blt.Motor_reset();
    velocity_L_temp = velocity_L;
    velocity_R_temp = velocity_R;
    velocity_L = blt.Get_processed_motor_Value('L');
    //pc_serial.printf("L : %f\n",velocity_L);
    velocity_R = blt.Get_processed_motor_Value('R');
    //pc_serial.printf("R : %f\n",velocity_R);
    if((velocity_L != velocity_L_temp) || (velocity_R != velocity_R_temp)){
      if((velocity_L == 0) && (velocity_R == 0)){
        velocity_L_set = 0.0;
        velocity_R_set = 0.0;
        velocity_L_now = velocity_L_set;
        velocity_R_now = velocity_L_set;
        newcommand = false;
        tick = 0;
      }
      else{
        velocity_L_set = velocity_L;
        velocity_R_set = velocity_R;
        velocity_L_delta = velocity_L_set - velocity_L_now;
        velocity_R_delta = velocity_R_set - velocity_R_now;
        newcommand = true;
      }
    }
    //motor_control(velocity_L,1);
    //motor_control(velocity_R,2);
    //wait_ms(main_delay);
  }
}

void initialize(){
  pc_serial.baud(115200);
  timer_int.attach(&timer_int_handler, 0.01);
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
