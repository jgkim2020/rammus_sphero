
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
DigitalOut myled(LED1);
Ticker timer_int;
Ticker sensor_tick;
Ticker controller_tick;
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
float Roll;
float Pitch;
float Yaw;
float Yaw_set;
float ctrl_px_P = 1;
float ctrl_py_P = 0.1;
float ctrl_r_P = 1;
float ctrl_fb_P = 1;
float ctrl_fb_D = 0.1;
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

void sensor_tick_handler(void)
{
  sensor.read();
  filter.updateIMU(sensor.gx(), sensor.gy(), sensor.gz(), sensor.ax(), sensor.ay(), sensor.az());
  Roll = filter.getRoll();
  Pitch = -filter.getPitch();
  Yaw = 360 - filter.getYaw();
  return;
}

void controller_tick_handler(void)
{
  // motor control input (L, R)
  // all control coefficients are
  // case 1 (parking): px(-ax, -ax) + py(ay, -ay)
  // case 2 (rotate): r(yaw_set - yaw, yaw - yaw_set)
  // case 3 (forward/backward): (spd, spd) + fb_p(ay ,-ay) + fb_d(ay', -ay')
  return;
}

int main(void)
{
  initialize();
  while(1)
  {
    pc.printf("%f %f %f\n", Roll, Pitch, Yaw);
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
  }
  return 0;
}

void initialize()
{
  pc_serial.baud(115200);
  timer_int.attach(&timer_int_handler, 0.01);
  motor1_control_pin_forward.period_us(400);
  motor1_control_pin_backward.period_us(400);
  motor2_control_pin_forward.period_us(400);
  motor2_control_pin_backward.period_us(400);
  pc.baud(115200);
  blt.baud(115200);
  blt.bluetooth.attach(&blt, &Serial_Receive::Receive_data, Serial::RxIrq);
  sensor.begin();
  sensor.calibrate('g');
  filter.begin(100);
  sensor_tick.attach(&sensor_tick_handler, 0.01);
  controller_tick.attach(&controller_tick_handler, 0.05);
  return;
}

void motor_control(float velocity, int motor_num)
{
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
