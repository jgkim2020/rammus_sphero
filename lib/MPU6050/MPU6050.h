//=============================================================================================
// MPU6050.h
//=============================================================================================
#ifndef MPU6050_h
#define MPU6050_h
#include "mbed.h"

class MPU6050 {
private:
  char data_write[2];
  char acc_read[6];
  char temp_read[2];
  char gyro_read[6];
  int16_t acc_rawval[3];
  int16_t temp_rawval;
  int16_t gyro_rawval[3];
  float acc_val[3];
  float temp_val;
  float gyro_val[3];
  void error_msg(int error, const char *error_msg);

public:
  MPU6050(void);
  void begin();
  void read();
  float gx() {return gyro_val[0];}
  float gy() {return gyro_val[1];}
  float gz() {return gyro_val[2];}
  float ax() {return acc_val[0];}
  float ay() {return acc_val[1];}
  float az() {return acc_val[2];}
};

#endif
