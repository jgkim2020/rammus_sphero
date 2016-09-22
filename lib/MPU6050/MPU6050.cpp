//=============================================================================================
// MPU6050.cpp
//=============================================================================================

#include "MPU6050.h"
#include "mbed.h"

#define MPU6050_ADDR      0xD0 // 8-bit address = 7-bit address*2 = 0x68*2 = 0xD0
#define MPU6050_PWR				0x6B
#define MPU6050_ACC_CONF  0x1C
#define MPU6050_GYRO_CONF 0x1B
#define ACC_XOUT_H 				0x3B
#define ACC_XOUT_L 				0x3C
#define ACC_YOUT_H 				0x3D
#define ACC_YOUT_L 				0x3E
#define ACC_ZOUT_H 				0x3F
#define ACC_ZOUT_L 				0x40
#define TEMP_OUT_H        0x41
#define TEMP_OUT_L        0x42
#define GYRO_XOUT_H 			0x43
#define GYRO_XOUT_L 			0x44
#define GYRO_YOUT_H 			0x45
#define GYRO_YOUT_L 			0x46
#define GYRO_ZOUT_H       0x47
#define GYRO_ZOUT_L       0x48

MPU6050::MPU6050() {
	data_write[0] = 0x00; data_write[1] = 0x00;
  acc_read[0] = 0x00; acc_read[1] = 0x00; acc_read[2] = 0x00; acc_read[3] = 0x00; acc_read[4] = 0x00; acc_read[5] = 0x00;
  temp_read[0] = 0x00; temp_read[1] = 0x00;
  gyro_read[0] = 0x00; gyro_read[1] = 0x00; gyro_read[2] = 0x00; gyro_read[3] = 0x00; gyro_read[4] = 0x00; gyro_read[5] = 0x00;
  acc_rawval[0] = 0; acc_rawval[1] = 0; acc_rawval[2] = 0;
  temp_rawval = 0;
  gyro_rawval[0] = 0; gyro_rawval[1] = 0; gyro_rawval[2] = 0;
  acc_val[0] = 0.0; acc_val[1] = 0.0; acc_val[2] = 0.0;
	acc_cval[0] = 0.0; acc_cval[1] = 0.0; acc_cval[2] = 0.0;
  temp_val = 0.0;
  gyro_val[0] = 0.0; gyro_val[1] = 0.0; gyro_val[2] = 0.0;
	gyro_cval[0] = -3.0; gyro_cval[1] = -1.0; gyro_cval[2] = 1.0;
	return;
}

void MPU6050::begin() {
	I2C i2c(I2C_SDA, I2C_SCL); // PB_9, PB_8
  int error = 0;
  data_write[0] = MPU6050_PWR;
  data_write[1] = 0x00;
  error += i2c.write(MPU6050_ADDR, data_write, 2, false)*1; // MPU6050 Power ON
  data_write[0] = MPU6050_ACC_CONF;
  error += i2c.write(MPU6050_ADDR, data_write, 2, false)*10; // Acclerometer 16384 LSB/g
  data_write[0] = MPU6050_GYRO_CONF;
  error += i2c.write(MPU6050_ADDR, data_write, 2, false)*100; // Gyroscope 131 LSB/dps
  error_msg(error, "MPU setup error");
	return;
}

void MPU6050::read() {
	I2C i2c(I2C_SDA, I2C_SCL); // PB_9, PB_8
  int error = 0;
  // Read accelerometer, gyroscope
  // i2c.read(int address, char *data, int length{#, bool repeated#})
  // i2c.write(int address, const char *data, int length{#, bool repeated#})
  this->data_write[0] = ACC_XOUT_H;
  error += i2c.write(MPU6050_ADDR, this->data_write, 1, true)*1;
  error += i2c.read(MPU6050_ADDR, this->acc_read, 6, false)*10;
  this->data_write[0] = TEMP_OUT_H;
  error += i2c.write(MPU6050_ADDR, this->data_write, 1, true)*100;
  error += i2c.read(MPU6050_ADDR, this->temp_read, 2, false)*1000;
  this->data_write[0] = GYRO_XOUT_H;
  error += i2c.write(MPU6050_ADDR, this->data_write, 1, true)*10000;
  error += i2c.read(MPU6050_ADDR, this->gyro_read, 6, false)*100000;
  // Merge MSB and LSB
  this->acc_rawval[0] = (int16_t)((uint8_t)this->acc_read[0] << 8) | ((uint8_t)this->acc_read[1]);
  this->acc_rawval[1] = (int16_t)((uint8_t)this->acc_read[2] << 8) | ((uint8_t)this->acc_read[3]);
  this->acc_rawval[2] = (int16_t)((uint8_t)this->acc_read[4] << 8) | ((uint8_t)this->acc_read[5]);
  this->temp_rawval = (int16_t)((uint8_t)this->temp_read[0] << 8) | ((uint8_t)this->temp_read[1]);
  this->gyro_rawval[0] = (int16_t)((uint8_t)this->gyro_read[0] << 8) | ((uint8_t)this->gyro_read[1]);
  this->gyro_rawval[1] = (int16_t)((uint8_t)this->gyro_read[2] << 8) | ((uint8_t)this->gyro_read[3]);
  this->gyro_rawval[2] = (int16_t)((uint8_t)this->gyro_read[4] << 8) | ((uint8_t)this->gyro_read[5]);
  // Apply scale factor1
  for(int i = 0; i < 3; i++) this->acc_val[i] = this->acc_rawval[i]/16384.0*9.8; // unit: m/s/s
  this->temp_val = this->temp_rawval/340.0 + 36.53; // unit: degrees Celsius
  for(int i = 0; i < 3; i++) this->gyro_val[i] = this->gyro_rawval[i]/131.0; // unit: rad/s
  this->gyro_val[0] -= this->gyro_cval[0];
  this->gyro_val[1] -= this->gyro_cval[1];
  this->gyro_val[2] -= this->gyro_cval[2];
  this->error_msg(error, "MPU read error");
	return;
}

void MPU6050::calibrate() {
	int i;
	char option = 'g';
	if(option == 'g')
	{
		this->gyro_cval[0] = 0;
		this->gyro_cval[1] = 0;
		this->gyro_cval[2] = 0;
		for(i = 0; i < 100; i++)
		{
			this->read();
			this->gyro_cval[0] += this->gyro_val[0];
			this->gyro_cval[1] += this->gyro_val[1];
			this->gyro_cval[2] += this->gyro_val[2];
			wait_ms(10);
		}
		this->gyro_cval[0] *= 0.01;
		this->gyro_cval[1] *= 0.01;
		this->gyro_cval[2] *= 0.01;
		return;
	}
	else if(option == 'a')
	{
		return;
	}
	else
	{
		return;
	}
}

void MPU6050::error_msg(int error, const char *error_msg) {
	DigitalOut myled(LED1);
	Serial pc(SERIAL_TX, SERIAL_RX); // PA_2, PA_3
  if(error != 0) {
  //  while(1) {
      myled = !myled;
      pc.printf("%s: %d\n", error_msg, error);
    //}
  }
	return;
}
