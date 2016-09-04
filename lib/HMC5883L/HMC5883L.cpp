//=============================================================================================
// HMC5883L.cpp
//=============================================================================================

#include "HMC5883L.h"
#include "mbed.h"

#define HMC5883L_ADDR     0x3C // 8-bit address = 7-bit address*2 = 0x1E*2 = 0x3C
#define HMC5883L_CONF_A 	0x00
#define HMC5883L_CONF_B 	0x01
#define HMC5883L_MODE 		0x02
#define MAG_XOUT_H 				0x03
#define MAG_XOUT_L 				0x04
#define MAG_YOUT_H 				0x07
#define MAG_YOUT_L 				0x08
#define MAG_ZOUT_H 				0x05
#define MAG_ZOUT_L 				0x06

HMC5883L::HMC5883L(void) {
  data_write[0] = 0x00; data_write[1] = 0x00;
  mag_read[0] = 0x00; mag_read[1] = 0x00; mag_read[2] = 0x00; mag_read[3] = 0x00; mag_read[4] = 0x00; mag_read[5] = 0x00;
  mag_rawval[0] = 0; mag_rawval[1] = 0; mag_rawval[2] = 0;
  mag_val[0] = 0.0; mag_val[1] = 0.0; mag_val[2] = 0.0;
}

void HMC5883L::begin(void) {
  I2C i2c(I2C_SDA, I2C_SCL);
  int error = 0;
  data_write[0] = HMC5883L_MODE;
  error += i2c.write(HMC5883L_ADDR, data_write, 2, false)*1; // HMC5883L Continuous mode
  data_write[0] = HMC5883L_CONF_A;
  data_write[1] = 0x18;
  error += i2c.write(HMC5883L_ADDR, data_write, 2, false)*10; // HMC5883L 75Hz mode
  data_write[0] = HMC5883L_CONF_B;
  data_write[1] = 0x60;
  error += i2c.write(HMC5883L_ADDR, data_write, 2, false)*100; // Magnetometer 1.52 mG/LSB
  error_msg(error, "HMC setup error");
}

void HMC5883L::read(void) {
  I2C i2c(I2C_SDA, I2C_SCL);
  int error = 0;
  // Read magnetometer register
  data_write[0] = MAG_XOUT_H;
  error += i2c.write(HMC5883L_ADDR, data_write, 1, true)*1;
  error += i2c.read(HMC5883L_ADDR, mag_read, 6, false)*10;
  // Merge MSB and LSB
  mag_rawval[0] = (int16_t)((uint8_t)mag_read[0] << 8) | ((uint8_t)mag_read[1]);
  mag_rawval[1] = (int16_t)((uint8_t)mag_read[4] << 8) | ((uint8_t)mag_read[5]);
  mag_rawval[2] = (int16_t)((uint8_t)mag_read[2] << 8) | ((uint8_t)mag_read[3]);
  // Apply scale factor
  for(int i = 0; i < 3; i++) mag_val[i] = mag_rawval[i]*1.52; // unit: mG
  error_msg(error, "HMC read error");
}

void HMC5883L::error_msg(int error, const char *error_msg) {
	DigitalOut myled(LED1);
  Serial pc(SERIAL_TX, SERIAL_RX); // PA_2, PA_3
  if(error != 0) {
    while(1) {
      myled = !myled;
      pc.printf("%s: %d\n", error_msg, error);
      wait_ms(2500);
    }
  }
}
