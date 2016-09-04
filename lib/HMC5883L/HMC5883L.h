//=============================================================================================
// HMC5883L.h
//=============================================================================================
#ifndef HMC5883L_h
#define HMC5883L_h
#include "mbed.h"

class HMC5883L {
private:
  char data_write[2];
  char mag_read[6];
  int16_t mag_rawval[3];
  float mag_val[3];
  void error_msg(int error, const char *error_msg);

public:
  HMC5883L(void);
  void begin();
  void read();
  float mx() {return mag_val[0];}
  float my() {return mag_val[1];}
  float mz() {return mag_val[2];}
};
#endif
