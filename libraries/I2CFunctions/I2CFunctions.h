#ifndef libI2C // preventing duplicate libraries
#define libI2C

#include "Arduino.h"

class I2CFunctions {
  public:
    I2CFunctions();  
    uint8_t readI2C (int deviceAddress, uint8_t registerAddress, uint8_t& result);
	void readMultiI2C (int deviceAddress, uint8_t registerAddress, uint8_t readByteArr[], int numBytes, uint8_t& result); 
	void writeMultiI2C (int deviceAddress, uint8_t registerAddress, uint8_t writeByteArr[], int numBytes, uint8_t& result);
};


#endif