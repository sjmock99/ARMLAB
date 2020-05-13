// Stephen Mock
// I2C Library
// Date: 5/12/20

#include "I2CFunctions.h"
#include <Wire.h>

//Constructor for I2CFunctions Object
//
 
I2CFunctions::I2CFunctions() {
	Wire.begin(); // starting I2C communications
}

// ALL ADDRESSES NEED TO BE INTS...

/**
 * Purpose of this method is to read in a specific device address and its register, and return it as 
 * a uint8_t (byte) 
 *  
 * @param deviceAddress defines the specific I2C device to be accessed
 * @param registerAddress defines the specific register of the device to be read from
 * @param result determines if the I2C worked as intended
 * @return a uint8_t (8 bit) value was read
 */

uint8_t I2CFunctions::readI2C(int deviceAddress, uint8_t registerAddress, uint8_t &result) {
  
  uint8_t registerData;
  Wire.beginTransmission(deviceAddress);              // set sensor target
  Wire.write(registerAddress);                        // set memory pointer
  result = Wire.endTransmission();
  
  if (Wire.available() <= 1) {
    Wire.requestFrom(deviceAddress, 1);     // request num bytes
    registerData = Wire.read(); 
  }
  return registerData;           // the returned byte from this function is the content from registerAddress
}



/**
 * Purpose of this method is to read in a specific device address and its register, and write to a 
 * array which stores all of the number of bytes 
 *  
 * @param deviceAddress defines the specific I2C device to be accessed
 * @param registerAddress defines the specific register of the device to be read from
 * @param readByteArr[] Array that stores all the bytes read
 * @param numBytes number of bytes to read
 * @param result determines if the I2C worked as intended
 * @return a uint8_t (8 bit) value was read
 */

void I2CFunctions::readMultiI2C(int deviceAddress, uint8_t registerAddress, uint8_t readByteArr[], int numBytes, uint8_t &result) {
  
  Wire.beginTransmission(deviceAddress);              // set sensor target
  Wire.write(registerAddress);                        // set memory pointer
  result = Wire.endTransmission();

  int ii;
  
  Wire.requestFrom(deviceAddress, numBytes);     // request num bytes
  if (Wire.available() <= numBytes) {
    
    for (ii = 0; ii < numBytes; ii++) {
      readByteArr[ii] = Wire.read(); // filling the array
    }
  }
}



/**
 * Purpose of this method is to read write n bits to specific device at its register, and returns a 0 if 
 * the operation was successful 
 *
 * @param deviceAddress defines the specific I2C device to be accessed
 * @param registerAddress defines the specific register of the device to be read from
 * @param newRegisterByteArr[] is an array of bytes that a user wants to write each index's data to a register
 * @param numBytes number of bytes 
 * @param result determines if the I2C worked as intended
 * @return a byte that simply returns 0 if the write operation was successful
 */


void I2CFunctions::writeMultiI2C(int deviceAddress, uint8_t registerAddress, uint8_t writeByteArr[], int numBytes, uint8_t &result) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
    
  for (int ii = 0; ii < numBytes; ii++) {
    // writing multiple bytes 
    Wire.write(writeByteArr[ii]); 
  }
  result = Wire.endTransmission(); // Wire.endTransmission(); returns 0 if write operation was successful
} 

