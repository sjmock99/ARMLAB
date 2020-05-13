// Stephen Mock
// SPI Library
// Date: 5/12/20


#include "SPIFunctions.h"
#include <SPI.h>

/** 
 *Constructor for SPIFunctions Object
 *
 *@param slaveSelect: the slave Select Pin for the specific sensor
 */
 
SPIFunctions::SPIFunctions(int slaveSelect) {
  _slaveSelect = slaveSelect;
  SPI.begin();
  pinMode(_slaveSelect, OUTPUT);
}

/**
 * Writing one byte
 * @param tx_msg One byte to write
 */ 

 
void SPIFunctions::writeSPI(uint8_t tx_msg){
  digitalWrite(_slaveSelect, LOW);
  SPI.transfer(tx_msg);
  digitalWrite(_slaveSelect,HIGH);
}


/** Writing an array of bytes 
 *
 *@param tx_msg[] the array of bytes to write 
 *@param n_bytes number of bytes to write
 */
 
void SPIFunctions::writeMultiSPI(uint8_t tx_msg[], int n_bytes) {
  digitalWrite(_slaveSelect,LOW);        // Begin SPI conversation
  int ii;
  for (ii = 0; ii < n_bytes; ii++) {
    SPI.transfer(tx_msg[ii]);
  }
  digitalWrite(_slaveSelect,HIGH);       // Terminate SPI conversation 
}



/** Read one byte from one register
 *
 *@param tx_msg: the one byte register to read from
 *@return the byte read from the register
 */
 
uint8_t SPIFunctions::readSPI(uint8_t tx_msg) {
  byte result;   // result to return
  digitalWrite(_slaveSelect, LOW);
  SPI.transfer(tx_msg);
  result = SPI.transfer(0x00);
  digitalWrite(_slaveSelect, HIGH);
  return(result);
}



/**
 * Write ALL tx_len bytes, then read ALL rx_len bytes
 *
 *@param tx_msg[]: array of bytes to write
 *@param tx_len: number of registers to write
 *@param rx_msg[]: array to store data that is read from register
 *@param rx_len: number of bytes that will be read 
 */

void SPIFunctions::writeThenReadSPI(uint8_t tx_msg[], int tx_len, uint8_t rx_msg[], int rx_len){
  digitalWrite(_slaveSelect, LOW); // begin communication
  
  int ii; // counting variables for tx_len
  int jj;  // rx_len

  // write tx_len inputs
  for (ii = 0; ii < tx_len; ii++) {
    SPI.transfer(tx_msg[ii]);
  }


  // read rx_len values
  for (jj = 0; jj < rx_len; jj++) {
    rx_msg[jj] = SPI.transfer(0x00);
  }
  
  digitalWrite(_slaveSelect, HIGH); // end communication
}


/**
 * Write a byte, then read right after. Length of array of bytes to write and array of data to store should be the same
 *
 *@param tx_msg[]: array of bytes to write
 *@param rx_msg[]: array to store data that is read from register
 *@param msg_len: number of bytes to write and read
 */

void SPIFunctions::writeAndReadSPI(uint8_t tx_msg[], uint8_t rx_msg[], int msg_len) {
  digitalWrite(_slaveSelect, LOW); // begin communication
  
  int ii; // counting variables for msg_len 
  // tx_msg and rx_msg must be same length in order to work as intended

  // write tx_len inputs, and then read and store in rx_msg[]
  for (ii = 0; ii < msg_len; ii++) {
    SPI.transfer(tx_msg[ii]);
	rx_msg[ii] = SPI.transfer(0x00);
  }

  digitalWrite(_slaveSelect, HIGH); // end communication

}
	

