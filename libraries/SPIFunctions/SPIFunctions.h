#ifndef libSPI // preventing duplicate libraries
#define libSPI

#include "Arduino.h"

class SPIFunctions {
  public: 
    SPIFunctions(int slaveSelect);
    void writeSPI(uint8_t tx_msg);
    void writeMultiSPI(uint8_t tx_msg[], int n_bytes);
    void writeThenReadSPI(uint8_t tx_msg[], int tx_len, uint8_t rx_msg[], int rx_len);
	byte readSPI(byte tx_msg);
	void writeAndReadSPI(uint8_t tx_msg[], uint8_t rx_msg[], int msg_len);

  private:
    int _slaveSelect;
};


#endif

