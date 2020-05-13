
//Stephen Mock
// With Library!
//11/30/19

#define sensorAddress 0x18 // default sensor address
#define tempAddress 0x05 // address the temperature register is at
#define configAddress 0x01 // configuration bit
#define criticalAddress 0x04 // critical address bit

#include <I2CFunctions.h>

byte MSB, LSB, result; 
int TEMP_Raw;
float TEMP_degC;
I2CFunctions i2cObj;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // starting serial
  
  uint8_t configArr[2] = {0, 0x40};
  i2cObj.writeMultiI2C(sensorAddress, configAddress, configArr, 2, result);
  

  uint8_t criticalArr[2] = {0x0F, 0xF0};
  i2cObj.writeMultiI2C(sensorAddress, criticalAddress, criticalArr, 2, result);
}

void loop() {
  uint8_t tempData[2] = {};
  i2cObj.readMultiI2C(sensorAddress, tempAddress, tempData, 2, result);
  
    MSB = tempData[0];
    LSB = tempData[1];

    MSB = MSB & 0b00011111;  // Mask away the three flag bits
    //easier to read when the mask is written in binary instead of hex

    if ((MSB & 0b00010000) == 0b00010000)  {          // if sign bit =1 then temp < 0°C
    MSB = MSB & 0b00001111;                             // mask away the SIGN bit
    TEMP_Raw = (((int)MSB) << 8) | LSB;    // combine the MSB & LSB
    TEMP_Raw-= 256;   // convert to negative value: note suggested datasheet calculation has an error!
     }
    else  // temp > 0°C  then the sign bit = 0  – so no need to mask it away
     {
    TEMP_Raw= (((int)MSB) << 8) | LSB;
     }
     
    TEMP_degC =TEMP_Raw*0.0625; // Need to shift all the data bytes backwards by 2^-4 (according to data sheet)
 
  Serial.println(TEMP_degC);
  Serial.println(result);
  delay(100);
}




