// Purpose: Script to test the ALS31313 Sensor alone, using various filtering techniques. The current filtering technique is to average 100 samples every second of magnetic data in X,Y,Z
// Result: Filter works as expected and reduces sensor noise. Further testing of the lag as a result of the filter can be evaluated.


// Header Files
#include <TimerOne.h> // Using Timer1


//I2C
#include <math.h> // For PI
#include <I2CFunctions.h> // Custom library
I2CFunctions i2cObj; // Creating I2CObj 

// Return values of endTransmission in the Wire library
#define kNOERROR 0
#define kDATATOOLONGERROR 1
#define kRECEIVEDNACKONADDRESSERROR 2
#define kRECEIVEDNACKONDATAERROR 3
#define kOTHERERROR 4


//ALS31313 SETUP
int ALS31313Address = 96; // Address of the ALS31313 with nothing to the address pins
uint8_t error; // actual error message if needed

//Loop
int counter = 1; // how many times the magnetic data has been counted, used for the averaging of the data
int sumMX, sumMY, sumMZ; // Total sum of the mag data in X,Y,Z 
float avgMX, avgMY, avgMZ; // Averaged value of the mag data after 1s, 100 samples

void setup() {

  Serial.begin(115200); //115200 Baud Rate !

  //I2C SETUP
    I2Cwrite32bit(ALS31313Address, 0x35, 0x2C413534); //Enter customer access mode on the ALS31313
    if (error != kNOERROR)
    {
      Serial.print("Error while trying to enter customer access mode. error = ");
      Serial.println(error);
    }    
    // Setting the mode for I2C loop
    uint32_t value0x27;
    
    // Read the register the I2C loop mode is in
    value0x27 = I2Cread32bit(ALS31313Address, 0x27); // read and return
    if (error != kNOERROR){
      Serial.print("Unable to read the ALS31300. error = ");
      Serial.println(error);}
      
    // I2C loop mode is in bits 2 and 3 so mask them out
    // and set them to the no loop mode
    value0x27 = (value0x27 & 0xFFFFFFF3) | (0x0 << 2);
    
    // Write the new values to the register the I2C loop mode is in
    I2Cwrite32bit(ALS31313Address, 0x27, value0x27);
    if (error != kNOERROR){
      Serial.print("Unable to read the ALS31300. error = ");
      Serial.println(error);
    }
    delay(2500);
    Timer1.initialize(10000); // reprents the timer that is constantly reading the encoder, every 10 millis (hundredth of a second). The input for Timer1 is in nanoseconds.
    Timer1.attachInterrupt(timer1Function); // allowing for interrupts (of timer1)
    Timer1.start(); //starting the timer
}

void loop() {
}


// Main functions

/*
 * This function takes magnetic data measurements every 1/100 of a second, and averages the 100 points every second. The value is then outputted to serial.
 */

void timer1Function(void) {
  interrupts(); // enable interupts, I2C disables them! THIS MUST BE DONE OTHERWISE the interrupt cannot trigger.
  float magArr[3] = {}; //index 0: mx, index 1: my, index2: mz
  readALS31313(ALS31313Address, magArr);

  sumMX = sumMX + magArr[0]; // adding to a running sum for data in X
  sumMY = sumMY + magArr[1]; // adding to a running sum for data in Y
  sumMZ = sumMZ + magArr[2]; // adding to a running sum for data in Y

  if (counter % 100 == 0) {
    avgMX = sumMX / counter; // Getting an averaged value (dividing by 100)
    avgMY = sumMY / counter;
    avgMZ = sumMZ / counter;
    sumMX = 0; // resetting
    sumMY = 0; 
    sumMZ = 0; 
    Serial.print(avgMX); // Printing averaged data, in Gauss 
    Serial.print(",");
    Serial.print(avgMY);
    Serial.print(",");
    Serial.println(avgMZ);
    counter = 1; //resetting counter
  } else {
    counter++; // if I havent reached 100 iterations, just count up every timer
//    Serial.print("Counter: ");
//    Serial.println(counter);
  }
}
/*

ALS31313 FUNCTIONS!

  /*
   * This function reads the magnetic data in the X,Y,Z, accessing data from registers 0x28,0x29 and correctly concatenating it. 
   * @return there is not output, but an array (magData[]) is implictly returned
   * @param deviceAddress address of the device
   * @param magData[] an array to hold angle X,Y,Z data
   */
   
void readALS31313(int deviceAddress, float magData[]) {
  uint8_t arrRegister28[4] = {}; // creating empty arrays to pass into other function
  uint8_t arrRegister29[4] = {};  
  I2CreadRegister32bit(deviceAddress, 0x28, arrRegister28); // First 32 bit data register (0x28)
  I2CreadRegister32bit(deviceAddress, 0x29, arrRegister29); // Second 32 bit data register (0x29)

  // Byte Manipulation
  int16_t xMag = (int8_t)arrRegister28[0]; // Converting to 2's Complement
  xMag = (xMag << 4) | (0x0F & arrRegister29[1]); // Adding the 4 bits from second register
  int16_t yMag = (int8_t)arrRegister28[1];
  yMag = (yMag << 4) | ((0xF0 & arrRegister29[2]) >> 4 );
  int16_t zMag = (int8_t)arrRegister28[2];
  zMag = (zMag << 4) | (0x0F & arrRegister29[2]);

  // Storing Variables in Array, magData[0] = angle in XY, magData[1] is angle in XZ, magData[2] = angle in YZ
//magData[0] = atan2f(float(yMag), float(xMag)) * 180.0 / M_PI; // converting to an angle, and multipling to degrees from radians
//magData[1] = atan2f(float(xMag), float(zMag)) * 180.0 / M_PI; 
//magData[2] = atan2f(float(zMag), float(yMag)) * 180.0 / M_PI;
  magData[0] = xMag; // raw data
  magData[1] = yMag;
  magData[2] = zMag;
}

  /*
   * This function returns data at a specific address, and register and stores it in a 32bit array (4 byte arr)
   * @return there is not output, but an array (readArr[]) is implictly returned
   * @param deviceAddress address of the device
   * @param address the specific address of a register being used
   * @param readArr[] the array to hold the 32 bit data (4, 8 bit chunks)
   */
   
// Using I2C, read 32 bits of data from the address on the device at the bus address
void I2CreadRegister32bit(int deviceAddress, uint8_t address, uint8_t readArr[])
{
  // if the device accepted the address,
  // request 4 bytes from the device
  // and then read them, MSB first
  i2cObj.readMultiI2C(deviceAddress, address, readArr, 4, error); // using my custom library
  
  if (error != kNOERROR)
  {
    Serial.print("Unable to read the ALS31300. error = ");
    Serial.println(error);
  }
}

  /*
   * This function returns data at a specific address, and register but returns it at as a uint32_t of what was read
   * @return there is not output, but an array (readArr[]) is implictly returned
   * @param deviceAddress address of the device
   * @param address the specific address of a register being used
   */
   
uint32_t I2Cread32bit(int deviceAddress, uint8_t address)
{
  uint32_t value;
  uint8_t readArr[4] = {};
  i2cObj.readMultiI2C(deviceAddress, address, readArr, 4, error);

  // if the device accepted the address,
  // request 4 bytes from the device
  // and then read them, MSB first
  
  if (error != kNOERROR)
  {
    Serial.print("Unable to read the ALS31300. error = ");
    Serial.println(error);
  }

  // Combining the data from the array into a 32bit value
  uint32_t value = readArr[0] << 24;
  value += readArr[1] << 16;
  value += readArr[2] << 8;
  value += readArr[3];
  
  return value;
}

 /*
   * This function writes a 32bit value to a specific address, and register.
   * @param deviceAddress address of the device
   * @param address the specific address of a register being used
   * @param value the 32bit value you are writing.
   */

void I2Cwrite32bit(int deviceAddress, uint8_t address, uint32_t value)
{
  // Write the address that is to be written to the device
  // and then the 4 bytes of data, MSB first
  uint8_t writeArr[4] = {(byte)(value >> 24), (byte)(value >> 16), (byte)(value >> 8), (byte)(value)};
  i2cObj.writeMultiI2C(deviceAddress, address, writeArr, 4, error);
}
