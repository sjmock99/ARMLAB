// Purpose: Script to test the ALS31313, at various rotation positions. Implementating PID Control, as well as state machine to get to specific angles.
// Result: State machine, PI control works rotates the magnet to relatively the right location (+-2 degrees). Sensor works to read the measurements.
// Date : 5/12/20

// Header Files

//SPI
#include <SPIFunctions.h> // Custom Library
SPIFunctions spiObj(53); // Slave select on Pin 53 (MEGA)

//I2C
#include <math.h> 
#include <I2CFunctions.h> // Custom Library
I2CFunctions i2cObj;

//Timing (Two Timers!)
#include <TimerOne.h>
#include <MsTimer2.h>

// Return values of endTransmission in the Wire library
#define kNOERROR 0
#define kDATATOOLONGERROR 1
#define kRECEIVEDNACKONADDRESSERROR 2
#define kRECEIVEDNACKONDATAERROR 3
#define kOTHERERROR 4


//ALS31313 SETUP
int ALS31313Address = 96; // Address of the ALS31313 with nothing to the address pins
uint8_t error; // actual error message if needed

// LS7366R
const uint8_t write_MDR0 = 0x88;            // command to configure MDRO
const uint8_t read_MDR0 = 0x48;             // command to read MDR0
const uint8_t write_MDR1 = 0x90;           // command to configure MDR1
const uint8_t read_MDR1 = 0x50;            // command to read MDR1
const uint8_t read_CNTR = 0x60;             // command to read CNTR
const uint8_t clear_CNTR = 0x20;            // command to clear CNTR
const uint8_t non_quad = 0b00;              // non quadrature mode
const uint8_t one_count = 0b01;             // 1 pulse counted per cycle (A high)
const uint8_t two_count = 0b10;             // 2 pulses counted per cycle (A,B high)
const uint8_t four_count = 0b11;            // 4 pulses counted per cycle (A,B high and low)
const uint8_t one_byte = 0b11;              // CNTR is 1 byte long
const uint8_t two_byte = 0b10;              // CNTR is 2 bytes long
const uint8_t three_byte = 0b01;            // CNTR is 3 bytes long
const uint8_t four_byte = 0b00;             // CNTR is 4 bytes long
int CNTR_bytes = 4;                // default length of CNTR register
uint8_t MDR0 = 0x00;          // variables to be editted based on configuration
uint8_t MDR1 = 0x00;          // variables to be editted based on configuration



// Encoder
long countEncoder = 0;  // signed count, make it a long to hold 32 bits
float encoderDegree;

//ALS31313
int counter = 1; // how many times the magnetic data has been counted, used for the averaging of the data
int sumMX, sumMY, sumMZ; // Total sum of the mag data in X,Y,Z 
float avgMX, avgMY, avgMZ; // Averaged value of the mag data after 1s, 100 samples

// Button for State Machine
const int buttonPin = 2;     // the number of the pushbutton pin
int state = 0; // starting state (polling)

//PID / Motor
int enA = 3; // Enable pin for motor driver (PWM)
int in1 = 4; // Direction Control 
int in2 = 5; // Direction Control
boolean pause = false;
double setPoint; // master angle
double Input; // slave angle
double Output; // controller output
double Kp = 0.00135, Ki = 0.00021, Kd = 0;
int pwm_val = 0; // pwm variable for motor
double timer1dt = 0.1; // pwm loop time interval, 0.01s
double error_integral = 0;
double previous_error = 0;

// Timing / Rotation
double setPointArr[4] = {0, 90, 180, 270}; // array of values to transverse through
int setPointInd = 0; // indicies of the array
unsigned long pauseTimer = 0;


void setup() {

  Serial.begin(115200);

  //I2C SETUP

  I2Cwrite32bit(ALS31313Address, 0x35, 0x2C413534); //Enter customer access mode on the ALS31313
  if (error != kNOERROR)
  {
    Serial.print("Error while trying to enter customer access mode. error = ");
    Serial.println(error);
  }

  // Setting the mode for I2C loop
  uint32_t value0x27;
  
  // Read the register the I2C loop mode is
  value0x27 = I2Cread32bit(ALS31313Address, 0x27); // read and return
  if (error != kNOERROR) {
    Serial.print("Unable to read the ALS31300. error = ");
    Serial.println(error);
  }

  // I2C loop mode is in bits 2 and 3 so mask them out
  // and set them to the no loop mode
  value0x27 = (value0x27 & 0xFFFFFFF3) | (0x0 << 2);

  // Write the new values to the register the I2C loop mode is in
  I2Cwrite32bit(ALS31313Address, 0x27, value0x27);
  if (error != kNOERROR) {
    Serial.print("Unable to read the ALS31300. error = ");
    Serial.println(error);
  }

  // SPI SETUP
  configureMDR0(four_count); // doing quadracount
  configureMDR1(four_byte); // setting output to four bytes
  clearCNTR(); // clearing whatever data is in the encoder

  // Motor Setup

  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Adjust PID Values
  setPoint = setPointArr[0]; // set point start at 0th index

  delay(2500);

  // Timing
  pinMode(buttonPin, INPUT);
  Timer1.initialize((int)(timer1dt * 1000000)); // reprents the timer that is constantly reading the encoder, every 100 millis (tenth of a second)
  Timer1.stop(); // Dont start immediately
  Timer1.attachInterrupt(timer1Function); //Whenever timer1 interrupts, goes to timer1Function
  MsTimer2::set(10, timer2Function); // .01s period, sampling 100 times per second, timer 2 works in milliseconds...
  motorOff();
  //Serial.println("Transition to State 0");
}

// State machine being used for the main loop!
void loop() {
  switch (state) {
    case 0: { // Poll for button Press
        //Serial.println("Waiting : state 0");
        if (digitalRead(buttonPin)) {
          //Serial.println("Transition to State 1");
          state = 1;
          delay(1000);
        }
        break;
      }
    case 1: { // Initialize Timer, go to whatever value is referenced in original setpoint array
        Timer1.start();
        break;
      }
    case 2: { // Wait until button press, polling
        Timer1.stop();
        //Serial.println("Waiting : state 2");
        if (digitalRead(buttonPin)) {
          //Serial.println("Transition to State 3");
          state = 3;
          delay(1000);
        }
        break;
      }
    case 3: { // Initialize Timer2, Which enables the magnet sensor to start taking measurements
        MsTimer2::start();
        state = 4;
        break;
      }
    case 4: { // Wait until its time to disable Timer2
        if (digitalRead(buttonPin)) {
          MsTimer2::stop();
          sumMX = 0; // resetting summed values
          sumMY = 0;
          sumMZ = 0;
          counter = 1; //resetting counter
          //Serial.println("Transition to State 0");
          state = 0;
          delay(1000);
        }
      }

  }
}

// SETUP FUNCTIONS

/*
 * Function that configures register MDR0 (LS766R Quad Counter), based on the number of counts wanted (quadracount generally).
 * @param quad_counts variable the represents how many counts wanted
 */
 
void configureMDR0(uint8_t quad_counts) {
  MDR0 &= ~0b11;              // clear lowest two bits of MDR0
  MDR0 |= quad_counts;        // configure lowest two bits
  uint8_t tx_msg[2] = {write_MDR0, MDR0};
  spiObj.writeMultiSPI(tx_msg, 2);
}

/*
 * Function that configures register MDR1, which controls the number of outputted bytes. (Generally 4 bytes)
 * @param CNTR_len variable the represents how many bytes to return
 */
 
void configureMDR1(uint8_t CNTR_len) {
  CNTR_bytes = 4 - CNTR_len; // in case of lower bit output modes
  MDR1 &= ~0b11;          // clear lowest two bits of MDR1
  MDR1 |= CNTR_len;       // configure lowest two bits

  uint8_t tx_msg[2] = {write_MDR1, MDR1};
  spiObj.writeMultiSPI(tx_msg, 2);
}

/*
 * Clears the counter of the quadrature counter.
 */
void clearCNTR(void) {
  uint8_t tx_msg = clear_CNTR;
  spiObj.writeSPI(tx_msg); // cant be ambigious with just write
}


// Main functions

/*

  Timer FUNCTIONS!

*/

/*
 * This function calls the PI Control() function every 0.1 seconds
 */

void timer1Function(void) {
  PIControl();
}

/*
 * This function takes magnetic data measurements every 1/100 of a second, and averages the 100 points every second. The value is then outputted to serial.
 */
 
void timer2Function(void) {
  // Looping read of ALS31313 sensor in terms of how often it measures (100ms)
  interrupts(); // enable interrupts for I2C
  float magArr[3] = {}; //index 0: mx, index 1: my, index2: mz
  readALS31313(ALS31313Address, magArr);
  sumMX = sumMX + magArr[0]; // adding to a running sum for data in X
  sumMY = sumMY + magArr[1]; // adding to a running sum for data in Y
  sumMZ = sumMZ + magArr[2]; // adding to a running sum for data in Y

  if (counter % 100 == 0) {
    avgMX = sumMX / counter; // Getting an averaged value (dividing by 10)
    avgMY = sumMY / counter;
    avgMZ = sumMZ / counter;
    sumMX = 0; // resetting
    sumMY = 0;
    sumMZ = 0;
    //    Serial.print(avgMX);
    //    Serial.print(",");
    //    Serial.print(avgMY);
    //    Serial.print(",");
    Serial.println(avgMZ);
    counter = 1; //resetting counter
  } else {
    counter++; // if I havent reached 10 iterations, just count up every timer
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

  uint8_t arrRegister28[4] = {};
  uint8_t arrRegister29[4] = {};
  I2CreadRegister32bit(deviceAddress, 0x28, arrRegister28); // First 32 bit data register (0x28)
  I2CreadRegister32bit(deviceAddress, 0x29, arrRegister29); // Second 32 bit data register (0x29)

  // Byte Manipulation
  int16_t xMag = (int8_t)arrRegister28[0]; // Converting to 2's Complement
  xMag = (xMag << 4) | (0x0F & arrRegister29[1]); // Adding the 4 bits from second register
  int16_t yMag = (int8_t)arrRegister28[1];
  yMag = (yMag << 4) | ((0xF0 & arrRegister29[2]) >> 4);
  int16_t zMag = (int8_t)arrRegister28[2];
  zMag = (zMag << 4) | (0x0F & arrRegister29[2]);

  //  magData[0] = xMag; // raw data
  //  magData[1] = yMag;
  //  magData[2] = zMag;

  // Storing Variables in Array, magData[0] = angle in XY, magData[1] is angle in XZ, magData[2] = angle in YZ
  magData[0] = atan2f(float(yMag), float(xMag)) * 180.0 / M_PI; // converting to an angle, and multipling because radians
  magData[1] = atan2f(float(zMag), float(xMag)) * 180.0 / M_PI;
  magData[2] = atan2f(float(zMag), float(yMag)) * 180.0 / M_PI;
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

  i2cObj.readMultiI2C(deviceAddress, address, readArr, 4, error);
  // if the device accepted the address,
  // request 4 bytes from the device
  // and then read them, MSB first

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


/*

  ENCODER FUNCTIONS / TIMING!

*/


/*
 * This function reads the raw tick count from the quadrature counter. 
 * @return a long is returned which represents the raw tick count.
 */
 
long readCount(void) {
  uint8_t tx_msg[1] = {read_CNTR};
  // asking to read
  uint8_t rx_msg[4] = {};
  spiObj.writeThenReadSPI(tx_msg, 1, rx_msg, CNTR_bytes);

  countEncoder = (int8_t)rx_msg[0]; // converting to signed integers
  int ii;
  for (ii = 1; ii < (CNTR_bytes); ii++) {
    countEncoder = ((countEncoder) << 8 | (rx_msg[ii]));
  }
  return countEncoder;
}

/*
   PID / MOTOR CONTROL FUNCTIONS
*/


/*
 * This function implements PI control based on the encoder reading. The reference is the value provided before the program runs, and the input is the encoder angle reading.
 * 
 */

void PIControl() {
  encoderDegree = ((float)readCount()) / (360 / (29.86 * 12)); // Dividing by tick count for the specific motor I am using 
  Serial.print("Encoder Degree: ");
  Serial.println(encoderDegree);
  Input = encoderDegree;
  //    Serial.print("Input : ");
  //    Serial.println(Input);

  // Stopping condition, when reference and real is within 2 degrees
  if (abs(setPoint - Input) < 2) {
    digitalWrite(in1, LOW); // stop motor
    digitalWrite(in2, LOW);
    analogWrite(enA, 0); // stop the motor
    error_integral = 0; // reset error integral component
    previous_error = 0;
    setPointInd++; // advancing the index of reference angle
    if (setPointInd >= 4) {
      setPointInd = 0; // basically modulus operation
    }
    setPoint = setPointArr[setPointInd]; // go through the array of reference angles
    state = 2; // time to pause
    delay(2500); // quick pause

  } else { // Keep reading data and updating PWM (motor speed)
    Output = controlUpdate(Input, setPoint); // a value from -1 to 1 (saturation limits). Actual PI Control Function
    pwm_val = (int)((abs(Output)) * 255); // Adjusting for 0-225 range (8bits) for PWM value
    //      Serial.print("Output: ");
    //      Serial.println(Output);
    //      Serial.print("pwm_val: ");
    //      Serial.println(pwm_val);

    // Direction control
    if (Output > 0) {
      clockWiseRotate();
    } else {
      counterClockWiseRotate();
    }
    analogWrite(enA, pwm_val); // writing out speed
  }
}

// PID Controller code
// in = input, ref = reference (setPoint)

/*
 * This function is the actual PI Controller code. It calculates the respective error and uses saturation control.
 * @param in is the input which represents the measured encoder angle in my example.
 * @param ref is the reference (setPoint) that is determined before.
 * @return the return is the relative value that is calculated by the error and controller.
 */
 
double controlUpdate(double in, double ref) {
  double error = ref - in;                    // calculate error
  //    Serial.print("Error: ");
  //    Serial.println(error);
  error_integral += error * timer1dt; // carried between calculations
  double error_derivative = (error - previous_error) / timer1dt;

  //u represents the error for this iteration, through the controller
  double u = Kp * error + Ki * error_integral + Kd * error_derivative;
  //  Serial.print("U: ");
  //  Serial.println(u);


  // Setting upper limits
  if (u > 1) {
    u = 1;
  }
  else if (u < -1) {
    u = -1;
  }

  previous_error = error;
  return u;
}

/*
 * Simple clockwise rotation, setting the in1,in2 pins.
 */
void clockWiseRotate() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

/*
 * Simple counter-clockwise rotation, setting the in1,in2 pins.
 */
void counterClockWiseRotate() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

/*
 * Turning the motor off... setting enable and both in1,in2 to off.
 */
void motorOff() {
  analogWrite(enA, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}
