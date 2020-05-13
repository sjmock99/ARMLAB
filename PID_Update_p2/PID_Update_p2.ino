// Purpose: To test a PID Controller at 6V, using my Pololu Motor
// Result: Works within ~+-2 degree accuracy. The encoder resolution on the motor is only about ~1 degree anyways.
// Date : 5/12/20

#include <SPI.h>
#include <SPIFunctions.h>
SPIFunctions spiObj(53); // Slave select on 53

#include <TimerOne.h>


// Motors
int enA = 3;
int in1 = 4;
int in2 = 5;


// PID
boolean pause = false;
double setPoint; // master angle
double Input; // slave angle
double Output; // controller output
double Kp = 0.00135, Ki = 0.00021, Kd = 0;
int pwm_val = 0; // pwm variable for motor
double timer1dt = 0.1; // pwm loop time interval, 0.01s 
// 10^6 * timer1dt = microseconds --> 10^5 microseconds = 0.1s
double error_integral = 0;
double previous_error = 0;

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


// Constants for Measurement
long count = 0;                  // signed count, make it a long to hold 32 bits
int CNTR_bytes = 4;                // default length of CNTR register
uint8_t MDR0 = 0x00;          // variables to be editted based on configuration
uint8_t MDR1 = 0x00;          // variables to be editted based on configuration
float encoderDegree;


// Timing
double setPointArr[4] = {90, 180, 270, 360}; // array of values to transverse through
int setPointInd = 0; // indicie of the array
unsigned long pauseTimer = 0; // how long to pause

void setup() {
  
  
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  delay(1000);

  // Adjust PID Values
  setPoint = setPointArr[0]; // set point start at 0th index

  Serial.begin(115200);
  SPI.begin();

  configureMDR0(four_count); // doing quadracount
  configureMDR1(four_byte); // setting output to four bytes
  clearCNTR(); // clearing whatever data is in the encoder

  delay(2500);

  // Timing
  Timer1.initialize((int)(timer1dt * 1000000)); // reprents the timer that is constantly reading the encoder, every 100 millis (tenth of a second)
  Timer1.attachInterrupt(timer1Function);

  
}

void loop() {}

// SETUP FUNCTIONS

void configureMDR0(uint8_t quad_counts) {
  MDR0 &= ~0b11;              // clear lowest two bits of MDR0
  MDR0 |= quad_counts;        // configure lowest two bits
  uint8_t tx_msg[2] = {write_MDR0, MDR0};
  spiObj.writeMultiSPI(tx_msg, 2);
}

void configureMDR1(uint8_t CNTR_len) {
  CNTR_bytes = 4 - CNTR_len; // in case of lower bit output modes

  MDR1 &= ~0b11;          // clear lowest two bits of MDR1
  MDR1 |= CNTR_len;       // configure lowest two bits

  uint8_t tx_msg[2] = {write_MDR1, MDR1};
  spiObj.writeMultiSPI(tx_msg, 2);
}

void clearCNTR(void) {
  uint8_t tx_msg = clear_CNTR;
  spiObj.writeSPI(tx_msg); // cant be ambigious with just write
}


// Main functions

// Looping read and PWM Out Function associated with Timer1
void timer1Function(void) {
  encoderDegree = ((float)readCount()) / (360 / (29.86 * 12));
  Serial.print("Encoder Degree: ");
  Serial.println(encoderDegree);
  Input = encoderDegree;
  //    Serial.print("Input : ");
  //    Serial.println(Input);

  if (!pause) {
    // Stopping condition, when reference and real is within 2 degrees
    if (abs(setPoint - Input) < 2) {
      pause = true;
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      analogWrite(enA, 0); // writing out speed
      error_integral = 0; // reset error integral component
      previous_error = 0;
      Serial.println("PAUSE!");
      setPointInd++; // advancing the index
      if (setPointInd >= 4) {
        setPointInd = 0; // basically modulus operation
      }

      setPoint = setPointArr[setPointInd]; // go through the array

    } else { // Keep reading data and updating PWM (motor speed)

      Output = controlUpdate(Input, setPoint); // a value from -1 to 1

      pwm_val = (int)((abs(Output)) * 255);

      Serial.print("Output: ");
      Serial.println(Output);
      Serial.print("pwm_val: ");
      Serial.println(pwm_val);

      // Direction control
      if (Output > 0) {
        clockWiseRotate();
      } else {
        counterClockWiseRotate();
      }

      analogWrite(enA, pwm_val); // writing out speed
    }
  } else {
    // Waiting 5000s between
    if (pauseTimer >= 5000) {
      pause = false;
      pauseTimer = 0;
    } else {
      pauseTimer += timer1dt * 1000; // 100 millis per iteration
    }
  }

}


// PID Controller code
// in = input, ref = reference (setPoint)
double controlUpdate(double in, double ref) {
  double error = ref - in;                    // calculate error
  //    Serial.print("Error: ");
  //    Serial.println(error);
  error_integral += error * timer1dt; // carried between calculations
  double error_derivative = (error - previous_error) / timer1dt;

  //u represents the error for this iteration, through the controller
  double u = Kp * error + Ki * error_integral + Kd * error_derivative;

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

long readCount(void) { 
  uint8_t tx_msg[1] = {read_CNTR};
  // asking to read
  uint8_t rx_msg[4] = {};
  spiObj.writeThenReadSPI(tx_msg, 1, rx_msg, CNTR_bytes);

  count = (int8_t)rx_msg[0]; // converting to signed integers
  int ii;
  for (ii = 1; ii < (CNTR_bytes); ii++) {
    count = ((count) << 8 | (rx_msg[ii]));
  }
  return count;
}

void clockWiseRotate() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

void counterClockWiseRotate() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}


void motorOff() {
  analogWrite(enA, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}
