
/*
MPL115A1 sparkfun breakout baropressure meter
 SDN       : pin 7
 CSN       : pin 10
 SDI/MOSI  : pin 11
 SDO/MISO  : pin 12
 SCK       : pin 13
*/


#include <SPIFunctions.h>

#define PRESH   0x80
#define   PRESL   0x82
#define   TEMPH   0x84
#define   TEMPL   0x86

#define A0MSB   0x88
#define A0LSB   0x8A
#define B1MSB   0x8C
#define B1LSB   0x8E
#define   B2MSB   0x90
#define B2LSB   0x92
#define C12MSB   0x94
#define   C12LSB   0x96

#define CONVERT   0x24   

#define chipSelectPin 10
#define shutDown 7

float A0_;
float B1_;
float B2_;
float C12_;
SPIFunctions spiObj(chipSelectPin);


void setup() {
  Serial.begin(9600);

  // initalize the data ready and chip select pins:
  pinMode(shutDown, OUTPUT);
  digitalWrite(shutDown, HIGH);
  delay (10);
 
  // read registers that contain the chip-unique parameters to do the math
  unsigned int A0H = spiObj.readSPI(A0MSB);
  unsigned int A0L = spiObj.readSPI(A0LSB);
         A0_ = (A0H << 5) + (A0L >> 3) + (A0L & 0x07) / 8.0;
         Serial.print("A0_ : ");
         Serial.println(A0_);

         
 
  unsigned int B1H = spiObj.readSPI(B1MSB);
  unsigned int B1L = spiObj.readSPI(B1LSB);
         B1_ = ( ( ( (B1H & 0x1F) * 0x100)+B1L) / 8192.0) - 3;    
         Serial.print("B1_ : ");
         Serial.println(B1_);        
  

  unsigned int B2H = spiObj.readSPI(B2MSB);
  unsigned int B2L = spiObj.readSPI(B2LSB);
        B2_ = ( ( ( (B2H - 0x80) << 8) + B2L) / 16384.0 ) - 2 ;
        Serial.print("B2_ : ");
        Serial.println(B2_);
      
       
                
  unsigned int C12H = spiObj.readSPI(C12MSB);
  unsigned int C12L = spiObj.readSPI(C12LSB);
          C12_ = ( ( ( C12H * 0x100 ) + C12L) / 16777216.0 )  ;
          Serial.print("C12_ : ");
          Serial.println(C12_);
          
 }

void loop() {
   Serial.print("The pressure is : ");
   Serial.print(baropPessure());
   Serial.println(" kPa");
   delay(1000);
} 

////Read registers
//unsigned int readRegister(byte thisRegister ) {
//  unsigned int result = 0;   // result to return
//  digitalWrite(chipSelectPin, LOW);
//    delay(10);
//    SPI.transfer(thisRegister);
//    result = SPI.transfer(0x00);
//  digitalWrite(chipSelectPin, HIGH);
//  return(result);
//}

//read pressure
float baropPessure(){
    uint8_t convert[2] = {0x24, 0x00};
    spiObj.writeMultiSPI(convert, 2);
    delay(3);
    
    uint8_t tempAndPressure[4] = {PRESH, PRESL, TEMPH, TEMPL};
    uint8_t results[4] = {};
    spiObj.writeAndReadSPI(tempAndPressure, results, 4);
    
//    Serial.print("presH: ");
//    Serial.println(results[0]);
    unsigned int presH = results[0];
//    Serial.print("presL: ");
//    Serial.println(results[1]);
    unsigned int presL = results[1];
//    Serial.print("tempH: ");
//    Serial.println(results[2]);
    unsigned int tempH = results[2];
//    Serial.print("tempL: ");
//    Serial.println(results[3]);
    unsigned int tempL = results[3];


  unsigned long press = ((presH *256) + presL)/64; // 10 BIT --> 16 bit / 64
  unsigned long temp  = ((tempH *256) + tempL)/64; // 10 BIT --> 16 bit / 64 

  float pressure = A0_+(B1_+C12_*temp)*press+B2_*temp;
  float preskPa = pressure * (65.0/1023.0)+50.0;
 
return(preskPa);
}
