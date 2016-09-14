//****************************************************************************
//Includes

#include <i2c_t3.h> // the I2C library that replaces Wire.h for the Teensy 3.2
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <FlexCAN.h>
#include <EEPROM.h>


//****************************************************************************
//Defines
#define PS0Pin      2
#define resetPin    22
#define IMUIntPin   6
#define STPin       17
#define ledPin      13

#define serialBaud  9600

#define sampleFreq  10


//****************************************************************************
//Instantiations
elapsedMillis sampleTimer;

bool ledState = LOW;

Adafruit_BNO055 bno = Adafruit_BNO055();


//****************************************************************************
//Function Declarations


//****************************************************************************
//Setup

void setup() {

  pinMode(ledPin, OUTPUT);
 
  pinMode(PS0Pin,OUTPUT);
  digitalWrite(PS0Pin,LOW);
  pinMode(resetPin,OUTPUT);
  digitalWrite(resetPin,LOW);
  pinMode(STPin,OUTPUT);
  digitalWrite(STPin,LOW);

  pinMode(IMUIntPin,INPUT);
  
  delay(20);
  digitalWrite(resetPin,HIGH);

  //Start the serial Conneciton
  Serial.begin(serialBaud);

  //Initialize the BNO
  bno.begin(bno.OPERATION_MODE_CONFIG);
  bno.write8(bno.BNO055_PAGE_ID_ADDR, 1); //switch to memory page 1
  byte currentAccelConfig = bno.read8(bno.BNO055_ACCEL_DATA_X_LSB_ADDR) & 0xFC; //read the 6 msb of the accel config
  byte newAccelConfig = currentAccelConfig | 0x3; //update the config to have 16G acceleration
  bno.write8(bno.BNO055_ACCEL_DATA_X_LSB_ADDR, newAccelConfig); //write the update to the config register
  //bno.setMode(bno.OPERATION_MODE_AMG);
  bno.setMode(bno.OPERATION_MODE_IMUPLUS);

  //Use the external crystal
  bno.setExtCrystalUse(true);
  
}


//****************************************************************************
//Main Loop

void loop() {


}
