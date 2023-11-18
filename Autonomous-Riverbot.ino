#include <ServoEasing.hpp>
#include "src/HCSR04.h"
#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "BluefruitConfig.h"
#include "motor_directions.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

//bluetooth settings
#define FACTORYRESET_ENABLE         0
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}
// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);
// the packet buffer
extern uint8_t packetbuffer[];

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
HCSR04 hc(3, 2); //initialisation class HCSR04 (trig pin , echo pin)
ServoEasing servo1;

//global constants & variables
float currentDist;
const float stopDist = 20; //distance in centimeters that the car will stop
int servoPos = 90;
bool manualOverride = true;
int manualSpeed = 150;
int milliStore = 0;
int milliPulse = 60;

/******************************************************************************************************/
/* * * * * * * * * * * * * * * *  Setup * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/******************************************************************************************************/
void setup()
{
  Serial.begin(9600);
  //start the motor shield
  if (!AFMS.begin()) {
    while (1);
  }
  
  servo1.attach(10, 90); //servo attached to pin 10
  servo1.setSpeed(150);
  servo1.setEasingType(EASE_CUBIC_IN_OUT);

/* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }


  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));
}

/******************************************************************************************************/
/* * * * * * * * * * * * * * * *  Loop  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/******************************************************************************************************/
void loop()
{
  if (manualInput()) return; //the bluetooth functionality, checks for any data
  if (manualOverride) return; //responsible for the override stop functionality

  if (millis() >= milliStore + milliPulse){ 
    float currentDist = hc.dist(); //store distance value for this loop
    milliStore = millis();
  }
  int speed = constrain(map(currentDist, 50, 120, 100, 255), 100, 255); //variable movement speed based on distances
  Serial.println(currentDist);
  Serial.println(speed);

  if (currentDist > stopDist) {
    goFC(speed);
    return;
  }
  else if(currentDist <= stopDist) {
    stop(50);
    if (decideDirection()) {
      for(int i = 0; i <= 2000 || hc.dist() <= 40; i++) {
        turnRC(100);
      }
      stop(1);
    }
    else {
      for(int i = 0; i <= 2000 || hc.dist() <= 40; i++) {
        turnLC(100);
      }
      stop(1);
    }
  }
}

/******************************************************************************************************/
/* * * * * * * * * * * * * * * *  Functions * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/******************************************************************************************************/

//Decide Right = True
bool decideDirection() //finds average distance between left and right directions, then returns the higher valued direction
{
  float leftDistAverage = 0;
  float rightDistAverage = 0;
  servo1.easeTo(90, 1000);

  servo1.startEaseTo(160, 50);
    delay(200);
    for(int i = 0; i <= 15; i++) {
      leftDistAverage = leftDistAverage + hc.dist();
      if (manualInput()) break;
    }
    leftDistAverage = leftDistAverage/15;

  servo1.easeTo(90, 1000);
  servo1.startEaseTo(20, 50);
    delay(200);
    for(int i = 0; i <= 15; i++) {
      rightDistAverage = rightDistAverage + hc.dist();
      if (manualInput()) break;
    }
    rightDistAverage = rightDistAverage/15;

  servo1.startEaseTo(90, 1000);
  if (rightDistAverage >= leftDistAverage) return true;
  else return false;
}

//determines if a manual input has been sent and performs requested action
bool manualInput()
{
  uint8_t len = readPacket(&ble, 60);
  if (len == 0) return false;

  // Color
  if (packetbuffer[1] == 'C') {
    uint8_t red = packetbuffer[2];
    uint8_t green = packetbuffer[3];
    uint8_t blue = packetbuffer[4];
    Serial.print ("RGB #");
    if (red < 0x10) Serial.print("0");
    Serial.print(red, HEX);
    if (green < 0x10) Serial.print("0");
    Serial.print(green, HEX);
    if (blue < 0x10) Serial.print("0");
    Serial.println(blue, HEX);
  }

  // Buttons
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    Serial.print ("Button "); Serial.print(buttnum);
    if (pressed) {
      Serial.println(" pressed");
    } else {
      Serial.println(" released");
    }

    switch(buttnum) {
      case 1:
        manualOverride = true;
        Serial.println("overridden");
        stop(1);
        break;
      case 2:
        manualOverride = false;
        Serial.println("unoverride");
        break;
      case 3:
        while (pressed){
          manualSpeed = manualSpeed - 10;
          int constrainSpeed = constrain(manualSpeed, 50, 255);
          manualSpeed = constrainSpeed;
          Serial.println(manualSpeed);
        }
        break;
      case 4:
        while (pressed){
          manualSpeed = manualSpeed + 10;
          int constrainSpeed = constrain(manualSpeed, 50, 255);
          manualSpeed = constrainSpeed;
          Serial.println(manualSpeed);
        }
        break;
      case 5:
        if (pressed) {
          goFC(manualSpeed);
        }
        else {
          Serial.println("stop forward");
          stop(1);
        }
        break;
      case 6:
        if (pressed) {
          goBC(manualSpeed);

        } 
        else {
          stop(1);
        }
        break;
      case 7:
        if (pressed) {
          turnLC(manualSpeed);
        } 
        else {
          stop(1);
        }
        break;
      case 8:
        if (pressed) {
          turnRC(manualSpeed);
        } 
        else {
          stop(1);
        }
        break;
    }
  }
  return false;
}