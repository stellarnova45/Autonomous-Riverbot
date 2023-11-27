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
const float stopDist = 30; //distance in centimeters that the car will stop
int servoPos = 90;
bool manualOverride = true;
bool stillPressed = false;
int manualSpeed = 150;
unsigned long currentMilli = 0;
unsigned long lastMilli = 0;
unsigned long milliPulse = 300;
bool lastServo = false;

/******************************************************************************************************/
/* * * * * * * * * * * * * * * * * * * * * Setup  * * * * * * * * * * * * * * * * * * * * * * * * * * */
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
/* * * * * * * * * * * * * * * * * * * * * Loop * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/******************************************************************************************************/
void loop()
{
  if (manualInput()) return; //the bluetooth functionality, checks for any data
  if (stillPressed) return; //prevents continuation of code when interrupted by input
  if (manualOverride) return; //responsible for the override stop functionality
  currentMilli = millis();
  Serial.println(currentMilli);

  if (currentMilli >= lastMilli + milliPulse) {
    if (lastServo) { 
      servo1.startEaseTo(125, 350);
      lastServo = false;
    }
    else {
      servo1.startEaseTo(55, 350);
      lastServo = true;
    }
    lastMilli = millis();
  }
  
  float currentDist = hc.dist(); //store distance value for this loop
  int speed = constrain(map(currentDist, 50, 120, 100, 200), 100, 200); //variable movement speed based on distances
  Serial.print("Distance: "); Serial.println(currentDist);
  Serial.print("Speed: "); Serial.println(speed);

  if (currentDist > stopDist) {
    goFC(speed);
    return;
  }
  else if(currentDist <= stopDist) {
    stop(50);
    if (decideDirection()) {
      while(currentDist <= 50) {
        turnRC(75);
        currentDist = hc.dist();
        Serial.println(currentDist);
        delay(60);
      }
      delay(700);
      stop(1);
    }
    else {
      if (manualOverride) return;
      while(currentDist <= 50) {
        turnLC(75);
        currentDist = hc.dist();
        Serial.println(currentDist);
        delay(60);
      }
      delay(700);
      stop(1);
    }
    speed = 50;
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
  servo1.easeTo(90, 1000); //set sensor to middle in case it isn't already

  servo1.startEaseTo(160, 50); //non-blocking movement
    delay(200);
    for(int i = 0; i <= 15; i++) { //takes 15 pulses of distance for average
      leftDistAverage = leftDistAverage + hc.dist();
      if (manualInput()) break; //manualInput takes 60 milliseconds. This is critical to this function.
    }
  leftDistAverage = leftDistAverage/15;

  servo1.easeTo(90, 1000);
  servo1.startEaseTo(20, 50); //non-blocking movement
    delay(200);
    for(int i = 0; i <= 15; i++) { //takes 15 pulses of distance for average
      rightDistAverage = rightDistAverage + hc.dist();
      if (manualInput()) break; //manualInput takes 60 milliseconds. This is critical to this function.
    }
  rightDistAverage = rightDistAverage/15;

  servo1.easeTo(90, 1000); // return sensor to middle
  delay(500);
  if (manualOverride) return false;
  if (rightDistAverage >= leftDistAverage) return true;
  else return false;
}

/**********************************************************************************************************************************************************/
/**********************************************************************************************************************************************************/

//determines if a manual input has been sent and performs requested action
bool manualInput()
{
  uint8_t len = readPacket(&ble, 60); //60 millisecond timeout, some functions rely on this. I know its bad but I'll fix it if I get the time.
  if (stillPressed) {
    len = 1;
  }
  if (len == 0) return false; //no new button presses

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
      case 1: //start manual only controls
        manualOverride = true;
        Serial.println("overridden");
        stop(1);
        break;
      case 2: //return to automatic driving
        manualOverride = false;
        Serial.println("unoverride");
        break;
      case 3: //decrease speed of manual inputs
        if (pressed){
          manualSpeed = manualSpeed - 10;
          int constrainSpeed = constrain(manualSpeed, 50, 255);
          manualSpeed = constrainSpeed;
          Serial.print("Speed is now: "); Serial.println(manualSpeed);
          stillPressed = true;
        }
        else stillPressed = false; return;
        break;
      case 4: //increase speed of manual inputs
        if (pressed){
          manualSpeed = manualSpeed + 10;
          int constrainSpeed = constrain(manualSpeed, 50, 255);
          manualSpeed = constrainSpeed;
          Serial.print("Speed is now: "); Serial.println(manualSpeed);
          stillPressed = true;
        }
        else stillPressed = false; return;
        break;
      case 5: //up dpad, goes forward
        if (pressed) {
          goFC(manualSpeed);
          stillPressed = true;
        }
        else {
          Serial.println("stop forward");
          stop(1);
          stillPressed = false;
        }
        break;
      case 6: //down dpad, goes backward
        if (pressed) {
          goBC(manualSpeed);
          stillPressed = true;
        } 
        else {
          stop(1);
          stillPressed = false;
        }
        break;
      case 7: //left dpad, turns left
        if (pressed) {
          turnLC(manualSpeed);
          stillPressed = true;
        } 
        else {
          stop(1);
          stillPressed = false;
        }
        break;
      case 8: //right dpad, turns right
        if (pressed) {
          turnRC(manualSpeed);
          stillPressed = true;
        } 
        else {
          stop(1);
          stillPressed = false;
        }
        break;
    }
  }

  /*
  // Color ***potential for manual control over RGB LEDs***
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
  */

  return true; //sets loop to immediately check for new input
}