#define ENABLE_EASE_CUBIC
#define ENABLE_EASE_BOUNCE
#define ENABLE_EASE_BACK
#include <ServoEasing.hpp>
#include "src/HCSR04.h"
#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "BluefruitConfig.h"
#include "motor_directions.h"
#include "RunningAverage.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

//bluetooth settings
#define FACTORYRESET_ENABLE         0
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
// A small helper
void error(const __FlashStringHelper*err) {
  //Serial.println(err);
  while (1);
}
// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);
// the packet buffer
extern uint8_t packetbuffer[];
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

HCSR04 hc(2, 3); //initialisation class HCSR04 (trig pin , echo pin)
ServoEasing servo1;

RunningAverage distRA(7);

//global constants & variables
float currentDist = 40;
float stopDist = 30; //distance in centimeters that the car will stop
bool manualOverride = true;
bool stillPressed = false;
int manualSpeed = 150;
bool lastServo = false;
bool servoMid = true;
unsigned long lastMill = 0;
unsigned long millPulse = 60;
float distAverage;

/******************************************************************************************************/
/* * * * * * * * * * * * * * * * * * * * * Setup  * * * * * * * * * * * * * * * * * * * * * * * * * * */
/******************************************************************************************************/
void setup()
{
  //Serial.begin(9600);
  //start the motor shield
  if (!AFMS.begin()) {
    while (1);
  }
  
  servo1.attach(10, 90); //servo attached to pin 10

/* Initialise the module */
  //Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  //Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    //Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }


  /* Disable command echo from Bluefruit */
  ble.echo(false);

  //Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  //Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  //Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  //Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */


  //Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    //Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  //Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  //Serial.println(F("******************************"));


  servo1.setEasingType(EASE_BOUNCE_OUT);
  servo1.setSpeed(50);
  delay(500);
  servo1.easeTo(0);
  servo1.easeTo(180, 100);
  servo1.easeTo(90);

}

/******************************************************************************************************/
/* * * * * * * * * * * * * * * * * * * * * Loop * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/******************************************************************************************************/
void loop()
{
  if (manualInput()) return; //the bluetooth functionality, checks for any data
  if (stillPressed) return; //prevents continuation of code when interrupted by input
  if (manualOverride) return; //responsible for the override stop functionality

  if (!servo1.isMoving()) {
    if (lastServo) {
      if (!servoMid) {
        servo1.setEasingType(EASE_CUBIC_OUT);
        servo1.startEaseTo(90, 350);
        servoMid = true;
        stopDist = 20;
      }
      else{
        servo1.setEasingType(EASE_CUBIC_IN);
        servo1.startEaseTo(165, 350);
        servoMid = false;
        lastServo = false;
        stopDist = 30;
      }
    }
    else {
      if (!servoMid) {
        servo1.setEasingType(EASE_CUBIC_OUT);
        servo1.startEaseTo(90, 350);
        servoMid = true;
        stopDist = 20;
      }
      else{
        servo1.setEasingType(EASE_CUBIC_IN);
        servo1.startEaseTo(10, 350);
        servoMid = false;
        lastServo = true;
        stopDist = 30;
      }
    }
  }
  if(millis() - lastMill >= millPulse){
    currentDist = hc.dist(); //store distance value for this loop
    distRA.addValue(currentDist);
    distAverage = distRA.getAverage();
    lastMill = millis();
  }

  int speed = constrain(map(distAverage, 150, 180, 90, 255), 90, 255); //variable movement speed based on distances
  //Serial.print("Distance: "); Serial.println(distAverage);
  //Serial.print("Speed: "); Serial.println(speed);

  if (currentDist > stopDist) {
    goFC(speed);
    return;
  }
  else if(currentDist <= stopDist) {
    stop(50);
    if (decideDirection()) {
      while(currentDist <= 70) {
        turnRC(75);
        currentDist = hc.dist();
        //Serial.println(currentDist);
        delay(60);
      }
      delay(950);
      stop(1);
    }
    else {
      if (manualOverride) return;
      while(currentDist <= 70) {
        turnLC(75);
        currentDist = hc.dist();
        //Serial.println(currentDist);
        delay(60);
      }
      delay(950);
      stop(1);
    }
    speed = 50;
  }
}

/******************************************************************************************************/
/* * * * * * * * * * * * * * * *  Functions * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/******************************************************************************************************/

//Decide which way to turn (Right = Return True)
bool decideDirection() //finds average distance between left and right directions, then returns the higher valued direction
{
  int pulseCount = 0;
  float leftDistAverage = 0;
  float rightDistAverage = 0;
  servo1.setEasingType(EASE_CUBIC_IN_OUT);
  servo1.easeTo(90, 1000); //move sensor to middle in case it isn't already

  servo1.startEaseTo(160, 50); //non-blocking movement
    delay(200);
    while(servo1.isMoving()) { //takes 15 pulses of distance for average
      leftDistAverage = leftDistAverage + hc.dist();
      if (manualInput()) break; //manualInput takes 60 milliseconds. This is critical to this function.
      pulseCount++;
    }
  leftDistAverage = leftDistAverage/pulseCount;
  pulseCount = 0;

  servo1.easeTo(90, 1000);
  servo1.startEaseTo(20, 50); //non-blocking movement
    delay(200);
    while(servo1.isMoving()) { //takes 15 pulses of distance for average
      rightDistAverage = rightDistAverage + hc.dist();
      if (manualInput()) break; //manualInput takes 60 milliseconds. This is critical to this function.
      pulseCount++;
    }
  rightDistAverage = rightDistAverage/pulseCount;

  servo1.easeTo(90, 1000); //return sensor to middle
  delay(500);
  if (manualOverride) return false;
  if (rightDistAverage >= leftDistAverage) return true;
  else return false;
}

/**********************************************************************************************************************************************************/
/**********************************************************************************************************************************************************/

//determines if a manual input has been sent over bluetooth and performs requested action
bool manualInput()
{
  uint8_t len = readPacket(&ble, 60); //60 millisecond timeout, some functions rely on this. I know that's bad but I'll fix it if I get the time.
  if (stillPressed) {
    len = 1;
  }
  if (len == 0) return false; //no new button presses

  // Buttons
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    //Serial.print ("Button "); //Serial.print(buttnum);
    if (pressed) {
      //Serial.println(" pressed");
    } else {
      //Serial.println(" released");
    }

    switch(buttnum) {
      case 1: //start manual only controls
        manualOverride = true;
        //Serial.println("overridden");
        stop(1);
        servo1.setEasingType(EASE_BOUNCE_OUT);
        servo1.startEaseTo(90, 40);
        break;
      case 2: //return to automatic driving
        manualOverride = false;
        //Serial.println("unoverride");
        servo1.setEasingType(EASE_CUBIC_IN_OUT);
        break;
      case 3: //decrease speed of manual inputs
        if (pressed){
          manualSpeed = manualSpeed - 50;
          int constrainSpeed = constrain(manualSpeed, 60, 255);
          manualSpeed = constrainSpeed;
          //Serial.print("Speed is now: "); //Serial.println(manualSpeed);
        }
        break;
      case 4: //increase speed of manual inputs
        if (pressed){
          manualSpeed = manualSpeed + 50;
          int constrainSpeed = constrain(manualSpeed, 60, 255);
          manualSpeed = constrainSpeed;
          //Serial.print("Speed is now: "); //Serial.println(manualSpeed);
        }
        break;
      case 5: //up dpad, goes forward
        if (pressed) {
          goFC(manualSpeed);
          stillPressed = true;
        }
        else {
          //Serial.println("stop forward");
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
    //Serial.print ("RGB #");
    if (red < 0x10) //Serial.print("0");
    //Serial.print(red, HEX);
    if (green < 0x10) //Serial.print("0");
    //Serial.print(green, HEX);
    if (blue < 0x10) //Serial.print("0");
    //Serial.println(blue, HEX);
  }
  */

  return true; //sets loop to immediately check for new input
}