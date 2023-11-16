#include <Servo.h>
#include "HCSR04.h"
#include "Adafruit_MotorShield.h"
#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

//bluetooth settings
#define FACTORYRESET_ENABLE         1
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

//create motor shield object and 4 motors
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *Motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *Motor2 = AFMS.getMotor(2);
Adafruit_DCMotor *Motor3 = AFMS.getMotor(3);
Adafruit_DCMotor *Motor4 = AFMS.getMotor(4);

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
HCSR04 hc(2, 3); //initialisation class HCSR04 (trig pin , echo pin)
Servo servo1;

//constants & variables
const unsigned long distFactor = 276; //allows configuration of how fine the distance definition is (1000 = 10ms delay at 100 speed)(currently set to approximate 1 unit per mm)
const unsigned long angleFactor = 880; //allows configuration of how fine the angle definition is
const float stopDist = 20; //distance in centimeters that the car will stop
int servoPos = 90;

void setup()
{
  Serial.begin(9600);
  //start the motor shield
  if (!AFMS.begin()) {
    while (1);
  }

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

/***************************************************************/ 

  servo1.attach(10); //servo attached to pin 4
  servo1.write(servoPos);
  delay(200);
}

void loop()
{
  int currentDist = hc.dist(); //store distance value for this loop
  int speed = constrain(map(currentDist, 50, 400, 100, 255), 100, 255); //variable movement speed based on distances

  if (currentDist > stopDist) {
    goFC(speed);
  }
  else if(currentDist <= stopDist) {
    stop(50);
    int turnAngle = 0;
    while (turnAngle = 0) {
      turnAngle = findAngle();
      switch(turnAngle) { //decide how to avoid obstacle based on readings
        case 1:
          turnL(100, 90);
          break;
        case 2:
          turnL(100, 45);
          break;
        case 3:
          turnR(100, 45);
          break;
        case 4:
          turnR(100, 90);
          break;
        default:
          break;
      }
    }
    delay(200);
  }
}

//functions definitions
void goF(int speed, int dist) //drives straight forward
{
    Motor1->setSpeed(speed-2);
    Motor2->setSpeed(speed-2);
    Motor3->setSpeed(speed);
    Motor4->setSpeed(speed);

    Motor1->run(FORWARD);
    Motor2->run(FORWARD);
    Motor3->run(FORWARD);
    Motor4->run(FORWARD);
    delay((dist*distFactor)/speed);
}

void goFC(int speed) //drives straight forward
{
    Motor1->setSpeed(speed-2);
    Motor2->setSpeed(speed-2);
    Motor3->setSpeed(speed);
    Motor4->setSpeed(speed);

    Motor1->run(FORWARD);
    Motor2->run(FORWARD);
    Motor3->run(FORWARD);
    Motor4->run(FORWARD);
}

void goB(int speed, int dist) //drives straight backward
{
    Motor1->setSpeed(speed);
    Motor2->setSpeed(speed);
    Motor3->setSpeed(speed);
    Motor4->setSpeed(speed);

    Motor1->run(BACKWARD);
    Motor2->run(BACKWARD);
    Motor3->run(BACKWARD);
    Motor4->run(BACKWARD);
    delay((dist*distFactor)/speed);
}

void turnL(int speed, int angle) //turns vehicle left in place
{
    Motor1->setSpeed(speed);
    Motor2->setSpeed(speed);
    Motor3->setSpeed(speed);
    Motor4->setSpeed(speed);

    Motor1->run(BACKWARD);
    Motor2->run(BACKWARD);
    Motor3->run(FORWARD);
    Motor4->run(FORWARD);
    delay((angle*angleFactor)/speed);
}

void turnR(int speed, int angle) //turns vehicle right in place
{
    Motor1->setSpeed(speed);
    Motor2->setSpeed(speed);
    Motor3->setSpeed(speed);
    Motor4->setSpeed(speed);

    Motor1->run(FORWARD);
    Motor2->run(FORWARD);
    Motor3->run(BACKWARD);
    Motor4->run(BACKWARD);
    delay((angle*angleFactor)/speed);
}

void stop(int time) //stops motors
{
    Motor1->setSpeed(0);
    Motor2->setSpeed(0);
    Motor3->setSpeed(0);
    Motor4->setSpeed(0);

    Motor1->run(BRAKE);
    Motor2->run(BRAKE);
    Motor3->run(BRAKE);
    Motor4->run(BRAKE);
    delay(time);
}

int findAngle() //probes for highest distance at 4 different angles
{
  float distance = 0;
  int angleChoice = 0;

  servo1.write(130);
  delay(100);
  if(hc.dist() >= distance) {
    angleChoice = 2;
  }
  
  servo1.write(170);
  delay(100);
  if(hc.dist() >= distance) {
    angleChoice = 1;
  }

  servo1.write(50);
  delay(300);
  if(hc.dist() >= distance) {
    angleChoice = 3;
  }

  servo1.write(10);
  delay(100);
  if(hc.dist() >= distance) {
    angleChoice = 4;
  }

  servo1.write(90);
  return(angleChoice);
}
