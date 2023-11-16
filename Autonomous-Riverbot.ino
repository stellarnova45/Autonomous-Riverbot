#include <Servo.h>
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

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
HCSR04 hc(2, 3); //initialisation class HCSR04 (trig pin , echo pin)
Servo servo1;

//global constants & variables
const float stopDist = 20; //distance in centimeters that the car will stop
int servoPos = 90;
bool manualOverride = true;

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
  if (manualInput()) return;
  if (manualOverride) return;

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

int findAngle() //probes for highest distance at 4 different angles
{
  float distance = 0;
  int angleChoice = 0;

  servo1.write(130);
  delay(100);
  if(hc.dist() >= distance) {
    angleChoice = 2;
  }
  
  servo1.write(50);
  delay(200);
  if(hc.dist() >= distance) {
    angleChoice = 3;
  }

  servo1.write(170);
  delay(300);
  if(hc.dist() >= distance) {
    angleChoice = 1;
  }

  servo1.write(10);
  delay(400);
  if(hc.dist() >= distance) {
    angleChoice = 4;
  }

  servo1.write(90);
  return(angleChoice);
}

bool manualInput() //determines if a manual input has been sent and performs requested action
{
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
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
        break;
      case 2:
        manualOverride = false;
        break;
      case 3:
        break;
      case 4:
        break;
      case 5:
        if (pressed) {
          goFC(150);
        } else {
          stop(1);
        }
        break;
      case 6:
        if (pressed) {
          goBC(150);

        } else {
          stop(1);
        }
        break;
      case 7:
        if (pressed) {
          turnLC(100);
        } else {
          stop(1);
        }
        break;
      case 8:
        if (pressed) {
          turnRC(100);
        } else {
          stop(1);
        }
        break;
    }
  }
  return false;
}