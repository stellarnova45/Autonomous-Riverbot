#include "src/Adafruit_MotorShield.h"

const unsigned long distFactor = 276; //allows configuration of how fine the distance definition is (1000 = 10ms delay at 100 speed)(currently set to approximate 1 unit per mm)
const unsigned long angleFactor = 880; //allows configuration of how fine the angle definition is

//create motor shield object and 4 motors
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *Motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *Motor2 = AFMS.getMotor(2);
Adafruit_DCMotor *Motor3 = AFMS.getMotor(3);
Adafruit_DCMotor *Motor4 = AFMS.getMotor(4);

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
    Motor3->setSpeed(speed-2);
    Motor4->setSpeed(speed-2);

    Motor1->run(BACKWARD);
    Motor2->run(BACKWARD);
    Motor3->run(BACKWARD);
    Motor4->run(BACKWARD);
    delay((dist*distFactor)/speed);
}

void goBC(int speed) //drives straight backward
{
    Motor1->setSpeed(speed);
    Motor2->setSpeed(speed);
    Motor3->setSpeed(speed-2);
    Motor4->setSpeed(speed-2);

    Motor1->run(BACKWARD);
    Motor2->run(BACKWARD);
    Motor3->run(BACKWARD);
    Motor4->run(BACKWARD);
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

void turnLC(int speed) //turns vehicle left in place
{
    Motor1->setSpeed(speed);
    Motor2->setSpeed(speed);
    Motor3->setSpeed(speed);
    Motor4->setSpeed(speed);

    Motor1->run(BACKWARD);
    Motor2->run(BACKWARD);
    Motor3->run(FORWARD);
    Motor4->run(FORWARD);
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

void turnRC(int speed) //turns vehicle right in place
{
    Motor1->setSpeed(speed);
    Motor2->setSpeed(speed);
    Motor3->setSpeed(speed);
    Motor4->setSpeed(speed);

    Motor1->run(FORWARD);
    Motor2->run(FORWARD);
    Motor3->run(BACKWARD);
    Motor4->run(BACKWARD);
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