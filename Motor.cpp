#include <Arduino.h>
#include "Motor.h"

Motor::Motor(short pinA, short pinB, short speedPin, short deadZone, short zeroThreshold) : 
    pinA(pinA), pinB(pinB), speedPin(speedPin), deadZone(deadZone), zeroThreshold(zeroThreshold) {
}

void Motor::init() {
  pinMode(this->pinA, OUTPUT);
  pinMode(this->pinB, OUTPUT);
  pinMode(this->speedPin, OUTPUT);
}

void Motor::left(int speed) {
  this->setMotor(speed, LEFT);
}

void Motor::right(int speed) {
  this->setMotor(speed, RIGHT);
}

void Motor::stop() {
  this->setSpeed(0);
}

void Motor::setSpeed(int speed) {
  if (speed < this->zeroThreshold) {
    analogWrite(this->speedPin, 0);
  } else {
    analogWrite(this->speedPin, 255);
  }
//  analogWrite(this->speedPin, (speed < this->zeroThreshold ? 0 : constrain(map(speed, 0, 255, this->deadZone, 255), 0, 200)));
}

void Motor::setMotor(int speed, Dir dir) {
  this->setSpeed(speed);
  if (dir == LEFT) {
     digitalWrite(this->pinA,LOW);
     digitalWrite(this->pinB,HIGH);
  } else {
     digitalWrite(this->pinA,HIGH);
     digitalWrite(this->pinB,LOW);
  }
}

void Motor::setMotor(int speed) {
  if (speed > 0) {
    this->setMotor(speed, RIGHT);
  } else {
    this->setMotor(-speed, LEFT);
  }
}
