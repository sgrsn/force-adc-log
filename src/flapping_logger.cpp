/******************************************************************************
*   @file    example.cpp
*   @brief   example to configure and get data from AD7177 ADC
*   @author  Hidaka Sato
*******************************************************************************/
#include <iomanip>
#include <memory>
#include <string>
#include <MsTimer2.h>
#include "AS5601.hpp"
#include "motor_driver.hpp"

const double RATE = 1000;
const uint8_t photointerrupter_pin = 37;
void TimerCallBack();
void flappingAngleSetZero();
AS5601 as5601;
//MotorDriver motor(12, 10, 11);

void setup() {
  Serial.begin(115200);
  Serial.println("start");

  as5601.init();

  uint8_t dir = 12;
  uint8_t pwm = 10;
  uint8_t reset = 11;
  pinMode(dir, OUTPUT);
  pinMode(pwm, OUTPUT);
  pinMode(reset, OUTPUT);
  digitalWrite(dir, 1);
  digitalWrite(reset, 0);
  digitalWrite(pwm, 1);

  //motor.setPwmFrequency(20000);
  //motor.setDirection(0);
  //motor.setDuty(0.5);


  attachInterrupt(digitalPinToInterrupt(photointerrupter_pin), flappingAngleSetZero, FALLING);

  MsTimer2::set(1. / RATE * 1e3, TimerCallBack);
  MsTimer2::start();
}

void loop() { 
}

void TimerCallBack()
{
  const double encoder2flap = -10.0 / 40.0;

  double rad = as5601.update();
  double flap_rad = rad * encoder2flap;
  double flap_hz = as5601.getRadPerSec() * encoder2flap / (2.*M_PI);

  std::stringstream ss;
  ss.str("");
  ss << "f: " << flap_hz << ", ";
  ss << "rad: " << flap_rad;
  Serial.println(ss.str().c_str());
}

void flappingAngleSetZero()
{
  as5601.setRevZero();
  detachInterrupt(digitalPinToInterrupt(photointerrupter_pin));
}