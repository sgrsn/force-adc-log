/******************************************************************************
*   @file    example.cpp
*   @brief   example to configure and get data from AD7177 ADC
*   @author  Hidaka Sato
*******************************************************************************/
#include <iomanip>
#include <memory>
#include <string>
#include <MsTimer2.h>
#include "AD7177.hpp"
#include "AS5601.hpp"

const double RATE = 1000;
double Fx = 0;
double Fz = 0;
void TimerCallBack();

AD7177 ad7177;
AS5601 as5601;

void setup() {
  Serial.begin(115200);
  Serial.println("start");

  ad7177.init();
  as5601.init();

  MsTimer2::set(1. / RATE * 1e3, TimerCallBack);
  MsTimer2::start();
}

void loop() {  
}

void TimerCallBack()
{
  /*レンジの設定値が最大出力電圧となる*/
  const double voltage2ustrain_x = 500./5.0;
  const double voltage2ustrain_z = 1000./5.0;
  const double ustrain2force_x = 0.05304;
  const double ustrain2force_z = 0.05858;

  ad7177.update();
  Fx = ad7177.get_ch0_voltage() * voltage2ustrain_x * ustrain2force_x;
  Fz = ad7177.get_ch1_voltage() * voltage2ustrain_z * ustrain2force_z;

  const double encoder2flap = -10.0 / 40.0;

  double rad = as5601.update();
  double flap_rad = rad * encoder2flap;
  double flap_hz = as5601.getRadPerSec() * encoder2flap / (2.*M_PI);

  std::stringstream ss;
  ss.str("");
  ss << "f: " << flap_hz << ", ";
  ss << "x: " << Fx;
  ss << ", z: " << Fz;
  Serial.println(ss.str().c_str());
}
