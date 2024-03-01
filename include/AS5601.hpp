#pragma once

#include <stdint.h>
#include <Wire.h>
#include <cmath>

#define AS5600_AS5601_DEV_ADDRESS      0x36
#define AS5600_AS5601_REG_RAW_ANGLE    0x0C

class AS5601
{
private:
  double rad_;
  double last_rad_;
  double rev_;
  uint16_t last_raw_;
  double last_t_;
  double rad_per_sec_;

public:
  AS5601() : rad_(0), rev_(0), last_raw_(0), last_t_(0)
  {
  }

  void init()
  {
    // I2C init
    Wire.begin();
    Wire.setClock(400000);
  }

  double update()
  {
    double time = micros();
    uint16_t raw = getRawData();

    /*周回判定*/
    if(fabs(raw - last_raw_) > 2000)
    {
      if(raw < last_raw_)
        rev_ += 1.0;
      else
        rev_ -= 1.0;
    }
    rad_ = (double)raw  / 4096 * 2*M_PI + rev_ * 2*M_PI;

    rad_per_sec_ = (rad_ - last_rad_) / ((time - last_t_)*1.0e-6);

    last_rad_ = rad_;
    last_raw_ = raw;
    last_t_ = time;
    return rad_;
  }

  double getRad() { return rad_; }
  double getRadPerSec() {return rad_per_sec_; }

  uint16_t getRawData()
  {
    // Read RAW_ANGLE value from encoder
    Wire.beginTransmission(AS5600_AS5601_DEV_ADDRESS);
    Wire.write(AS5600_AS5601_REG_RAW_ANGLE);
    Wire.endTransmission(false);
    Wire.requestFrom(AS5600_AS5601_DEV_ADDRESS, 2);
    uint16_t RawAngle = 0;
    RawAngle  = ((uint16_t)Wire.read() << 8) & 0x0F00;
    RawAngle |= (uint16_t)Wire.read();
    return RawAngle;
  }

  void setRevZero()
  {
    rev_ = 0;
  }
};