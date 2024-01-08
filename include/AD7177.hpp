#pragma once

#include <iomanip>
#include <memory>
#include <string>
#include <unordered_map>
#include "ad717x.h"

class AD7177
{
private:
  ad717x_dev device;
  ad717x_dev *pdevice;
  ad717x_st_reg reg[15] = {};

  std::unordered_map<std::string, double> voltage_;

public:
  AD7177()
  {
    voltage_["ch0"] = 0;
    voltage_["ch1"] = 0;
  }

  void init()
  {
    reg[0].addr = AD717X_ADCMODE_REG;
    reg[0].value = AD717X_ADCMODE_REG_REF_EN | AD717X_ADCMODE_REG_MODE(0) | AD717X_ADCMODE_REG_CLKSEL(0);
    reg[0].size = 2;

    /*Interface mode register*/
    reg[1].addr = AD717X_IFMODE_REG;
    reg[1].value = AD717X_IFMODE_REG_DATA_WL16;// | AD717X_IFMODE_REG_DATA_STAT;
    reg[1].size = 2;

    /*GPIO Configuration Register*/
    reg[2].addr = AD717X_GPIOCON_REG;
    reg[2].value = 0x0800;
    reg[2].size = 2;

    /*Offset Register*/
    reg[3].addr = AD717X_OFFSET0_REG;
    reg[3].value = 0x800000;
    reg[3].size = 3;

    reg[4].addr = AD717X_OFFSET1_REG;
    reg[4].value = 0x800000;
    reg[4].size = 3;

    /*ID register; returns 16-bit model-specific ID; unique value for AD7177-2 is 0x4FDX*/
    reg[5].addr = AD717X_ID_REG;
    reg[5].value = 0x0000;
    reg[5].size = 2;

    /*Enable CH0*/
    reg[6].addr = AD717X_CHMAP0_REG;
    reg[6].value = AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(0) | AD717X_CHMAP_REG_AINNEG(1);
    reg[6].size = 2;
    /*Enable CH1*/
    reg[7].addr = AD717X_CHMAP1_REG;
    reg[7].value = AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(2) | AD717X_CHMAP_REG_AINNEG(3);
    reg[7].size = 2;

    /*Set as differential input with AIN0 positive input and AIN1 negative input*/
    reg[8].addr = AD717X_SETUPCON0_REG;
    reg[8].value = AD717X_SETUP_CONF_REG_BI_UNIPOLAR | AD717X_SETUP_CONF_REG_AINBUF_P | AD717X_SETUP_CONF_REG_AINBUF_N | AD717X_SETUP_CONF_REG_REF_SEL(0b00);
    reg[8].size = 2;
    
    reg[9].addr = AD717X_SETUPCON1_REG;
    reg[9].value = AD717X_SETUP_CONF_REG_BI_UNIPOLAR | AD717X_SETUP_CONF_REG_AINBUF_P | AD717X_SETUP_CONF_REG_AINBUF_N | AD717X_SETUP_CONF_REG_REF_SEL(0b00);
    reg[9].size = 2;

    /*
    01001 2500 SPS
    01010 1000 SPS
    01011 500 SPS
    01101 200 SPS
    01110 100 SPS
    */
    /*Digital filter settings*/
    reg[10].addr = AD717X_FILTCON0_REG;
    //reg[10].value = AD717X_FILT_CONF_REG_ENHFILTEN | AD717X_FILT_CONF_REG_ENHFILT(0b010) | AD717X_FILT_CONF_REG_ORDER(0) | AD717X_FILT_CONF_REG_ODR(0b00111);
    reg[10].value = AD717X_FILT_CONF_REG_ODR(0b01001);
    reg[10].size = 2;

    reg[11].addr = AD717X_FILTCON1_REG;
    //reg[11].value = AD717X_FILT_CONF_REG_ENHFILTEN | AD717X_FILT_CONF_REG_ENHFILT(0b010) | AD717X_FILT_CONF_REG_ORDER(0) | AD717X_FILT_CONF_REG_ODR(0b00111);
    reg[11].value = AD717X_FILT_CONF_REG_ODR(0b01001);
    reg[11].size = 2;

    /*Data register, where ADC conversion results are stored.*/
    reg[12].addr = AD717X_DATA_REG;
    reg[12].value = 0;
    reg[12].size = 4;

    reg[13].addr = AD717X_STATUS_REG;
    reg[13].value = 0;
    reg[13].size = 1;

    device.regs = reg;
    device.num_regs = 15;
    device.useCRC = AD717X_DISABLE;

    pdevice = &device;

    ad717x_init_param init_param;
    init_param.spi_init.spi = &SPI;
    init_param.spi_init.cs_pin = SS;
    init_param.spi_init.spi_mode = SPI_MODE3;
    init_param.regs = reg;
    init_param.num_regs = 15;

    AD717X_Init(&pdevice, init_param);

    // Show message //
    AD717X_ReadRegister(pdevice, AD717X_ID_REG);
    std::stringstream ss;
    ss << "[" << std::hex << AD717X_ID_REG << "] : " << AD717X_GetReg(pdevice, AD717X_ID_REG)->value;
    //Serial.println(ss.str().c_str());
    ss.str("");

    AD717X_WriteRegister(pdevice, AD717X_CHMAP0_REG);
    AD717X_WriteRegister(pdevice, AD717X_CHMAP0_REG);
    AD717X_WriteRegister(pdevice, AD717X_CHMAP1_REG);
    AD717X_WriteRegister(pdevice, AD717X_SETUPCON0_REG);
    AD717X_WriteRegister(pdevice, AD717X_SETUPCON1_REG);
    AD717X_WriteRegister(pdevice, AD717X_FILTCON0_REG);
    AD717X_WriteRegister(pdevice, AD717X_FILTCON1_REG);
  }

  double get_ch0_voltage()
  {
    return voltage_["ch0"];
  }
  double get_ch1_voltage()
  {
    return voltage_["ch1"];
  }

  void update()
  {
    int32_t data;
    AD717X_ReadData(pdevice, &data);

    const double Gain = 0x555555;
    const double Vref = 5.0;
    const double offset = 0x800000;
    const double voltage = (((double)data/2/Gain) * (double)0x400000 + (offset-(double)0x800000)) / (double)0x800000 / 0.75 * Vref - 2.5;
    
    int32_t ch = -1;
    AD717X_ReadRegister(pdevice, AD717X_STATUS_REG); 
    ch = (AD717X_GetReg(pdevice, AD717X_STATUS_REG)->value) & (uint32_t)0b11;

    if(ch == 0)
      voltage_["ch0"] = voltage;
    else if(ch == 1)
      voltage_["ch1"] = voltage;
  }

};