/******************************************************************************
*   @file    example.cpp
*   @brief   example to configure and get data from AD7177 ADC
*   @author  Hidaka Sato
*******************************************************************************/
#include <iomanip>
#include <memory>
#include <string>
#include <MsTimer2.h>
#include "ad717x.h"

#include <iostream>
#include <vector>
#include <numeric>
#include <cmath>
#include <algorithm>

const uint8_t RDY_PIN = 12;

const double RATE = 1000;
double Fx = 0;
double Fz = 0;
void TimerCallBack();

ad717x_dev *pdevice;

void setup() {
  /* initiate serial communication */
  Serial.begin(115200);
  Serial.println("start");

  std::stringstream ss;
  
  ad717x_st_reg reg[15] = {};
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
  //reg[10].value = AD717X_FILT_CONF_REG_ENHFILTEN | AD717X_FILT_CONF_REG_ENHFILT(0b010) | AD717X_FILT_CONF_REG_ORDER(0) | AD717X_FILT_CONF_REG_ODR(0b00111);]
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

  ad717x_dev device;
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

  MsTimer2::set(1. / RATE * 1e3, TimerCallBack);
  MsTimer2::start();

  while(true)
  {

  }
}

void lim_size(std::vector<double>& vec, size_t size)
{
  if (vec.size() >= size) {
    // 最大サイズに達していれば、最も古いデータを削除
    vec.erase(vec.begin());
  }
}

// ハイパスフィルタ関数
std::vector<double> highPassFilter(const std::vector<double>& data, double cutoff) {
    std::vector<double> filteredData;
    for (size_t i = 1; i < data.size(); ++i) {
        double diff = data[i] - data[i-1];
        filteredData.push_back(diff);
    }
    return filteredData;
}

// 外れ値を検出する関数
std::vector<bool> detectOutliers(const std::vector<double>& data, double threshold) {
    std::vector<bool> isOutlier(data.size(), false);
    double mean = std::accumulate(data.begin(), data.end(), 0.0) / data.size();
    double sq_sum = std::inner_product(data.begin(), data.end(), data.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / data.size() - mean * mean);

    for (size_t i = 0; i < data.size(); ++i) {
        if (std::abs(data[i] - mean) > threshold * stdev) {
            isOutlier[i] = true;
        }
    }
    return isOutlier;
}

// 元のデータから外れ値を除去する関数
std::vector<double> removeOutliers(const std::vector<double>& originalData, 
                                   const std::vector<double>& filteredData, 
                                   const std::vector<bool>& isOutlier, 
                                   double cutoff) {
    std::vector<double> cleanedData;
    size_t j = 0; // filteredDataのインデックス
    for (size_t i = 1; i < originalData.size(); ++i) {
        if (std::abs(filteredData[j]) > cutoff && isOutlier[j]) {
            // 外れ値をスキップ
        } else {
            cleanedData.push_back(originalData[i]);
        }
        j++;
    }
    return cleanedData;
}

void loop() {  
}

std::vector<double> v0;
std::vector<double> v1;

double get_voltage(int32_t data)
{
  const double Gain = 0x555555;
  const double Vref = 5.0;
  const double offset = 0x800000;
  const double voltage = (((double)data/2/Gain) * (double)0x400000 + (offset-(double)0x800000)) / (double)0x800000 / 0.75 * Vref - 2.5;
  return voltage;
}

void TimerCallBack()
{
  /*レンジの設定値が最大出力電圧となる*/
  const double voltage2ustrain_x = 500./5.0;
  const double voltage2ustrain_z = 1000./5.0;
  const double ustrain2force_x = 0.05304;
  const double ustrain2force_z = 0.05858;

  const size_t MAX_SIZE = 50;

  int32_t data;
  AD717X_ReadData(pdevice, &data);
  double voltage = get_voltage(data);

  int32_t ch = -1;
  AD717X_ReadRegister(pdevice, AD717X_STATUS_REG); 
  ch = (AD717X_GetReg(pdevice, AD717X_STATUS_REG)->value) & (uint32_t)0b11;

  if(ch == 0)
    v0.push_back(voltage);
  else if(ch == 1)
    v1.push_back(voltage);
  lim_size(v0, MAX_SIZE);
  lim_size(v1, MAX_SIZE);
  
  double cutoff = 0.1; // ハイパスフィルタのカットオフ値
  double threshold = 1; // 外れ値検出の閾値

  auto filteredData0 = highPassFilter(v0, cutoff);
  auto isOutlier0 = detectOutliers(filteredData0, threshold);
  auto cleanedData0 = removeOutliers(v0, filteredData0, isOutlier0, cutoff);

  auto filteredData1 = highPassFilter(v1, cutoff);
  auto isOutlier1 = detectOutliers(filteredData1, threshold);
  auto cleanedData1 = removeOutliers(v1, filteredData1, isOutlier1, cutoff);
  
  if(cleanedData0.size() > 0)
    Fx = cleanedData0.back() * voltage2ustrain_x * ustrain2force_x;
  if(cleanedData1.size() > 0)
    Fz = cleanedData1.back() * voltage2ustrain_z * ustrain2force_z;

  std::stringstream ss;
  ss.str("");
  //ss << "t: " << millis() << ", ";
  ss << "x: " << Fx;
  ss << ", z: " << Fz;
  Serial.println(ss.str().c_str());
/*
  ss.str("");
  ss << "元のデータ: ";
  for (auto d : v0) ss << d << " ";
  ss << "\nハイパスフィルタ後: ";
  for (auto d : filteredData) ss << d << " ";
  ss << "\nisOutlier: ";
  for (auto d : isOutlier) ss << d << " ";
  ss << "\n外れ値除去後のデータ: ";
  for (auto d : cleanedData) ss << d << " ";
  ss << std::endl;
  Serial.println(ss.str().c_str());*/
}
