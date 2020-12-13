/*
  library for the MPU9250, reading out Accelerometer, Gyroscope and the Magnetometer via SPI
  Magnetometer is read out via the auxiliary I2C bus
  Created by Stefan Schlee, 2020.
*/

#ifndef MPU9250_SPI_h
#define MPU9250_SPI_h

#include "Arduino.h"
#include <SPI.h>

//Gyroscope Sensitivity in Degrees per Second +-
typedef enum
{
  GYRO_SENS_250,
  GYRO_SENS_500,
  GYRO_SENS_1000,
  GYRO_SENS_2000
} Gyro_Sensitivity_t;

//Accerlerometer Sensitivity in g +-
typedef enum
{
  ACC_SENS_2,
  ACC_SENS_4,
  ACC_SENS_8,
  ACC_SENS_16,
} Acc_Sensitivity_t;

/*
Accelerometer in g
Gyroscope in Degrees per Second
Magnetometer unscaled (not important for direction)
Magnetometer Output in Gyroscope Koordinatensystem
*/
typedef struct{
  float Acc_X, Acc_Y, Acc_Z;
  float Gyro_X, Gyro_Y, Gyro_Z;
  float Mag_X, Mag_Y, Mag_Z;

} MPU9250_Data_t;

/*
  Wird von Data abgezogen
  Acc, Gyr, Mag; jeweils x y z
*/
typedef struct{
  float Acc_X_off, Acc_Y_off, Acc_Z_off;
  float Gyro_X_off, Gyro_Y_off, Gyro_Z_off;
  float Mag_X_off, Mag_Y_off, Mag_Z_off;
} MPU9250_Offsets_t;

/*
MPU9250 Sensor Class
*/
class MPU9250
{
public:
  MPU9250(uint8_t SS_Pin, SPIClass &Spi, SPISettings SpiSettings, MPU9250_Offsets_t Offsets);

  void init(Gyro_Sensitivity_t Gyro_sens,uint8_t DLPF_Gyro, Acc_Sensitivity_t Acc_sens, uint8_t DLPF_Acc);
  MPU9250_Data_t get_Data();

private:
  uint8_t SS_Pin;
  SPIClass &Spi;
  SPISettings SpiSettings;

  Gyro_Sensitivity_t Gyro_Sens;
  Acc_Sensitivity_t Acc_Sens;
  MPU9250_Offsets_t Mpu_offsets;

  void write_Magnetometer(uint8_t Register, uint8_t Data);

  //Gyro_Sensitivity_t to LSB
  const float Gyro_LSB[4] = {
      131.0, 65.5, 32.8, 16.4};

  //Acc_Sensitivity_t to LSB
  const float Acc_LSB[4] = {
      16384.0, 8192.0, 4096.0, 2048.0};

  //I2C address of Magnetometer
  const uint8_t MAGaddress = 0x0C;

  //Magnetometer Sensitivity Ajustment values
  uint8_t ASAX, ASAY, ASAZ;
};

#endif
