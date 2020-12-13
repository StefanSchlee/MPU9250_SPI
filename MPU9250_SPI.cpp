/*
  library for the MPU9250, reading out Accelerometer, Gyroscope and the Magnetometer via SPI
  Magnetometer is read out via the auxiliary I2C bus
  Created by Stefan Schlee, 2020.
*/

#include "Arduino.h"
#include "MPU9250_SPI.h"

MPU9250::MPU9250(uint8_t SS_Pin, SPIClass &Spi, SPISettings SpiSettings, MPU9250_Offsets_t Offsets) : Spi(Spi)
{
  this->SS_Pin = SS_Pin;
  this->SpiSettings = SpiSettings;
  Mpu_offsets = Offsets;

  pinMode(SS_Pin, OUTPUT);
  digitalWrite(SS_Pin, HIGH);
}


/*
Activates the Sensors and initializes the control registers
Sensitivity given by enumerations
DLPF: 0-6 (Datasheet for Bandwith)
*/
void MPU9250::init(Gyro_Sensitivity_t Gyro_sens, uint8_t DLPF_Gyro, Acc_Sensitivity_t Acc_sens, uint8_t DLPF_Acc)
{
  //save Sensitivity Settings
  Gyro_Sens = Gyro_sens;
  Acc_Sens = Acc_sens;

  uint8_t buffer[4]; //read write buffer

  delay(20); //wait for MPU

  Spi.beginTransaction(SpiSettings);

  ///Set SPI only mode, enable I2C Master module
  digitalWrite(SS_Pin, LOW);
  buffer[0] = 106;                 //register number + write
  buffer[1] = (1 << 4) | (1 << 5); //SPI only bit + EN Master
  Spi.transfer(buffer, 2);
  digitalWrite(SS_Pin, HIGH);
  delayMicroseconds(5);

  //Gyro sens
  digitalWrite(SS_Pin, LOW);
  buffer[0] = 27;               //register number + write
  buffer[1] = (Gyro_sens << 3); //
  Spi.transfer(buffer, 2);
  digitalWrite(SS_Pin, HIGH);
  delayMicroseconds(5);

  //Gyro DLPF
  digitalWrite(SS_Pin, LOW);
  buffer[0] = 26; //register number + write
  buffer[1] = DLPF_Gyro;
  Spi.transfer(buffer, 2);
  digitalWrite(SS_Pin, HIGH);
  delayMicroseconds(5);

  //Acc sensitivity
  digitalWrite(SS_Pin, LOW);
  buffer[0] = 28;              //register number + write
  buffer[1] = (Acc_sens << 3); //
  Spi.transfer(buffer, 2);
  digitalWrite(SS_Pin, HIGH);
  delayMicroseconds(5);

  //Acc DLPF
  digitalWrite(SS_Pin, LOW);
  buffer[0] = 29; //register number + write
  buffer[1] = DLPF_Acc;
  Spi.transfer(buffer, 2);
  digitalWrite(SS_Pin, HIGH);
  delayMicroseconds(5);

  ///////// Init Magnetometer //////////////
  //read Ajustment Values
  write_Magnetometer(0x0A, 15); //Fuse access mode

  //register address
  digitalWrite(SS_Pin, LOW);
  buffer[0] = 38;   //register number + write
  buffer[1] = 0x10; //start of Ajustment Registers
  Spi.transfer(buffer, 2);
  digitalWrite(SS_Pin, HIGH);
  delayMicroseconds(5);

  //enable I2C Master + lenght of read data
  digitalWrite(SS_Pin, LOW);
  buffer[0] = 39;           //register number + write
  buffer[1] = 3 | (1 << 7); //EN bit + 3bytes to read
  Spi.transfer(buffer, 2);
  digitalWrite(SS_Pin, HIGH);
  delayMicroseconds(5);

  delay(10); //wait for mpu to get data

  //get Adjustment values
  digitalWrite(SS_Pin, LOW);
  buffer[0] = 73 |(1 << 7);   //register number + read
  Spi.transfer(buffer, 4);
  digitalWrite(SS_Pin, HIGH);
  delayMicroseconds(5);

  ASAX = buffer[1];
  ASAY = buffer[2];
  ASAZ = buffer[3];

  write_Magnetometer(0x0A, 0); //back to power down

  delay(10); 

  write_Magnetometer(0x0A, 22); //ctrl1, Continous 2 + 16Bit Ausgabe

  //register address
  digitalWrite(SS_Pin, LOW);
  buffer[0] = 38;   //register number + write
  buffer[1] = 0x03; //start of Measurement Registers
  Spi.transfer(buffer, 2);
  digitalWrite(SS_Pin, HIGH);
  delayMicroseconds(5);

  //enable I2C Master + lenght of read data
  digitalWrite(SS_Pin, LOW);
  buffer[0] = 39;           //register number + write
  buffer[1] = 7 | (1 << 7); //EN bit + 7bytes to read
  Spi.transfer(buffer, 2);
  digitalWrite(SS_Pin, HIGH);
  delayMicroseconds(5);

  Spi.endTransaction();
}

void MPU9250::write_Magnetometer(uint8_t Register, uint8_t Data)
{
  uint8_t buffer[2]; //write buffer
  uint8_t oldRegister;
  uint8_t oldMasterStatus;

  //read current Master Status
  digitalWrite(SS_Pin, LOW);
  buffer[0] = 39 | (1 << 7); //register number + read
  Spi.transfer(buffer, 2);
  digitalWrite(SS_Pin, HIGH);
  delayMicroseconds(5);
  oldMasterStatus = buffer[1];

  //read current read Register
  digitalWrite(SS_Pin, LOW);
  buffer[0] = 38 | (1 << 7); //register number + read
  Spi.transfer(buffer, 2);
  digitalWrite(SS_Pin, HIGH);
  delayMicroseconds(5);
  oldRegister = buffer[1];

  //disable I2C Master
  digitalWrite(SS_Pin, LOW);
  buffer[0] = 39; //register number + write
  buffer[1] = 0;  //EN bit
  Spi.transfer(buffer, 2);
  digitalWrite(SS_Pin, HIGH);
  delayMicroseconds(5);

  //Device adress + write
  digitalWrite(SS_Pin, LOW);
  buffer[0] = 37;         //register number + write
  buffer[1] = MAGaddress; //mag address + write
  Spi.transfer(buffer, 2);
  digitalWrite(SS_Pin, HIGH);
  delayMicroseconds(5);

  //register address
  digitalWrite(SS_Pin, LOW);
  buffer[0] = 38; //register number + write
  buffer[1] = Register;
  Spi.transfer(buffer, 2);
  digitalWrite(SS_Pin, HIGH);
  delayMicroseconds(5);

  //data out
  digitalWrite(SS_Pin, LOW);
  buffer[0] = 99; //register number + write
  buffer[1] = Data;
  Spi.transfer(buffer, 2);
  digitalWrite(SS_Pin, HIGH);
  delayMicroseconds(5);

  //enable I2C Master
  digitalWrite(SS_Pin, LOW);
  buffer[0] = 39;           //register number + write
  buffer[1] = 1 | (1 << 7); //EN bit + 1 Byte to write
  Spi.transfer(buffer, 2);
  digitalWrite(SS_Pin, HIGH);
  delayMicroseconds(5);

  delay(10); //wait for data to be written

  //disable I2C Master
  digitalWrite(SS_Pin, LOW);
  buffer[0] = 39; //register number + write
  buffer[1] = 0;  //EN bit
  Spi.transfer(buffer, 2);
  digitalWrite(SS_Pin, HIGH);
  delayMicroseconds(5);

  //register address
  digitalWrite(SS_Pin, LOW);
  buffer[0] = 38; //register number + write
  buffer[1] = oldRegister;
  Spi.transfer(buffer, 2);
  digitalWrite(SS_Pin, HIGH);
  delayMicroseconds(5);

  //Device adress + read
  digitalWrite(SS_Pin, LOW);
  buffer[0] = 37;                    //register number + write
  buffer[1] = MAGaddress | (1 << 7); //mag address + read
  Spi.transfer(buffer, 2);
  digitalWrite(SS_Pin, HIGH);
  delayMicroseconds(5);

  //Restore Master Status
  digitalWrite(SS_Pin, LOW);
  buffer[0] = 39;              //register number + write
  buffer[1] = oldMasterStatus; //EN bit + 1 Byte to write
  Spi.transfer(buffer, 2);
  digitalWrite(SS_Pin, HIGH);
  delayMicroseconds(5);
}


MPU9250_Data_t MPU9250::get_Data()
{
  uint8_t buffer[21]; //read buffer
  int16_t Acc_rawX, Acc_rawY, Acc_rawZ, Gyr_rawX, Gyr_rawY, Gyr_rawZ;
  int16_t Mag_rawX, Mag_rawY, Mag_rawZ;
  MPU9250_Data_t newData;

  //read raw data
  Spi.beginTransaction(SpiSettings);

  buffer[0] = 59 | (1 << 7); //register +  read
  digitalWrite(SS_Pin, LOW);
  Spi.transfer(buffer, 21);
  digitalWrite(SS_Pin, HIGH);

  //convert raw data Zeit:0
  Acc_rawX = (buffer[1] << 8) | buffer[2];
  Acc_rawY = (buffer[3] << 8) | buffer[4];
  Acc_rawZ = (buffer[5] << 8) | buffer[6];

  Gyr_rawX = (buffer[9] << 8) | buffer[10];
  Gyr_rawY = (buffer[11] << 8) | buffer[12];
  Gyr_rawZ = (buffer[13] << 8) | buffer[14];

  Mag_rawX = (buffer[16] << 8) | buffer[15];
  Mag_rawY = (buffer[18] << 8) | buffer[17];
  Mag_rawZ = (buffer[20] << 8) | buffer[19];

  //convert to float Zeit:5µs
  newData.Acc_X = (float)Acc_rawX / Acc_LSB[Acc_Sens] - Mpu_offsets.Acc_X_off;
  newData.Acc_Y = (float)Acc_rawY / Acc_LSB[Acc_Sens] - Mpu_offsets.Acc_Y_off;
  newData.Acc_Z = (float)Acc_rawZ / Acc_LSB[Acc_Sens] - Mpu_offsets.Acc_Z_off;

  newData.Gyro_X = (float)Gyr_rawX / Gyro_LSB[Gyro_Sens] - Mpu_offsets.Gyro_X_off;
  newData.Gyro_Y = (float)Gyr_rawY / Gyro_LSB[Gyro_Sens] - Mpu_offsets.Gyro_Y_off;
  newData.Gyro_Z = (float)Gyr_rawZ / Gyro_LSB[Gyro_Sens] - Mpu_offsets.Gyro_Z_off;

  Mag_rawX = Mag_rawX * (((ASAX-128) >> 8) + 1);
  Mag_rawY = Mag_rawY * (((ASAY-128) >> 8) + 1);
  Mag_rawZ = Mag_rawZ * (((ASAZ-128) >> 8) + 1);


  newData.Mag_X = (float)Mag_rawY - Mpu_offsets.Mag_X_off;
  newData.Mag_Y = (float)Mag_rawX - Mpu_offsets.Mag_Y_off;
  newData.Mag_Z = (float)-Mag_rawZ - Mpu_offsets.Mag_Z_off;


  Spi.endTransaction();


  return newData;
}