#include <Wire.h>

#define MPU9250_ADDRESS_MAIN 0x68
#define MPU9250_ADDRESS_MAG  0x0C

#define MPU9250_REG_WHO_AM_I         0x75
#define MPU9250_REG_WHO_AM_I_DEFAULT 0x71

#define MPU9250_REG_PWR_MGMT_1       0x6B
#define MPU9250_REG_CONFIG           0x1A
#define MPU9250_REG_GYRO_CONFIG      0x1B

#define MPU8250_REG_GYRO_XOUT_H      0x43

enum GyroMode {
  GyroMode_250dps = 0,
  GyroMode_500dps,
  GyroMode_1000dps,
  GyroMode_2000dps,
};

/* i2c functionalities */
uint8_t i2c_read_reg(uint8_t address, uint8_t reg);
void i2c_write_reg(uint8_t address, uint8_t reg, uint8_t value);
void i2c_read_range(uint8_t address, uint8_t reg_base, uint8_t count, uint8_t *dest);

void verify_device()
{
  byte who = i2c_read_reg(MPU9250_ADDRESS_MAIN, MPU9250_REG_WHO_AM_I);

  if (MPU9250_REG_WHO_AM_I_DEFAULT == who) {
    Serial.println("Device verified (MPU-9250)");
  } else {
    Serial.print("Wrong device code: ");
    Serial.println(who, HEX);

    while (true)
      ; // hang
  }
}

/* state global variables, should be turned into a class instance */
GyroMode gyro_mode = GyroMode_1000dps;

/* stolen from some open source project :-) (beerware, remember to buy author a beer) 
 * https://github.com/kriswiner/MPU-9250/blob/master/MPU9250BasicAHRS.ino */
const float gyro_coefficients[4] = {
  250.0/32768.0,
  500.0/32768.0,
  1000.0/32768.0,
  2000.0/32768.0
};

void setup()
{
  Serial.begin(115200);
  Serial.println("Startup sequence...");
  delay(100);

  Serial.println("Starting I2C port...");
  Wire.begin();
  delay(100);

  // 1. verification
  verify_device();

  // 2. wake-up device, auto-select clock source
  i2c_write_reg(MPU9250_ADDRESS_MAIN, MPU9250_REG_PWR_MGMT_1, 1);

  // 3. set configuration register to known value
  i2c_write_reg(MPU9250_ADDRESS_MAIN, MPU9250_REG_CONFIG, 0);

  // 4. configure gyro
  i2c_write_reg(MPU9250_ADDRESS_MAIN, MPU9250_REG_GYRO_CONFIG, gyro_mode << 3);

  Serial.println("Gyro configured...");
  delay(100);

  Serial.println("Starting measurements...");
}

void loop()
{
  // fetch gyro data
  uint8_t rawData[6];
  i2c_read_range(MPU9250_ADDRESS_MAIN, MPU8250_REG_GYRO_XOUT_H, 6, &rawData[0]);
  int16_t gyro_x = ((int16_t)rawData[0] << 8) | rawData[1];
  int16_t gyro_y = ((int16_t)rawData[2] << 8) | rawData[3];
  int16_t gyro_z = ((int16_t)rawData[4] << 8) | rawData[5];

  Serial.print(gyro_x * gyro_coefficients[gyro_mode]);
  Serial.print(" ");
  Serial.print(gyro_y * gyro_coefficients[gyro_mode]);
  Serial.print(" ");
  Serial.println(gyro_z * gyro_coefficients[gyro_mode]);

  delay(100);
}

uint8_t i2c_read_reg(uint8_t address, uint8_t reg)
{
  uint8_t data;
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(address, (uint8_t)1);
  data = Wire.read();
  return data;
}

void i2c_write_reg(uint8_t address, uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void i2c_read_range(uint8_t address, uint8_t reg_base, uint8_t count, uint8_t *dest)
{  
  Wire.beginTransmission(address);
  Wire.write(reg_base);
  Wire.endTransmission(false);
  uint8_t i = 0;
  Wire.requestFrom(address, count);

  while (Wire.available())
    dest[i++] = Wire.read();
}

