#include <Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

const char* ssid     = "...";
const char* password = "...";

#define MPU9250_ADDRESS_MAIN 0x68
#define MPU9250_ADDRESS_MAG  0x0C

#define MPU9250_REG_WHO_AM_I         0x75
#define MPU9250_REG_WHO_AM_I_DEFAULT 0x71

#define MPU9250_REG_PWR_MGMT_1       0x6B
#define MPU9250_REG_PWR_MGMT_2       0x6C
#define MPU9250_REG_CONFIG           0x1A
#define MPU9250_REG_GYRO_CONFIG      0x1B

#define MPU9250_REG_GYRO_XOUT_H      0x43
#define MPU9250_REG_FIFO_EN          0x23
#define MPU9250_REG_USER_CTRL        0x6A
#define MPU9250_REG_SMPLRT_DIV       0x19

#define MPU9250_REG_FIFO_COUNTH      0x72
#define MPU9250_REG_FIFO_R_W         0x74

#define MPU9250_REG_GYRO_OFFS_USR    0x13

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
int16_t i2c_read_int16(uint8_t address, uint8_t reg);

/* state global variables, should be turned into a class instance */
GyroMode gyro_mode = GyroMode_250dps;

/* stolen from some open source project :-) (beerware, remember to buy author a beer) 
 * https://github.com/kriswiner/MPU-9250/blob/master/MPU9250BasicAHRS.ino */
const float gyro_coefficients[4] = {
  250.0/32768.0,
  500.0/32768.0,
  1000.0/32768.0,
  2000.0/32768.0
};

void mpu9250_reset()
{
  i2c_write_reg(MPU9250_ADDRESS_MAIN, MPU9250_REG_PWR_MGMT_1, 0x80);
  delay(100);
}

void mpu9250_verify()
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

const float KILO_DIVIDER = 12;

void mpu9250_setup()
{
  i2c_write_reg(MPU9250_ADDRESS_MAIN, MPU9250_REG_PWR_MGMT_1, 1);
  i2c_write_reg(MPU9250_ADDRESS_MAIN, MPU9250_REG_PWR_MGMT_2, 7 << 3); // enable only gyro
  i2c_write_reg(MPU9250_ADDRESS_MAIN, MPU9250_REG_CONFIG, 3);
  i2c_write_reg(MPU9250_ADDRESS_MAIN, MPU9250_REG_GYRO_CONFIG, gyro_mode << 3);
  i2c_write_reg(MPU9250_ADDRESS_MAIN, MPU9250_REG_SMPLRT_DIV, KILO_DIVIDER);
  i2c_write_reg(MPU9250_ADDRESS_MAIN, MPU9250_REG_FIFO_EN, 0x70); // enable FIFO for gyro X, Y, Z
  i2c_write_reg(MPU9250_ADDRESS_MAIN, MPU9250_REG_USER_CTRL, 0x44); // reset FIFO and enable FIFO
}

void mpu9250_calibrate()
{
  mpu9250_reset();
  i2c_write_reg(MPU9250_ADDRESS_MAIN, MPU9250_REG_PWR_MGMT_1, 1);
  i2c_write_reg(MPU9250_ADDRESS_MAIN, MPU9250_REG_PWR_MGMT_2, 7 << 3); // disable accelerometer
  i2c_write_reg(MPU9250_ADDRESS_MAIN, MPU9250_REG_CONFIG, 1); // Bandwidth 184Hz, delay 2.9ms, Fs 1Khz
  i2c_write_reg(MPU9250_ADDRESS_MAIN, MPU9250_REG_SMPLRT_DIV, 0);
  i2c_write_reg(MPU9250_ADDRESS_MAIN, MPU9250_REG_GYRO_CONFIG, GyroMode_250dps << 3);

  int32_t last_avarage[3] = {0};

  for (int k = 0; k < 10; k ++)
  {
    i2c_write_reg(MPU9250_ADDRESS_MAIN, MPU9250_REG_FIFO_EN, 0x70); // enable FIFO for gyro X, Y, Z
    i2c_write_reg(MPU9250_ADDRESS_MAIN, MPU9250_REG_USER_CTRL, 0x44);
    delay(100); // collect samples  
    i2c_write_reg(MPU9250_ADDRESS_MAIN, MPU9250_REG_FIFO_EN, 0); // disable FIFO
  
    int32_t fifo_size = i2c_read_int16(MPU9250_ADDRESS_MAIN, MPU9250_REG_FIFO_COUNTH)/6; // read FIFO sample count
  
    Serial.print("calibration: samples from FIFO: "); Serial.println(fifo_size, DEC);
  
    uint8_t data[6];
    int32_t avarage[3] = {0};
    
    for (int i = 0; i < fifo_size; i ++)
    {
      i2c_read_range(MPU9250_ADDRESS_MAIN, MPU9250_REG_FIFO_R_W, 6, data);
  
      avarage[0] += (int16_t)(((int16_t)data[0] << 8) | data[1] );
      avarage[1] += (int16_t)(((int16_t)data[2] << 8) | data[3] );
      avarage[2] += (int16_t)(((int16_t)data[4] << 8) | data[5] );
    }
 
    avarage[0] = (avarage[0]/fifo_size/4.f);
    avarage[1] = (avarage[1]/fifo_size/4.f);
    avarage[2] = (avarage[2]/fifo_size/4.f);

    Serial.print("Avarages         : ");
    for (int a = 0; a < 3; a ++) {
      Serial.print(avarage[a]);
      Serial.print(", ");
    }
    Serial.println();

    avarage[0] += last_avarage[0];
    avarage[1] += last_avarage[1];
    avarage[2] += last_avarage[2];

    Serial.print("Corrected Avarages: ");
    for (int a = 0; a < 3; a ++) {
      Serial.print(avarage[a]);
      Serial.print(", ");
    }

    Serial.println();

    last_avarage[0] = avarage[0];
    last_avarage[1] = avarage[1];
    last_avarage[2] = avarage[2];
  
    /* I have no idea why this order.. - to be checked with spec */
    data[0] = (-avarage[1] >> 8) & 0xff;
    data[1] = (-avarage[1]     ) & 0xff;
    data[2] = (-avarage[2] >> 8) & 0xff;
    data[3] = (-avarage[2]     ) & 0xff;
    data[4] = (-avarage[0] >> 8) & 0xff;
    data[5] = (-avarage[0]     ) & 0xff;
  
    /* apply offsets */
    for (int i = 0; i < 6; i ++)
    {
      i2c_write_reg(MPU9250_ADDRESS_MAIN, MPU9250_REG_GYRO_OFFS_USR + i, data[i]);
    }
  }
}

void wifi_connect()
{
  Serial.print("Connecting to SSID: ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
 
  Serial.println("Got IP: ");
  Serial.println(WiFi.localIP());
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Startup sequence...");
  delay(100);

  Serial.println("Starting I2C port...");
  Wire.begin();
  delay(500);
  mpu9250_reset();
  delay(500);

  Serial.println("Checking device ID...");
  mpu9250_verify();

  Serial.println("Calibrating device...");
  mpu9250_calibrate();

  Serial.println("Setting up device...");
  mpu9250_setup();

  wifi_connect();

  delay(100);

  Serial.println("Starting measurements...");
}

enum {
  Pitch = 0,
  Yaw,
  Roll
};

void loop()
{
  static float gyro[3] = {0.f};

  int32_t fifo_size = i2c_read_int16(MPU9250_ADDRESS_MAIN, MPU9250_REG_FIFO_COUNTH)/6; // read FIFO sample count

  Serial.print("FIFO: "); Serial.println(fifo_size);

  for (int i = 0; i < fifo_size; i ++)
  {
    uint8_t data[6];
    i2c_read_range(MPU9250_ADDRESS_MAIN, MPU9250_REG_FIFO_R_W, 6, data);

    float gx = (int16_t)(((int16_t)data[0] << 8) | data[1]) * gyro_coefficients[gyro_mode];
    float gy = (int16_t)(((int16_t)data[2] << 8) | data[3]) * gyro_coefficients[gyro_mode];
    float gz = (int16_t)(((int16_t)data[4] << 8) | data[5]) * gyro_coefficients[gyro_mode];

    gyro[0] += gx / (1000.f / KILO_DIVIDER);
    gyro[1] += gy / (1000.f / KILO_DIVIDER);
    gyro[2] += gz / (1000.f / KILO_DIVIDER);
  }

  // send data
#pragma pack(push, 1)
  struct {
        uint8_t pad1;
        uint8_t flags;
        float fl[12];
    } data;
#pragma pack(pop)

  data.flags = 3;

  // translate axis, just because. :-)
  data.fl[0 + Roll] = data.fl[9 + Roll] = gyro[Yaw] * 0.0174532925;
  data.fl[0 + Pitch] = data.fl[9 + Pitch] = gyro[Roll]  * 0.0174532925;
  data.fl[0 + Yaw] = data.fl[9 + Yaw] = gyro[Pitch]  * 0.0174532925;

  if (fifo_size > 0)
  {
    WiFiUDP udp;
    udp.beginPacket(IPAddress(192, 168, 0, 102), 5555);
    udp.write((const uint8_t *)&data, sizeof(data));
    udp.endPacket();
  }

  delay(10);
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

int16_t i2c_read_int16(uint8_t address, uint8_t reg)
{
  uint8_t data[2];
  int16_t result;
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(address, (uint8_t)2);
  data[0] = Wire.read();
  data[1] = Wire.read();

  result = ((uint16_t)data[0] << 8) | data[1];
  
  return result;
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

