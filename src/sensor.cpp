#include "sensor.hpp"

float Ax,Ay,Az,Wp,Wq,Wr,dp,dq,dr,Voltage;
INA3221 ina3221(INA3221_ADDR40_GND);

uint8_t init_i2c()
{
  //Wire1.begin(25,21);          // join i2c bus (address optional for master)
  Wire1.begin(25,21,400000UL);
  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;
  for (short i = 0; i < 256; i++)
  {
    Wire1.beginTransmission (i);          // Begin I2C transmission Address (i)
    if (Wire1.endTransmission () == 0)  // Receive 0 = success (ACK response) 
    {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);     // PCF8574 7 bit address
      Serial.println (")");
      count++;
    }
  }
  Serial.print ("Found ");      
  Serial.print (count, DEC);        // numbers of devices
  Serial.println (" device(s).");
  return count;
}

uint8_t mpu6886_byte_read(uint8_t reg_addr)
{
  uint8_t data;
  Wire1.beginTransmission (MPU6886_ADDRESS);
  Wire1.write(reg_addr);
  Wire1.endTransmission();
  Wire1.requestFrom(MPU6886_ADDRESS, 1);
  data = Wire1.read();
  return data;
}

void mpu6886_byte_write(uint8_t reg_addr, uint8_t data)
{
  Wire1.beginTransmission (MPU6886_ADDRESS);
  Wire1.write(reg_addr);
  Wire1.write(data);
  Wire1.endTransmission();
}

void sensor_init(void)
{
  init_i2c();

  //Config imu filter
  //Cutoff frequency
  //filter_config Gyro Accel
  //0 250    218.1 log140　Bad
  //1 176    218.1 log141　Bad
  //2 92     99.0  log142 Bad これはヨーガカクカクする log256
  //3 41     44.8  log143 log188　Good! log257
  //4 20     21.2
  //5 10     10.2
  //6 5      5.1
  //7 3281   420.0
  uint8_t data;
  const uint8_t filter_config = 2;//(今の所2はノイズが多くてダメ、log188は3)

  //Mdgwick filter 実験
  // filter_config=0において実施
  //beta =0 次第に角度増大（角速度の積分のみに相当する）
  //beta=0.5

  M5.IMU.Init();
  //IMUのデフォルトI2C周波数が100kHzなので400kHzに上書き
  Wire1.begin(25,21,400000UL);

 //F_CHOICE_B
  data = mpu6886_byte_read(MPU6886_GYRO_CONFIG);
  Serial.printf("GYRO_CONFIG %d\r\n", data);
  mpu6886_byte_write(MPU6886_GYRO_CONFIG, data & 0b11111100);
  data = mpu6886_byte_read(MPU6886_GYRO_CONFIG);
  Serial.printf("Update GYRO_CONFIG %d\r\n", data);

  //Gyro
  //DLPG_CFG
  data = mpu6886_byte_read(MPU6886_CONFIG);
  Serial.printf("CONFIG %d\r\n", data);
  mpu6886_byte_write(MPU6886_CONFIG, (data&0b11111100)|filter_config);
  data = mpu6886_byte_read(MPU6886_CONFIG);
  Serial.printf("Update CONFIG %d\r\n", data);

  //Accel
  //ACCEL_FCHOCE_B & A_DLPF_CFG
  data = mpu6886_byte_read(MPU6886_ACCEL_CONFIG2);
  Serial.printf("ACCEL_CONFIG2 %d\r\n", data);
  mpu6886_byte_write(MPU6886_ACCEL_CONFIG2, (data & 0b11110111) | filter_config);
  data = mpu6886_byte_read(MPU6886_ACCEL_CONFIG2);
  Serial.printf("Update ACCEL_CONFIG2 %d\r\n", data);

  //Voltage & Current Seonsor
  ina3221.begin(&Wire1);
  ina3221.reset();  

}

void sensor_read(void)
{
  float ax, ay, az, gx, gy, gz, acc_norm, rate_norm;
  float filterd_v;
  static float dp, dq, dr; 

  M5.IMU.getAccelData(&ax, &ay, &az);
  M5.IMU.getGyroData(&gx, &gy, &gz);
  Ax = ay;
  Ay = ax;
  Az = az;
  dp = Wp;
  dq = Wq;
  dr = Wr;
  Wp = gy*(float)DEG_TO_RAD;
  Wq = gx*(float)DEG_TO_RAD;
  Wr =-gz*(float)DEG_TO_RAD;
  if(Wp>8.0||Wp<-8.0)Wp = dp;
  if(Wq>8.0||Wq<-8.0)Wq = dq;
  if(Wr>8.0||Wr<-8.0)Wr = dr;

  #if 1
  Voltage = ina3221.getVoltage(INA3221_CH2);
  #endif

}