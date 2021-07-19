// Accel parts taken from: https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/
// Send to Python parts taken from: https://thepoorengineer.com/en/arduino-python-plot/#arduino

unsigned long timer = 0;
long loopTime = 50;   // microseconds

#include<Wire.h>
const int MPU_addr=0x68;  // I2C address of the MPU-6050
double AcX,AcY,AcZ,GyX,GyY,GyZ,accAngleX,accAngleY,gyroAngleX,gyroAngleY;
double AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
double roll, pitch, yaw;
char userInput;
float g = 9.81;
int c = 0;

void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0x00);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(115200);
  //calculate_IMU_error();
  timer = micros();
}

void loop() {
  timeSync(loopTime);
  //int val = analogRead(0) - 512;
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,6,true);  // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g we divide raw values by 16384 according to datasheet
  AcX = (Wire.read()<<8|Wire.read()) * (g/16384.0) ;  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)   
  AcY = (Wire.read()<<8|Wire.read()) * (g/16384.0) ; // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = (Wire.read()<<8|Wire.read()) * (g/16384.0) ; // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AcY / sqrt(pow(AcX, 2) + pow(AcZ, 2))) * 180 / PI) - 0.58; // AcErrorX ~(0.58) See the calculate_IMU_error() custom function for more details
  accAngleY = (atan(-1 * AcX / sqrt(pow(AcY, 2) + pow(AcZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)

  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000;
  previousTime = currentTime;
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  GyX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  
  // Correct the outputs with the calculated error values - IMPLEMENT THIS CALCULATION?
  GyX = GyX + 0.56; // GyroErrorX ~(-0.56)
  GyY = GyY - 2; // GyroErrorY ~(2)
  GyZ = GyZ + 0.79; // GyroErrorZ ~ (-0.8)

  gyroAngleX = gyroAngleX + GyX * elapsedTime; // deg/s * s = deg .   - THIS + gyroAngleX ?? Why ?? 
  gyroAngleY = gyroAngleY + GyY * elapsedTime;

  // Complementary filter - combine acceleromter and gyro angle values
  gyroAngleX = 0.96 * gyroAngleX + 0.04 * accAngleX;
  gyroAngleY = 0.96 * gyroAngleY + 0.04 * accAngleY;

  roll = gyroAngleX;
  pitch = gyroAngleY;

  // This one is a pain in the arse and still drifts
  //yaw =  yaw + GyZ * elapsedTime;
  
  sendToPC(&AcX, &AcY, &AcZ, &roll, &pitch, &yaw);
  //sendToPC(&AcX, &AcY, &AcZ, &accAngleX, &accAngleY, &yaw);
  //sendToPC(&AcX, &AcY, &AcZ, &GyX, &GyY, &yaw);
}

void timeSync(unsigned long deltaT)
{
  unsigned long currTime = micros();
  long timeToDelay = deltaT - (currTime - timer);
  if (timeToDelay > loopTime) // or 5000
  {
    delay(timeToDelay / 1000);
    delayMicroseconds(timeToDelay % 1000);
  }
  else if (timeToDelay > 0)
  {
    delayMicroseconds(timeToDelay);
  }
  else
  {
      // timeToDelay is negative so we start immediately
  }
  timer = currTime + timeToDelay;
}

void sendToPC(int* data1, int* data2, int* data3, int* data4, int* data5, int* data6)
{
  byte* byteData1 = (byte*)(data1);
  byte* byteData2 = (byte*)(data2);
  byte* byteData3 = (byte*)(data3);
  byte* byteData4 = (byte*)(data4);
  byte* byteData5 = (byte*)(data5);
  byte* byteData6 = (byte*)(data6);
  byte buf[12] = {byteData1[0], byteData1[1],
                 byteData2[0], byteData2[1],
                 byteData3[0], byteData3[1],
                 byteData4[0], byteData4[1],
                 byteData5[0], byteData5[1],
                 byteData6[0], byteData6[1]};
  Serial.write(buf, 12);
}

void sendToPC(double* data1, double* data2, double* data3, double* data4, double* data5, double* data6)
{
  byte* byteData1 = (byte*)(data1);
  byte* byteData2 = (byte*)(data2);
  byte* byteData3 = (byte*)(data3);
  byte* byteData4 = (byte*)(data4);
  byte* byteData5 = (byte*)(data5);
  byte* byteData6 = (byte*)(data6);
  byte buf[24] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                 byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                 byteData3[0], byteData3[1], byteData3[2], byteData3[3],
                 byteData4[0], byteData4[1], byteData4[2], byteData4[3],
                 byteData5[0], byteData5[1], byteData5[2], byteData5[3],
                 byteData6[0], byteData6[1], byteData6[2], byteData6[3]};
  Serial.write(buf, 24);
}


void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 6, true);
    AcX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AcY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AcZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AcY) / sqrt(pow((AcX), 2) + pow((AcZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AcX) / sqrt(pow((AcY), 2) + pow((AcZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 6, true);
    GyX = Wire.read() << 8 | Wire.read();
    GyY = Wire.read() << 8 | Wire.read();
    GyZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyX / 131.0);
    GyroErrorY = GyroErrorY + (GyY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}
