// получаем углы при помощи DMP
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include "Wire.h"
MPU6050 mpu;
float angleX = 180;
float angleY = 180;
float angleZ = 180;
uint32_t timer;

String DMP_X = "DMP_X: ";
String DMP_Y = " DMP_Y: ";
String DMP_Z = " DMP_Z: ";
String X = "NULL";
String Y = "NULL";
String Z = "NULL";

int Arduino_addr = 0x04;
String DMP_end = "  END";
String Get_R_Pi;

void setup() {
  Wire.begin(Arduino_addr); // Указание адреса Arduino Uno R3
  Wire.onReceive(read_R_Pi); //Регистрация события чтения данных Raspberry Pi
  Serial.begin(19200);
  mpu.initialize();
  initDMP();
}

void loop() {
  getAngles();
  X = DMP_X + String(angleX);
  Y = DMP_Y + String(angleY);
  Z = DMP_Z + String(angleZ);
  // timer = micros();
  //  Serial.print("timer: ");
  //  Serial.print(timer);
  //  Serial.print(" ");
  // Serial.print(DMP_X + angleX);
  // Serial.print(DMP_Y + angleY);
  // Serial.println(DMP_Z + angleZ + DMP_end);
}


// НУЖНЫЕ ПЕРЕМЕННЫЕ
const float toDeg = 180.0 / M_PI;
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// инициализация
void initDMP() {
  devStatus = mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  mpuIntStatus = mpu.getIntStatus();
  packetSize = mpu.dmpGetFIFOPacketSize();
}


// получение углов в angleX, angleY, angleZ
void getAngles() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    angleX = ypr[2] * toDeg + 180;
    angleY = ypr[1] * toDeg + 180;
    angleZ = ypr[0] * toDeg + 180;
  }
}


//Передча данных Raspberry Pi
void read_R_Pi() {
  Get_R_Pi = Wire.read();
  if (Get_R_Pi == 'X') {
    Wire.write("DMP_X: " + byte(angleX));
  }
  if (Get_R_Pi == 'Y') {
    Wire.write("DMP_Y: " + byte(angleY));
  }
    if (Get_R_Pi == 'Z') {
    Wire.write("DMP_Z: " + byte(angleZ));
  }
  Wire.write(" END ");
}
