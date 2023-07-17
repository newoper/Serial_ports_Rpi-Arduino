#include <Wire.h>
#include "Kalman.h"

Kalman kalmanX;
Kalman kalmanY;

uint8_t IMUAddress = 0x68;
uint8_t nbytes = 14;
uint8_t n_MPU;
uint8_t n2_MPU;

uint8_t AFS_SELa = 0; //диапазон шкалы акселерометра
uint8_t AFS_SELg = 0; //диапазон шкалы гироскопа

uint8_t F_x = 0;
uint8_t F_y = 0;

/*
  Настройка чувствительности акселерометра - полный диапазон шкалы (по умолчанию +/- 2g)
  Масштабирование показания датчиков в зависимости от настройки чувствительности:

  |AFS_SEL|диапазон шаклы|масштабирование на|перевод в СИ   |||||||||
  Акселерометр
  |   0   |     +-2g     |     16384        |    aX/32768*2     |||||||||
  |   1   |     +-4g     |     8192         |    aX/32768*4     |||||||||
  |   2   |     +-6g     |     4096         |    aX/32768*6     |||||||||
  |   3   |     +-8g     |     2048         |    aX/32768*8     |||||||||
  Температура
  |   0   |от -40 до 85 С|     340          |    t/340          |||||||||
  Гироскоп
  |   0   |     +-250    |     131          |    gX/32768*250   |||||||||
  |   1   |     +-500    |     65.5         |    gX/32768*500   |||||||||
  |   2   |     +-1000   |     32.8         |    gX/32768*1000  |||||||||
  |   3   |     +-2000   |     16.4         |    gX/32768*2000  |||||||||

*/


//указание переменных для настройки диапазона гироскопа и акселерометра
uint8_t AFS_CONFa = 0x10;
uint8_t AFS_CONFg = 0x10;
uint8_t coeffa = 0;
uint16_t coeffg = 0;

int16_t aX;
int16_t aY;
int16_t aZ;

uint16_t AFS_SELt = 340;
int16_t tR;

int16_t gX;
int16_t gY;
int16_t gZ;

double gyroXangle = 0; // Angle calculate using the gyro
double gyroYangle = 0;

double kalAngleX; // Calculate the angle using a Kalman filter
double kalAngleY;

String str_Kal_X = "aX_KAL:";
String str_Kal_Y = " aY_KAL:";

uint32_t timer;
volatile boolean flag = false;
volatile boolean MPU_write_flag;

#define Arduino_addr 0x48


void setup() {
  Serial.begin(9600); //настроить последовательный порт для вывода
  Wire.begin(Arduino_addr); // подключение к шине I2C как мастер
  Wire.beginTransmission(IMUAddress);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  int16_t fifo_23[6];
  int16_t fifo_6A[4];

  Wire.beginTransmission(IMUAddress);
  Wire.write(0x23);
  fifo_23[0] = 0x01; //Указание для передачи FIFO [температура, гиро_Х, гиро_Y, гиро_Z, акселб, включает включает аксель в FIFO]
  fifo_23[1] = 0x01; 
  fifo_23[2] = 0x01; 
  fifo_23[3] = 0x01; 
  fifo_23[4] = 0x01; 
  fifo_23[5] = 0x01; 
  Wire.write(0x01);
  Wire.write(0x01);
  Wire.write(0x01);
  Wire.write(0x01);
  Wire.write(0x01);
  //Wire.write(fifo_23, 6); // Настройка регистра 0х23
  Wire.endTransmission(true);

  Wire.beginTransmission(IMUAddress);
  Wire.write(0x6A); //
  fifo_6A[0] = 0x01;
  fifo_6A[1] = 0x00;
  fifo_6A[2] = 0x00;
  fifo_6A[3] = 0x00;
  Wire.write(0x01);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(0x01);
//fifo_6A = [0x01, 0x00, 0x00, 0x00]; //Указание для передачи FIFO [вкл, разрешает I2CMaster для MPU, включает SPI(Вместо I2C),сбрасывает буфер FIFO (FIFO надо выключить]
  //Wire.write(fifo_6A, 4); // Настройка регистра 0х23
  Wire.endTransmission(true);




  Wire.onRequest(write_R_Pi);

  //Назначение переменных под нужную точность
  switch (AFS_SELa)
  {
    case 0:
      coeffa = 2;
      AFS_CONFa = 0x00;
      break;
    case 1:
      coeffa = 4;
      AFS_CONFa = 0x01;
      break;
    case 2:
      coeffa = 6;
      AFS_CONFa = 0x10;
      break;
    case 3:
      coeffa = 8;
      AFS_CONFa = 0x11;
      break;
    default:
      coeffa = 2;
      AFS_CONFa = 0x00;
      break;
  }

  switch (AFS_SELg)
  {
    case 0:
      coeffg = 250;
      AFS_CONFg = 0x00;
      break;
    case 1:
      coeffg = 500;
      AFS_CONFg = 0x01;
      break;
    case 2:
      coeffg = 1000;
      AFS_CONFg = 0x10;
      break;
    case 3:
      coeffg = 2000;
      AFS_CONFg = 0x11;
      break;
    default:
      coeffg = 250;
      AFS_CONFg = 0x00;
      break;
  }



  Wire.beginTransmission(IMUAddress);
  Wire.write(0x1C);                  //обращаемся к регистру ACCEL_CONFIG (1C hex)
  Wire.write(AFS_CONFa);                  //Установить биты регистра как 00010000 (диапазон полной шкалы +/- 8g)
  Wire.endTransmission(true);
  // Настроить чувствительность гироскопа - полный диапазон шкалы (по умолчанию +/- 250 град / с)
  Wire.beginTransmission(IMUAddress);
  Wire.write(0x1B);                   // Обращаемся к регистру GYRO_CONFIG (1B hex)
  Wire.write(AFS_CONFg);                   // Установить биты регистра как 00010000 (полная шкала 1000 град / с)
  Wire.endTransmission(true);
  // delay(20);
  timer = micros();
}

unsigned MPU_period = 100; // Period oprosa MPU
unsigned long MPU_time = timer;

void loop() {
  Serial.print("aX" + int(aX));
  //Serial.print(str_Kal_Y + int(aY));
  
  
  if (millis() - MPU_time >= MPU_period) {
    if (flag == false) {
      read_MPU();
      Kal_ang();
      MPU_time = millis();
      Serial.print(str_Kal_X + int(kalAngleX));
      Serial.println(str_Kal_Y + int(kalAngleY));
    }

  }


}

void read_MPU() {
  Wire.beginTransmission(IMUAddress); // Открывает канал связи по шине I2C с ведомым устройством.
  Wire.write(0x3B); //передача начинается с регистра 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  n_MPU = Wire.available();
  Serial.print(n_MPU);
  Serial.print("MPU_available: " + String(n_MPU));
  n2_MPU = Wire.requestFrom(IMUAddress, nbytes); //Отправляет запрос на определенное количество байтов от ведущего устройства к ведомому.
  //акселерометр
  Serial.print("    ");
  Serial.print("MPU_requestFrom: " + String(n2_MPU));

  //true — после запроса отправляет STOP, освобождая шину I2C. false — после запроса отправляет RESTART. Шина не освобождается и можно отправлять дополнительные запросы. По умолчанию — true.
  Serial.print("    MPU endTransmission: " + String(MPU_write_flag));
  Serial.print("    ");
  if (n2_MPU == 14) {
    aX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    aY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    aZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    // температура
    tR = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    //гироскоп
    gX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    gY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    gZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  }
  MPU_write_flag = Wire.endTransmission(true); //Отправляет данные, которые были поставлены в очередь методом write() и завершает передачу.
}


void Kal_ang() {
  //Перевод показателей датчика из условных единиц в системы СИ (в количестве единиц графитации g)
  double aX_si = (double(aX) / double(32767)) * double(coeffa);
  double aY_si = (double(aY) / double(32767)) * double(coeffa);
  double aZ_si = (double(aZ) / double(32767)) * double(coeffa);

  double gX_si = (double(gX) / double(32767)) * double(coeffg);
  double gY_si = (double(gY) / double(32767)) * double(coeffg);
  double gZ_si = (double(gZ) / double(32767)) * double(coeffg);

  //расчет угла альфа через тангенс и перевод из радианы в градусы
  double aXangle = (atan2(aY_si, aZ_si) + PI) * RAD_TO_DEG;
  double aYangle = (atan2(aX_si, aZ_si) + PI) * RAD_TO_DEG;

  gyroXangle += kalmanX.getRate() * ((double)(micros() - timer) / 1000000);
  gyroYangle += kalmanY.getRate() * ((double)(micros() - timer) / 1000000);

  kalAngleX = kalmanX.getAngle(aXangle, gX_si, (double)(micros() - timer) / 1000000);
  kalAngleY = kalmanY.getAngle(aYangle, gY_si, (double)(micros() - timer) / 1000000);
  timer = micros();
  f_xyi();

}

void f_xyi() {
  if (int(kalAngleX) <= 256) {
    F_x = 0;
  }
  else {
    F_x = 1;
  }

  if (int(kalAngleY) <= 256) {
    F_y = 0;
  }
  else {
    F_y = 1;
  }
}


void write_R_Pi() {
  flag = true;
  //Kal_ang();
  //delay(500);
  Wire.write(int(F_x));
  Wire.write(int(kalAngleX));
  Wire.write(int(F_y));
  Wire.write(int(kalAngleY));
  flag = false;
  //delay(500);
  Serial.print("Rasbperry Pi write    X: " + String(kalAngleX) + "Y: " + String(kalAngleY));
  Serial.print("    ");

}
