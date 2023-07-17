#define COMPL_K 0.07
#include <GyverFilters.h>
#include <Wire.h>
#include "Kalman.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
MPU6050 mpu;

float angleX_DMP = 180;
float angleY_DMP = 180;
float angleZ_DMP = 180;

float gyroXangle_COMPL = 180;
float gyroYangle_COMPL = 180;

Kalman kalmanX;
Kalman kalmanY;


uint8_t IMUAddress = 0x68;
uint8_t nbytes = 14;

uint8_t AFS_SELa = 0; //диапазон шкалы акселерометра
uint8_t AFS_SELg = 0; //диапазон шкалы гироскопа
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


double gyroXangle = 180; // Angle calculate using the gyro
double gyroYangle = 180;
double AngleX_comp = 180; // Calculate the angle using a Kalman filter
double AngleY_comp = 180;
double kalAngleX; // Calculate the angle using a Kalman filter
double kalAngleY;



String str_Kal_X = " aX_KAL:";
String str_Kal_Y = " aY_KAL:";

String str_COPL_X = " aX_COPL:";
String str_COPL_Y = " aY_COPL:";

String str_DMP_X = " aX_DMP:";
String str_DMP_Y = " aY_DMP:";

uint32_t timer;


void setup() {
  Serial.begin(9600); //настроить последовательный порт для вывода
  Wire.begin(); // подключение к шине I2C как мастер

  //  Wire.setClock(400000ul); // значение (в герцах) желаемых часов связи. Допустимые значения: 100000 (стандартный режим) и 400000 (быстрый режим).
  //Некоторые процессоры также поддерживают 10000 (низкоскоростной режим), 1000000 (быстрый режим плюс) и 3400000 (высокоскоростной режим).
  //См. Документацию по конкретному процессору, чтобы убедиться, что нужный режим поддерживается.

  mpu.initialize();
  initDMP();

  Wire.beginTransmission(IMUAddress);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

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
  delay(20);
  timer = micros();
}


void loop() {

  Wire.beginTransmission(IMUAddress); // Открывает канал связи по шине I2C с ведомым устройством.
  Wire.write(0x3B); //передача начинается с регистра 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  Wire.endTransmission(false); //Отправляет данные, которые были поставлены в очередь методом write() и завершает передачу.
  //true — после запроса отправляет STOP, освобождая шину I2C. false — после запроса отправляет RESTART. Шина не освобождается и можно отправлять дополнительные запросы. По умолчанию — true.

  Wire.requestFrom(IMUAddress, nbytes); //Отправляет запрос на определенное количество байтов от ведущего устройства к ведомому.
  //акселерометр
  aX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  aY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  aZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  // температура
  tR = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  //гироскоп
  gX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

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

  gyroXangle_COMPL += gX_si * ((float)(micros() - timer) / 1000000);
  gyroYangle_COMPL += gY_si * ((float)(micros() - timer) / 1000000);

  //Комплементарный фильтр
  AngleX_comp = ((float)(1 - COMPL_K) * (AngleX_comp + (gX_si * (float)(micros() - timer) / 1000000))) + (COMPL_K * aXangle);
  AngleY_comp = ((float)(1 - COMPL_K) * (AngleY_comp + (gY_si * (float)(micros() - timer) / 1000000))) + (COMPL_K * aYangle);






  /*

    Serial.println();
    Serial.print("NEW->");
    Serial.println();

    Serial.print("aXs:");
    Serial.print(aX_si);
    Serial.print("g ");

    Serial.print("aYs:");
    Serial.print(aY_si);
    Serial.print("g ");


    Serial.print("aZs:");
    Serial.print(aZ_si);
    Serial.print("g ");


    Serial.print("       |         ");


    Serial.print("gXs:");
    Serial.print(gX_si);
    Serial.print("°/сек ");

    Serial.print("gYs:");
    Serial.print(gY_si);
    Serial.print("°/сек ");


    Serial.print("gZs:");
    Serial.print(gZ_si);
    Serial.print("°/сек ");


    Serial.print("       |         ");

    Serial.print("angleX:");
    Serial.print(kalAngleX);
    Serial.print("° ");

    Serial.print("angleY:");
    Serial.print(kalAngleY);
    Serial.print("° ");


    Serial.print("       |         ");

    Serial.print("GyroAngleX:");
    Serial.print(gyroXangle);
    Serial.print("° ");

    Serial.print("GyroAngleY:");
    Serial.print(gyroYangle);
    Serial.print("° ");

  */
  //getAngles()

  

  // Углы через Калмана
  // Serial.print("aX_KAL:");
  Serial.print(str_Kal_X + kalAngleX);
  //Serial.print("aY_KAL:");
  Serial.print(str_Kal_Y + kalAngleY);

  //Комплементарный фильтр
  // Serial.print(" aX_COPL: ");
  Serial.print(str_COPL_X + AngleX_comp);
  //Serial.print(" aY_COPL: ");
  Serial.println(str_COPL_Y + AngleY_comp);

 /*
  //Углы через DMP
  // Serial.print(" aX_DMP: ");
  Serial.print(str_DMP_X + angleX_DMP);

  //  Serial.print(" aY_DMP: ");
  Serial.println(str_DMP_Y + angleY_DMP);

 
      Serial.print("aZ:");
      Serial.print(aZ);
      Serial.print("        ");

      Serial.print("a:");
      Serial.print(abs(aZ)+abs(aX)+abs(aY));
      Serial.print("        ");

      Serial.print("tR:");
      Serial.print(tR);
      Serial.print("        ");

      Serial.print("gX:");
      Serial.print(gX);
      Serial.print(" ");

      Serial.print("gY:");
      Serial.print(gY);
      Serial.print(" ");

      Serial.print("gZ:");
      Serial.print(gZ);
      Serial.print(" ");
      Serial.println();
  */
}










// НУЖНЫЕ ПЕРЕМЕННЫЕ
const float toDeg = 180.0 / M_PI;
uint8_t mpuIntStatus;   // содержит текущий байт состояния прерывания от MPU
uint8_t devStatus;      // возвращаем статус после каждой операции с устройством (0 = успех,! 0 = ошибка)
uint16_t packetSize;    // ожидаемый размер пакета DMP (по умолчанию 42 байта)
uint16_t fifoCount;     // подсчет всех байтов в текущий момент в FIFO
uint8_t fifoBuffer[64]; // буфер хранения FIFO
Quaternion q;           // [w, x, y, z]         кватернион контейнер
VectorFloat gravity;    // [x, y, z]            вектор гравитации
float ypr[3];           // [yaw, pitch, roll]   контейнер рыскания/тангажа/крена и вектор силы тяжести

// инициализация
void initDMP() {
  devStatus = mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  mpuIntStatus = mpu.getIntStatus();
  packetSize = mpu.dmpGetFIFOPacketSize();
}

// получение углов в angleX, angleY, angleZ по DMP
void getAngles() {

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    angleX_DMP = ypr[2] * toDeg + 180;
    angleY_DMP = ypr[1] * toDeg + 180;
    angleZ_DMP = ypr[0] * toDeg + 180;
  }
}
