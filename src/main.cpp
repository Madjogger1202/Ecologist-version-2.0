////////////////////////////////////////////////////////////////////////
//                                                                    //                         
//    ┌◄──────────────────────────────────────►┐                      //                       
//    │     Ecologist Cansat V2                │                      //                         
//    │                                        │                      //                     
//    │     Writen by Madjogger1202            │                      //
//    │                                        │                      //
//    │►remember, ist wednesday my dudes────┬─►│                      //
//    │ │           xxxxxxxxxxxx            │  │                      //  
//    │ │  xxxxxxxxxx┌┬─┬──┬─┬┐xxxxxxxxxxxxx│  │                      //
//    │ │  xxxxxxx┌──┴┤x├──┤x├┴──┐ xxxxxxxxx│  │                      //
//    └─┘► xxxxxxx│   └─┘  └─┘   │ xxxxxxxxx▼ ─┘                      //
//         xxxxxxx│xxxxxxxxxxxxxx│xxxxxxxxxx                          //
//                ├┬─────────────┴┐                                   //
//         ───────┴┘              └────┐                              //
//    xxxxx                            │       xxx                    //         
//    xxxxxxxxxxxxxxxxxxxx         xxxxxxxxxxxxxxxx                   //         
//    xxxxxxxxxxxxxxxxxxx─────────xxxxxxxxxxxxxxxx                    //         
//                                                                    //
////////////////////////////////////////////////////////////////////////                                


//////////////////////////////////////////
#define OW_SKIP_ROM 0xCC                // Пропуск этапа адресации на шине 
#define OW_DS18B20_CONVERT_T 0x44       // Команда на начало замера
#define OW_DS18B20_READ_SCRATCHPAD 0xBE // Чтение скратчпада ds18b20
#define DS18B20_SCRATCHPAD_SIZE 9       // Размер скратчпада ds18b20
//////////////////////////////////////////

// CAUTION!! к чему подключаться и полезные примечания: 
#define RAD_IRQ 4
#define RAD_1 41
#define RAD_2 40
#define RAD_3 39
#define RAD_4 38
#define RAD_5 37
#define RAD_6 23
#define RAD_7 35
#define RAD_8 34

#define DS18B20_PIN 31          // пин данных с термометра
#define MH_Z19B_IRQ 6           // вывод ШИМ с датчика CO2, также 6 рерывание в коде
#define NRF24_CS 30             // CS радиомодуля
#define NRF24_CE 29             // CE радиомодуля
#define GPS_UART Serial1        // 
#define GPS_DEF_BOD 9600        // нельзя поменять, записав в память GPS, он не запоминает
#define GPS_DEF_RATE 1          // same thing 
#define PM25_LED 43             // Просвечивающий светодиод у датчика PM2.5
#define PM25_OUT A0             // Вывод чувствительного у датчика PM2.5
#define SD_CS 27                // CS sd карты *модуля*
#define LORA_NSS 26             // CS лоры
#define LORA_DIO0 5             // первое прерывание с лоры
#define LORA_RST 25             // Пин перезагрузки лоры
#define BUZZER_PIN 24           // Пищалка, активируется просто подачей напряжения
#define BEST_SPI_SPEED 5E6      // частота, на которой все точно работает нормально



////////////////////////////////////////// подключение всхе библиотек, все необходимые исходники лежат в папке с библиотеками
#include <Arduino.h>                    //
#include <math.h>                       //
#include <SPI.h>                        //
#include <OneWire.h>                    //
#include <SD.h>                         //
#include <LoRa.h>                       //
#include <SparkFun_ADXL345.h>           //
#include "nRF24L01.h"                   //
#include "RF24.h"                       //
#include "printf.h"                     //  
#include "Adafruit_SGP30.h"             //
#include <SHT3x.h>                      //
#include <iarduino_Pressure_BMP.h>      //
#include <iarduino_GPS_NMEA.h>          //
//////////////////////////////////////////


///////////////////////////////////
unsigned int samplingTime = 280; //
unsigned int deltaTime = 40;     // для датчика PM2.5
unsigned int sleepTime = 9680;   //
///////////////////////////////////

///////////////////////////////////
volatile long long timer_high;   //  Переменные для ШИМ сигнала с датчика CO2
volatile long long timer_low;    //
///////////////////////////////////

////////////////////////////////////
uint32_t fastTimer;               //
uint32_t fastTimerDel = 100;      //
                                  //
uint32_t slowTimer;               //   таймеры для общего цикла
uint32_t slowTimerDel = 500;      //
                                  //
uint32_t ds18b20_timer;           //
                                  //
uint32_t rad_log_timer;           // 
////////////////////////////////////

////////////////////////////////////
int measurePin = PM25_OUT;        // 
int ledPower = PM25_LED;          //
                                  // для датчика PM2.5
float voMeasured = 0;             //
float calcVoltage = 0;            //
float dustDensity = 0;            //
////////////////////////////////////

volatile uint16_t doze[256];      // для хранения почти нашего спектра импульсов
volatile uint16_t dozeAbs;        // просто кол-во импульсов

iarduino_GPS_NMEA    gps;         // по факту - абстрактный объект, которому уже потом нужно скормить нард uart шину
SHT3x SHT;                        // I2C
OneWire  ds(DS18B20_PIN);         // OneWire
Adafruit_SGP30 sgp;               // I2C
iarduino_Pressure_BMP sensor;     // I2C
ADXL345 adxl = ADXL345();         // I2C
RF24 radio(NRF24_CS, NRF24_CE);   // CS, CE (см в гугл таблице)

File myFile;                      // объект файла для СД карты


struct frstSrt      //  при помощипеременных trsh я уравнвешиваю пакеты и делаю их одинаковой структуры
{                   //  это позволит сразу принимать не в массив байт, а в переменные с явным приведением типов
  uint8_t id = 0;   // 1 byte
  int16_t x_acs;    // 2 bytes
  int16_t y_acs;    // 2 bytes
  int16_t z_acs;    // 2 bytes
  int16_t trsh1;    // 2 bytes
  float trsh2;      // 4 bytes
  float trsh3;      // 4 bytes
  float trsh4;      // 4 bytes
  float presssure;  // 4 bytes
  uint32_t counter; // 4 bytes
} fastData;         

struct secStr
{
  uint8_t id = 1;   // 1 byte
  int16_t pm25;     // 2 bytes
  int16_t tVOC;     // 2 bytes
  int16_t rad_qw;   // 2 bytes
  int16_t co2_ppm;  // 2 bytes
  float lanGPS;     // 4 bytes
  float lonGPS;     // 4 bytes
  float humidVal;   // 4 bytes
  float temp;       // 4 bytes
  uint32_t counter; // 4 bytes
} slowData;         // 29 bytes: 29 / 250Kbps = 0.9ms - max max delay (in theory)  вот только при малых значениях таймеров 
                    //                                        пакеты начинали теряться, то есть лучше делать delta ~90+ ms 

void readDoze();
void CO2int();
long int getCO2Data();
bool ds18b20_read_t(float & temperatur);
bool ds18b20_convert_t();
void printSDfast();
void printSDslow();
void fastDataMeasureNsend();
void slowDataMeasureNSend();
void clarDoze();
void writeRadLog(); 

void setup()
{

  Serial.begin(115200);                          // *для отладки* - по порядку выведенные пины:
                                                 //     GND  *
                                                 //     RX   *
                                                 //     *    TX           
  
//  attachInterrupt(RAD_IRQ, readDoze, RISING);          // прерывание для дозиметра (если все перестало работать - два раза ударить по слешу и дозиметр перестанет дергать аппарат, а потом перепрошить stm8)
  
  pinMode(MH_Z19B_IRQ, INPUT);
  attachInterrupt(MH_Z19B_IRQ, CO2int, CHANGE);        // прерывание для считывания ШИМ сигнала с датчика CO2 

  SPI.begin();                                    // инициализируем работу с SPI
  SPI.setDataMode(SPI_MODE3);                     // чтобы лора не смогла подмять шину на старте
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);    // LoRa: NSS, RST, DIO0  ;  dio0 - прерывание
  if (!LoRa.begin(long(433E6)))                   // частота не сильно влияет, в случае её замены нужно проверять, что числе идёт как long
  {
    Serial.println("Starting LoRa failed!");
    tone(BUZZER_PIN, 6000);                       // по идее - пищать должна будет сразу после старта (первый паттерн пищания)
  }
  else
  {
    Serial.println("LoRa started sucsessfully");
    // LoRa.enableCrc();                          // раскомментировать если настройки начнут сбивать контрольную сумму
    LoRa.setCodingRate4(8);
    LoRa.setSignalBandwidth(62.5E3);
    LoRa.setSpreadingFactor(5);

  }

  printf_begin();                           // для вывода с nrf
  radio.begin();                            // (для макета) красный - желтый - оранжевый  (с конца платы, с внешней стороны)
  radio.setChannel(100);                    // Указываем канал передачи данных (от 0 до 127), 5 - значит передача данных осуществляется на частоте 2,405 ГГц (на одном канале может быть только 1 приёмник и до 6 передатчиков)
  radio.setDataRate(RF24_250KBPS);          // Указываем скорость передачи данных (RF24_250KBPS, RF24_1MBPS, RF24_2MBPS), RF24_1MBPS - 1Мбит/сек
  radio.setPALevel(RF24_PA_MAX);            // Указываем мощность передатчика (RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm, RF24_PA_HIGH=-6dBm, RF24_PA_MAX=0dBm)
  radio.openWritingPipe(0x1234567899LL);
  radio.setAutoAck(false);
  radio.printDetails();

  //////////////////////////////////////////////////////
  if (!SD.begin(SD_CS))                               // обычно всё проходит штатно, но если не инициализировалось - имеет смысл проверить карточку
  {                                                   //
    Serial.println(" SD card initialization failed!");//
    tone(BUZZER_PIN, 1000);                           //
    delay(400);                                       //
    noTone(BUZZER_PIN);                               //
                                                      //
  }                                                   //
  myFile = SD.open("test.txt", FILE_WRITE);           //
  if (myFile) {                                       //
    myFile.println("got ya");                         //
    myFile.close();                                   //
  } else                                              //
    Serial.println("error opening test.txt");         //
                                                      //
  myFile = SD.open("test.txt");                       //
  if (myFile)                                         //
    myFile.close();                                   //
  else                                                //
    Serial.println("error opening file");             //
  delay(100);                                         //
  //////////////////////////////////////////////////////



  ///////////////////////////////////////////////////
  if (! sgp.begin())                               // инициализация i2c-шных датчиков воздуха, если первый не заработает, значит шина i2c легла
    Serial.println("SGP30 sensor not found :(");   //
                                                   //
  SHT.Begin();                                     //
  ///////////////////////////////////////////////////

  //////////////////////////////// в инициализации акселерометра таже можно в теории добавить настройку всех коэффициентов, но пока что это не нужно
  adxl.powerOn();               //
  adxl.setRangeSetting(16);     //
  ////////////////////////////////

  sensor.begin();                               // инициализация BMP280, можно настраивать bool iarduino_Pressure_BMP::begin(float = (0.0F))
  
  ////////////////////////////////////////////////    Инициализация GPS
  Serial1.begin(9600);                          //
  uint8_t bytes[] = {                           // ну а вдруг((   в теории должно переключать GPS в быстрый режим, но на модуле не работает сохранение настроек(
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12
  };                                            //
  Serial1.write(bytes, sizeof(bytes));          //
  gps.begin(Serial1);                           //
  ////////////////////////////////////////////////
}

/*
    Было бы более красиво всё закинуть в int main(), но в полевых условиях могут возникнуть конфликты в ненастроенной ide
*/

void loop()
{
  if (millis() >= fastTimer)
  {
    fastDataMeasureNsend();
    fastTimer = millis() + fastTimerDel;
  }
  if (millis() >= slowTimer)
  {
    slowDataMeasureNSend();
    slowTimer = millis() + slowTimerDel;
  }
  if (millis() >= rad_log_timer)
  {
    writeRadLog();
    clarDoze();
    rad_log_timer = millis() + 4000;
  }
}


void fastDataMeasureNsend()
{
  int x, y, z;
  adxl.readAccel(&x, &y, &z);                     // иррационально, но поможет изберать фантомных проблем сторонних компиляторов
  fastData.x_acs = x;
  fastData.y_acs = y;
  fastData.z_acs = z; 
  sensor.read(2);                                 // 2 - int-овое значение, 1 или 2, для норм вывода давления - 2
  fastData.presssure = sensor.pressure;
  fastData.counter++;
  radio.write(&fastData, sizeof(fastData));
  Serial.print(millis());
  Serial.print(",");
  Serial.print(fastData.x_acs);
  Serial.print(",");
  Serial.print(fastData.y_acs);
  Serial.print(",");
  Serial.print(fastData.z_acs);
  Serial.print(",");
  Serial.print(fastData.presssure);
  Serial.print(",");
  Serial.println(fastData.counter);
  //  SD.begin(SD_CS);                             // дергать карточку перед записью каждый раз нежелательно
  myFile = SD.open("Eco.txt", FILE_WRITE);

  if (myFile)
  {
    myFile.print(millis());
    myFile.print(",");
    myFile.print(fastData.x_acs);
    myFile.print(",");
    myFile.print(fastData.y_acs);
    myFile.print(",");
    myFile.print(fastData.z_acs);
    myFile.print(",");
    myFile.print(fastData.presssure);
    myFile.print(",");
    myFile.println(fastData.counter);
    myFile.close();
    noTone(BUZZER_PIN);
  }
  else
  {
    tone(BUZZER_PIN, 1000);
    delay(100);
    SPI.begin();
    SD.begin(SD_CS);
    SPI.setDataMode(SPI_MODE3);
    //    LoRa.begin((433E6));
    Serial.println(myFile);
  }
}

void slowDataMeasureNSend()
{
  digitalWrite(ledPower, LOW);
  delayMicroseconds(samplingTime);
  voMeasured = analogRead(measurePin);
  delayMicroseconds(deltaTime);
  digitalWrite(ledPower, HIGH);
  delayMicroseconds(sleepTime);
  calcVoltage = voMeasured * (5.0 / 1024);
  dustDensity = 170 * calcVoltage - 0.1;
  if ( dustDensity < 0)
  {
    dustDensity = 0.00;
  }
  slowData.pm25 = dustDensity;



  if (millis() / 100 >= ds18b20_timer) //  раз в 3 итерации выполняется снятие данных с датчика
  {
    if (millis() / 400 != 0)        //  при первой итерации - пропускается блок снятия показаний, после чего посылается запрос на температуру (датчик не может мгновенно дать показания)
    { //                                                                                    |
      float temp;                   //                                                                                    |                                                                                    |
      if (ds18b20_read_t(temp))     //                                                                                    |
        slowData.temp = temp;       //                                                                                    |
      else                          //                                                                                    |
        slowData.temp = NAN;        //   в случае исключения в переменную пишем, что она была посчитана неверно           |
    }
    ds18b20_timer = millis() / 100 + 10; //                                                                                    |
    ds18b20_convert_t();            // <<|--------------------------------------------------------------------------------/
  }

  SHT.UpdateData();
  slowData.humidVal = SHT.GetRelHumidity();
  sgp.IAQmeasure();
  slowData.tVOC = sgp.TVOC;
  slowData.co2_ppm = getCO2Data();
  slowData.rad_qw = dozeAbs;
  doze[0] = 0;
  slowData.counter++;
  radio.write(&slowData, sizeof(slowData));
  if (!(slowData.counter % 9))
  {
    gps.read();
    slowData.lanGPS = gps.longitude;
    slowData.lonGPS = gps.latitude;
    //     SPI.begin();
    //     SPI.setDataMode(SPI_MODE3);

  }
  if (!(slowData.counter % 18))
  {
    LoRa.begin(long(433E6));
    LoRa.enableCrc();
    LoRa.setCodingRate4(8);
    LoRa.setSignalBandwidth(62.5E3);
    LoRa.setSpreadingFactor(5);
    LoRa.beginPacket();
    LoRa.print(slowData.lanGPS, 7);
    LoRa.print(" ");
    LoRa.print(slowData.lonGPS, 7);
    LoRa.endPacket(1);
  }
  //  SD.begin(27);
  myFile = SD.open("Eco2.txt", FILE_WRITE);

  if (myFile)
  {
    myFile.print(millis());
    myFile.print(",");
    myFile.print(slowData.counter);
    myFile.print(",");
    myFile.print(slowData.pm25);
    myFile.print(",");
    myFile.print(slowData.tVOC);
    myFile.print(",");
    myFile.print(slowData.rad_qw);
    myFile.print(",");
    myFile.print(slowData.co2_ppm);
    myFile.print(",");
    myFile.print(slowData.humidVal);
    myFile.print(",");
    myFile.print(slowData.temp);
    myFile.print(",");
    myFile.print(slowData.lanGPS, 7);
    myFile.print(",");
    myFile.println(slowData.lonGPS, 7);
    myFile.close();
  }
  else
  {
    tone(BUZZER_PIN, 1000);
  }
}

bool ds18b20_convert_t()
{
  if (!ds.reset()) // даем reset на шину
  {
    return false;
  }
  ds.write(OW_SKIP_ROM, 1);
  ds.write(OW_DS18B20_CONVERT_T, 1);
  return true;
}
bool ds18b20_read_t(float & temperatur)
{
  if (!ds.reset()) // даем резет на шину
  { 
    return false;
  }
  ds.write(OW_SKIP_ROM, 1); // Пропускаем этап адресации
  uint8_t scratchpad[DS18B20_SCRATCHPAD_SIZE];
  ds.write(OW_DS18B20_READ_SCRATCHPAD, 1);
  ds.read_bytes(scratchpad, sizeof(scratchpad));
  uint8_t crc_actual = scratchpad[DS18B20_SCRATCHPAD_SIZE - 1]; // Берем контрольную сумму, которую насчитал у себя датчик и положил в последний байт скратчпада
  uint8_t crc_calculated = OneWire::crc8(scratchpad, DS18B20_SCRATCHPAD_SIZE - 1); // Считаем сами по всем байтам скратчпада кроме последнего
  float temp;
  if (crc_calculated != crc_actual)
  {
    return false;
  }
  uint16_t uraw_temp;
  uraw_temp = scratchpad[0] | (static_cast<uint16_t>(scratchpad[1]) << 8);
  int16_t raw_temp;
  memcpy(&raw_temp, &uraw_temp, sizeof(raw_temp));
  temp = raw_temp / 16.f;
  temperatur = temp;
  return true;
}

void readDoze()
{
  dozeAbs++;        // можно просто реализовать подсчет импульсов
  uint8_t dozeImp;  // для распределения
  bitWrite(dozeImp, 0, digitalRead(41));
  bitWrite(dozeImp, 1, digitalRead(40));
  bitWrite(dozeImp, 2, digitalRead(39));
  bitWrite(dozeImp, 3, digitalRead(38));

  bitWrite(dozeImp, 4, digitalRead(37));
  bitWrite(dozeImp, 5, digitalRead(36));
  bitWrite(dozeImp, 6, digitalRead(35));
  bitWrite(dozeImp, 7, digitalRead(34));
  doze[dozeImp]++;
  Serial.println(doze[dozeImp]);           // для отладки и калибровки чувствительности 

}

void clarDoze()
{
  for(uint8_t i = 0; i<256; i++)
    doze[i]=0;
  dozeAbs=0;
}

void writeRadLog()                            // подразумевается, что лог пишется не особо часто, ибо на ячейку по 5 байт(символьно), значит 1 280 байт на запись
{
  myFile = SD.open("RadLg.txt", FILE_WRITE);

  if (myFile)
  {
    for(uint8_t i = 0; i<256; i++)
    {
      myFile.print(doze[i]);
      myFile.print(",");
    }
    myFile.println();
  }

}

void CO2int()
{
  if (digitalRead(MH_Z19B_IRQ))
    timer_high = micros();
  else
    timer_low = micros() - timer_high;
}

long int getCO2Data()
{
  timer_low /= 1000;
  timer_high = 1004 - timer_low;
  timer_low = 5000 * (timer_low - 2) / (timer_low + timer_high - 4);  // коэффициент 5000 стандартный, на некоторых модулях по дефолту идет 2000
  if ((timer_low > 0) && (timer_low < 5000))
    return timer_low;
  else
    return slowData.co2_ppm;
}