#define OW_SKIP_ROM 0xCC                // Пропуск этапа адресации на шине 
#define OW_DS18B20_CONVERT_T 0x44       // Команда на начало замера
#define OW_DS18B20_READ_SCRATCHPAD 0xBE // Чтение скратчпада ds18b20
#define DS18B20_SCRATCHPAD_SIZE 9       // Размер скратчпада ds18b20

#include <Arduino.h>
#include <math.h> 
#include <SPI.h>
#include <OneWire.h>
#include <SD.h>
#include <LoRa.h>
#include <SparkFun_ADXL345.h>  
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include "Adafruit_SGP30.h"
#include <SHT3x.h>
#include <iarduino_GPS_NMEA.h>
#include <iarduino_Pressure_BMP.h>            

unsigned int samplingTime = 280;
unsigned int deltaTime = 40;
unsigned int sleepTime = 9680;
volatile long long timer_high;
volatile long long timer_low;

uint32_t fastTimer;
uint32_t fastTimerDel = 150;

uint32_t slowTimer;
uint32_t slowTimerDel = 400;
uint32_t ds18b20_timer;
int measurePin = A0;
int ledPower = 43;
float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;

volatile uint16_t doze[256];

SHT3x SHT;
iarduino_GPS_NMEA gps;      
OneWire  ds(31);  
Adafruit_SGP30 sgp;
iarduino_Pressure_BMP sensor; 
ADXL345 adxl = ADXL345(); 
RF24 radio(30, 29);
File myFile;


struct frstSrt
{
    uint8_t id = 0;   // 1 byte
    int16_t x_acs;    // 2 bytes
    int16_t y_acs;    // 2 bytes
    int16_t z_acs;    // 2 bytes
    int16_t trsh1;    // 2 bytes
    float trsh2;
    float trsh3;
    float trsh4;
    float presssure;  // 4 bytes
    uint32_t counter; // 4 bytes
}fastData;

struct secStr
{
  uint8_t id = 1;   // 1 byte
  int16_t pm25;    // 2 bytes
  int16_t tVOC;    // 2 bytes
  int16_t rad_qw;  // 2 bytes
  int16_t co2_ppm; // 2 bytes
  float lanGPS;     // 4 bytes
  float lonGPS;     // 4 bytes
  float humidVal;   // 4 bytes
  float temp;       // 4 bytes
  uint32_t counter; // 4 bytes
} slowData;           // 29 bytes: 29 / 250Kbps = 0.9ms - max max delay (in theory)

void readDoze();
void CO2int();
long int getCO2Data();
bool ds18b20_read_t(float & temperatur);
bool ds18b20_convert_t();
void printSDfast();
void printSDslow();
void fastDataMeasureNsend();
void slowDataMeasureNSend();
int main()
{
  attachInterrupt(4, readDoze, RISING);
  attachInterrupt(6, CO2int, CHANGE);
  Serial.begin(115200);
  Serial1.begin(9600);                         //  Инициируем работу с аппаратной шиной UART для получения данных от GPS модуля на скорости 9600 бит/сек.
  
  SPI.begin();                                               // инициализируем работу с SPI
  SPI.setDataMode(SPI_MODE3); 
  LoRa.setPins(26, 25, 5);
  if (!LoRa.begin(433E6))
    Serial.println("Starting LoRa failed!");
  else
    Serial.println("LoRa started sucsessfully");

  printf_begin(); 
  radio.begin();                            // (для макета) красный - желтый - оранжевый  (с конца платы, с внешней стороны) 
  radio.setChannel(100);                    // Указываем канал передачи данных (от 0 до 127), 5 - значит передача данных осуществляется на частоте 2,405 ГГц (на одном канале может быть только 1 приёмник и до 6 передатчиков)
  radio.setDataRate(RF24_250KBPS);          // Указываем скорость передачи данных (RF24_250KBPS, RF24_1MBPS, RF24_2MBPS), RF24_1MBPS - 1Мбит/сек
  radio.setPALevel(RF24_PA_MAX);            // Указываем мощность передатчика (RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm, RF24_PA_HIGH=-6dBm, RF24_PA_MAX=0dBm)
  radio.openWritingPipe(0x1234567899LL);
  radio.setAutoAck(false);
  radio.printDetails();
  
  if (!SD.begin(27))
  {
      Serial.println("initialization failed!");
      tone(24, 1000);
      delay(4000);
      noTone(24);
      tone(24, 6000);
      delay(4000);
      noTone(24);
      tone(24, 1000);
      delay(4000);
      noTone(24);
      tone(24, 6000);
      delay(4000);
      noTone(24);
       
  }  
  myFile = SD.open("test.txt", FILE_WRITE);
  if (myFile) {
    myFile.println("got ya");
    myFile.close();
  } else
  Serial.println("error opening test.txt");

  myFile = SD.open("test.txt");
  if (myFile)
    myFile.close();
  else
    Serial.println("error opening file");
  gps.begin(Serial1);
  gps.timeZone(3); 
  delay(100);
  if (! sgp.begin())
    Serial.println("SGP30 sensor not found :(");
  SHT.Begin();

  while(1)
  {
    if(millis()>=fastTimer)
    {
        fastDataMeasureNsend();
        fastTimer+=fastTimerDel;
    }
    if(millis()>=slowTimer)
    {
        slowDataMeasureNSend();
        slowTimer+=slowTimerDel;
    }
  }
  
}

void fastDataMeasureNsend()
{
    int x, y, z;
    adxl.readAccel(&x, &y, &z); 
    fastData.x_acs = x;
    fastData.y_acs = y;
    fastData.z_acs = z;
    sensor.read(2);
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
    myFile = SD.open("its_wednesday_dd.csv", FILE_WRITE);

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
    }
}

void slowDataMeasureNSend()
{
    digitalWrite(ledPower,LOW);
    delayMicroseconds(samplingTime);
    voMeasured = analogRead(measurePin);
    delayMicroseconds(deltaTime);
    digitalWrite(ledPower,HIGH);
    delayMicroseconds(sleepTime);
    calcVoltage = voMeasured*(5.0/1024);
    dustDensity = 0.17*calcVoltage-0.1;
    if ( dustDensity < 0)
    {
      dustDensity = 0.00;
    }
    slowData.pm25 = int16_t(dustDensity*1000);

    gps.read(); 
    slowData.lanGPS=gps.latitude;
    slowData.lonGPS=gps.longitude;

    if (millis()/100>=ds18b20_timer)//  раз в 3 итерации выполняется снятие данных с датчика
    {  
    if (millis()/400 != 0)          //  при первой итерации - пропускается блок снятия показаний, после чего посылается запрос на температуру (датчик не может мгновенно дать показания)
    {                               //                                                                                    |
      float temp;                   //                                                                                    |                                                                                    |
      if(ds18b20_read_t(temp))      //                                                                                    |
        slowData.temp = temp;       //                                                                                    |
      else                          //                                                                                    |
        slowData.temp = NAN;        //   в случае исключения в переменную пишем, что она была посчитана неверно           |
    }
    ds18b20_timer = millis()/100+10;  //                                                                                    |
    ds18b20_convert_t();            // <<|--------------------------------------------------------------------------------/
    }    
    
    SHT.UpdateData();
    slowData.humidVal = SHT.GetRelHumidity();   
    sgp.IAQmeasure();
    slowData.tVOC = sgp.TVOC;
    slowData.co2_ppm = getCO2Data();
    slowData.rad_qw = doze[0];
    doze[0]=0;
    slowData.counter = 0;
    radio.write(&slowData, sizeof(slowData));
    LoRa.beginPacket();
    LoRa.print(slowData.lanGPS);
    LoRa.print(" ");
    LoRa.print(slowData.lonGPS);
    LoRa.endPacket();
    myFile = SD.open("its_wednesday_dd_bt_slowww.csv", FILE_WRITE);

    if (myFile) 
    {
      myFile.print(millis());
      myFile.print(",");
      myFile.print(",");
      myFile.print(",");
      myFile.print(",");
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
  detachInterrupt(4);
  if (!ds.reset()) // даем резет на шину
 {   attachInterrupt(4, readDoze, RISING);
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
    attachInterrupt(4, readDoze, RISING);
    return false;
  }
  uint16_t uraw_temp;
  uraw_temp = scratchpad[0] | (static_cast<uint16_t>(scratchpad[1]) << 8);
  int16_t raw_temp;
  memcpy(&raw_temp, &uraw_temp, sizeof(raw_temp));
  temp = raw_temp / 16.f;
  temperatur = temp;
  attachInterrupt(4, readDoze, RISING);
  return true;
}

void readDoze()
{
    doze[0]++;
//  bitWrite(doze[], 0, digitalRead(41));
//  bitWrite(doze[], 1, digitalRead(40));
//  bitWrite(doze[], 2, digitalRead(39));
//  bitWrite(doze[], 3, digitalRead(38));

//  bitWrite(doze[], 4, digitalRead(37));
//  bitWrite(doze[], 5, digitalRead(36));
//  bitWrite(doze[], 6, digitalRead(35));
//  bitWrite(doze[], 7, digitalRead(34));
  Serial.println(doze[0]);
  
}

void CO2int()
{
  if(digitalRead(6))
  timer_high = micros();
  else
  timer_low = micros()-timer_high;
}

long int getCO2Data()
{ 
    timer_low /= 1000;
    timer_high = 1004 - timer_low;
    timer_low = 5000*(timer_low - 2)/(timer_low + timer_high - 4);
    return timer_low;
}