#include <Arduino.h>
  
#define Radio_CE 12                     // пин chip enable для радио
#define Radio_CSN 13                    // пин chip select для радио
#define Acsel_pin 27                    // пин chip select для акселерометра
#define Temperature_pin 42              // пин chip select для термометра
#define Pressure_pin 26                 // пин chip select для барометра
#define Sd_pin 41                       // пин chip select для SD карты
#define MH_Z14A_PWM 18                  // пин для ШИМ датчика СО2 ()



#define OW_SKIP_ROM 0xCC                // Пропуск этапа адресации на шине 
#define OW_DS18B20_CONVERT_T 0x44       // Команда на начало замера
#define OW_DS18B20_READ_SCRATCHPAD 0xBE // Чтение скратчпада ds18b20
#define DS18B20_SCRATCHPAD_SIZE 9       // Размер скратчпада ds18b20


#include <SD.h>
#include <TinyGPS.h> 
#include <OneWire.h>
#include <string.h>  // для memcpy
#include <stdint.h>  // для int8_t, uint8_t и т.п.
#include <math.h> // для NAN
#include "Adafruit_BMP280.h"
#include "SparkFun_ADXL345.h"
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(Radio_CE, Radio_CSN);        // Создаём объект radio для работы с библиотекой RF24, указывая номера выводов nRF24L01+ (CE, CSN)
Adafruit_BMP280 bmp(Pressure_pin);      // создаём объект bmp для работы с барометром
OneWire  ds(36);                        // создаём объект ds для работы с термометром
ADXL345 adxl = ADXL345(Acsel_pin);      // создаём объект ADXL345 для работы с акселерометром
TinyGPS gps;
float FLAT, FLON, ALT;

int x, y, z;                            // переменные для ускорений по 3 осям

bool buzz;

volatile unsigned long long co2_th;
volatile unsigned long long co2_tl;
volatile int cntr_co2;

struct telemetry_p1       //Создаем структуру
{
  bool id = 0;
  float temp_str;      // переменная для температуры
  float press_str;     // переменная для давления с барометра
  int x_str;           ////////////////////////////////////////////////
  int8_t y_str;           //  переменные для ускорений с акселерометра  //
  int16_t z_str;           ////////////////////////////////////////////////
  float gps_lat;
  float gps_lon;
  float GPStime;
  uint32_t timer;      // переменная для подсчета выполненных циклов программы
} data_1;

struct telemetry_p2    //Создаем структуру
{
  bool id = 1;
  float O2_percent;
  float CO_ppm;
  int GPSaltit;
  int8_t MH_Z14A_temp; // переменная для хранения температуры с датчика СО2
  int16_t CO2_ppm;     // переменная для хранения значения CO2 в ppm
  float NO2_ppm;
  float NH3_ppm;  
  float rad;
  uint32_t timer;      // переменная для подсчета выполненных циклов программы
} data_2;
  
volatile int counter = 0;   //
void rad_tick();            //

long int ds18b20_timer = 0 ;                                         //
long int bmp280_timer =  0 ;                                         //
long int adxl345_timer = 0 ;                                         //
long int gps_timer =     0 ;                                         //
long int co2_timer =     0 ;                                         //
long int o2_timer =      0 ;                                         //
long int gaz_x3_timer =  0 ;                                         //
long int rad_timer =     0 ;                                         //
long int radio_timer =     0 ;                                         //

int gas_turn;

int CO2_timer;

void rad_tick();
bool ds18b20_convert_t();
bool ds18b20_read_t(float & temperatur);
boolean get_MH_Z14A_data(int16_t &ppm);
boolean get_Radiation_value(float &doze);
bool readgps();
void Co2_tl();
void Co2_th();
void getTVOC_val();
void getPM25_val();
void getHumidity_val();
void make_noise();
void stop_noise();

void setup() { 
 
  pinMode(46, INPUT_PULLUP);
  pinMode(47, INPUT_PULLUP);
  pinMode(48, INPUT_PULLUP);
  pinMode(16, 1);
  digitalWrite(16, 1);
  pinMode(Sd_pin, OUTPUT);
  SD.begin(Sd_pin);
  //////////////////////////////////////////////
  pinMode(7, INPUT);                          //
  attachInterrupt(7, rad_tick, FALLING);      //
  //////////////////////////////////////////////
  pinMode(18, INPUT);
  attachInterrupt(0, Co2_th, RISING);  
  pinMode(19, INPUT);
  attachInterrupt(1, Co2_tl, FALLING);  

  Serial.begin(9600);                         //  Инициируем работу с аппаратной шиной UART для получения данных от GPS модуля на скорости 9600 бит/сек.
  //gps.begin(Serial);                          //  Инициируем расшифровку строк NMEA указав объект используемой шины UART.
  SPI.begin();                                               // инициализируем работу с SPI
  SPI.setDataMode(SPI_MODE3);                                // насотройка SPI
  delay(100);  
  radio.begin();                                             // Инициируем работу nRF24L01+
  radio.setChannel(120);                                     // Указываем канал передачи данных (от 0 до 127), 5 - значит передача данных осуществляется на частоте 2,405 ГГц (на одном канале может быть только 1 приёмник и до 6 передатчиков)
  radio.setDataRate(RF24_2MBPS);                           // Указываем скорость передачи данных (RF24_250KBPS, RF24_1MBPS, RF24_2MBPS), RF24_1MBPS - 1Мбит/сек
  radio.setPALevel(RF24_PA_MAX);                            // Указываем мощность передатчика (RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm, RF24_PA_HIGH=-6dBm, RF24_PA_MAX=0dBm)
  radio.openWritingPipe(0x1234567899LL);                     // Открываем трубу с идентификатором 0x1234567899LL для передачи данных (на одном канале может быть открыто до 6 разных труб, которые должны отличаться только последним байтом идентификатора)
  bmp.begin();
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* режим работы      */
                  Adafruit_BMP280::SAMPLING_X2,     /* коэф. температуры */
                  Adafruit_BMP280::SAMPLING_X16,    /* коэф. давления    */
                  Adafruit_BMP280::FILTER_X16,      /* фильтр            */
                  Adafruit_BMP280::STANDBY_MS_500); /* время ожидания    */

  adxl.powerOn();                     // вывод датчика из режима пониженного энергопотребления (на случай, если он был случайно включён)
  adxl.setRangeSetting(16);           // настройка чувствительности (макс - 16)
  delay(1000);   
  get_MH_Z14A_data(data_2.CO2_ppm);               
}
void loop()
{


  // ToDo - перенастроить на малую точность, но большую скорость замера
  ////////////////////////////////////          ДЛЯ РАБОТЫ С ДАТЧИКОМ ТЕМПЕРАТУРЫ DS18B20
  if (millis()/100>=ds18b20_timer)        //  раз в 3 итерации выполняется снятие данных с датчика
  {  
    if (data_1.timer != 0)            //  при первой итерации - пропускается блок снятия показаний, после чего посылается запрос на температуру (датчик не может мгновенно дать показания)
    {                               //                                                                                    |
      float temp;                   //                                                                                    |                                                                                    |
      if(ds18b20_read_t(temp))      //                                                                                    |
        data_1.temp_str = temp;       //                                                                                    |
      else                          //                                                                                    |
        data_1.temp_str = NAN;        //   в случае исключения в переменную пишем, что она была посчитана неверно           |
    }
    ds18b20_timer = millis()/100+10;  //                                                                                    |
    ds18b20_convert_t();            // <<|--------------------------------------------------------------------------------/
  }                                 //



  adxl.readAccel(&x, &y, &z);                         // запись значений для ускорений в указанные переменные
  data_1.x_str = x;                                   /////////////////////////////////////////////////////////////
  data_1.y_str = y;                                   //   запись в структурную переменную телеметрии ускорений  //
  data_1.z_str = z;                                   /////////////////////////////////////////////////////////////

  data_1.press_str = bmp.readPressure();              //

  get_MH_Z14A_data(data_2.CO2_ppm);                   // получение данных с датчика CO2

  /*
  if(millis()/100 >= gps_timer)
  { 
    if(!readgps()){}
    else 
    {
     unsigned long aG; 
      gps.f_get_position(&data_1.gps_lat, &data_1.gps_lon, &aG);
      unsigned long timeee;
      unsigned long dateee;
      gps.get_datetime(&dateee,&timeee, &aG);                        
      data_2.GPSaltit = gps.f_altitude();
      data_1.GPStime = float(timeee);
    }
    gps_timer = millis()/100 + 5;
  }
    ToDo -  нужно переделать c минимальным возможным временем парсинга данных
  */

  if(millis()/100 >= rad_timer)
  {
    get_Radiation_value(data_2.rad);
    rad_timer = millis()/100 + 600;
  }
  
    radio.write(&data_1, sizeof(data_1));                 // отправка в эфир пакета данных
    radio.write(&data_2, sizeof(data_2));                 // отправка в эфир пакета данных
    data_1.timer++;
    data_2.timer++;                                       // + 1 выполненный цикл

    File allData = SD.open("Eco.csv", FILE_WRITE);
    if (allData)
    {
    
      allData.print(data_1.GPStime);
      allData.print(";");
      allData.print(data_2.GPSaltit);
      allData.print(";");
      allData.print(data_1.gps_lat, 7);
      allData.print(";");
      allData.print(data_1.gps_lon, 7);
      allData.print(";");
      allData.print(data_1.press_str);
      allData.print(";");
      allData.print(data_1.temp_str);
      allData.print(";");
      allData.print(data_1.x_str);
      allData.print(";");
      allData.print(data_1.y_str);
      allData.print(";");
      allData.print(data_1.z_str);
      allData.print(";");
      allData.print(data_2.O2_percent);
      allData.print(";");
      allData.print(data_2.CO2_ppm);
      allData.print(";");
      allData.print(data_2.CO_ppm);
      allData.print(";");
      allData.print(data_2.NO2_ppm);
      allData.print(";");
      allData.print(data_2.NH3_ppm);
      allData.print(";");
      allData.print(data_2.rad);
      allData.print(";");
      allData.print(data_2.timer);
      allData.print(";");
      allData.println(millis()/1000);
      allData.close();
    }
    else
    {
      if(buzz)
      {
        stop_noise();
      }
      else
      {
        make_noise();
      }
    }
  }
  ////////////////////////////////////////////////////
  







void rad_tick()
{
  counter++;       //
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
    return false;

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

boolean get_MH_Z14A_data(int16_t &ppm)
{
  unsigned long tl;
  unsigned long th;
  int16_t co2_last = data_2.CO2_ppm;
  th = co2_tl/1000 ;
  tl = 1004 - th;
  ppm =  5000 * (th-2)/(th+tl-4); // расчёт для диапазона от 0 до 5000ppm 
  if(ppm<0)
  {
    ppm = co2_last;
  }
  
}
void Co2_th()
{
  co2_th=micros();
}
void Co2_tl()
{
  co2_tl=micros()-co2_th;
}


boolean get_Radiation_value(float &doze)
{
  doze = float(counter/1.25);
  counter = 0;
  return 1;
}


bool readgps()
{
  while (Serial.available())
  {
    char b = Serial.read();
    if('\r' != b)
    {
      if (gps.encode(b))
      return true;
    }
  }
  return false;
}

void getTVOC_val()
{
  // ToDo . алгоритм должен будет считывать показания с датчика органики

}

void getPM25_val()
{
  // ToDo . алгоритм будет считывать и распределять показания с 
}

