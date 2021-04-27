
#define OW_SKIP_ROM 0xCC                // Пропуск этапа адресации на шине 
#define OW_DS18B20_CONVERT_T 0x44       // Команда на начало замера
#define OW_DS18B20_READ_SCRATCHPAD 0xBE // Чтение скратчпада ds18b20
#define DS18B20_SCRATCHPAD_SIZE 9       // Размер скратчпада ds18b20

#include <Arduino.h>

#include <OneWire.h>                    // библиотека для работы с барометром по одноименному протоколу
#include <Adafruit_BMP280.h>            // библиотека для работы с барометром
#include <nRF24L01.h>                   // суб-библиотека для радиомодуля
#include <RF24.h>                       // основная библиотека для радиомодуля
#include <SparkFun_ADXL345.h>
#include <Wire.h>
#include <SPI.h>


#define BMP_CS  32    // пин подключения 
#define ADXL_CS  33   //
#define DS18B20 31    //

#define NRF_CS  30    //
#define NRF_CE  29    //

#define LORA_CS  26   //
#define LORA_INT 5    //
#define LORA_RST 25   //

#define SD_CS  27     // 

#define GP2Y_PIN A0
#define GP2Y_LED 28

#define MH_Z19B 6 

#define BUZZER_PIN 24

OneWire  ds(DS18B20); 
ADXL345 adxl = ADXL345(ADXL_CS); 
Adafruit_BMP280 bmp(BMP_CS); 
RF24 radio(NRF_CE, LORA_CS);


/* это неправильно, но пока что я поставлю точно рабочий костыль вместо динамического размера пакета
struct frstSrt
{
    uint8_t id = 0;   // 1 byte
    int16_t x_acs;    // 2 bytes
    int16_t y_acs;    // 2 bytes
    int16_t z_acs;    // 2 bytes
    float presssure;  // 4 bytes
    uint32_t counter; // 4 bytes
}fastData;            // 15 bytes: 15 / 250Kbps = 0.4 ms - min max delay 
*/
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
}fastData, midData; 

struct secStr
{
    uint8_t id = 1;   // 1 byte
    int16_t pm25;    // 2 bytes ПОМЕНЯТЬ НА UINT
    int16_t tVOC;    // 2 bytes ПОМЕНЯТЬ НА UINT
    int16_t rad_qw;  // 2 bytes ПОМЕНЯТЬ НА UINT
    int16_t co2_ppm; // 2 bytes ПОМЕНЯТЬ НА UINT
    float lanGPS;     // 4 bytes
    float lonGPS;     // 4 bytes
    float humidVal;   // 4 bytes
    float temp;       // 4 bytes
    uint32_t counter; // 4 bytes
} slowData;           // 29 bytes: 29 / 250Kbps = 0.9ms - max max delay (in theory)

bool measureAcs(int16_t x, int16_t y, int16_t z);
uint16_t measureRadiation();
float measureTemp();
bool getGPSdata(float LAN, float LON);
float measurePressure();
uint16_t measureTVOC();
uint16_t getRadVal();
void radIRQ();
float getHumid(); 
boolean ds18b20_convert_t();
boolean ds18b20_read_t(float & temperatur);


void setup()
{
    SPI.begin();                                               // инициализируем работу с SPI
    SPI.setDataMode(SPI_MODE3);                                // настройка SPI
    delay(100);  
    adxl.powerOn();                     // вывод датчика из режима пониженного энергопотребления (на случай, если он был случайно включён)
    adxl.setRangeSetting(16);           // настройка чувствительности (макс - 16)
    bmp.begin();
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* режим работы      */
                  Adafruit_BMP280::SAMPLING_X2,     /* коэф. температуры */
                  Adafruit_BMP280::SAMPLING_X16,    /* коэф. давления    */
                  Adafruit_BMP280::FILTER_X16,      /* фильтр            */
                  Adafruit_BMP280::STANDBY_MS_500); /* время ожидания    */
    radio.begin();                                             // Инициируем работу nRF24L01+
    radio.setChannel(120);                                     // Указываем канал передачи данных (от 0 до 127), 5 - значит передача данных осуществляется на частоте 2,405 ГГц (на одном канале может быть только 1 приёмник и до 6 передатчиков)
    radio.setDataRate(RF24_2MBPS);                           // Указываем скорость передачи данных (RF24_250KBPS, RF24_1MBPS, RF24_2MBPS), RF24_1MBPS - 1Мбит/сек
    radio.setPALevel(RF24_PA_HIGH);                            // Указываем мощность передатчика (RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm, RF24_PA_HIGH=-6dBm, RF24_PA_MAX=0dBm)
    radio.openWritingPipe(0x1234567899LL);                     // Открываем трубу с идентификатором 0x1234567899LL для передачи данных (на одном канале может быть открыто до 6 разных труб, которые должны отличаться только последним байтом идентификатора)



}

void loop()
{


  
}



bool measureAcs(int16_t x, int16_t y, int16_t z)
{
    
    return 0;
}

uint16_t measureRadiation()
{

    
}

void radIRQ()
{

}


float measureTemp()
{

    
}


bool getGPSdata(float LAN, float LON)
{

    return 0;
}


float measurePressure()
{

    
}


uint16_t measureTVOC()
{

    
}


uint16_t getRadVal()
{

    
}


float getHumid()
{

    
}
 
boolean ds18b20_convert_t()
{
    if (!ds.reset()) // даем reset на шину
    {
      return false;
    }
    ds.write(OW_SKIP_ROM, 1);
    ds.write(OW_DS18B20_CONVERT_T, 1);
    return true;
}
boolean ds18b20_read_t(float & temperatur)
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
