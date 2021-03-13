#include <Arduino.h>

#define BMP_CS  41
#define ADXL_CS  40
#define NRF_CS  39
#define NRF_CE  38
#define LORA_CS  37
#define LORA_INT 36
#define SD_CS  35

#define GP2Y_PIN A0

struct frstSrt
{
    uint8_t id = 0;   // 1 byte
    int16_t x_acs;    // 2 bytes
    int16_t y_acs;    // 2 bytes
    int16_t z_acs;    // 2 bytes
    float presssure;  // 4 bytes
    uint32_t counter; // 4 bytes
}fastData;            // 15 bytes: 15 / 250Kbps = 0.4 ms - min max delay 

struct secStr
{
    uint8_t id = 1;   // 1 byte
    uint16_t pm25;    // 2 bytes
    uint16_t tVOC;    // 2 bytes
    uint16_t rad_qw;  // 2 bytes
    uint16_t co2_ppm; // 2 bytes
    float lanGPS;     // 4 bytes
    float lonGPS;     // 4 bytes
    float humidVal;   // 4 bytes
    float temp;       // 4 bytes
    uint32_t counter; // 4 bytes
} slowData;           // 29 bytes: 29 / 250Kbps = 0.9ms - max max delay

bool measureAcs(int16_t x, int16_t y, int16_t z);
uint16_t measureRadiation();
float measureTemp();
bool getGPSdata(float LAN, float LON);
float measurePressure();
uint16_t measureTVOC();
uint16_t getRadVal();
float getHumid(); 

void setup()
{


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
 