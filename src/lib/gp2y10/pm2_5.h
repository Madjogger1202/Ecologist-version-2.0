#include <Arduino.h>

uint16_t voltage;
uint16_t PMval;

uint16_t measureTime;
uint8_t measureTimes;
float coefPM;


void begin();
void measure();
void setCoef();
void setMeasureTimes();


void begin(int pin)
{
    pinMode(pin, OUTPUT);
    return;
}

void measure()
{
    for(int i = 0; i<= 8)
    {
        
    }
}