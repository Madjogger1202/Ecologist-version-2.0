/*
    This is library for SGP30 sensor

    *RU*
    датчик находится в режиме калибровки первые 15 сек после включения (инициализации)
*/

#define _SENSOR_ADRESS 0x58
#define _rb 4

#include <Wire.h>

bool writeCommand(uint8_t first, uint8_t second);
bool sensorBegin();
bool getData();
bool countCRC(uint8_t frst, uint8_t second, uint8_t crc);

bool sensorBegin()
{
    Wire.begin(_SENSOR_ADRESS);
    writeCommand(0x20, 0x03);
    delay(10);
    return 0;
}
//

bool getData(uint8_t &tVOC)
{
    long tm = millis();
    writeCommand(0x20, 0x08);
    while(!Wire.available())
    {
        if(millis()>=(tm+30))
        return 1;
    }
    uint8_t recieve[6];
    uint8_t counter;
    Wire.requestFrom(_SENSOR_ADRESS, 6);
    delay(1);
    while(Wire.available())
    {
        recieve[counter] = Wire.read();
        counter++;
    }
    if(!countCRC(recieve[0], recieve[1], recieve[2]))
    {
        tVOC = recieve[1];
        return 0;
    }
    else
    {
        return 1;
    }
    

}
//

// алгоритм для контрольной суммы взял из библиотеки adafruit, не до конца понял как они в даташите её предлагают считать 
bool countCRC(uint8_t frst, uint8_t second, uint8_t crc)
{
    uint8_t right_crc;
    uint8_t data[2] = {frst, second};   
    for (uint8_t i = 0; i < 2; i++) 
    {
        right_crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) 
        {
            if (right_crc & 0x80)
                right_crc = (right_crc << 1) ^ 0x31;
            else
                right_crc <<= 1;
        }
     }
    if(right_crc == crc)
        return 0;
    else
        return 1;
}

bool writeCommand(uint8_t first, uint8_t second)
{
    Wire.beginTransmission(_SENSOR_ADRESS);
    Wire.write(first);
    Wire.write(second);
    Wire.endTransmission();  
    return 0;
}