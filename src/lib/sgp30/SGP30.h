/*
    This is library for SGP30 sensor

    *RU*
    датчик находится в режиме калибровки первые 15 сек после включения (инициализации)
*/

#define SENSOR_ADRESS 0x58
#define typ_delay 10

#include <Wire.h>

bool writeCommand(uint8_t first, uint8_t second);
bool sensorBegin();
uint8_t getData(uint16_t &tVOC);
bool countCRC(uint8_t frst, uint8_t second, uint8_t crc);

bool sensorBegin()
{
    Wire.begin(SENSOR_ADRESS);
    writeCommand(0x20, 0x03);
    delay(typ_delay*2);
    return 0;
}
//

uint8_t getData(uint16_t &tVOC)        // получаем значения tVOC, остальное выкидываем (+проверка CRC)
{
    writeCommand(0x20, 0x08);              // пишем составную команду 0x2008 - триггер запуска замера
    uint8_t recieve[6];                    // весь буфер, не выкидывать же всё
    uint8_t counter = 0;                   // костыльная замена циклу for
    Wire.requestFrom(SENSOR_ADRESS, 6);    // запрашиваем 6 байт с датчика
    if(Wire.available()!=6)                // 6 ли байт нам пришло
        return 2;
    while(Wire.available())   // берём 6 байт (пока что есть в буфере и кол-во байт <=6 )
    {
        recieve[counter] = Wire.read();
        counter++;
    }
    if(!countCRC(recieve[3], recieve[4], recieve[5]))   // проверяем контрольную сумму, если она норм - пишем значение
    {
        tVOC = 0;               //
	    tVOC = recieve[3];      // вроде отлажено, по крайней мере из 0x20 и 0x33 0x2033 склеивает
	    tVOC <<= 8;             //
	    tVOC |= recieve[4];     //
        return 0;
    }
    else
    {
        return 1;               // уходим, ничего не делая 
    }
    

}
//

// подсчёт контрольной суммы 
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

bool writeCommand(uint8_t first, uint8_t second)   // должно работать без проблем, а как на практике - проверю когда датчик приедет уже
{
    Wire.beginTransmission(SENSOR_ADRESS);
    Wire.write(first);
    Wire.write(second);
    Wire.endTransmission();  
    return 0;
}