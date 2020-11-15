#include <Arduino.h>
#include "SoftwareSerial.h"

SoftwareSerial dataPort(0, 3);

uint16_t energy[200];
uint8_t counter;

void setup()
{
  dataPort.begin(9600);
}

void loop()
{
    int recent = analogRead(A2);
    if (recent >= 10)
    {
        energy[counter] = recent;
        energy[counter] += analogRead(A2);
        energy[counter] += analogRead(A2);
        energy[counter] += analogRead(A2);
        energy[counter] /= 4;
        counter++;
    
   
    } 
    if (dataPort.available())
    {
        if (dataPort.read() == 0xFF)
        {
            dataPort.write(counter);
            for (int i = 0; i < counter; i++)
            {
                dataPort.write(energy[i]);
            }
        } 
    }
}
