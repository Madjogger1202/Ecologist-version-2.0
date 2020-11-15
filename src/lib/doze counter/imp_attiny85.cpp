#include <Arduino.h>
#include "SoftwareSerial.h"

SoftwareSerial dataPort(PB0, PB1);

uint16_t energy;

void setup()
{
    
    DDRB &=~(1<<2); // ставим PB4 на вход 
    DDRB |= 1<<2;   // в выход PB2
    ADCSRA |= ((1 << ADPS2) | (1 << ADPS0));  //Биту ADPS2 и 0 присваиваем единицу - коэффициент деления 32
    ADCSRA &= ~ (1 << ADPS1); // теперь имеем число 101 в регистре ADPS        
    dataPort.begin(9600);
}

void loop()
{
    int recent = analogRead(A2);
    if (recent >= 10)
    {
        energy = recent;
        energy += analogRead(A2);
        energy += analogRead(A2);
        energy += analogRead(A2);
        energy /= 4;
        PORTB |= 1<<2;
        dataPort.write(energy);
        PORTB &= ~(1<<2);
    } 
}
