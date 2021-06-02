#include <Arduino.h> 
 
#define PPM_COEF 5000

long int getCO2Data();
void attach_ints(int for_low, int for_high);
volatile long long timer_high;
volatile long long timer_low;
void low_sub();
void high_sub();


void attach_ints(int for_low, int for_high)
{
    attachInterrupt(digitalPinToInterrupt(for_low), low_sub, FALLING);
}

long int getCO2Data()
{ 
    timer_low /= 1000;
    timer_high = 1004 - timer_low;
    timer_low = PPM_COEF*(timer_low - 2)/(timer_low + timer_high - 4);
    return timer_low;
}

void low_sub()
{
    timer_low = micros()-timer_high;
}

void high_sub()
{
    timer_high = micros();
}
