/*
  Analog input, analog output, serial output

  Reads an analog input pin, maps the result to a range from 0 to 255 and uses
  the result to set the pulse width modulation (PWM) of an output pin.
  Also prints the results to the Serial Monitor.

  The circuit:
  - potentiometer connected to analog pin 0.
    Center pin of the potentiometer goes to the analog pin.
    side pins of the potentiometer go to +3.3V and ground
  - LED connected from digital pin 9 to ground

  created 29 Dec. 2008
  modified 9 Apr 2012
  by Tom Igoe
  modified 28 Feb 2017 for use with sduino
  by Michael Mayer

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/AnalogInOutSerial
*/

// These constants won't change. They're used to give names to the pins used:
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
//const int analogOutPin = LED_BUILTIN; // Analog output pin that the LED is attached to

int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)
uint32_t tim;
void setup() {
  // initialize serial communications at 9600 bps:
  pinMode(A0, INPUT);
  pinMode(5, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(15, OUTPUT);
  
  
}

void loop() {
  int32_t rec = analogRead(A0);
  if(rec >=800)
  {
    rec+= analogRead(A0);
    rec+= analogRead(A0);
    rec+= analogRead(A0);
    rec+= analogRead(A0);
    rec+= analogRead(A0);
    rec+= analogRead(A0);
    rec+= analogRead(A0);
    rec >>= 4;
    uint8_t rec_byte = rec;
    digitalWrite(7, bitRead(rec_byte,7));
    digitalWrite(8, bitRead(rec_byte,6));
    digitalWrite(9, bitRead(rec_byte,5));
    digitalWrite(11, bitRead(rec_byte,4));

    digitalWrite(12, bitRead(rec_byte,3));
    digitalWrite(13, bitRead(rec_byte,2));
    digitalWrite(14, bitRead(rec_byte,1));
    digitalWrite(15, bitRead(rec_byte,0));
    digitalWrite(5, 1);
    delayMicroseconds(80);
    digitalWrite(5, 0);
    
  }
}
