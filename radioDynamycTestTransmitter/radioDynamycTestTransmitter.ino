#include <SPI.h>
#include "RF24.h"

RF24 radio(8, 9);

uint32_t radTim=3000;

struct frst
{
  const uint8_t id = 0;
  uint32_t timeMs;
  uint16_t trsh;
  
} first;

struct sec
{
  const uint8_t id = 1;
  uint16_t trsh;
  
} secon;


void setup() {
  radio.begin();
  radio.enableDynamicPayloads();
  radio.setAutoAck( true ) ;
      radio.setChannel(100);                                // Указываем канал приёма данных (от 0 до 127), 5 - значит приём данных осуществляется на частоте 2,405 ГГц (на одном канале может быть только 1 приёмник и до 6 передатчиков)
  radio.setDataRate     (RF24_250KBPS);                   // Указываем скорость передачи данных (RF24_250KBPS, RF24_1MBPS, RF24_2MBPS), RF24_1MBPS - 1Мбит/сек
  radio.setPALevel      (RF24_PA_MAX); 
  radio.openWritingPipe(0xEEFAFDFDEELL);
  radio.powerUp() ;
  
}

void loop() {
  // put your main code here, to run repeatedly:
  if(radTim < millis())
  {
    first.timeMs = millis();
    radio.write( &first, sizeof(first));
    radTim = millis()+300;    
  }
  secon.trsh = millis()/1000;
  radio.write( &secon, sizeof(secon));
  delay(100);

  
}
