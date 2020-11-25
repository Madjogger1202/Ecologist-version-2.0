
void setup() {
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
  if(rec >=800) // тут ставится порог отсчёта
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
