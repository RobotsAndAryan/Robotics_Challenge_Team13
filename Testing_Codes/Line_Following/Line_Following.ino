int pins[] = {2, 3, 4, 5, 6, 7, 8, 9, 10};

void setup() {
  delay(5000);
  Serial.begin(115200);
  
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  digitalWrite(11, HIGH);
  digitalWrite(12, HIGH);
}

void loop() {
  int raw[9];

  for(int i=0; i<9; i++) {
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], HIGH);
  }
  delayMicroseconds(15);

  for(int i=0; i<9; i++) {
    pinMode(pins[i], INPUT);
    raw[i] = 1000; 
  }

  long st = micros();
  while(micros() - st < 1000) {
    for(int i=0; i<9; i++) {
      if(raw[i] == 1000 && digitalRead(pins[i]) == LOW) {
        raw[i] = micros() - st;
      }
    }
  }

  for(int i=0; i<9; i++) {
    Serial.print(raw[i]);
    Serial.print("\t");
  }
  Serial.println();
  delay(100);
}