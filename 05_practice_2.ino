#define PIN_LED 7

uint8_t cnt = 0;

void setup() {
  Serial.begin(115200);
  
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, false);

  while(!Serial);
  
  delay(1000);
}

void loop() {
  digitalWrite(PIN_LED, true);
  delay(100);
  digitalWrite(PIN_LED, false);
  delay(100);

  cnt++;
  if(cnt > 5) {
    digitalWrite(PIN_LED, true);
    while(true);
  }
}
