#define PIN_LED 13

uint16_t count, toggle;

void setup() {
  Serial.begin(115200);
  
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, toggle);

  while(!Serial);
}

void loop() {
  Serial.println(++count);
  toggle = toggle_state(toggle);
  digitalWrite(PIN_LED, toggle);
  delay(1000);
}

uint16_t toggle_state(uint16_t toggle) {
  return !toggle;
}
