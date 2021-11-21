#define PIN_IR A5
#define PIN_SERVO 10
#define PIN_RESET 4

#include <avr/wdt.h>

#include "servohandler.h"
#include "medianfilter.h"
#include "distfilter.h"

static float sensorToDist(short raw) {
  //return 4.8f / (value / 1024.0f - 0.02f) + 0.25f;
  constexpr static float coeff[] = {2.56189235e+06, -4.66124879e+03,  9.35303775e+00};
  const float x = 1.0f / raw;
  return x * x * coeff[0] + x * coeff[1] + coeff[2];
}

ServoHandler servoHandler;
MedianFilter<sensorToDist> medianFilter;
DistFilter distFilter;

// Enable watchdog and wait for it to expire
void reset() {
  wdt_enable(WDTO_15MS);
  while(true);
}

void setup() {
  pinMode(PIN_RESET, INPUT_PULLUP);
  
  Serial.begin(57600);
  servoHandler.init();

  // Keep servo at neutral position for calibration
  while(digitalRead(PIN_RESET) == LOW);
  
  medianFilter.init();
  distFilter.init();
}

void loop() {
  if(digitalRead(PIN_RESET) == LOW)
    reset();
  
  if(medianFilter.ready()) {
    distFilter.push(medianFilter.read());
  }
  
  float pos = distFilter.calc();
  if(pos < 25.5f)
    servoHandler.setTarget(1);
  else
    servoHandler.setTarget(-1);

  servoHandler.update();
}
