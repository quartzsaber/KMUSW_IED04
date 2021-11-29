#define PIN_IR A5
#define PIN_SERVO 10
#define PIN_RESET 4

#include <avr/wdt.h>

#include "servohandler.h"
#include "medianfilter.h"

static float irDist(short raw) {
  constexpr static float coeff[] = {-7.22652450e+09,  8.02403089e+07, -1.84167372e+05,  2.09313908e+02};
  const float x = 1 / (float) raw;
  return x * x * x * coeff[0] + x * x * coeff[1] + x * coeff[2] + coeff[3];
}

ServoHandler servoHandler;
MedianFilter<irDist> medianFilter;

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
}

void loop() {
  static constexpr int prescaler = 1000;
  //static constexpr float Kp = 0.77f;
  //static constexpr float Kp = 0.925f;
  static constexpr float Kp = 0.91f;

  if(digitalRead(PIN_RESET) == LOW)
    reset();

  if(medianFilter.ready()) {
    float pos = medianFilter.read();
    float error = (255 - pos) / prescaler;

    float target = Kp * error;

    // Maps -1...1 range to -8...8 deg
    servoHandler.setTarget(target);

    Serial.print("min:0,max:500,target:255,dist:");
    Serial.println(pos);
  }
  servoHandler.update();
}
