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

static float fmap(float val, float vmin, float vmax, float tmin, float tmax) {
  return (val - vmin) / (vmax - vmin) * (tmax - tmin) + tmin;
}

ServoHandler servoHandler;
MedianFilter<irDist> filter;

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
  
  filter.init();
}

void loop() {
  static constexpr int prescaler = 1000;
  static constexpr float Kp = 2.5f;
  static constexpr float Kd = 40.0f;

  static float prevError = 0;

  if(digitalRead(PIN_RESET) == LOW)
    reset();

  if(filter.ready()) {
    float dist = filter.read();
    float error = (255 - dist) / prescaler;

    float pterm = Kp * error;
    float dterm = Kd * (error - prevError);

    prevError = error;
    float target = pterm + dterm;
    servoHandler.setTarget(target);

    Serial.print("dist:");
    Serial.print(dist);
    Serial.print(",pterm:");
    Serial.print(fmap(pterm, -1, 1, 510, 610));
    Serial.print(",dterm:");
    Serial.print(fmap(dterm, -1, 1, 510, 610));
    Serial.print(",duty_target:");
    Serial.print(map(servoHandler.getTarget(), 1000, 2000, 410, 510));
    Serial.print(",duty_curr:");
    Serial.print(map(servoHandler.getCurr(), 1000, 2000, 410, 510));
    Serial.print(",min:100,low:200,dist_target:255,high:310,max:410\n");
  }
  servoHandler.update();
}
