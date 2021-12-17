#define PIN_IR A5
#define PIN_SERVO 10

#include <avr/wdt.h>

#include "servohandler.h"
#include "medianfilter.h"

static float irDist(short raw) {
  constexpr static float coeff[] = {7.10699063e+09, -5.38569675e+07,  1.98948099e+05, -1.35788594e+02};
  const float x = 1 / (float) raw;
  float dist = x * x * x * coeff[0] + x * x * coeff[1] + x * coeff[2] + coeff[3];
  return min(max(dist, 0), 500);
}

static float fmap(float val, float vmin, float vmax, float tmin, float tmax) {
  return (val - vmin) / (vmax - vmin) * (tmax - tmin) + tmin;
}

ServoHandler servoHandler;
MedianFilter<irDist> filter;

void setup() {
  Serial.begin(57600);
  servoHandler.init();

#if 0
  Serial.setTimeout(20);
  while(true) {
    int val = Serial.parseInt();
    if(val >= 300) {
      Serial.println(val);
      servoHandler.setMicroseconds(val);
    }
    servoHandler.update();
  }
#endif

#if 0
  while(true) {
    for(int i=1200; i <= 1800; i++) {
      servoHandler.setMicroseconds(i);
      servoHandler.update();
      delay(10);
    }
    for(int i=1800; i >= 1200; i--) {
      servoHandler.setMicroseconds(i);
      servoHandler.update();
      delay(10);
    }
  }
#endif
  
  filter.init();

#if 0
  servoHandler.setTarget(0.1f);
  while(true) {
    if(filter.ready()) {
      Serial.print("IR:");
      Serial.print(filter.read());
      Serial.print(",min:150,max:750,err:275\n");
    }
    servoHandler.update();
  }
#endif
}

void loop() {
  static constexpr int prescaler = 1000;
  static constexpr int dist_target = 255;
  static constexpr float Kp = 0.67;
  static constexpr float Kd = 36;
  static constexpr float Ki = 0.003;

  static uint16_t lastSerial = 0;
  static float dist_ema;
  static float prevError;
  static float pterm;
  static float dterm;
  static float iterm;
  static float target;
  static float error;

  if(filter.ready()) {
    float measure = filter.read();
    dist_ema = dist_ema * 0.25f + measure * 0.75f;

    error = (dist_target - dist_ema) / prescaler;

    pterm = Kp * error;
    dterm = Kd * (error - prevError);
    iterm += Ki * error;

    if(abs(dterm) > 0.4f)
      dterm = 0;
    if(abs(iterm) > 0.4f)
      iterm = 0;

    if(iterm * error < 0)
      iterm *= 0.2f;

    prevError = error;
    target = pterm + dterm + iterm;
    servoHandler.setTarget(target);
  }

  uint16_t currTime = millis();
  if(currTime - lastSerial >= 100) {
    lastSerial += 100;

    Serial.print("IR:");
    Serial.print(dist_ema);
    Serial.print(",T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(fmap(pterm, -1, 1, 510, 610));
    Serial.print(",D:");
    Serial.print(fmap(dterm, -1, 1, 510, 610));
    Serial.print(",I:");
    Serial.print(fmap(iterm, -1, 1, 510, 610));
    Serial.print(",DTT:");
    Serial.print(map(servoHandler.getTarget(), 1000, 2000, 410, 510));
    Serial.print(",DTC:");
    Serial.print(map(servoHandler.getCurr(), 1000, 2000, 410, 510));
    Serial.print(",-G:245,+G:265,m:0,M:800\n");
  }
  
  servoHandler.update();
}
