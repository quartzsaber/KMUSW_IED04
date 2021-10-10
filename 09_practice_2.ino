// Arduino pin assignment
#define PIN_LED 9
#define PIN_TRIG 12
#define PIN_ECHO 13

// configurable parameters
#define SND_VEL 346.0 // sound velocity at 24 celsius degree (unit: m/s)
#define INTERVAL 25 // sampling interval (unit: ms)
#define _DIST_MIN 100 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300 // maximum distance to be measured (unit: mm)
#define MEDIAN_A_LEN 3
#define MEDIAN_B_LEN 10
#define MEDIAN_C_LEN 30
#define STRINGIFIER(X) #X
#define STRINGIFY(X) STRINGIFIER(X)

// NOT copied from the internet :)
// Coded by Yeomin Yoon (20213030)

// A plain swap function
void swap(float& a, float& b) {
  float tmp = a;
  a = b;
  b = tmp;
}

// Inplace quickselect to select target-th largest element
// Returns the target-th largest element
// Post condition: arr[target] == target-th largest element
float qselect(float* arr, uint16_t target, uint16_t begin, uint16_t end) {
  // Overflow-free average (only works when begin <= end)
  uint16_t mid = begin + ((end - begin) / 2);

  // Select pivot and move it to back
  const float pivot = arr[mid];
  swap(arr[mid], arr[end - 1]);

  // Partition the array
  uint16_t store = begin;
  for(uint16_t i = begin; i < end - 1; i++) {
    if(arr[i] < pivot) {
      swap(arr[store], arr[i]);
      store++;
    }
  }

  // Move pivot to right place
  swap(arr[store], arr[end - 1]);

  // Tail optimization. yay.
  if(store == target)
    return arr[store];
  else if(store < target)
    return qselect(arr, target, store + 1, end);
  else
    return qselect(arr, target, begin, store);
}

// This also is not a copy
// Using template may generate better code due to constant optimization
template<uint16_t SIZE>
class MedianStorage {
public:
  uint16_t idx;
  uint16_t len;
  float arr[SIZE];

  MedianStorage() {
    idx = 0;
    len = 0;
    // No need to initialize arr
  }

  // Push a new value into ring buffer
  void push(float val) {
    arr[idx] = val;
    idx = (idx + 1) % SIZE;
    if(len < SIZE)
      len++;
  }

  // Check if all entries of ring buffer if initialized
  bool ready() const {
    return len >= SIZE;
  }

  // Calculate median value. Assumes ready() is true
  // Time complexity: O(n) on average
  float calc() const {
    // First copy the internal state
    float copied[SIZE];
    memcpy(copied, arr, sizeof(arr));

    // Then perform inplace quickselect to find median
    if(SIZE % 2 == 1)
      return qselect(copied, SIZE / 2, 0, SIZE);
    else {
      // Second quickselect may be replaced with a find-max-loop
      // ...which would be faster on a platform with memory prefetcher
      float a = qselect(copied, SIZE / 2, 0, SIZE);
      float b = qselect(copied, SIZE / 2 - 1, 0, SIZE / 2);
      return (a + b) / 2;
    }
  }
};

// global variables
float timeout; // unit: us
float dist_min, dist_max, dist_raw, dist_a, dist_b, dist_c; // unit: mm
unsigned long last_sampling_time; // unit: ms
float scale; // used for pulse duration to distance conversion
MedianStorage<MEDIAN_A_LEN> medianA;
MedianStorage<MEDIAN_B_LEN> medianB;
MedianStorage<MEDIAN_C_LEN> medianC;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO,INPUT);

// initialize USS related variables
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  timeout = (INTERVAL / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  dist_raw = 0.0; // raw distance output from USS (unit: mm)
  dist_a = 0.0;
  dist_b = 0.0;
  dist_c = 0.0;
  scale = 0.001 * 0.5 * SND_VEL;

// initialize serial port
  Serial.begin(230400);

// initialize last sampling time
  last_sampling_time = 0;
}

void loop() {
// wait until next sampling time. 
// millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  if(millis() < last_sampling_time + INTERVAL) return;

// get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG,PIN_ECHO);
  medianA.push(dist_raw);
  medianB.push(dist_raw);
  medianC.push(dist_raw);
  if(medianA.ready())
    dist_a = medianA.calc();
  if(medianB.ready())
    dist_b = medianB.calc();
  if(medianC.ready())
    dist_c = medianC.calc();

// output the read value to the serial port
  Serial.print("Min:0,raw:");
  Serial.print(dist_raw);
  Serial.print(",med" STRINGIFY(MEDIAN_A_LEN) ":");
  Serial.print(dist_a);
  Serial.print(",med" STRINGIFY(MEDIAN_B_LEN) ":");
  Serial.print(dist_b);
  Serial.print(",med" STRINGIFY(MEDIAN_C_LEN) ":");
  Serial.print(dist_c);
//  Serial.print(map(dist_ema,0,400,100,500));
  Serial.println(",Max:500");

// turn on the LED if the distance is between dist_min and dist_max
  if(dist_raw < dist_min || dist_raw > dist_max) {
    analogWrite(PIN_LED, 255);
  }
  else {
    analogWrite(PIN_LED, 0);
  }

// update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  float reading;
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  reading = pulseIn(ECHO, HIGH, timeout) * scale; // unit: mm
  if(reading < dist_min || reading > dist_max) reading = 0.0; // return 0 when out of range.
  return reading;
}
