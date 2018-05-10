#include <Arduino.h>
#include <Stepper.h>
#include <Ultrasonic.h>

#define REF_MEASURE_COUNT 30
#define DISTANCE_THRESHOLD_PERCENT 0.90
#define THRESHOLD_MAX_DURATION 2000
#define TIME_BETWEEN_READS 200
#define PIN_SWITCH 8

// The gear reduction ratio is approximately 64:1 (tested: 63.68395:1)
// 64 steps per motor rotation x 63.684 gear ratio = 4076 steps per full revolution (approximately)
const int stepsPerRevolution = 4076;

Stepper stepper(stepsPerRevolution, 2, 3, 4, 5);
Ultrasonic ultrasonic(6, 7);

unsigned int refMeasure = 0;
unsigned long thresholdReachedTimestamp = 0;
unsigned long lastReadTimestamp = 0;
bool sprayActive = false;
bool checkSwitch = false;

void setup() {
  Serial.begin(115200);

  pinMode(PIN_SWITCH, INPUT_PULLUP);

  stepper.setSpeed(7);

  // Assumes that the spray cap is not pushed

  // Mesures the distance 5 times, as reference
  unsigned int nbMeasures = 0;
  unsigned int distanceSum = 0;
  while(nbMeasures < REF_MEASURE_COUNT) {
    delay(TIME_BETWEEN_READS);
    unsigned int distance = ultrasonic.distanceRead();
    if(distance != 0) {
      distanceSum += ultrasonic.distanceRead();
      nbMeasures++;
    }
  }
  refMeasure = (int)round(distanceSum / REF_MEASURE_COUNT);
  Serial.print("Ref distance value: ");
  Serial.println(refMeasure);
}

void loop() {
  if(!sprayActive) {
    // - Let's avoid any echo from the previous reading
    if(millis() - lastReadTimestamp > TIME_BETWEEN_READS) {
      lastReadTimestamp = millis();

      // - If the distance is less than 90%, the threshold is reached
      unsigned int distance = ultrasonic.distanceRead();
      if(distance != 0 && distance < refMeasure * DISTANCE_THRESHOLD_PERCENT) {
        if(thresholdReachedTimestamp == 0) {
          thresholdReachedTimestamp = millis();
          Serial.print("Threshold with distance: ");
          Serial.println(distance);
        } else if(millis() - thresholdReachedTimestamp >= THRESHOLD_MAX_DURATION) {
          // If after few seconds, the distance is still not closed to the reference, spray 1 time
          if(!sprayActive) {
            Serial.println("Spray active");
            sprayActive = true;
          }
        }
      } else if(thresholdReachedTimestamp != 0) {
        thresholdReachedTimestamp = 0;
        Serial.println("Latched");
      }
    }

    // Active built-in led when threshold reached
    digitalWrite(LED_BUILTIN, thresholdReachedTimestamp != 0 ? HIGH : LOW);
  } else {
    // step 1/100 of a revolution:
    stepper.step(stepsPerRevolution / 100);

    if(!checkSwitch && digitalRead(PIN_SWITCH) == HIGH) {
      checkSwitch = true;
    } else if(checkSwitch && digitalRead(PIN_SWITCH) == LOW) {
      checkSwitch = false;
      sprayActive = false;
      thresholdReachedTimestamp = 0;
      Serial.println("Spray inactive");
    }
  }
}
