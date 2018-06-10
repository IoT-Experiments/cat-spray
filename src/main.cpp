#include <Arduino.h>
#include <Stepper.h>
#include <Ultrasonic.h>

#define REF_MEASURE_COUNT 30
#define DISTANCE_THRESHOLD_PERCENT 0.90
#define THRESHOLD_MAX_DURATION 2000
#define TIME_BETWEEN_READS 200
#define TIME_BETWEEN_SPRAY 10000
#define PIN_SWITCH 8

#define PIN_STEPPER_INPUT_1 2
#define PIN_STEPPER_INPUT_2 3
#define PIN_STEPPER_INPUT_3 4
#define PIN_STEPPER_INPUT_4 5
#define PIN_ULTRASONIC_ECHO 6
#define PIN_ULTRASONIC_TRIG 7

// The gear reduction ratio is approximately 64:1 (tested: 63.68395:1)
// 64 steps per motor rotation x 63.684 gear ratio = 4076 steps per full revolution (approximately)
const int stepsPerRevolution = 4076;

Stepper stepper(stepsPerRevolution,
  PIN_STEPPER_INPUT_1, PIN_STEPPER_INPUT_2,
  PIN_STEPPER_INPUT_3, PIN_STEPPER_INPUT_4);
Ultrasonic ultrasonic(PIN_ULTRASONIC_ECHO, PIN_ULTRASONIC_TRIG);

unsigned int refMeasure = 0;
unsigned long thresholdReachedTimestamp = 0;
unsigned long lastReadTimestamp = 0;
unsigned long lastSprayTimestamp = 0;
bool sprayActive = false;
bool checkSwitch = false;

void powerOffStepper();

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
    digitalWrite(LED_BUILTIN, HIGH);
    unsigned int distance = ultrasonic.distanceRead();
    if(distance != 0) {
      distanceSum += ultrasonic.distanceRead();
      nbMeasures++;
    }
    digitalWrite(LED_BUILTIN, LOW);
  }
  refMeasure = (int)round(distanceSum / REF_MEASURE_COUNT);
  Serial.print("Ref distance value: ");
  Serial.println(refMeasure);
}

void loop() {
  if(!sprayActive) {
    // - Let's avoid any echo from the previous reading
    if(millis() - lastReadTimestamp > TIME_BETWEEN_READS && millis() - lastSprayTimestamp > TIME_BETWEEN_SPRAY) {
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
      lastSprayTimestamp = millis();
      checkSwitch = false;
      sprayActive = false;
      thresholdReachedTimestamp = 0;
      powerOffStepper();
      Serial.println("Spray inactive");
    }
  }
}

void powerOffStepper() {
  digitalWrite(PIN_STEPPER_INPUT_1, LOW);
  digitalWrite(PIN_STEPPER_INPUT_2, LOW);
  digitalWrite(PIN_STEPPER_INPUT_3, LOW);
  digitalWrite(PIN_STEPPER_INPUT_4, LOW);
}
