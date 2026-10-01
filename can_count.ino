#include <Wire.h>
#include "Adafruit_VL53L0X.h"

bool listening = false;
unsigned long lastUpdateTime = 0;

const char CMD_START[] = "start_listen";
const char CMD_STOP[] = "stop_listen";

const int NUM_SECTIONS = 1;
const unsigned long UPDATE_INTERVAL_MS = 1000;
const int XSHUT_PINS[NUM_SECTIONS] = { 2 };                 // unique i2c pins for each sensor
const uint8_t SENSOR_ADDRESSES[NUM_SECTIONS] = { 0x30 };    // unique I2C address for each sensor
const float ONE_CAN_DISTANCE_MM[NUM_SECTIONS] = { 495.0 };  // how much distance a can takes
const float CAN_PITCH_MM[NUM_SECTIONS] = { 61.0 };          // distance of can pitch
const float EMPTY_THRESHOLD_MM[NUM_SECTIONS] = { 520.0 };   // distance considered empty


Adafruit_VL53L0X sensors[NUM_SECTIONS];



void initSensors() {

  for (int i = 0; i < NUM_SECTIONS; i++) {
    pinMode(XSHUT_PINS[i], OUTPUT);
    digitalWrite(XSHUT_PINS[i], LOW);
  }

  delay(20);

  for (int i = 0; i < NUM_SECTIONS; i++) {
    digitalWrite(XSHUT_PINS[i], HIGH);
    delay(10);

    if (!sensors[i].begin(SENSOR_ADDRESSES[i], false, &Wire)) {
      Serial.print("{\"error\": \"sensor_");
      Serial.print(i + 1);
      Serial.println("_init_failed\"}");
    }
  }
}




int calculateCanCount(int section, uint16_t distance) {
  if (distance >= EMPTY_THRESHOLD_MM[section]) {
    return 0;
  }

  float difference = ONE_CAN_DISTANCE_MM[section] - (float)distance;
  int additionalCans = round(difference / CAN_PITCH_MM[section]);

  int count = 1 + additionalCans;

  if (count < 0) {
    count = 0;
  }

  return count;
}




void sendCanCounts() {
  Serial.print("{");

  for (int i = 0; i < NUM_SECTIONS; i++) {
    VL53L0X_RangingMeasurementData_t measure;
    sensors[i].rangingTest(&measure, false);

    int count = 0;

    if (measure.RangeStatus != 4) {
      count = calculateCanCount(i, measure.RangeMilliMeter);
    }

    Serial.print("\"section_");
    Serial.print(i + 1);
    Serial.print("\": ");
    Serial.print(count);

    if (i < NUM_SECTIONS - 1) {
      Serial.print(", ");
    }
  }

  Serial.println("}");
}




void handleSerial() {
  static String command = "";

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      command.trim();

      if (command.equals(CMD_START)) {
        Serial.println("{\"status\": \"started\"}");
        listening = true;
      } else if (command.equals(CMD_STOP)) {
        Serial.println("{\"status\": \"stopped\"}");
        listening = false;
      } else if (command.length() > 0) {
        Serial.print("{\"error\": \"unknown_command\", \"command\": \"");
        Serial.print(command);
        Serial.println("\"}");
      }

      command = "";
    } else {
      command += c;
    }
  }
}




void setup() {
  Serial.begin(115200);

  Wire.begin();

  initSensors();
}




void loop() {
  handleSerial();

  if (listening && millis() - lastUpdateTime >= UPDATE_INTERVAL_MS) {
    lastUpdateTime = millis();
    sendCanCounts();
  }
}
