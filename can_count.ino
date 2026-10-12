#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// ============================================================
// TCA9548A CONFIG
// ============================================================

#define MUX_ADDRESS 0x70

// Sensor 0 is connected to TCA channel 2
// Sensor 1 is connected to TCA channel 3
const uint8_t SENSOR_CHANNELS[2] = { 2, 3 };

const uint8_t SENSOR_ADDRESS = 0x29;
/*
    only 7 cans can fit a column

    cans: distance
    0: 530 - 590
    1: 510 - 535
    2: 430 - 460
    3: 385 - 390
    4: 305 - 315
    5: 230 - 240
    6: 165 - 170
    7: 84 - 85
*/

#include <map>
#include <utility>

const std::map<std::pair<float, float>, int> canCountMap = {
  {{540.0, 590.0}, 0},
  {{510.0, 535.0}, 1},
  {{430.0, 460.0}, 2},
  {{385.0, 390.0}, 3},
  {{305.0, 315.0}, 4},
  {{230.0, 240.0}, 5},
  {{165.0, 170.0}, 6},
  {{84.0, 85.0},  7}
};

// ============================================================
// SERIAL / LISTENING
// ============================================================

bool listening = false;
bool listeningRaw = false;

unsigned long lastUpdateTime = 0;
unsigned long lastRawUpdateTime = 0;

const char CMD_START[] = "start";
const char CMD_STOP[] = "stop";
const char CMD_LISTEN_RAW[] = "listen_raw";


// ============================================================
// SENSOR CONFIG
// ============================================================

const int NUM_COLUMNS = 2;

const unsigned long UPDATE_INTERVAL_MS = 1000;

const float ONE_CAN_DISTANCE_MM[NUM_COLUMNS] = {
  530.0,
  530.0
};

const float CAN_PITCH_MM[NUM_COLUMNS] = {
  61.0,
  61.0
};

const float EMPTY_BAND_MIN_MM[NUM_COLUMNS] = {
  520.0,
  520.0
};

const int NUM_READINGS = 10;


// ============================================================
// VL53L0X OBJECTS
// ============================================================

Adafruit_VL53L0X sensors[NUM_COLUMNS];


// ============================================================
// TCA9548A
// ============================================================

void selectChannel(uint8_t channel) {
  Wire.beginTransmission(MUX_ADDRESS);
  Wire.write(1 << channel);
  Wire.endTransmission();

  // Give the mux a moment to switch
  delay(2);
}


// ============================================================
// CHECK SENSOR
// ============================================================

bool sensorExists(uint8_t channel) {
  selectChannel(channel);

  Wire.beginTransmission(SENSOR_ADDRESS);

  return Wire.endTransmission() == 0;
}


// ============================================================
// INITIALISE SENSORS
// ============================================================

void initSensors() {

  // Check TCA9548A first
  Wire.beginTransmission(MUX_ADDRESS);

  if (Wire.endTransmission() != 0) {
    Serial.println("{\"error\":\"tca9548a_not_found\"}");
    return;
  }

  Serial.println("{\"status\":\"tca9548a_found\"}");


  // Initialise each sensor through its own mux channel
  for (int i = 0; i < NUM_COLUMNS; i++) {

    uint8_t channel = SENSOR_CHANNELS[i];

    selectChannel(channel);

    Serial.print("{\"sensor\":");
    Serial.print(i + 1);
    Serial.print(",\"channel\":");
    Serial.print(channel);
    Serial.print(",\"address\":\"0x29\",\"found\":");

    if (sensorExists(channel)) {

      Serial.println("true}");

      // Initialise VL53L0X on this mux channel
      if (!sensors[i].begin(SENSOR_ADDRESS, false, &Wire)) {

        Serial.print("{\"error\":\"sensor_");
        Serial.print(i);
        Serial.println("_init_failed\"}");

      } else {

        Serial.print("{\"sensor\":");
        Serial.print(i);
        Serial.println(",\"status\":\"ready\"}");
      }

    } else {

      Serial.println("false}");
    }
  }
}


// ============================================================
// GET AVERAGE DISTANCE
// ============================================================

float getAverageDistance(int column) {
  selectChannel(SENSOR_CHANNELS[column]);

  const int NUM_SAMPLES = 15;
  float readings[NUM_SAMPLES];
  int validReadings = 0;

  for (int i = 0; i < NUM_SAMPLES; i++) {
    VL53L0X_RangingMeasurementData_t measure;

    sensors[column].rangingTest(&measure, false);

    // Accept only valid measurements
    if (measure.RangeStatus == 0 && measure.RangeMilliMeter > 0) {
      readings[validReadings++] =
        measure.RangeMilliMeter;
    }

    delay(10);
  }

  if (validReadings == 0) {
    return -1.0;
  }

  // Sort readings
  for (int i = 0; i < validReadings - 1; i++) {
    for (int j = i + 1; j < validReadings; j++) {
      if (readings[j] < readings[i]) {
        float temp = readings[i];
        readings[i] = readings[j];
        readings[j] = temp;
      }
    }
  }

  // Return median
  if (validReadings % 2 == 1) {
    return readings[validReadings / 2];
  }

  return (
           readings[validReadings / 2 - 1] + readings[validReadings / 2])
         / 2.0;
}


// ============================================================
// CALCULATE CAN COUNT
// ============================================================

int calculateCanCount(int column, float distance) {
  if (distance < 0) {
    return 0;
  }

  int closestCount = 0;
  float smallestDifference = 1000000.0;

  for (const auto& entry : canCountMap) {
    float midpoint =
      (entry.first.first + entry.first.second) / 2.0;

    float difference = abs(distance - midpoint);

    if (difference < smallestDifference ||
        (difference == smallestDifference &&
         entry.second > closestCount)) {
      smallestDifference = difference;
      closestCount = entry.second;
    }
  }

  return closestCount;
}


// ============================================================
// SEND CAN COUNTS (EMIT AS ONE ATOMIC LINE)
// ============================================================

void sendCanCounts() {

  String json = "[";

  for (int i = 0; i < NUM_COLUMNS; i++) {

    float avgDistance = getAverageDistance(i);

    int count = 0;

    if (avgDistance >= 0) {
      count = calculateCanCount(i, avgDistance);
    }

    if (i > 0) json += ",";

    json += "{\"channel\":";
    json += String(SENSOR_CHANNELS[i]);

    json += ",\"count\":";
    json += String(count);

    json += ",\"live_distance\":";

    if (avgDistance < 0) {
      json += "null";
    } else {
      json += String(avgDistance, 2);
    }

    json += "}";
  }

  json += "]";

  Serial.println(json);
}

// ============================================================
// SEND RAW DISTANCES (EMIT AS ONE ATOMIC LINE)
// ============================================================

void sendDistances() {

  String json = "{";

  for (int i = 0; i < NUM_COLUMNS; i++) {

    float avgDistance = getAverageDistance(i);

    if (i > 0) json += ",";

    json += "\"channel_";
    json += String(SENSOR_CHANNELS[i]);
    json += "\":";

    if (avgDistance < 0) {
      json += "null";
    } else {
      json += String(avgDistance, 2);
    }
  }

  json += "}";

  Serial.println(json);
}


// ============================================================
// SERIAL COMMAND HANDLER
// ============================================================

void handleSerial() {

  static String command = "";

  while (Serial.available()) {

    char c = Serial.read();

    if (c == '\n' || c == '\r') {

      command.trim();

      if (command.equals(CMD_START)) {

        Serial.println("{\"status\":\"started\"}");

        listening = true;

      } else if (command.equals(CMD_STOP)) {

        Serial.println("{\"status\":\"stopped\"}");

        listening = false;
        listeningRaw = false;

      } else if (command.equals(CMD_LISTEN_RAW)) {

        Serial.println("{\"status\":\"raw_started\"}");

        listeningRaw = true;

      } else if (command.length() > 0) {

        Serial.print("{\"error\":\"unknown_command\",\"command\":\"");
        Serial.print(command);
        Serial.println("\"}");
      }

      command = "";

    } else {

      command += c;
    }
  }
}


// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  Wire.begin();

  delay(100);

  initSensors();
}


// ============================================================
// LOOP
// ============================================================

void loop() {

  handleSerial();


  // Normal can-count output
  if (listening && millis() - lastUpdateTime >= UPDATE_INTERVAL_MS) {

    lastUpdateTime = millis();

    sendCanCounts();
  }


  // Raw distance output
  if (listeningRaw && millis() - lastRawUpdateTime >= UPDATE_INTERVAL_MS) {

    lastRawUpdateTime = millis();

    sendDistances();
  }
}