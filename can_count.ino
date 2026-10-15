/*
  ESP32 - PLUG AND PLAY DISTANCE SYSTEM
  can connect 64 VL53L1X sensors maximum using TCA9548A multiplexers at addresses 0x70–0x77.

  this sends data over serial looking like:
  [{"channel":1,"live_distance":548.00},{"channel":2,"live_distance":503.00}]
  [{"channel":1,"live_distance":543.00},{"channel":2,"live_distance":501.00}]
  [{"channel":1,"live_distance":547.00},{"channel":2,"live_distance":501.00}]

  the frontend will then get the data and identify what was taken out based on distance differences using TS code
*/

#include <Wire.h>
#include <VL53L1X.h>

#define I2C_SDA 21
#define I2C_SCL 22

#define SENSOR_ADDRESS 0x29

const uint8_t MUX_MIN_ADDRESS = 0x70;
const uint8_t MUX_MAX_ADDRESS = 0x77;

// for each 0 - 7 show shelf number -> row_id : 0 == 1
const int CHANNELS_PER_MULTIPLEXER = 8;
const int NUM_OF_MULTIPLEXERS = 8;

// get the maximum number of channels per multiplexers
const int MAX_SENSORS = CHANNELS_PER_MULTIPLEXER * NUM_OF_MULTIPLEXERS;

const unsigned long UPDATE_INTERVAL_MS = 1000;

struct SensorInfo {
  uint8_t muxAddress;
  uint8_t channel;
  bool ready;
};

SensorInfo sensorInfo[MAX_SENSORS];
VL53L1X sensors[MAX_SENSORS];

int sensorCount = 0;

bool listening = false;
unsigned long lastUpdateTime = 0;

const char CMD_START[] = "start";
const char CMD_STOP[] = "stop";


bool muxExists(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}


void selectChannel(uint8_t muxAddress, uint8_t channel) {
  Wire.beginTransmission(muxAddress);
  Wire.write(1 << channel);
  Wire.endTransmission();

  delay(2);
}


bool sensorExists(uint8_t muxAddress, uint8_t channel) {
  selectChannel(muxAddress, channel);

  Wire.beginTransmission(SENSOR_ADDRESS);
  return Wire.endTransmission() == 0;
}


void initSensors() {
  for (uint8_t mux = MUX_MIN_ADDRESS;
       mux <= MUX_MAX_ADDRESS;
       mux++) {

    if (!muxExists(mux)) {
      continue;
    }

    Serial.printf(
      "{\"status\":\"mux_found\",\"mux\":%u}\n",
      mux);

    for (uint8_t channel = 0;
         channel < CHANNELS_PER_MULTIPLEXER;
         channel++) {

      if (sensorCount >= MAX_SENSORS) {
        Serial.println("{\"status\":\"sensor_limit_reached\"}");
        return;
      }

      if (!sensorExists(mux, channel)) {
        continue;
      }

      int index = sensorCount;

      sensors[index].setBus(&Wire);
      sensors[index].setTimeout(500);

      if (!sensors[index].init()) {
        Serial.printf(
          "{\"error\":\"sensor_init_failed\",\"mux\":%u,\"channel\":%u}\n",
          mux, channel);
        continue;
      }

      sensors[index].setDistanceMode(VL53L1X::Long);
      sensors[index].startContinuous(50);

      sensorInfo[index].muxAddress = mux;
      sensorInfo[index].channel = channel;
      sensorInfo[index].ready = true;

      sensorCount++;

      Serial.printf(
        "{\"status\":\"sensor_ready\",\"mux\":%u,\"channel\":%u}\n",
        mux, channel);
    }
  }

  Serial.printf(
    "{\"status\":\"scan_complete\",\"sensors_found\":%d}\n",
    sensorCount);
}


float getLiveDistance(int index) {
  if (index < 0 || index >= sensorCount) {
    return -1.0;
  }

  if (!sensorInfo[index].ready) {
    return -1.0;
  }

  uint8_t mux = sensorInfo[index].muxAddress;
  uint8_t channel = sensorInfo[index].channel;

  selectChannel(mux, channel);

  uint16_t distance = sensors[index].read();

  bool timedOut = sensors[index].timeoutOccurred();

  bool validStatus =
    sensors[index].ranging_data.range_status
    == VL53L1X::RangeValid;

  if (timedOut || !validStatus || distance == 0) {
    return -1.0;
  }

  return distance;
}


void sendDistances() {
  Serial.print("[");

  for (int i = 0; i < sensorCount; i++) {
    if (i > 0) {
      Serial.print(",");
    }

    float distance = getLiveDistance(i);

    int row_id = sensorInfo[i].muxAddress - MUX_MIN_ADDRESS;

    Serial.print("{\"row_id\":");
    Serial.print(row_id);

    Serial.print(",\"channel\":");
    Serial.print(sensorInfo[i].channel);

    Serial.print(",\"live_distance\":");

    if (distance < 0) {
      Serial.print("null");
    } else {
      Serial.print(distance, 2);
    }

    Serial.print("}");
  }

  Serial.println("]");
}


void handleSerial() {
  static String command = "";

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      command.trim();

      if (command.equals(CMD_START)) {
        listening = true;
        lastUpdateTime = millis();

        Serial.println("{\"status\":\"started\"}");

      } else if (command.equals(CMD_STOP)) {
        listening = false;

        Serial.println("{\"status\":\"stopped\"}");

      } else if (command.length() > 0) {
        Serial.println("{\"error\":\"unknown_command\"}");
      }

      command = "";

    } else if (command.length() < 100) {
      command += c;
    }
  }
}


void setup() {
  Serial.begin(115200);

  Wire.begin(I2C_SDA, I2C_SCL);

  delay(100);

  initSensors();
}


void loop() {
  handleSerial();

  unsigned long now = millis();

  if (listening && now - lastUpdateTime >= UPDATE_INTERVAL_MS) {

    lastUpdateTime = now;

    sendDistances();
  }
}
