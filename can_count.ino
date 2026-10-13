#include <Wire.h>
#include <VL53L1X.h>
#include <map>
#include <utility>
#include <math.h>

// ============================================================
// TCA9548A CONFIG
// ============================================================

#define MUX_ADDRESS 0x70

// Sensor 0 -> TCA channel 2
// Sensor 1 -> TCA channel 3
const uint8_t SENSOR_CHANNELS[2] = {2, 3};
const uint8_t SENSOR_ADDRESS = 0x29;

// ============================================================
// CAN COUNT MAP
// ============================================================

// for the older sensor
// const std::map<std::pair<float, float>, int> canCountMap = {
//     {{540.0, 590.0}, 0},
//     {{510.0, 535.0}, 1},
//     {{430.0, 460.0}, 2},
//     {{385.0, 390.0}, 3},
//     {{305.0, 315.0}, 4},
//     {{230.0, 240.0}, 5},
//     {{165.0, 170.0}, 6},
//     {{84.0, 85.0},  7}
// };

// for the new sensor:
const std::map<std::pair<float, float>, int> canCountMap = {
    {{500.0, 1000.0}, 0},
    {{450.0, 470.0},  1},
    {{390.0, 410.0},  2},
    {{335.0, 350.0},  3},
    {{270.0, 280.0},  4},
    {{230.0, 240.0},  5},
    {{125.0, 130.0},  6},
    {{60.0, 70.0},    7}
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
    530.0, 530.0
};

const float CAN_PITCH_MM[NUM_COLUMNS] = {
    61.0, 61.0
};

const float EMPTY_BAND_MIN_MM[NUM_COLUMNS] = {
    520.0, 520.0
};

const int NUM_READINGS = 10;

// ============================================================
// VL53L1X OBJECTS
// ============================================================

VL53L1X sensors[NUM_COLUMNS];

bool sensorReady[NUM_COLUMNS] = {false, false};

// ============================================================
// TCA9548A
// ============================================================

void selectChannel(uint8_t channel) {
    Wire.beginTransmission(MUX_ADDRESS);
    Wire.write(1 << channel);
    Wire.endTransmission();

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
    Wire.beginTransmission(MUX_ADDRESS);

    if (Wire.endTransmission() != 0) {
        Serial.println("{\"error\":\"tca9548a_not_found\"}");
        return;
    }

    Serial.println("{\"status\":\"tca9548a_found\"}");

    for (int i = 0; i < NUM_COLUMNS; i++) {
        uint8_t channel = SENSOR_CHANNELS[i];

        selectChannel(channel);

        Serial.print("{\"sensor\":");
        Serial.print(i + 1);
        Serial.print(",\"channel\":");
        Serial.print(channel);
        Serial.print(",\"address\":\"0x29\",\"found\":");

        if (!sensorExists(channel)) {
            Serial.println("false}");
            continue;
        }

        Serial.println("true}");

        selectChannel(channel);

        sensors[i].setBus(&Wire);
        sensors[i].setTimeout(500);

        if (!sensors[i].init()) {
            Serial.print("{\"error\":\"sensor_");
            Serial.print(i + 1);
            Serial.println("_init_failed\"}");
            continue;
        }

        sensors[i].setDistanceMode(VL53L1X::Long);
        sensors[i].setMeasurementTimingBudget(50000);
        sensors[i].startContinuous(50);

        sensorReady[i] = true;

        Serial.print("{\"sensor\":");
        Serial.print(i + 1);
        Serial.println(",\"status\":\"ready\"}");
    }
}

// ============================================================
// GET MEDIAN DISTANCE
// ============================================================

float getAverageDistance(int column) {
    if (column < 0 || column >= NUM_COLUMNS) {
        return -1.0;
    }

    if (!sensorReady[column]) {
        return -1.0;
    }

    selectChannel(SENSOR_CHANNELS[column]);

    const int NUM_SAMPLES = 15;

    float readings[NUM_SAMPLES];
    int validReadings = 0;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        uint16_t distance = sensors[column].read();

        bool timedOut = sensors[column].timeoutOccurred();

        bool validStatus =
            sensors[column].ranging_data.range_status
            == VL53L1X::RangeValid;

        if (!timedOut && validStatus && distance > 0) {
            readings[validReadings++] = distance;
        }

        delay(10);
    }

    if (validReadings == 0) {
        return -1.0;
    }

    // Sort valid readings in ascending order.
    for (int i = 0; i < validReadings - 1; i++) {
        for (int j = i + 1; j < validReadings; j++) {
            if (readings[j] < readings[i]) {
                float temp = readings[i];
                readings[i] = readings[j];
                readings[j] = temp;
            }
        }
    }

    // Return median.
    if (validReadings % 2 == 1) {
        return readings[validReadings / 2];
    }

    return (
        readings[validReadings / 2 - 1] +
        readings[validReadings / 2]
    ) / 2.0;
}

// ============================================================
// CALCULATE CAN COUNT
// ============================================================



int calculateCanCount(int column, float distance) {
    if (distance < 0) {
        return 0;
    }

    for (const auto& entry : canCountMap) {
        float minDistance = entry.first.first;
        float maxDistance = entry.first.second;

        if (distance >= minDistance && distance <= maxDistance) {
            return entry.second;
        }
    }

    int closestCount = 1;
    float smallestDifference = 1e9;

    for (const auto& entry : canCountMap) {
        int count = entry.second;

        if (count == 0) {
            continue;
        }

        float midpoint =
            (entry.first.first + entry.first.second) / 2.0;

        float difference = fabs(distance - midpoint);

        if (difference < smallestDifference) {
            smallestDifference = difference;
            closestCount = count;
        }
    }

    return closestCount;
}


// ============================================================
// SEND CAN COUNTS
// Atomic JSON line
// ============================================================

void sendCanCounts() {
    String json = "[";

    for (int i = 0; i < NUM_COLUMNS; i++) {
        float avgDistance = getAverageDistance(i);

        int count = 0;

        if (avgDistance >= 0) {
            count = calculateCanCount(i, avgDistance);
        }

        if (i > 0) {
            json += ",";
        }

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
// SEND RAW DISTANCES
// Atomic JSON line
// ============================================================

void sendDistances() {
    String json = "{";

    for (int i = 0; i < NUM_COLUMNS; i++) {
        float avgDistance = getAverageDistance(i);

        if (i > 0) {
            json += ",";
        }

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
                listening = true;
                lastUpdateTime = millis();

                Serial.println("{\"status\":\"started\"}");

            } else if (command.equals(CMD_STOP)) {
                listening = false;
                listeningRaw = false;

                Serial.println("{\"status\":\"stopped\"}");

            } else if (command.equals(CMD_LISTEN_RAW)) {
                listeningRaw = true;
                lastRawUpdateTime = millis();

                Serial.println("{\"status\":\"raw_started\"}");

            } else if (command.length() > 0) {
                Serial.print(
                    "{\"error\":\"unknown_command\",\"command\":\""
                );
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

    // Normal can-count output.
    if (listening &&
        millis() - lastUpdateTime >= UPDATE_INTERVAL_MS) {

        lastUpdateTime = millis();

        sendCanCounts();
    }

    // Raw distance output.
    if (listeningRaw &&
        millis() - lastRawUpdateTime >= UPDATE_INTERVAL_MS) {

        lastRawUpdateTime = millis();

        sendDistances();
    }
}
