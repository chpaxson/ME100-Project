// Ian Zhang

// Parts: Adafruit Feather ESP32 V2, Sparkfun LSM6DSO, 2 Pin Push Button, 10k Ohm Resistor
// Required Library: https://github.com/sparkfun/SparkFun_Qwiic_6DoF_LSM6DSO_Arduino_Library

#include "SparkFunLSM6DSO.h"
#include "Wire.h"
#include <WiFi.h>
#include <String.h>

LSM6DSO myIMU;

// WiFi
const char* ssid = "kiwi";
const char* password = "ChArLeS1";
const char* host = "192.168.12.1";
const int port = 10000;

// Reset Button Variables
const int resetPin = 5;
int resetButton = 0;
bool resetState = true;
int roll_offset = 0;
int pitch_offset = 0;
int yaw_offset = 0;

// Q Button Variables
const int qPin = 38;
int qButton = 0;
bool qState = true;

// Static Error from Basic_Readings Sketch
float accXerror = -0.015;
float accYerror = -0.035;
float accZerror = 0.025;
float gyroZerror = 0.125;

// Yaw Calculation Variables
float elapsedTime, currentTime, previousTime;
double yaw_gyro = 0;

// Controller Output Variables
double roll = 0;
double pitch = 0;
double yaw = 0;
double x = 0;
double y = 0;

// Rounding Function
int roundToNearest5(double value) {
  return 5 * round(value / 5.0);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  pinMode(resetPin, INPUT_PULLDOWN);
  pinMode(qPin, INPUT_PULLDOWN);

  Wire.begin();
  delay(10);

  if (myIMU.begin()) {
    Serial.println("Ready.");
  } else {
    Serial.println("Could not connect to IMU.");
    Serial.println("Freezing");
  }
  if (myIMU.initialize(BASIC_SETTINGS)) {
    Serial.println("Loaded Settings.");
  }
}

void loop() {
  // Client
  WiFiClient client;
  if (client.connect(host, port)) {
    Serial.println("Connected to server!");
    while (true) {
      // Q Button
      qButton = digitalRead(qPin);
      if (qButton == LOW && qState) {
        qState = false;
        client.println("Q");
        Serial.println("Q");
        client.stop();
      }
      if (qButton == HIGH && qState) {
        // Roll and Pitch Calculated from Accelerometer Readings
        double roll_acc = 90 - atan2(myIMU.readFloatAccelZ() - accZerror, myIMU.readFloatAccelY() - accYerror) * RAD_TO_DEG;
        double pitch_acc = 90 - atan2(myIMU.readFloatAccelZ() - accZerror, myIMU.readFloatAccelX() - accXerror) * RAD_TO_DEG;

        // Yaw Calculated from Gyroscope Readings
        previousTime = currentTime;
        currentTime = millis();
        elapsedTime = (currentTime - previousTime) / 1000;
        yaw_gyro = yaw_gyro + (myIMU.readFloatGyroZ() - gyroZerror) * elapsedTime;

        // Rounding for Noise Reduction
        roll = roundToNearest5(roll_acc) - roll_offset;
        pitch = -(roundToNearest5(pitch_acc) - pitch_offset);
        yaw = -(roundToNearest5(yaw_gyro) - yaw_offset);

        // Orientation Resetting
        resetButton = digitalRead(resetPin);
        if (resetButton == HIGH && resetState) {
          resetState = false;
          roll_offset += roll;
          pitch_offset -= pitch;
          yaw_offset -= yaw;
        }
        if (resetButton == LOW && !resetState) {
          resetState = true;
        }

        // Scale and Normalize Yaw
        double yaw_scaled = yaw * (1.0 / 60.0);
        if (abs(yaw) > 60)
          yaw_scaled = (yaw / abs(yaw));

        // Calculate Direction Vector from Roll and Pitch
        x = roll * (1.0 / 60.0);
        y = pitch * (1.0 / 60.0);

        // Normalize Vectors Outside of Unit Circle
        double magnitude = sqrt(x * x + y * y);
        if (magnitude > 1) {
          x /= magnitude;
          y /= magnitude;
        }

        String M = "M ";
        String space = " ";
        String cmd = M + x + space + y + space + yaw_scaled;

        // Client Output
        client.println(cmd);

        // Serial Output
        Serial.println(cmd);

        delay(20);
      }
    }
  } else {
    Serial.println("Failed to connect");
    delay(100);
  }
}
