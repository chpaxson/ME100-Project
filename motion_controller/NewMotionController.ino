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

        String yaw_cmd = "X";
        if (abs(yaw) > 10) {
          if (yaw > 0) {
            yaw_cmd = "CW";
          } else {
            yaw_cmd = "CCW";
          }
        }

        x = roll * (1.0 / 60.0);
        y = pitch * (1.0 / 60.0);
        double magnitude = sqrt(x * x + y * y);

        double angle = atan2(y, x) * RAD_TO_DEG;
        String direction = "X";
        if (magnitude > 1.0 / 6.0) {
          /*if (angle >= -22.5 && angle < 22.5) {
            direction = "E";
          } else if (angle >= 22.5 && angle < 67.5) {
            direction = "NE";
          } else if (angle >= 67.5 && angle < 112.5) {
            direction = "N";
          } else if (angle >= 112.5 && angle < 157.5) {
            direction = "NW";
          } else if (angle >= 157.5 || angle < -157.5) {
            direction = "W";
          } else if (angle >= -157.5 && angle < -112.5) {
            direction = "SW";
          } else if (angle >= -112.5 && angle < -67.5) {
            direction = "S";
          } else if (angle >= -67.5 && angle < -22.5) {
            direction = "SE";
          }*/
          if (angle >= -20 && angle < 20) {
            direction = "E";
          } else if (angle >= 25 && angle < 65) {
            direction = "NE";
          } else if (angle >= 70 && angle < 110) {
            direction = "N";
          } else if (angle >= 115 && angle < 155) {
            direction = "NW";
          } else if (angle >= 160 || angle < -155) {
            direction = "W";
          } else if (angle >= -160 && angle < -110) {
            direction = "SW";
          } else if (angle >= -115 && angle < -65) {
            direction = "S";
          } else if (angle >= -70 && angle < -25) {
            direction = "SE";
          }
        } else {
          direction = "X";
        }

        String M = "M ";
        String space = " ";
        String cmd = M + direction + space + yaw_cmd;

        // Client Output
        client.println(cmd);

        // Serial Output
        Serial.println(cmd);

        delay(5);
      }
    }
  } else {
    Serial.println("Failed to connect");
    delay(500);
  }
}
