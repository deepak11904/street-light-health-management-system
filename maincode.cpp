/*
  IoT-based Earthing Detection System with ESP32

  Project Description:
  This Arduino sketch serves as the core firmware for an IoT-based system
  designed to detect earthing faults in street lighting infrastructure. The
  system leverages an ESP32 microcontroller to monitor real-time electrical
  parameters using voltage and current sensors. Its primary function is to
  identify unsafe conditions, trigger a swift power shutoff to prevent
  accidents, and send a comprehensive alert to a centralized cloud dashboard.

  The code is structured into three main sections:
  1.  **Configuration:** Defines Wi-Fi credentials, sensor pin assignments, and
      safety thresholds. These values should be calibrated based on your
      specific hardware and operational requirements.
  2.  **Function Definitions:** Contains modular functions for specific tasks,
      such as connecting to Wi-Fi, reading sensor data, and communicating
      with a cloud service.
  3.  **Main Program Logic (setup() and loop()):** The standard Arduino
      functions that handle initialization and the continuous monitoring loop.

  Hardware Used:
  - ESP32 Microcontroller
  - ZMPT101B Voltage Sensor
  - Current Sensor (e.g., SCT-013, ACS712)
  - Relay Module (to control power supply)
  - Power Supply (e.g., Solar Power, Battery)

  Software & Libraries:
  - Arduino IDE with ESP32 Board Package
  - Wi-Fi library (built-in)
  - Future Integration: Libraries for MQTT or HTTP communication.
*/

// --- 1. CONFIGURATION ---

#include <WiFi.h>

// Wi-Fi credentials for connecting to the network
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// Pin assignments for hardware components
const int VOLTAGE_SENSOR_PIN = 34; // ZMPT101B analog pin
const int CURRENT_SENSOR_PIN = 35; // Current sensor analog pin
const int POWER_RELAY_PIN = 25;    // Digital pin to control the relay

// Safety thresholds to detect faults
// NOTE: These values are placeholders and must be carefully calibrated
//       to match your electrical system's specifications.
const float VOLTAGE_THRESHOLD = 5.0; // Voltage in Volts (e.g., for leakage detection)
const float CURRENT_THRESHOLD = 0.5; // Current in Amperes (e.g., for overload)

// Calibration constants for sensors
// These values convert the raw ADC readings into meaningful units (Volts and Amperes).
const float VOLTAGE_SENSITIVITY = 0.05; // Volts per raw reading
const float CURRENT_SENSITIVITY = 0.1;  // Amperes per raw reading

// Variables for storing sensor data and system status
float voltageRMS = 0.0;
float currentRMS = 0.0;
bool isFaultDetected = false;

// --- 2. FUNCTION DEFINITIONS ---

// Function to establish a connection to the specified Wi-Fi network.
// It will continuously attempt to connect until a connection is successful.
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to Wi-Fi network: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected successfully!");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());
}

// Function to read the voltage from the ZMPT101B sensor and calculate the RMS value.
// It reads a large number of samples over a period to ensure a stable and accurate reading.
float read_voltage_sensor() {
  long sumOfSquares = 0;
  long startTime = millis();
  int numSamples = 0;

  // Read samples for one full second to get a stable RMS value
  while (millis() - startTime < 1000) {
    int sensorValue = analogRead(VOLTAGE_SENSOR_PIN);
    sumOfSquares += (long)sensorValue * sensorValue;
    numSamples++;
  }
  float avgSquare = (float)sumOfSquares / numSamples;
  float rms = sqrt(avgSquare) * VOLTAGE_SENSITIVITY;
  return rms;
}

// Function to read the current from the sensor and calculate the RMS value.
// Similar to the voltage function, it samples over time for accuracy.
float read_current_sensor() {
  long sumOfSquares = 0;
  long startTime = millis();
  int numSamples = 0;

  while (millis() - startTime < 1000) {
    int sensorValue = analogRead(CURRENT_SENSOR_PIN);
    sumOfSquares += (long)sensorValue * sensorValue;
    numSamples++;
  }
  float avgSquare = (float)sumOfSquares / numSamples;
  float rms = sqrt(avgSquare) * CURRENT_SENSITIVITY;
  return rms;
}

// Function to send the collected data to the cloud service.
// This is a placeholder function. In a real-world application, you would
// replace this with code for an MQTT, HTTP, or other cloud API client.
void send_data_to_cloud(float voltage, float current, bool faultStatus) {
  Serial.println("--- Sending Data to Cloud Dashboard ---");
  Serial.print("Voltage RMS: ");
  Serial.print(voltage);
  Serial.println(" V");
  Serial.print("Current RMS: ");
  Serial.print(current);
  Serial.println(" A");
  Serial.print("Fault Detected: ");
  Serial.println(faultStatus ? "YES" : "NO");
  Serial.println("-------------------------------------");

  // Example of a data payload you might send:
  // String payload = "{\"voltage\":" + String(voltage) + ",\"current\":" + String(current) + ",\"fault\":" + (faultStatus ? "true" : "false") + "}";
  // client.publish("data_topic", payload);
}

// Function to trigger the power shutoff mechanism.
// This function controls the relay connected to the lamp pole's power supply.
void trigger_power_shutoff() {
  Serial.println("--- FATAL FAULT DETECTED! INITIATING EMERGENCY SHUTOFF ---");
  // Assuming a HIGH signal deactivates the relay (Normally Closed) to cut power.
  digitalWrite(POWER_RELAY_PIN, HIGH);
}

// --- 3. MAIN PROGRAM LOGIC ---

void setup() {
  // Initialize the Serial Monitor for debugging
  Serial.begin(115200);

  // Set the relay pin as an output and ensure power is initially ON
  pinMode(POWER_RELAY_PIN, OUTPUT);
  digitalWrite(POWER_RELAY_PIN, LOW); // Assumes a LOW signal keeps the relay active (power ON)

  // Connect to Wi-Fi
  setup_wifi();
}

void loop() {
  // Read sensor data
  voltageRMS = read_voltage_sensor();
  currentRMS = read_current_sensor();

  // Check for fault conditions based on defined thresholds
  if (voltageRMS > VOLTAGE_THRESHOLD || currentRMS > CURRENT_THRESHOLD) {
    isFaultDetected = true;
    trigger_power_shutoff();
  } else {
    isFaultDetected = false;
    // Power remains ON, no action needed for the relay
  }

  // Send the collected data and status to the cloud
  send_data_to_cloud(voltageRMS, currentRMS, isFaultDetected);

  // Pause before the next measurement cycle to avoid excessive data and power consumption
  delay(5000); // Wait 5 seconds
}

