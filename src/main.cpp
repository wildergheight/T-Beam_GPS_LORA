/*
 * LilyGO T-Beam ESP-NOW Controller (with ADS1115 ADC)
 * Author: Gemini
 * Date: August 28, 2025
 * * Description:
 * This sketch reads an analog joystick connected to an external ADS1115 ADC
 * over I2C and transmits the control data to the push cart receiver via
 * the ESP-NOW protocol.
 *
 * Hardware Setup:
 * - Connect ADS1115 SCL to T-Beam GPIO 22.
 * - Connect ADS1115 SDA to T-Beam GPIO 21.
 * - Connect Joystick Y-axis to ADS1115 A0.
 * - Connect Joystick X-axis to ADS1115 A1.
 *
 * Required Libraries:
 * - Adafruit ADS1X15
 */

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// --- Configuration ---
// IMPORTANT: Replace this with the MAC address of your push cart's ESP32.
uint8_t receiverMacAddress[] = {0x6C, 0xC8, 0x40, 0x86, 0x3C, 0x68};

// Create an instance of the ADS1115
Adafruit_ADS1115 ads;

// --- Joystick Calibration ---
// The raw ADC range depends on the gain setting. With GAIN_ONE (up to 4.096V)
// and a 3.3V joystick, the max value is around 26355.
const int ADC_MAX = 26355; 
const int JOYSTICK_CENTER = ADC_MAX / 2;
// A larger deadzone is needed for the higher resolution ADC.
const int JOYSTICK_DEADZONE = 400; 

// --- Data Structure ---
typedef struct ControlData {
    float throttle; // Range: -1.0 to 1.0
    float steering; // Range: -1.0 to 1.0
} ControlData;

ControlData controlData;

// --- ESP-NOW Functions ---
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // Callback after data is sent. Can be left empty.
}

float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//================================================================================
// Main Program: setup() and loop()
//================================================================================
void setup() {
    Serial.begin(115200);
    // Give some time for the serial monitor to start
    delay(1000);
    Serial.println("T-Beam ESP-NOW Controller (ADS1115) Initializing...");

    // Initialize I2C for the ADS1115
    Wire.begin(21, 22); // T-Beam's default I2C pins

    // Initialize the ADS1115
    if (!ads.begin()) {
        Serial.println("Failed to initialize ADS. Check wiring!");
        while (1);
    }
    // Set the gain to +/- 4.096V. This provides good precision for a 3.3V joystick.
    ads.setGain(GAIN_ONE);
    Serial.println("ADS1115 Initialized.");

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_register_send_cb(onDataSent);

    // Register the receiver as a peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    Serial.println("ESP-NOW Initialized. Ready to send data.");
}

void loop() {
    // Read analog joystick values from the ADS1115
    int16_t rawY = ads.readADC_SingleEnded(0); // Y-axis on A0
    int16_t rawX = ads.readADC_SingleEnded(1); // X-axis on A1

    // Apply deadzone and map to -1.0 to 1.0 range
    float throttle = 0.0;
    if (abs(rawY - JOYSTICK_CENTER) > JOYSTICK_DEADZONE) {
        // Invert Y-axis for intuitive control (up is forward)
        throttle = map_float(rawY, 0, ADC_MAX, -1.0, 1.0);
    }

    float steering = 0.0;
    if (abs(rawX - JOYSTICK_CENTER) > JOYSTICK_DEADZONE) {
        steering = map_float(rawX, 0, ADC_MAX, -1.0, 1.0);
    }
    
    // Constrain values to ensure they are within the expected range
    throttle = constrain(throttle, -1.0, 1.0);
    steering = constrain(steering, -1.0, 1.0);

    // Load data into the struct
    controlData.throttle = throttle;
    controlData.steering = steering;

    // Send the data via ESP-NOW
    esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *) &controlData, sizeof(controlData));

    if (result == ESP_OK) {
        // Serial.printf("Raw: X=%d, Y=%d  |  Sent: Thr=%.2f, Ste=%.2f\n", rawX, rawY, controlData.throttle, controlData.steering);
        
      } else {
        Serial.println("Error sending the data");
    }

    delay(50); // Send data at ~20 Hz
}
