/*
 * LilyGO T-Beam ESP-NOW Controller (with ADS1115 ADC & Toggle Button)
 * Author: Gemini
 * Date: September 16, 2025
 * * Description:
 * This sketch reads an analog joystick (ADS1115) and a toggle button,
 * then transmits the control data to a receiver via ESP-NOW.
 * The button on GPIO 38 acts as a toggle (push-on, push-off).
 *
 * Hardware Setup:
 * - ADS1115 SCL -> T-Beam GPIO 22
 * - ADS1115 SDA -> T-Beam GPIO 21
 * - Joystick Y-axis -> ADS1115 A0
 * - Joystick X-axis -> ADS1115 A1
 * - Push Button -> T-Beam GPIO 38 and GND
 */
#define XPOWERS_CHIP_AXP2101

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include "XPowersLib.h"

#ifndef CONFIG_PMU_SDA
#define CONFIG_PMU_SDA 21
#endif

#ifndef CONFIG_PMU_SCL
#define CONFIG_PMU_SCL 22
#endif

#ifndef CONFIG_PMU_IRQ
#define CONFIG_PMU_IRQ 35
#endif

// --- Configuration ---
uint8_t receiverMacAddress[] = {0x6C, 0xC8, 0x40, 0x86, 0x3C, 0x68};

// --- Pin Definitions ---
const int BUTTON_PIN = 38;

// --- I2C Device Instances ---
Adafruit_ADS1115 ads;
XPowersPMU power;

// --- Joystick Calibration ---
const int ADC_MAX = 26400;
const int JOYSTICK_CENTER_X = 13010;
const int JOYSTICK_CENTER_Y = 13350;
const int JOYSTICK_DEADZONE = 500;

// --- Button Debounce & Toggle Logic ---
bool g_button_toggle_state = false;    // The actual toggle state (0 or 1), defaults to 0.
int g_last_button_state = HIGH;        // Used to detect a press event.
unsigned long g_last_debounce_time = 0;
const unsigned long DEBOUNCE_DELAY = 50; // 50ms debounce delay.

// --- Data Structure ---
typedef struct ControlData {
    float throttle;     // Range: -1.0 to 1.0
    float steering;     // Range: -1.0 to 1.0
    bool button_state;  // ADDED: 0 for off, 1 for on
} ControlData;

ControlData controlData;

// --- ESP-NOW Functions ---
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // Callback after data is sent. Can be left empty.
}

float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// --- Low Battery LED Flashing ---
unsigned long lastBlinkTime = 0;
bool ledOn = false;

void checkBatteryAndFlash() {
    int percent = power.getBatteryPercent();
    if (percent < 10) {
        unsigned long now = millis();
        if (now - lastBlinkTime >= 500) { // toggle every 500ms = 1Hz blink
            ledOn = !ledOn;
            if (ledOn) {
                power.setChargingLedMode(XPOWERS_CHG_LED_ON);
            } else {
                power.setChargingLedMode(XPOWERS_CHG_LED_OFF);
            }
            lastBlinkTime = now;
        }
    } else {
        // restore LED to steady ON when battery is above 10%
        power.setChargingLedMode(XPOWERS_CHG_LED_ON);
    }
}


//================================================================================
// Main Program: setup() and loop()
//================================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("T-Beam ESP-NOW Controller (ADS1115 + Button) Initializing...");

    // ADDED: Configure the button pin with an internal pull-up resistor.
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    Serial.println("Button on GPIO 38 configured.");

    // Initialize I2C for PMU and ADS1115
    bool result = power.begin(Wire, AXP2101_SLAVE_ADDRESS, CONFIG_PMU_SDA, CONFIG_PMU_SCL);

    if (result == false) {
        Serial.println("PMU is not online..."); while (1)delay(50);
    }
    Serial.println("PMU AXP2101 init success!");
    power.setChargingLedMode(XPOWERS_CHG_LED_ON);
    Serial.println("PMU Initialized and LED is set to ON.");

    // Initialize the ADS1115
    if (!ads.begin()) {
        Serial.println("Failed to initialize ADS. Check wiring!");
        while (1);
    }
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
    power.enableGeneralAdcChannel();

}

void loop() {
    // --- ADDED: Button Toggle Logic ---
    int current_button_state = digitalRead(BUTTON_PIN);

    // Check for a button press (transition from HIGH to LOW)
    if (current_button_state == LOW && g_last_button_state == HIGH) {
        // Check if the debounce delay has passed since the last press
        if ((millis() - g_last_debounce_time) > DEBOUNCE_DELAY) {
            // Toggle the state
            g_button_toggle_state = !g_button_toggle_state;
            g_last_debounce_time = millis(); // Reset the debounce timer
        }
    }
    // Update the last known button state for the next loop iteration
    g_last_button_state = current_button_state;


    // --- Read analog joystick values from the ADS1115 ---
    int16_t rawY = ads.readADC_SingleEnded(0); // Y-axis on A0
    int16_t rawX = ads.readADC_SingleEnded(1); // X-axis on A1

    // Apply deadzone and map to -1.0 to 1.0 range
    float throttle = 0.0;
    if (abs(rawY - JOYSTICK_CENTER_Y) > JOYSTICK_DEADZONE) {
        throttle = map_float(rawY, 1744, ADC_MAX, -1.0, 1.0);
    }

    float steering = 0.0;
    if (abs(rawX - JOYSTICK_CENTER_X) > JOYSTICK_DEADZONE) {
        steering = map_float(rawX, 176, ADC_MAX, -1.0, 1.0);
    }
    
    // Constrain values to ensure they are within the expected range
    throttle = constrain(throttle, -1.0, 1.0);
    steering = constrain(steering, -1.0, 1.0);

    // Load data into the struct
    controlData.throttle = throttle;
    controlData.steering = steering;
    controlData.button_state = g_button_toggle_state; // <-- Assign the button's toggle state

    // Send the data via ESP-NOW
    esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *) &controlData, sizeof(controlData));

    if (result == ESP_OK) {
        // Updated the print statement to include the button state
        Serial.printf("Raw: X=%d, Y=%d | Sent: Thr=%.4f, Ste=%.4f, Btn=%d, BatteryPercent:%d\n", rawX, rawY, controlData.throttle, controlData.steering, controlData.button_state, power.getBatteryPercent());
    } else {
        Serial.println("Error sending the data");
    }

    checkBatteryAndFlash();
    delay(50); // Send data at ~20 Hz
}