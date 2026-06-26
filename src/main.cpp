/*
 * LilyGO T-Beam ESP-NOW Controller (with ADS1115 ADC & Toggle Button)
 * with Non-Blocking GPS Position Logging
 *
 * Author: Gemini
 * Date: October 1, 2025
 *
 * Description:
 * This sketch reads an analog joystick (ADS1115) and a toggle button,
 * then transmits the control data to a receiver via ESP-NOW.
 *
 * It also continuously reads data from the built-in GPS module in a
 * non-blocking way. Every 5 seconds, if a valid fix is available, it stores
 * the coordinates and timestamp into a vector.
 *
 * Hardware Setup:
 * - ADS1115 SCL -> T-Beam GPIO 22
 * - ADS1115 SDA -> T-Beam GPIO 21
 * - Joystick Y-axis -> ADS1115 A0
 * - Joystick X-axis -> ADS1115 A1
 * - Push Button -> T-Beam GPIO 38 and GND
 * - GPS Module (Onboard): Connected to Serial1 (TX:34, RX:12)
 */
#define XPOWERS_CHIP_AXP2101

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <WiFiUdp.h> // ADDED for Phone GPS
#include <esp_wifi.h> // Required for esp_wifi_set_channel
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include "XPowersLib.h"

// --- ADDED: GPS Libraries ---
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <vector> // Used to store a history of GPS coordinates

//LoRa Library
#include <RadioLib.h>


#ifndef CONFIG_PMU_SDA
#define CONFIG_PMU_SDA 21
#endif

#ifndef CONFIG_PMU_SCL
#define CONFIG_PMU_SCL 22
#endif

#ifndef CONFIG_PMU_IRQ
#define CONFIG_PMU_IRQ 35
#endif

// --- PHONE GPS CONFIG (Input your details here) ---
const char* hotspot_ssid = "Test Wifi Please Ignore";
const char* hotspot_pass = "freewifi";
const int udpPort = 2000; 

WiFiUDP udp;
char packetBuffer[255];

// --- Configuration ---
uint8_t receiverMacAddress[] = {0x6C, 0xC8, 0x40, 0x86, 0x3C, 0x68};

// --- Pin Definitions ---
const int BUTTON_PIN = 38;
const int AUTO_PIN = 25;

// --- I2C Device Instances ---
Adafruit_ADS1115 ads;
XPowersPMU power;

// --- Joystick Calibration ---
const int ADC_MAX = 26400;
const int JOYSTICK_CENTER_X = 13010;
const int JOYSTICK_CENTER_Y = 13350;
const int JOYSTICK_DEADZONE = 500;

// --- Button Debounce & Toggle Logic ---
bool g_button_toggle_state = false;
int g_last_button_state = HIGH;
unsigned long g_last_debounce_time = 0;
const unsigned long DEBOUNCE_DELAY = 50;

// --- ADDED: Variables for LED Status & Long Press ---
unsigned long g_last_gps_blink_time = 0;
bool g_gps_led_on = false;
unsigned long g_button_press_start_time = 0;
bool g_long_press_action_done = false;
const unsigned long LONG_PRESS_DURATION = 2000; // 2 seconds for a long press

//Variables for Manual/Auto Mode from 3 way switch
bool auto_mode = false;

//motor timing
int lastMotorCommandTime = 0;
int motorCommandDelay = 50; //milliseconds between motor commands

// --- ADDED: Variables for Double Press Detection ---
unsigned long g_last_press_time = 0;
const unsigned long DOUBLE_PRESS_WINDOW = 300; // Time in ms for a double press

// SX1276 radio = new Module(5, 2, 14); 
SX1276 radio = new Module(18, 26, 23); 

// Timing Constants
const unsigned long TX_INTERVAL = 1000;  // Send every 2 seconds
const unsigned long RX_TIMEOUT = 300;   // Listen for 0.5 seconds

// State Machine Variables
enum RadioState { IDLE, TRANSMITTING, LISTENING };
RadioState currentMode = IDLE;
unsigned long lastTxTime = 0;
unsigned long startListenTime = 0;

// --- ADDED: GPS Configuration ---
static const int GPS_RX_PIN = 12; // T-Beam v1.0/v1.1
static const int GPS_TX_PIN = 34; // T-Beam v1.0/v1.1
static const uint32_t GPS_BAUD = 9600;

double current_gps_lat = 0.0;
double current_gps_long = 0.0;

// TinyGPS++ object to process GPS data
TinyGPSPlus gps;

// The hardware serial port for the GPS module (UART 1)
HardwareSerial gpsSerial(1);

// --- ADDED: GPS Data Storage ---
struct GPSLog {
    double latitude;
    double longitude;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
};

std::vector<GPSLog> gps_log_history;
const int MAX_LOG_HISTORY = 2000; // Limit stored points to prevent memory overflow
unsigned long lastGpsLogTime = 0;
const unsigned long GPS_LOG_INTERVAL = 5000; // Log position every 5000 ms (5 seconds)

struct __attribute__((packed)) GpsPacket {
  uint8_t rh_to = 0xFF; uint8_t rh_from = 0x01; uint8_t rh_id = 0x00; uint8_t rh_flags = 0x00;
  uint32_t timestamp;
  double latitude;
  double longitude;
  int16_t alt_cm;
};

struct __attribute__((packed)) TelemPacket {
  uint8_t rh_to; uint8_t rh_from; uint8_t rh_id; uint8_t rh_flags;
  uint32_t status;
  double speed;
  double distance_to_target;
  int16_t heading;
};

// --- Data Structure ---
typedef struct ControlData {
    float throttle;
    float steering;
    bool button_state;
    bool auto_mode;
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
        if (now - lastBlinkTime >= 500) {
            ledOn = !ledOn;
            if (ledOn) {
                power.setChargingLedMode(XPOWERS_CHG_LED_ON);
            } else {
                power.setChargingLedMode(XPOWERS_CHG_LED_OFF);
            }
            lastBlinkTime = now;
        }
    } else {
        power.setChargingLedMode(XPOWERS_CHG_LED_ON);
    }
}

void loRaSetup() {
  Serial.println(F("Starting Radio..."));

  // Start SPI with T-Beam specific pins before radio.begin
  // SCK: 5, MISO: 19, MOSI: 27, NSS: 18
  SPI.begin(5, 19, 27, 18); 

  int state = radio.begin(433.0, 125.0, 7, 5, 0x12, 10, 8);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("Radio is good!"));
  } else {
    Serial.print(F("Radio failed, code: "));
    Serial.println(state);
    // Common codes: 
    // -2: Chip not found (Check Power/SPI)
    // -16: SPI error
    // while (true); 
  }
}

// Create a flag to track if we are currently mid-transmission
bool transmissionActive = false;

void handleLoRa() {
  unsigned long now = millis();

  switch (currentMode) {
    
    case IDLE:
      if (now - lastTxTime >= TX_INTERVAL) {
        GpsPacket myData;
        myData.timestamp = now;
        myData.latitude = current_gps_lat;
        myData.longitude = current_gps_long;
        myData.alt_cm = 4500;

        Serial.print(F("[LoRa] Starting Asynchronous TX... "));
        
        // Use startTransmit instead of transmit (NON-BLOCKING)
        int state = radio.startTransmit((uint8_t*)&myData, sizeof(myData));
        
        if (state == RADIOLIB_ERR_NONE) {
          currentMode = TRANSMITTING; 
          transmissionActive = true;
        }
        lastTxTime = now;
      }
      break;

    case TRANSMITTING:
      // Check if the hardware interrupt pin (DIO0) has gone HIGH
      // This indicates the radio is finished sending the packet
      if (digitalRead(26) == HIGH) {
        Serial.println(F("Done!"));
        radio.finishTransmit(); // Clean up radio state
        
        // Now switch to listening mode exactly like before
        radio.startReceive();
        startListenTime = millis();
        currentMode = LISTENING;
      }
      break;

    case LISTENING:
      if (digitalRead(26) == HIGH) {
        TelemPacket incoming;
        int state = radio.readData((uint8_t*)&incoming, sizeof(incoming));
        if (state == RADIOLIB_ERR_NONE) {
          Serial.print(F("SUCCESS! Speed: "));
          Serial.println(incoming.speed);
        }
        radio.standby();
        currentMode = IDLE;
      }
      else if (now - startListenTime >= RX_TIMEOUT) {
        Serial.println(F("[LoRa] RX Timeout."));
        radio.standby();
        currentMode = IDLE;
      }
      break;
  }
}

// --- NEW: Non-Blocking Phone GPS Processing ---
void processPhoneGPS() {

    if (WiFi.status() != WL_CONNECTED) {
        return; 
    }

    int packetSize = udp.parsePacket();
    if (packetSize) {
        int len = udp.read(packetBuffer, 255);
        for (int i = 0; i < len; i++) {
            // Feed each character from the UDP packet to TinyGPS++
            if (gps.encode(packetBuffer[i])) {
                if (gps.location.isValid()) {
                    current_gps_lat = gps.location.lat();
                    current_gps_long = gps.location.lng();
                }
            }
        }
    }
}

// --- ADDED: GPS Processing Function ---
// This function reads from the GPS serial port and feeds the data to the
// TinyGPS++ object. It should be called in every loop iteration.
void processGPS() {
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }

    if (gps.location.isValid()) {
            current_gps_lat = gps.location.lat();
            current_gps_long = gps.location.lng();
    }
}

// --- ADDED: Function to print all stored GPS data ---
void printGpsHistory() {
    Serial.println("\n\n--- GPS Log History (CSV Format) ---");
    Serial.println("Latitude,Longitude,Year,Month,Day,Hour,Minute,Second");

    if (gps_log_history.empty()) {
        Serial.println("No GPS data has been logged.");
    } else {
        for (const auto& log : gps_log_history) {
            Serial.printf("%.6f,%.6f,%d,%d,%d,%d,%d,%d\n",
                log.latitude, log.longitude, log.year, log.month, log.day,
                log.hour, log.minute, log.second);
        }
    }
    Serial.println("--- End of Log ---\n");
}

// --- ADDED: Function to clear GPS history and confirm with LED ---
void clearGpsHistory() {
    gps_log_history.clear();
    Serial.println("\n*** GPS Log History Erased! ***\n");

    // Quick triple-blink confirmation
    for (int i=0; i<3; i++) {
        power.setChargingLedMode(XPOWERS_CHG_LED_OFF);
        delay(75);
        power.setChargingLedMode(XPOWERS_CHG_LED_ON);
        delay(75);
    }
}

//================================================================================
// Main Program: setup() and loop()
//================================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("T-Beam ESP-NOW Controller (ADS1115 + Button + GPS) Initializing...");

    // Configure the button pin
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    Serial.println("Button on GPIO 38 configured.");

    // Configure the Auto 3 Way Switch
    pinMode(AUTO_PIN, INPUT_PULLUP);
    Serial.println("Switch on GPIO 25 configured.");

    // Initialize I2C for PMU
    if (!power.begin(Wire, AXP2101_SLAVE_ADDRESS, CONFIG_PMU_SDA, CONFIG_PMU_SCL)) {
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

    //Initialize LoRa
    loRaSetup();
    Serial.println("Initialized LoRa");

    // --- WIFI SETUP for Phone GPS ---
    Serial.println("Connecting to Phone Hotspot...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(hotspot_ssid, hotspot_pass);
    
    unsigned long wifi_timeout = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - wifi_timeout < 5000) {
        delay(500);
        Serial.print(".");
    }
    int channel = WiFi.channel();

    if(WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi Connected!");
        Serial.print("T-Beam IP: "); Serial.println(WiFi.localIP());
        udp.begin(udpPort);
    } else {
        Serial.println("\nWiFi Failed. Running without Phone GPS. Defaulting to Channel 1");

        WiFi.disconnect(true);
        WiFi.mode(WIFI_STA);

        channel = 1;
        // This forces the physical radio to stay on Channel 1
        esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);

    }

    WiFi.setSleep(false);

    // 3. Get the channel from the connected WiFi
    
    Serial.printf("ESP-NOW Home Channel: %d\n", channel);

    // // Register the receiver as a peer
    // esp_now_peer_info_t peerInfo = {};
    // memcpy(peerInfo.peer_addr, receiverMacAddress, 6);

    // // CHANGE THIS: Use the actual channel from the WiFi connection
    // peerInfo.channel = channel; 
    // peerInfo.encrypt = false;

    // --- ADDED: Initialize GPS Serial ---
    Serial.println("Initializing GPS...");
    // Note: T-Beam uses different pins for different versions. v1.1 uses 34, 12.
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_TX_PIN, GPS_RX_PIN);
    Serial.println("GPS Serial Port Initialized. Waiting for data...");

   // 1. MUST set mode first
    WiFi.mode(WIFI_STA); 

    // 2. Initialize ESP-NOW (This was missing!)
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // 3. Register the callback
    esp_now_register_send_cb(onDataSent);

    // 4. Setup Peer Info
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
    peerInfo.channel = channel; // Match the WiFi channel
    peerInfo.encrypt = false;

    // 5. Finally add the peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    power.enableGeneralAdcChannel();
}

void loop() {
    float time_start = millis();
    // --- Process any incoming GPS data ---
    // processGPS();
    processPhoneGPS();    

    //Process loRa messages
    handleLoRa();

    // -- Auto Logic
    int current_auto_state = digitalRead(AUTO_PIN);
    // Serial.println(current_auto_state);

    // --- Button Logic: Handle Toggle (short), Erase (double), and Dump (long) ---
    int current_button_state = digitalRead(BUTTON_PIN);

    // Detect the falling edge (the moment the button is pressed)
    if (current_button_state == LOW && g_last_button_state == HIGH) {
        if ((millis() - g_last_debounce_time) > DEBOUNCE_DELAY) {
            // Check if this press is within the double-press window
            if (millis() - g_last_press_time < DOUBLE_PRESS_WINDOW) {
                clearGpsHistory();
                g_last_press_time = 0; // Reset timer to prevent a triple-press action
            } else {
                // It's a single press, so toggle the state
                g_button_toggle_state = !g_button_toggle_state;
            }
            g_last_press_time = millis(); // Record the time of this press
            g_last_debounce_time = millis();
        }
    }
    
    // Long press (data dump) detection
    if (current_button_state == LOW) { // If button is currently pressed
        if (g_last_button_state == HIGH) { // If it was just pressed, start the timer
            g_button_press_start_time = millis();
        }
        // If it has been held long enough and we haven't already acted
        if (millis() - g_button_press_start_time > LONG_PRESS_DURATION && !g_long_press_action_done) {
            printGpsHistory();
            g_long_press_action_done = true; // Prevents this from running repeatedly
        }
    } else { // Button is not pressed
        g_long_press_action_done = false; // Reset the long press flag when released
    }
    g_last_button_state = current_button_state;


    // --- Read analog joystick values from the ADS1115 ---
    int16_t rawY = ads.readADC_SingleEnded(0);
    int16_t rawX = ads.readADC_SingleEnded(1);
    // int16_t rawY = 0;
    // int16_t rawX = 0;

    // Apply deadzone and map to -1.0 to 1.0 range
    float throttle = 0.0;
    if (abs(rawY - JOYSTICK_CENTER_Y) > JOYSTICK_DEADZONE) {
        throttle = map_float(rawY, 1744, ADC_MAX, -1.0, 1.0);
    }

    float steering = 0.0;
    if (abs(rawX - JOYSTICK_CENTER_X) > JOYSTICK_DEADZONE) {
        steering = map_float(rawX, 176, ADC_MAX, -1.0, 1.0);
    }            // Serial.printf("Waiting for GPS fix... Satellites in view: %d\n", gps.satellites.value());

    
    throttle = constrain(throttle, -1.0, 1.0);
    steering = constrain(steering, -1.0, 1.0);

    // Load data into the struct
    controlData.throttle = throttle;
    controlData.steering = steering;
    controlData.button_state = g_button_toggle_state;

    // Serial.println(steering);

    // Auto Control Logic (Supercedes manual control above)
 
    if (current_auto_state == 1){
        controlData.auto_mode = true;
        // IN AUTO MODE, THROTTLE IS RIGHT MOTOR AND STEERING IS LEFT MOTOR
        if (throttle > 0.5 && abs(steering < 0.5)){ // FORWARD
            controlData.throttle = -1;
            controlData.steering = 1;
        }
        else if (steering < -0.5) { // RIGHT
            controlData.throttle = -1;
            controlData.steering = 0.85;
        }
        else if (steering > 0.5) { // LEFT
            controlData.throttle = -0.85;
            controlData.steering = 1;
        }
        else if (throttle <= -0.5 && abs(steering <= 0.5)) { // SLOWER (BACK)
            controlData.throttle = -0.66;
            controlData.steering = 0.66;
        }
        else{
            controlData.throttle = 0;
            controlData.steering = 0;
        }
    }
    else {
        controlData.auto_mode = false;
    }

    // Serial.println(throttle);
    // Serial.println(steering);
    // Serial.println(auto_mode);

    // Serial.printf("Throttle/Left: %f, Steering/Right: %f, Manual/Auto: %d\n", controlData.throttle, controlData.steering, controlData.auto_mode);

    // Send the data via ESP-NOW
    if (millis() - lastMotorCommandTime > motorCommandDelay){
        esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *) &controlData, sizeof(controlData));
        // Serial.println(millis() - lastMotorCommandTime);
        if (result != ESP_OK) {
            Serial.print("ESP-NOW Send Error: ");
            Serial.println(result);
        }
        lastMotorCommandTime = millis();
    }
    
    

    // --- GPS Logging Logic (Non-Blocking) ---
    unsigned long currentTime = millis();
    if (currentTime - lastGpsLogTime >= GPS_LOG_INTERVAL) {
        lastGpsLogTime = currentTime; // Reset the timer

        if (gps.location.isValid()) {
            GPSLog new_log;
            new_log.latitude = gps.location.lat();
            new_log.longitude = gps.location.lng();
            new_log.year = gps.satellites.value();
            new_log.month = gps.date.month();
            new_log.day = gps.date.day();
            new_log.hour = gps.time.hour();
            new_log.minute = gps.time.minute();
            new_log.second = gps.time.second();

            if (gps_log_history.size() >= MAX_LOG_HISTORY) {
                gps_log_history.erase(gps_log_history.begin());
            }
            gps_log_history.push_back(new_log);

            Serial.printf("Logged GPS Position: Lat: %.6f, Lng: %.6f | Sats: %d | Total logs: %zu\n",
                          new_log.latitude, new_log.longitude, gps.satellites.value(), gps_log_history.size());
        } else {
            // Serial.printf("Waiting for GPS fix... Satellites in view: %d\n", gps.satellites.value());
        }
    }

    // --- Status LED Logic ---
    if (power.getBatteryPercent() < 10) {
        // Low battery warning takes priority over GPS status
        checkBatteryAndFlash();
    } else {
        // Otherwise, show GPS status
        if (gps.location.isValid()) {
            power.setChargingLedMode(XPOWERS_CHG_LED_ON); // Solid ON for GPS fix
        } else {
            // Blink slowly if searching for satellites
            if (currentTime - g_last_gps_blink_time >= 1000) { // 1-second interval
                g_last_gps_blink_time = currentTime;
                g_gps_led_on = !g_gps_led_on;
                power.setChargingLedMode(g_gps_led_on ? XPOWERS_CHG_LED_ON : XPOWERS_CHG_LED_OFF);
            }
        }
    }
    
    // delay(50);
    // Serial.print("Cycle Time: ");
    // Serial.println(millis() - time_start);
}