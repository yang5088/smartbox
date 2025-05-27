#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h> // Required for AHTX0 events
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include "time.h" // For time functions

TinyGPSPlus gps;

// 4G Module Serial Port and Pins
#define LTE_SERIAL_TX_PIN 18
#define LTE_SERIAL_RX_PIN 17
HardwareSerial LTE_Serial(1); // UART1 for ESP32-S3

// Screen definitions
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)

// Button pin definitions
#define UP_BUTTON_PIN 16
#define DOWN_BUTTON_PIN 15
#define SELECT_BUTTON_PIN 7
#define BACK_BUTTON_PIN 6
#define NUM_BUTTONS 4
#define DEBOUNCE_DELAY 50 // milliseconds

// UI Page Definitions
enum AppPage {
  PAGE_HOME,
  PAGE_GPS_INFO,
  PAGE_SPEEDOMETER,
  PAGE_IMU_DATA, // New page
  PAGE_SETTINGS,
  NUM_PAGES // Helper to count number of pages
};
AppPage currentPage = PAGE_HOME;
const char* pageTitles[NUM_PAGES] = {"Home", "GPS Info", "Speedometer", "IMU Data", "Settings"};

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_MPU6050 mpu;
Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp; // Default I2C address 0x77 or 0x76, library handles detection

#define TEMT6000_PIN 4 // ADC1_CH3 for TEMT6000 Analog Light Sensor

// Global variables for sensor readings
float aht_temperature_c = -999.0; // AHT20 temperature
float aht_humidity_percent = -999.0;    // AHT20 humidity
float bmp_temperature_c = -999.0; // BMP280 temperature
float bmp_pressure_hpa = -999.0;   // BMP280 pressure
int   temt6000_light_raw = -1;   // TEMT6000 raw ADC value

// Global Variables for Modem Status
int currentSignalRSSI = -1; // 0-31 is valid, 99 is not known/detectable
int currentSignalBER = -1;
unsigned long lastSignalCheckTime = 0;
const unsigned long signalCheckInterval = 30000; // 30 seconds
String atResponseBuffer = ""; // Buffer for AT command responses

// NTP Configuration (Global Variables)
const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "cn.pool.ntp.org";
const char* ntpServer3 = "time.windows.com";
long gmtOffset_sec = 8 * 3600; // UTC+8 for Beijing
int daylightOffset_sec = 0;   // No DST

struct tm timeinfo;
bool timeSynchronized = false;
unsigned long lastNtpSyncAttempt = 0;
unsigned long lastNtpSuccessTime = 0; // Tracks the last successful sync
const unsigned long ntpSyncInterval = 3600000; // Sync every 1 hour if successful
const unsigned long ntpRetryInterval = 60000;  // Retry every 1 minute if not yet synced
char timeString[6] = "--:--"; // Buffer for HH:MM formatted time
bool needsRedraw = false; // Flag to redraw display when data changes

// Global Weather String
String weatherSummary = "None"; 

// Settings Page Global Variables
bool isMpu6050Enabled = true;
int settingsMenu_selectedItem = 0;
const char* settingsMenuItems[] = {"NTP Status", "GPS Status", "MPU6050"};
const int settingsMenu_itemCount = sizeof(settingsMenuItems) / sizeof(char*);

// Global GPS Data Variables
bool gpsHasFix = false;
double gpsLatitude = 0.0;
double gpsLongitude = 0.0;
double gpsAltitude = 0.0;    // meters
double gpsSpeedKmph = 0.0;
double gpsCourseDeg = 0.0;   // heading
uint32_t gpsSatellites = 0; // Changed from int to uint32_t to match TinyGPS++ type
String gpsStatusString = "No GPS"; 
unsigned long lastGpsDataTime = 0; // To check for stale data

// Button state variables
int buttonPins[NUM_BUTTONS] = {UP_BUTTON_PIN, DOWN_BUTTON_PIN, SELECT_BUTTON_PIN, BACK_BUTTON_PIN};
bool lastButtonStates[NUM_BUTTONS];
bool currentButtonStates[NUM_BUTTONS];
unsigned long lastDebounceTime[NUM_BUTTONS] = {0};

// Global variables for MPU6050 sensor data
sensors_event_t accelEvent, gyroEvent, tempEvent;

// --- Function Declarations for Page Drawing ---
void drawCurrentPage();
void drawStatusBar();
void drawHomePage();
void drawGpsInfoPage();
void drawSpeedometerPage();
void drawImuDataPage();
void drawSettingsPage();
void readMpuData();
void readEnvironmentalSensors();
void clearLTEBuffer();
bool sendATCommand(const String& cmd, unsigned long timeout, bool expectOK);
bool checkModem();
bool getSignalStrength(int &rssi, int &ber);
bool powerOnGPS();
bool updateLocalTime();
void initNtpTime();
// --- End Function Declarations ---


void setup() {
  // initialize serial communication
  Serial.begin(115200);
  Serial.println(F("ESP32-S3 Project Initialized"));

  // Initialize LTE Serial
  // pinMode(LTE_SERIAL_RX_PIN, INPUT_PULLUP); // Often handled by begin
  // pinMode(LTE_SERIAL_TX_PIN, OUTPUT);      // Often handled by begin
  LTE_Serial.begin(115200, SERIAL_8N1, LTE_SERIAL_RX_PIN, LTE_SERIAL_TX_PIN);
  delay(1000); // Wait for modem to be ready

  Serial.print("Checking modem connection... "); Serial.println(checkModem() ? "OK" : "Failed");
  getSignalStrength(currentSignalRSSI, currentSignalBER);
  Serial.print("Initial RSSI: "); Serial.println(currentSignalRSSI);
  powerOnGPS();

  Serial.println("Attempting to enable NMEA sentences...");
  // Try a few common commands. Response indicates if command was accepted, not necessarily if GPS has fix.
  if (sendATCommand("AT+CGPSNMEA=1", 1000, true)) { // Some modules use this
      Serial.println("NMEA enabled via AT+CGPSNMEA=1");
  } else if (sendATCommand("AT+CNMEA=1", 1000, true)) { // Others use this (e.g. some SIMCom)
      Serial.println("NMEA enabled via AT+CNMEA=1");
  } else if (sendATCommand("AT+CGPSOUT=2", 1000, true)) { // For enabling GGA, GLL, GSA, GSV, RMC, VTG (e.g. some Telit modules)
      Serial.println("NMEA enabled via AT+CGPSOUT=2");
  } else if (sendATCommand("AT+CGNSNMEA=1", 1000, true)) { // Another variant
      Serial.println("NMEA enabled via AT+CGNSNMEA=1");
  } else {
      Serial.println("Failed to enable NMEA streaming with common commands. GPS data might not be received.");
      Serial.println("You might need to use AT+CGNSINF or AT+CGPSINFO for polling if streaming fails.");
  }

  // In setup(), after LTE_Serial.begin() and basic modem checks/GPS power on
  // Call this, but it will only succeed if network is actually up.
  // A more robust way would be to confirm PDP context activation first.
  if (currentSignalRSSI > 5 && currentSignalRSSI != 99) { // Basic check for signal
      Serial.println("Attempting initial NTP synchronization...");
      initNtpTime();
  } else {
      Serial.println("Skipping initial NTP sync due to low/no signal.");
  }

  // Initialize I2C bus if not already done by display
  // Wire.begin(); // Typically, display.begin() handles this. If MPU fails, uncomment.

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  Serial.println(F("SSD1306 Initialized"));

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println(F("Failed to find MPU6050 chip"));
    // while (1) { delay(10); } // Decide if to halt
  } else {
    Serial.println(F("MPU6050 Found!"));
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG); // Corrected constant
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  Serial.println("Initializing AHT20...");
  if (!aht.begin()) {
    Serial.println(F("Could not find AHT20? Check wiring!"));
    // Decide if to halt: for(;;);
  } else {
    Serial.println(F("AHT20 Found!"));
  }

  Serial.println("Initializing BMP280...");
  // bmp.begin() will try both 0x77 and 0x76 addresses.
  // unsigned status = bmp.begin(BMP280_ADDRESS_ALT); // Use if specific address is known (0x76 or 0x77)
  if (!bmp.begin()) { 
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or I2C address!"));
    // Decide if to halt: for(;;);
  } else {
    Serial.println(F("BMP280 Found!"));
    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  }
  
  // pinMode(TEMT6000_PIN, INPUT); // analogRead doesn't strictly require this, but good practice
  Serial.println("TEMT6000 Light Sensor on ADC pin " + String(TEMT6000_PIN));

  // Initialize button pins
  for (int i = 0; i < NUM_BUTTONS; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
    lastButtonStates[i] = digitalRead(buttonPins[i]);
    currentButtonStates[i] = lastButtonStates[i];
  }
  Serial.println(F("Buttons Initialized"));

  // Display initial page
  drawCurrentPage(); 
}

void readMpuData() {
    if (!isMpu6050Enabled) {
        // Set sensor data to zero or specific "disabled" values
        accelEvent.acceleration.x = 0; accelEvent.acceleration.y = 0; accelEvent.acceleration.z = 0;
        gyroEvent.gyro.x = 0; gyroEvent.gyro.y = 0; gyroEvent.gyro.z = 0;
        tempEvent.temperature = -273.15; // Absolute zero, clearly indicating disabled or error
        return; // Skip reading from sensor
    }
  mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);
}

// Clears any pending data from LTE_Serial
void clearLTEBuffer() {
  while (LTE_Serial.available()) {
    LTE_Serial.read();
  }
}

// Sends an AT command and stores the response in a global buffer or returns it.
// Returns true if "OK" is found in response (if expectOK is true)
// Response is available in global atResponseBuffer
bool sendATCommand(const String& cmd, unsigned long timeout, bool expectOK) {
  Serial.print("Sending AT: "); Serial.println(cmd); // Debug
  clearLTEBuffer();
  LTE_Serial.println(cmd);

  atResponseBuffer = "";
  unsigned long startTime = millis();
  bool foundOK = false;
  String line;

  while (millis() - startTime < timeout) {
    if (LTE_Serial.available()) {
      char c = LTE_Serial.read();
      atResponseBuffer += c;
      // Read line by line for easier parsing
      if (c == '\n') {
         Serial.print("LTE << "); Serial.print(line); // Debug: print each line
         if (line.startsWith("OK")) foundOK = true;
         if (line.startsWith("ERROR")) return false; // Or handle error specifically
         // Add other terminators like "> " for SMS input if needed in future
         if (expectOK && foundOK) break; 
         line = ""; // Reset line buffer
      } else if (c != '\r') { // Ignore carriage return for line buffering
         line +=c;
      }
    }
  }
  Serial.print("Full AT Response: "); Serial.println(atResponseBuffer); // Debug
  if (expectOK) return foundOK;
  return (atResponseBuffer.length() > 0); // Return true if any response if not expecting OK
}

bool checkModem() {
  return sendATCommand("AT", 1000, true);
}

bool getSignalStrength(int &rssi, int &ber) {
  if (sendATCommand("AT+CSQ", 1000, true)) {
    // Example Response: +CSQ: 19,0 (sometimes with OK on next line)
    // Find the line with +CSQ
    int csqIdx = atResponseBuffer.indexOf("+CSQ:");
    if (csqIdx != -1) {
      String csqLine = "";
      for(int i=csqIdx; i<atResponseBuffer.length(); ++i) {
         if(atResponseBuffer[i] == '\n') break;
         csqLine += atResponseBuffer[i];
      }
      // Parse "RSSI,BER" from "+CSQ: RSSI,BER"
      sscanf(csqLine.c_str(), "+CSQ: %d,%d", &rssi, &ber);
      return true;
    }
  }
  rssi = 99; ber = 99; // Indicate error or no value
  return false;
}

// Attempt to power on GPS module for DX-CT511N
bool powerOnGPS() {
  Serial.println("Powering on GPS with DX-CT511N specific commands...");
  if (sendATCommand("AT+MGPSC=1", 2000, true)) { // Increased timeout slightly
    Serial.println("AT+MGPSC=1 (GPS Power ON) OK.");
    // Module defaults to NMEA output at 1Hz after AT+MGPSC=1.
    // AT+GPSMODE=1 for hot start is a configuration, not strictly required for NMEA.
    // AT+MGPSGET=1 ensures NMEA is on, good to confirm.
    
    // Set Hot Start Mode
    if (sendATCommand("AT+GPSMODE=1", 1000, true)) {
      Serial.println("AT+GPSMODE=1 (Set Hot Start) OK.");
    } else {
      Serial.println("AT+GPSMODE=1 failed. Continuing..."); // Not critical for NMEA output itself
    }

    // Ensure NMEA output is enabled (though it's default)
    if (sendATCommand("AT+MGPSGET=1", 1000, true)) {
      Serial.println("AT+MGPSGET=1 (Enable NMEA Output) OK.");
    } else {
      Serial.println("AT+MGPSGET=1 failed. NMEA might still be default."); // Not critical if default is active
    }
    return true; // Primary success is AT+MGPSC=1
  } else {
    Serial.println("AT+MGPSC=1 (GPS Power ON) FAILED.");
    return false;
  }
}

void drawStatusBar() {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  String sigStr = "SIG:" + String(currentSignalRSSI);
  // Could convert RSSI to bars: 0-9 weak, 10-14 ok, 15-19 good, 20+ excellent
  // 31 is max. 99 is no signal.
  if (currentSignalRSSI == 99 || currentSignalRSSI < 0) sigStr = "SIG:---";

  display.print(sigStr);
  display.print(F(" | GPS:")); 
  display.print(gpsStatusString);
  // display.drawLine(0, 8, SCREEN_WIDTH-1, 8, SSD1306_WHITE); // Optional separator line
}

void drawHomePage() {
  readEnvironmentalSensors(); // Get fresh data

  // Display Time
  display.setTextSize(2); // Larger text for time
  display.setCursor(5, 10); // Top-left below status bar
  display.print(timeString); 
  display.setTextSize(1); // Reset text size for other elements

  // Environmental Sensor Data - adjust cursor to be below time
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 30); // Position below time display

  display.print(F("Temp (AHT): ")); display.print(aht_temperature_c, 1); display.println(F(" C"));
  display.print(F("Humid (AHT): ")); display.print(aht_humidity_percent, 1); display.println(F(" %"));
  display.print(F("Pres (BMP): ")); display.print(bmp_pressure_hpa, 1); display.println(F(" hPa"));
  // display.print(F("Temp (BMP): ")); display.print(bmp_temperature_c, 1); display.println(F(" C")); // Optional: BMP temp
  display.print(F("Light (アナ): ")); display.println(temt6000_light_raw); // Raw ADC value

  // Weather Display Placeholder
  display.setCursor(0, 45); // Positioned above the [Home Page] footer
  display.print(F("Weather: "));
  display.print(weatherSummary);

  display.setCursor(0,55); // Bottom of screen
  display.print(F("[Home Page]")); // Placeholder for page title area
}

void drawGpsInfoPage() {
  // Called when currentPage is PAGE_GPS_INFO
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 10); // Below status bar

  display.print(F("Lat: ")); display.println(gpsLatitude, 6); // 6 decimal places
  display.print(F("Lon: ")); display.println(gpsLongitude, 6);
  display.print(F("Alt: ")); display.print(gpsAltitude, 1); display.println(F(" m"));
  display.print(F("Spd: ")); display.print(gpsSpeedKmph, 1); display.println(F(" km/h"));
  display.print(F("Cog: ")); display.print(gpsCourseDeg, 1); display.println(F(" deg"));
  display.print(F("Sat: ")); display.println(gpsSatellites);
  display.print(F("Fix: ")); display.println(gpsHasFix ? "Yes" : "No");
  display.print(F("NMEA: ")); display.println(gps.sentencesWithFix()); // Display count of sentences with fix
  display.print(F("Chars: ")); display.println(gps.charsProcessed()); // Total NMEA chars processed
}

void drawSpeedometerPage() {
  display.setCursor(0, 10); // Below status bar
  display.setTextSize(1);
  display.print("Speed"); // Title for the page

  // Display speed in large font
  display.setTextSize(4); // Large font for speed integer part
  display.setCursor(10, 25); // Adjust for centering
  
  int speedInt = static_cast<int>(gpsSpeedKmph);
  if (speedInt < 10) display.print(F("0")); // Pad with 0 for single digit
  display.print(speedInt);

  // Display "km/h" unit smaller and to the right/bottom
  display.setTextSize(1);
  display.setCursor(90, 55); // Bottom right
  display.print(F("km/h"));
}

void drawImuDataPage() {
    readMpuData(); // This will now respect isMpu6050Enabled

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 10); // Y position below status bar

    if (!isMpu6050Enabled) {
        display.setTextSize(2); // Larger text
        display.setCursor(15, 25); // Centered
        display.println(F("MPU6050"));
        display.setCursor(15, 45);
        display.println(F("DISABLED"));
    } else {
        // Existing code to display accelerometer, gyroscope, and temperature data
        display.print(F("Accel X: ")); display.println(accelEvent.acceleration.x);
        display.print(F("Accel Y: ")); display.println(accelEvent.acceleration.y);
        display.print(F("Accel Z: ")); display.println(accelEvent.acceleration.z);
        
        display.print(F("Gyro X: ")); display.println(gyroEvent.gyro.x);
        display.print(F("Gyro Y: ")); display.println(gyroEvent.gyro.y);
        display.print(F("Gyro Z: ")); display.println(gyroEvent.gyro.z);

        display.print(F("Temp: ")); display.print(tempEvent.temperature, 1); display.println(F(" C"));
    }
}

void drawSettingsPage() {
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 10); // Start Y position below status bar

    for (int i = 0; i < settingsMenu_itemCount; i++) {
        if (i == settingsMenu_selectedItem) {
            display.print(F("> "));
        } else {
            display.print(F("  "));
        }

        // Set cursor for item text, ensuring alignment
        display.setCursor(12, 10 + (i * 10)); // 12 for X (after "> "), Y increments by 10px per line

        display.print(settingsMenuItems[i]);
        
        // Add specific value display logic after ":"
        // display.setCursor(70, 10 + (i * 10)); // Example X for value, adjust as needed
        // For simplicity, let's print directly after the item name + ": "
        display.print(F(": "));


        if (strcmp(settingsMenuItems[i], "NTP Status") == 0) {
            display.println(timeSynchronized ? F("Synced") : F("No Sync"));
        } else if (strcmp(settingsMenuItems[i], "GPS Status") == 0) {
            display.println(gpsStatusString);
        } else if (strcmp(settingsMenuItems[i], "MPU6050") == 0) {
            display.print(F("["));
            display.print(isMpu6050Enabled ? F("On") : F("Off"));
            display.println(F("]"));
        } else {
            display.println(); // Should not happen with current items
        }
    }
}

void readEnvironmentalSensors() {
  sensors_event_t humidity_event, temp_event;
  if (aht.getEvent(&humidity_event, &temp_event)) { // getEvent() returns true on success
      aht_temperature_c = temp_event.temperature;
      aht_humidity_percent = humidity_event.relative_humidity;
  } else {
      // Keep previous values or set to error indication
      Serial.println(F("Failed to read AHT20 data"));
      aht_temperature_c = -998.0; 
      aht_humidity_percent = -998.0;
  }

  // Check if BMP280 was found (e.g. by checking if bmp_pressure_hpa is still initial error value, or add a bmp_found flag)
  // For now, assume if begin() passed, it's readable, but real code might check sensor presence before read.
  // A more robust check for BMP280 would be to check if bmp.sensorID() returns a valid ID after begin().
  // Or, check if (!bmp.begin()) was true and set a flag.
  // For this implementation, we will rely on the initial value of bmp_pressure_hpa or a successful begin().
  // If bmp.begin() failed, these reads might return NaN or fixed values depending on library.
  bmp_temperature_c = bmp.readTemperature();
  bmp_pressure_hpa = bmp.readPressure() / 100.0F; // Convert Pa to hPa

  temt6000_light_raw = analogRead(TEMT6000_PIN);
}

void drawCurrentPage() {
  display.clearDisplay();
  drawStatusBar(); // Draw status bar first

  // Set text properties for page content (can be overridden by specific page functions)
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  switch (currentPage) {
    case PAGE_HOME:
      drawHomePage();
      break;
    case PAGE_GPS_INFO:
      drawGpsInfoPage();
      break;
    case PAGE_SPEEDOMETER:
      drawSpeedometerPage();
      break;
    case PAGE_IMU_DATA:
      drawImuDataPage();
      break;
    case PAGE_SETTINGS:
      drawSettingsPage();
      break;
    default:
      // Handle unknown page if necessary
      display.setCursor(10,20);
      display.print(F("Unknown Page"));
      break;
  }
  display.display();
}

void loop() {
  // bool needsRedraw = false; // Flag to redraw only when necessary - MOVED TO GLOBAL

  // Button handling logic
  for (int i = 0; i < NUM_BUTTONS; i++) {
    bool reading = digitalRead(buttonPins[i]);
    if (reading != lastButtonStates[i]) {
      lastDebounceTime[i] = millis();
    }

    if ((millis() - lastDebounceTime[i]) > DEBOUNCE_DELAY) {
      if (reading != currentButtonStates[i]) {
        currentButtonStates[i] = reading;
        if (currentButtonStates[i] == LOW) { // Button pressed
            needsRedraw = true; // Assume redraw is needed, can be overridden if not

            if (currentPage == PAGE_SETTINGS) {
                if (buttonPins[i] == UP_BUTTON_PIN) {
                    settingsMenu_selectedItem = (settingsMenu_selectedItem - 1 + settingsMenu_itemCount) % settingsMenu_itemCount;
                } else if (buttonPins[i] == DOWN_BUTTON_PIN) {
                    settingsMenu_selectedItem = (settingsMenu_selectedItem + 1) % settingsMenu_itemCount;
                } else if (buttonPins[i] == SELECT_BUTTON_PIN) {
                    if (strcmp(settingsMenuItems[settingsMenu_selectedItem], "MPU6050") == 0) {
                        isMpu6050Enabled = !isMpu6050Enabled;
                        Serial.print(F("MPU6050 toggled: ")); Serial.println(isMpu6050Enabled ? "Enabled" : "Disabled");
                        // If MPU was fully de-initialized, might need re-init. For now, assumed not.
                    }
                    // Add other SELECT actions for future settings items here
                } else if (buttonPins[i] == BACK_BUTTON_PIN) {
                    currentPage = PAGE_HOME;
                }
            } else { // Button handling for all other pages
                if (buttonPins[i] == UP_BUTTON_PIN) {
                    currentPage = (AppPage)((currentPage - 1 + NUM_PAGES) % NUM_PAGES);
                } else if (buttonPins[i] == DOWN_BUTTON_PIN) {
                    currentPage = (AppPage)((currentPage + 1) % NUM_PAGES);
                } else if (buttonPins[i] == SELECT_BUTTON_PIN) {
                    Serial.println(F("SELECT Button - Action TBD for this page"));
                    // Define SELECT actions for other pages if needed
                } else if (buttonPins[i] == BACK_BUTTON_PIN) {
                    currentPage = PAGE_HOME; // Default back action
                }
            }
        }
      }
    }
    lastButtonStates[i] = reading;
  }

  if (needsRedraw) {
    drawCurrentPage();
    needsRedraw = false; // Reset flag
  }

  if (millis() - lastSignalCheckTime > signalCheckInterval) {
    if (getSignalStrength(currentSignalRSSI, currentSignalBER)) {
      Serial.print("Updated RSSI: "); Serial.print(currentSignalRSSI);
      Serial.print(", BER: "); Serial.println(currentSignalBER);
      // Potentially set needsRedraw = true; if status bar should update immediately
      // For now, rely on next button press or periodic page refresh if any
      // To force redraw: needsRedraw = true; // Let's make status bar update if signal changes
      if (currentPage == PAGE_HOME || currentPage == PAGE_GPS_INFO) needsRedraw = true; // Redraw if on relevant pages
    }
    lastSignalCheckTime = millis();
  }

  while (LTE_Serial.available() > 0) {
    gps.encode(LTE_Serial.read());
  }

  // Update GPS data variables if new data is available
  if (gps.location.isUpdated() || gps.satellites.isUpdated() || gps.speed.isUpdated()) { // Check if any relevant part updated
      lastGpsDataTime = millis(); // Record time of any GPS update
      if (gps.location.isValid()) {
          gpsHasFix = true;
          gpsLatitude = gps.location.lat();
          gpsLongitude = gps.location.lng();
          gpsStatusString = "Fix (" + String(gps.satellites.value()) + ")";
      } else {
          // gpsHasFix = false; // Don't immediately set to false if other data is coming
          // gpsStatusString = "Searching...";
      }

      if (gps.altitude.isValid()) gpsAltitude = gps.altitude.meters();
      if (gps.speed.isValid()) gpsSpeedKmph = gps.speed.kmph();
      if (gps.course.isValid()) gpsCourseDeg = gps.course.deg();
      if (gps.satellites.isValid()) gpsSatellites = gps.satellites.value();

      // If location becomes invalid after having been valid, or age is too high
      if (!gps.location.isValid() || gps.location.age() > 7000) { // 7 seconds
          if (gpsHasFix) { // Was previously fixed
              needsRedraw = true; // Force redraw if fix is lost
          }
          gpsHasFix = false;
          if (gps.satellites.isValid() && gps.satellites.value() > 0) {
              gpsStatusString = "Search(" + String(gps.satellites.value()) + ")";
          } else if (gps.charsProcessed() > 0 && gps.sentencesWithFix() == 0 && gps.failedChecksum() < gps.passedChecksum()) {
              // Receiving data, but no fix yet and checksums are mostly good
              gpsStatusString = "Acquiring";
          }
          else {
              gpsStatusString = "No GPS";
          }
      }
       if (gpsHasFix) needsRedraw = true; // Redraw if we have a fix and data changes
  }

  // Check for stale data if nothing received from GPS for a while
  if (millis() - lastGpsDataTime > 10000 && gpsHasFix) { // 10 seconds without any GPS characters
      Serial.println("GPS data stale, resetting status.");
      gpsHasFix = false;
      gpsStatusString = "Stale";
      gpsSatellites = 0;
      needsRedraw = true;
  } else if (millis() - lastGpsDataTime > 15000 && !gpsHasFix && gpsStatusString != "No GPS") {
      // If no data at all for a while, and no fix, assume "No GPS"
      gpsStatusString = "No GPS";
      needsRedraw = true;
  }

  // NTP Logic
  unsigned long currentTime = millis();

  // Periodic sync or retry logic
  if (timeSynchronized && (currentTime - lastNtpSuccessTime > ntpSyncInterval)) {
      Serial.println("Periodic NTP sync initiated...");
      if (currentSignalRSSI > 5 && currentSignalRSSI != 99) {
        updateLocalTime(); // Attempt to re-sync
      } else {
        Serial.println("Skipping periodic NTP sync, no signal.");
      }
      lastNtpSyncAttempt = currentTime; // Update attempt time regardless
  } else if (!timeSynchronized && (currentTime - lastNtpSyncAttempt > ntpRetryInterval)) {
      Serial.println("Retrying NTP sync...");
      if (currentSignalRSSI > 5 && currentSignalRSSI != 99) { // Basic check for signal
        updateLocalTime();
      } else {
        Serial.println("Skipping NTP retry, no signal.");
      }
      lastNtpSyncAttempt = currentTime;
  }

  // If time is synced, continuously update the local timeinfo struct and timeString for display
  if (timeSynchronized) {
    static int lastMinute = -1;
    if (getLocalTime(&timeinfo, 0)) { // Quick non-blocking update
        if (timeinfo.tm_min != lastMinute) {
            sprintf(timeString, "%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
            lastMinute = timeinfo.tm_min;
            needsRedraw = true; // Redraw if time string changed
        }
    }
  } else {
    // Ensure timeString is "--:--" if not synced
    if (strcmp(timeString, "--:--") != 0) {
        strcpy(timeString, "--:--");
        needsRedraw = true;
    }
  }
  
  delay(10); // Adjust as needed
}

// --- NTP Functions ---
bool updateLocalTime() {
  if (!getLocalTime(&timeinfo, 5000)) { // 5 second timeout to get time
    Serial.println("Failed to obtain time via NTP.");
    // timeSynchronized = false; // Keep previous status until retry logic decides
    return false;
  }
  Serial.print("Time successfully synchronized/updated: ");
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S"); // Print full date and time
  timeSynchronized = true;
  lastNtpSuccessTime = millis();
  sprintf(timeString, "%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
  needsRedraw = true; // Request redraw to update time on screen
  return true;
}

void initNtpTime() {
  Serial.println("Configuring module's NTP servers (AT+QNTP)...");
  // Using primary and secondary NTP servers defined globally
  // The exact syntax for setting multiple servers can vary.
  // Common patterns: AT+QNTP="server1","server2" or AT+QNTP="server1",port,"server2",port
  // Based on "AT+QNTP查询/设置NTP服务器", a likely syntax for setting is:
  String ntpATCmd = "AT+QNTP=\"" + String(ntpServer1) + "\",\"" + String(ntpServer2) + "\"";
  // This creates: AT+QNTP="pool.ntp.org","cn.pool.ntp.org"

  if (sendATCommand(ntpATCmd, 5000, true)) { // Expect "OK"
    Serial.println("Module NTP servers configured via AT+QNTP (primary/secondary).");
  } else {
    Serial.println("Failed to configure module NTP servers with primary/secondary syntax.");
    // Try setting only primary server as a fallback or if that's the module's expectation
    ntpATCmd = "AT+QNTP=\"" + String(ntpServer1) + "\"";
    if (sendATCommand(ntpATCmd, 5000, true)) {
        Serial.println("Module NTP primary server configured via AT+QNTP (primary only).");
    } else {
        Serial.println("Failed to configure module NTP server via AT+QNTP (primary only also).");
    }
  }
  // After attempting to set module's NTP, proceed with ESP32 configTime
  Serial.println("Configuring ESP32 NTP (UTC+8, no DST). Servers: " + String(ntpServer1) + ", " + String(ntpServer2) + ", " + String(ntpServer3));
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2, ntpServer3);
  lastNtpSyncAttempt = millis(); 
  updateLocalTime(); 
}
