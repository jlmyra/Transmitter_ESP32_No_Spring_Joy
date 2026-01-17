/*
 * ESP-NOW Transmitter for MultiWii ESP32 Drone
 * Hardware: MH-ET LIVE MiniKit (ESP32)
 * 
 * UPDATED: 
 * - Added __attribute__((packed)) to match receiver
 * - Added Linear Throttle mapping (no center deadzone) for spring-less stick
 * - Added Deadzone for Yaw/Pitch/Roll
 */

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ============================================================================
//                              CONFIGURATION
// ============================================================================

// IMPORTANT: Replace with your drone's MAC address!
uint8_t RECEIVER_MAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 

// ------------ PIN CONFIGURATION (MH-ET LIVE MiniKit) ------------
#define PIN_THROTTLE  36  // Left stick Y-axis
#define PIN_YAW       39  // Left stick X-axis
#define PIN_PITCH     34  // Right stick Y-axis
#define PIN_ROLL      35  // Right stick X-axis

#define PIN_AUX1      26  // Switch 1
#define PIN_AUX2      27  // Switch 2
#define PIN_JS_BTN_L  4   // Left stick button
#define PIN_JS_BTN_R  16  // Right stick button

// ------------ JOYSTICK CALIBRATION ------------
// {min, center, max}
// Use the Serial Monitor to read your specific min/max values and update these!
int throttleCal[] = {0, 2048, 4095}; // Throttle doesn't use center, but keep format
int yawCal[]      = {0, 2048, 4095};
int pitchCal[]    = {0, 2048, 4095};
int rollCal[]     = {0, 2048, 4095};

// Reverse settings
// Set to 'false' if Stick Down/Left = 0 voltage
// Set to 'true' if Stick Down/Left = Max voltage
bool throttleReverse = false; // Set false so Down = 0 -> Send 0 -> RX maps to 1000
bool yawReverse      = true;
bool pitchReverse    = true;
bool rollReverse     = true;

// Deadzone for centered sticks (Yaw, Pitch, Roll)
#define DEADZONE 30 

// ============================================================================
//                    DATA STRUCTURE - MUST MATCH RECEIVER!
// ============================================================================

typedef struct __attribute__((packed)) struct_message {
  uint8_t throttle;  // 0-255
  uint8_t yaw;       // 0-255
  uint8_t pitch;     // 0-255
  uint8_t roll;      // 0-255
  uint8_t AUX1;      // 0 or 1
  uint8_t AUX2;      // 0 or 1
  uint8_t switches;  // Bit flags
} struct_message;

typedef struct __attribute__((packed)) struct_ack {
  uint8_t vbat;
  uint8_t rssi;
  int16_t heading;
  int16_t pitch;
  int16_t roll;
  int16_t alt;
  uint8_t flags;
} struct_ack;

// ============================================================================
//                              GLOBALS
// ============================================================================

struct_message txData;
struct_ack rxTelemetry;
LiquidCrystal_I2C lcd(0x27, 16, 2);
esp_now_peer_info_t peerInfo;

unsigned long lastTxTime = 0;
unsigned long lastLcdTime = 0;
unsigned long lastTelemetryTime = 0;
bool telemetryReceived = false;

// Smoothing
float sThrottle = 0, sYaw = 0, sPitch = 0, sRoll = 0;
const float alpha = 0.4; 

// ============================================================================
//                              FUNCTIONS
// ============================================================================

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Optional: Handle delivery success/fail
}

void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *data, int len) {
  if (len == sizeof(struct_ack)) {
    memcpy(&rxTelemetry, data, sizeof(rxTelemetry));
    telemetryReceived = true;
    lastTelemetryTime = millis();
  }
}

// Map function for Centered Sticks (Yaw, Pitch, Roll) with Deadzone
int mapJoystickValues(int val, int lower, int middle, int upper, bool reverse) {
  val = constrain(val, lower, upper);
  
  // Deadzone logic
  if (val > (middle - DEADZONE) && val < (middle + DEADZONE)) {
    return 128; // Center
  }
  
  int output;
  if (val < middle)
    output = map(val, lower, middle - DEADZONE, 0, 128);
  else
    output = map(val, middle + DEADZONE, upper, 128, 255);
    
  return (reverse ? 255 - output : output);
}

// Map function for Throttle (Linear, No Center Deadzone)
int mapThrottleValues(int val, int lower, int upper, bool reverse) {
  val = constrain(val, lower, upper);
  int output = map(val, lower, upper, 0, 255);
  return (reverse ? 255 - output : output);
}

void readInputs() {
  // Read Raw (ESP32 is 12-bit 0-4095)
  int rawThrottle = analogRead(PIN_THROTTLE);
  int rawYaw      = analogRead(PIN_YAW);
  int rawPitch    = analogRead(PIN_PITCH);
  int rawRoll     = analogRead(PIN_ROLL);

  // Smooth
  sThrottle = (sThrottle * (1.0 - alpha)) + (rawThrottle * alpha);
  sYaw      = (sYaw * (1.0 - alpha)) + (rawYaw * alpha);
  sPitch    = (sPitch * (1.0 - alpha)) + (rawPitch * alpha);
  sRoll     = (sRoll * (1.0 - alpha)) + (rawRoll * alpha);

  // Map Throttle (Linear)
  txData.throttle = mapThrottleValues((int)sThrottle, throttleCal[0], throttleCal[2], throttleReverse);

  // Map Others (Centered with Deadzone)
  txData.yaw   = mapJoystickValues((int)sYaw,   yawCal[0],   yawCal[1],   yawCal[2],   yawReverse);
  txData.pitch = mapJoystickValues((int)sPitch, pitchCal[0], pitchCal[1], pitchCal[2], pitchReverse);
  txData.roll  = mapJoystickValues((int)sRoll,  rollCal[0],  rollCal[1],  rollCal[2],  rollReverse);

  // Switches
  txData.AUX1 = !digitalRead(PIN_AUX1);
  txData.AUX2 = !digitalRead(PIN_AUX2);
  
  txData.switches = 0;
  if (!digitalRead(PIN_JS_BTN_L)) txData.switches |= 0x01;
  if (!digitalRead(PIN_JS_BTN_R)) txData.switches |= 0x02;
}

void updateLCD() {
  lcd.setCursor(0, 0);
  if (telemetryReceived && (millis() - lastTelemetryTime < 2000)) {
    lcd.print(rxTelemetry.vbat / 10.0, 1); lcd.print("V ");
    lcd.print(rxTelemetry.rssi); lcd.print("% ");
    lcd.print(rxTelemetry.flags & 0x01 ? "ARM" : "DIS");
  } else {
    lcd.print("Connecting...");
  }
  
  lcd.setCursor(0, 1);
  lcd.print("T:"); lcd.print(txData.throttle);
  lcd.print(" M:"); 
  if (txData.AUX1) lcd.print("STAB"); else lcd.print("ACRO");
}

// ============================================================================
//                              SETUP & LOOP
// ============================================================================

void setup() {
  Serial.begin(115200);
  
  // Pin Modes
  pinMode(PIN_AUX1, INPUT_PULLUP);
  pinMode(PIN_AUX2, INPUT_PULLUP);
  pinMode(PIN_JS_BTN_L, INPUT_PULLUP);
  pinMode(PIN_JS_BTN_R, INPUT_PULLUP);

  // LCD
  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  lcd.print("TX Starting...");

  // WiFi
  WiFi.mode(WIFI_STA);
  Serial.print("Transmitter MAC: ");
  Serial.println(WiFi.macAddress());

  // Validate receiver MAC address (check if still default broadcast)
  bool macIsDefault = true;
  for (int i = 0; i < 6; i++) {
    if (RECEIVER_MAC[i] != 0xFF) {
      macIsDefault = false;
      break;
    }
  }
  if (macIsDefault) {
    Serial.println("\n*** WARNING: RECEIVER_MAC not configured! ***");
    Serial.println("Edit line 21 with your drone's MAC address.");
    lcd.clear();
    lcd.print("SET DRONE MAC!");
    lcd.setCursor(0, 1);
    lcd.print("See Serial Mon");
    delay(3000);
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Register Peer
  memcpy(peerInfo.peer_addr, RECEIVER_MAC, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
  }

  // Safety Check: Wait for throttle down
  readInputs();
  while(txData.throttle > 10) {
    lcd.setCursor(0,0); lcd.print("WARNING:      ");
    lcd.setCursor(0,1); lcd.print("LOWER THROTTLE");
    readInputs();
    delay(100);
  }
  lcd.clear();
}

void loop() {
  unsigned long now = millis();

  if (now - lastTxTime >= 20) { // 50Hz
    lastTxTime = now;
    readInputs();
    esp_now_send(RECEIVER_MAC, (uint8_t *)&txData, sizeof(txData));
  }

  if (now - lastLcdTime >= 100) {
    lastLcdTime = now;
    updateLCD();
  }
}
