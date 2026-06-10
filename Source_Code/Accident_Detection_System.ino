#include <Wire.h>
#include <math.h>
#include <string.h>

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

#include <TinyGPS++.h>
#include <HardwareSerial.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// =====================================================
// WIFI CONFIGURATION
// =====================================================

const char* ssid = "xxxxxx";
const char* password = "xxxxxx";

// =====================================================
// TELEGRAM CONFIGURATION
// =====================================================

#define BOT_TOKEN "xxxxxxxxxxxxxx"
#define CHAT_ID "xxxxxxx"

// =====================================================
// FAST2SMS CONFIGURATION
// =====================================================

String fast2smsApiKey = "xxxxxxxxxxxxxxxxx";
String smsNumbers = "xxxxxxxxxxxxxxx";

// =====================================================
// PIN CONFIGURATION
// =====================================================

#define MPU_SDA 21
#define MPU_SCL 22

#define OLED_SDA 32
#define OLED_SCL 33

#define GPS_RX 16
#define GPS_TX 17

#define BUZZER_PIN 27
#define CANCEL_BUTTON_PIN 25

// =====================================================
// OLED CONFIGURATION
// =====================================================

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

TwoWire OLEDWire = TwoWire(1);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &OLEDWire, OLED_RESET);

// =====================================================
// GPS CONFIGURATION
// =====================================================

TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

double currentLat = 0.0;
double currentLng = 0.0;
float currentSpeed = 0.0;
bool gpsLocationAvailable = false;

// =====================================================
// MPU CONFIGURATION
// =====================================================

byte MPU_ADDR = 0x68;

float ACC_THRESHOLD = 1.8;
float GYRO_THRESHOLD = 180.0;

// Use 25.0 for testing.
// Later for final real-vehicle test, tune this value carefully.
float ANGLE_THRESHOLD = 25.0;

float baselineRoll = 0.0;
float baselinePitch = 0.0;

// =====================================================
// ACCIDENT DETECTION VARIABLES
// =====================================================

unsigned long lastImpactTime = 0;
unsigned long lastTiltTime = 0;
unsigned long detectionWindow = 3000;

unsigned long suspectedStartTime = 0;
unsigned long confirmationDelay = 10000;

bool impactDetectedOnce = false;
bool tiltDetectedOnce = false;

bool accidentSuspected = false;
bool accidentConfirmed = false;

bool alertCancelled = false;
unsigned long cancelledMessageTime = 0;
unsigned long cancelCooldownUntil = 0;

// =====================================================
// ALERT STATUS VARIABLES
// =====================================================

bool alertsStarted = false;
bool alertsFinished = false;
bool alertsCompleted = false;

bool telegramAttempted = false;
bool telegramSent = false;

bool smsAttempted = false;
bool smsSent = false;

const int ALERT_SEND_RETRIES = 3;

unsigned long lastPrintTime = 0;
unsigned long lastDisplayTime = 0;

// =====================================================
// MPU BASIC FUNCTIONS
// =====================================================

void writeRegister(byte reg, byte value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

byte readRegister(byte reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (byte)1);

  if (Wire.available()) {
    return Wire.read();
  }

  return 0;
}

int16_t read16() {
  int16_t high = Wire.read();
  int16_t low = Wire.read();
  return (high << 8) | low;
}

bool detectMPU() {
  byte addresses[] = {0x68, 0x69};

  for (int retry = 0; retry < 10; retry++) {
    for (int i = 0; i < 2; i++) {
      MPU_ADDR = addresses[i];

      Wire.beginTransmission(MPU_ADDR);
      byte error = Wire.endTransmission();

      if (error == 0) {
        byte whoami = readRegister(0x75);

        Serial.print("MPU found at address 0x");
        Serial.print(MPU_ADDR, HEX);
        Serial.print(" | WHO_AM_I = 0x");
        Serial.println(whoami, HEX);

        return true;
      }
    }

    Serial.println("Retrying MPU detection...");
    delay(300);
  }

  return false;
}

bool readMotion(float &accMagnitude, float &gyroMagnitude, float &roll, float &pitch) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (byte)14);

  if (Wire.available() != 14) {
    return false;
  }

  int16_t rawAx = read16();
  int16_t rawAy = read16();
  int16_t rawAz = read16();

  read16();

  int16_t rawGx = read16();
  int16_t rawGy = read16();
  int16_t rawGz = read16();

  float ax = rawAx / 2048.0;
  float ay = rawAy / 2048.0;
  float az = rawAz / 2048.0;

  float gx = rawGx / 16.4;
  float gy = rawGy / 16.4;
  float gz = rawGz / 16.4;

  accMagnitude = sqrt(ax * ax + ay * ay + az * az);
  gyroMagnitude = sqrt(gx * gx + gy * gy + gz * gz);

  roll = atan2(ay, az) * 180.0 / PI;
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

  return true;
}

float angleDifference(float currentAngle, float baseAngle) {
  float diff = currentAngle - baseAngle;

  while (diff > 180.0) diff -= 360.0;
  while (diff < -180.0) diff += 360.0;

  return fabs(diff);
}

// =====================================================
// OLED FUNCTIONS
// =====================================================

bool initOLED() {
  OLEDWire.begin(OLED_SDA, OLED_SCL);
  OLEDWire.setClock(100000);
  delay(500);

  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C, true, false)) {
    display.clearDisplay();
    display.display();
    Serial.println("OLED found at 0x3C");
    return true;
  }

  delay(500);

  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3D, true, false)) {
    display.clearDisplay();
    display.display();
    Serial.println("OLED found at 0x3D");
    return true;
  }

  return false;
}

void showOLEDMessage(String line1, String line2 = "", String line3 = "", String line4 = "") {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println(line1);

  display.setCursor(0, 16);
  display.println(line2);

  display.setCursor(0, 32);
  display.println(line3);

  display.setCursor(0, 48);
  display.println(line4);

  display.display();
}

void updateOLED(float accMag, float gyroMag, float rollChange, float pitchChange, const char* status) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("ACC:");
  display.print(accMag, 1);
  display.print(" G:");
  display.print(gyroMag, 0);

  display.setCursor(0, 12);
  display.print("GPS:");
  display.print(gpsLocationAvailable ? "LOCK" : "WAIT");
  display.print(" S:");
  display.print(currentSpeed, 0);

  display.setCursor(0, 24);
  display.print("WiFi:");
  display.print(WiFi.status() == WL_CONNECTED ? "OK" : "NO");

  display.setCursor(0, 36);

  if (alertsCompleted) {
    display.print("ALERTS SENT");
  }
  else if (accidentConfirmed && alertsStarted && !alertsFinished) {
    display.print("SENDING ALERTS");
  }
  else if (accidentConfirmed && alertsFinished) {
    display.print("ALERT RESULT");
  }
  else if (accidentConfirmed) {
    display.print("ACCIDENT CONFIRMED");
  }
  else {
    display.print(status);
  }

  display.setCursor(0, 48);

  if (accidentConfirmed) {
    display.print("TG:");
    display.print(telegramSent ? "OK" : "NO");
    display.print(" SMS:");
    display.print(smsSent ? "OK" : "NO");
  } 
  else {
    display.print("R:");
    display.print(rollChange, 0);
    display.print(" P:");
    display.print(pitchChange, 0);
  }

  display.display();
}

// =====================================================
// BASELINE CALIBRATION
// =====================================================

void calibrateBaseline() {
  Serial.println("Keep device still. Calibrating baseline...");
  showOLEDMessage("Keep Still", "Calibrating...");

  float sumSinRoll = 0.0;
  float sumCosRoll = 0.0;
  float sumSinPitch = 0.0;
  float sumCosPitch = 0.0;

  int validSamples = 0;

  for (int i = 0; i < 80; i++) {
    float accMag, gyroMag, roll, pitch;

    if (readMotion(accMag, gyroMag, roll, pitch)) {
      sumSinRoll += sin(roll * PI / 180.0);
      sumCosRoll += cos(roll * PI / 180.0);

      sumSinPitch += sin(pitch * PI / 180.0);
      sumCosPitch += cos(pitch * PI / 180.0);

      validSamples++;
    }

    delay(25);
  }

  if (validSamples > 0) {
    baselineRoll = atan2(sumSinRoll / validSamples, sumCosRoll / validSamples) * 180.0 / PI;
    baselinePitch = atan2(sumSinPitch / validSamples, sumCosPitch / validSamples) * 180.0 / PI;
  }

  Serial.print("Baseline Roll: ");
  Serial.println(baselineRoll);

  Serial.print("Baseline Pitch: ");
  Serial.println(baselinePitch);

  showOLEDMessage("Calibration", "Done");
  delay(1000);
}

// =====================================================
// GPS FUNCTION
// =====================================================

void updateGPS() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isValid()) {
    currentLat = gps.location.lat();
    currentLng = gps.location.lng();
    gpsLocationAvailable = true;
  }

  if (gps.speed.isValid()) {
    currentSpeed = gps.speed.kmph();

    if (currentSpeed < 2.0) {
      currentSpeed = 0.0;
    }
  }
}

// =====================================================
// BUZZER FUNCTIONS
// =====================================================

void passiveBuzzerTone(bool enable, int frequency) {
  static unsigned long lastToggleTime = 0;
  static bool buzzerState = false;

  if (!enable) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerState = false;
    return;
  }

  unsigned long currentMicros = micros();
  unsigned long halfPeriod = 1000000UL / (frequency * 2);

  if (currentMicros - lastToggleTime >= halfPeriod) {
    lastToggleTime = currentMicros;
    buzzerState = !buzzerState;
    digitalWrite(BUZZER_PIN, buzzerState);
  }
}

void handleBuzzer(const char* status) {
  unsigned long currentTime = millis();

  if (alertsCompleted) {
    passiveBuzzerTone(false, 0);
  }
  else if (accidentConfirmed) {
    passiveBuzzerTone(true, 2000);
  } 
  else if (accidentSuspected) {
    if ((currentTime / 300) % 2 == 0) {
      passiveBuzzerTone(true, 1500);
    } else {
      passiveBuzzerTone(false, 0);
    }
  } 
  else if (strcmp(status, "IMPACT DETECTED") == 0) {
    passiveBuzzerTone(true, 1800);
  } 
  else {
    passiveBuzzerTone(false, 0);
  }
}

// =====================================================
// CANCEL BUTTON FUNCTIONS
// =====================================================

bool cancelButtonPressed() {
  static unsigned long lastPressTime = 0;

  if (digitalRead(CANCEL_BUTTON_PIN) == LOW) {
    if (millis() - lastPressTime > 400) {
      lastPressTime = millis();
      return true;
    }
  }

  return false;
}

void resetAccidentState() {
  impactDetectedOnce = false;
  tiltDetectedOnce = false;

  accidentSuspected = false;
  accidentConfirmed = false;

  alertCancelled = false;

  alertsStarted = false;
  alertsFinished = false;
  alertsCompleted = false;

  telegramAttempted = false;
  telegramSent = false;

  smsAttempted = false;
  smsSent = false;

  lastImpactTime = 0;
  lastTiltTime = 0;
  suspectedStartTime = 0;

  passiveBuzzerTone(false, 0);
}

// =====================================================
// WIFI FUNCTION
// =====================================================

void connectWiFi() {
  Serial.println("Connecting to WiFi...");
  showOLEDMessage("Connecting", "WiFi...");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(300);

  WiFi.begin(ssid, password);

  int attempts = 0;

  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    showOLEDMessage("WiFi Connected", WiFi.localIP().toString());
    delay(1000);
  } 
  else {
    Serial.println("WiFi connection failed");
    showOLEDMessage("WiFi Failed", "Alert may fail");
    delay(1000);
  }
}

// =====================================================
// URL ENCODING FUNCTION
// =====================================================

String urlEncode(const String &str) {
  const char *hex = "0123456789ABCDEF";
  String encoded = "";

  for (int i = 0; i < str.length(); i++) {
    uint8_t c = (uint8_t)str.charAt(i);

    if (
      (c >= 'a' && c <= 'z') ||
      (c >= 'A' && c <= 'Z') ||
      (c >= '0' && c <= '9') ||
      c == '-' || c == '_' || c == '.' || c == '~'
    ) {
      encoded += (char)c;
    } 
    else {
      encoded += '%';
      encoded += hex[(c >> 4) & 0x0F];
      encoded += hex[c & 0x0F];
    }
  }

  return encoded;
}

// =====================================================
// ALERT MESSAGE FUNCTION
// =====================================================

String createAlertMessage() {
  String message = "ACCIDENT CONFIRMED\n\n";
  message += "Emergency Alert Generated\n\n";

  if (gpsLocationAvailable) {
    message += "Latitude: ";
    message += String(currentLat, 6);
    message += "\n";

    message += "Longitude: ";
    message += String(currentLng, 6);
    message += "\n";

    message += "Speed: ";
    message += String(currentSpeed, 2);
    message += " km/h\n\n";

    message += "Google Maps Link:\n";
    message += "https://maps.google.com/?q=";
    message += String(currentLat, 6);
    message += ",";
    message += String(currentLng, 6);
  } 
  else {
    message += "GPS Location: Not available\n";
    message += "Move GPS module near open sky/window";
  }

  return message;
}

// =====================================================
// TELEGRAM ALERT FUNCTION
// =====================================================

bool sendTelegramAlertOnce() {
  Serial.println("Preparing Telegram alert...");

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected. Reconnecting...");
    connectWiFi();
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Telegram failed: WiFi not connected");
    return false;
  }

  String message = createAlertMessage();
  String encodedMessage = urlEncode(message);

  String url = "https://api.telegram.org/bot";
  url += BOT_TOKEN;
  url += "/sendMessage?chat_id=";
  url += CHAT_ID;
  url += "&text=";
  url += encodedMessage;

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.begin(client, url);

  int httpCode = http.GET();

  Serial.print("Telegram HTTP Code: ");
  Serial.println(httpCode);

  String response = http.getString();
  Serial.println("Telegram Response:");
  Serial.println(response);

  http.end();

  if (httpCode == 200 && response.indexOf("\"ok\":true") >= 0) {
    Serial.println("Telegram alert sent successfully");
    return true;
  }

  Serial.println("Telegram alert failed");
  return false;
}

bool sendTelegramAlert() {
  for (int i = 1; i <= ALERT_SEND_RETRIES; i++) {
    Serial.print("Telegram attempt ");
    Serial.println(i);

    if (sendTelegramAlertOnce()) {
      return true;
    }

    delay(1500);
  }

  return false;
}

// =====================================================
// FAST2SMS ALERT FUNCTION
// =====================================================

bool sendFast2SMSAlertOnce() {
  Serial.println("Preparing SMS alert...");

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected. Reconnecting...");
    connectWiFi();
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("SMS failed: WiFi not connected");
    return false;
  }

  String message = createAlertMessage();
  String encodedMessage = urlEncode(message);

  String url = "https://www.fast2sms.com/dev/bulkV2?";
  url += "authorization=" + fast2smsApiKey;
  url += "&route=q";
  url += "&message=" + encodedMessage;
  url += "&language=english";
  url += "&flash=0";
  url += "&numbers=" + smsNumbers;

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.begin(client, url);

  int httpCode = http.GET();

  Serial.print("Fast2SMS HTTP Code: ");
  Serial.println(httpCode);

  String response = http.getString();
  Serial.println("Fast2SMS Response:");
  Serial.println(response);

  http.end();

  if (
    httpCode == 200 &&
    (
      response.indexOf("\"return\":true") >= 0 ||
      response.indexOf("\"return\": true") >= 0
    )
  ) {
    Serial.println("SMS sent successfully");
    return true;
  }

  Serial.println("SMS failed or API rejected request");
  return false;
}

bool sendFast2SMSAlert() {
  for (int i = 1; i <= ALERT_SEND_RETRIES; i++) {
    Serial.print("SMS attempt ");
    Serial.println(i);

    if (sendFast2SMSAlertOnce()) {
      return true;
    }

    delay(1500);
  }

  return false;
}

// =====================================================
// SEND ALL ALERTS FUNCTION
// =====================================================

void sendAllEmergencyAlerts() {
  if (alertsStarted) {
    return;
  }

  alertsStarted = true;
  alertsFinished = false;
  alertsCompleted = false;

  Serial.println("================================");
  Serial.println("ACCIDENT CONFIRMED");
  Serial.println("Sending Telegram and SMS alerts...");
  Serial.println("================================");

  showOLEDMessage("ACCIDENT CONFIRMED", "Sending Alerts...");

  telegramAttempted = true;
  telegramSent = sendTelegramAlert();

  delay(1000);

  smsAttempted = true;
  smsSent = sendFast2SMSAlert();

  alertsFinished = true;

  Serial.println("================================");
  Serial.println("ALERT SENDING RESULT");
  Serial.print("Telegram: ");
  Serial.println(telegramSent ? "SENT" : "FAILED");
  Serial.print("SMS: ");
  Serial.println(smsSent ? "SENT" : "FAILED");
  Serial.println("================================");

  if (telegramSent && smsSent) {
    alertsCompleted = true;

    Serial.println("Both Telegram and SMS sent successfully");
    Serial.println("Buzzer stopped");

    showOLEDMessage("ALERTS SENT", "TG OK + SMS OK");
    passiveBuzzerTone(false, 0);
  } 
  else {
    alertsCompleted = false;

    if (telegramSent && !smsSent) {
      showOLEDMessage("TG SENT", "SMS FAILED");
    } 
    else if (!telegramSent && smsSent) {
      showOLEDMessage("SMS SENT", "TG FAILED");
    } 
    else {
      showOLEDMessage("ALERT FAILED", "Check WiFi/API");
    }
  }
}

// =====================================================
// SETUP FUNCTION
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(CANCEL_BUTTON_PIN, INPUT_PULLUP);

  passiveBuzzerTone(false, 0);

  Wire.begin(MPU_SDA, MPU_SCL);
  Wire.setClock(100000);

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  Serial.println("System starting...");

  if (!initOLED()) {
    Serial.println("OLED not found");
    while (1);
  }

  showOLEDMessage("System Starting", "OLED OK");
  delay(1000);

  Serial.println("Checking MPU...");

  if (!detectMPU()) {
    Serial.println("MPU not detected");
    showOLEDMessage("MPU NOT FOUND", "Check wiring");
    while (1);
  }

  writeRegister(0x6B, 0x00);
  delay(100);

  writeRegister(0x1C, 0x18);
  writeRegister(0x1B, 0x18);

  Serial.println("MPU initialized successfully");
  showOLEDMessage("MPU OK", "Initializing...");
  delay(1000);

  calibrateBaseline();

  Serial.println("GPS initialized successfully");

  connectWiFi();

  showOLEDMessage("System Ready", "Monitoring...");
  delay(1000);

  Serial.println("System ready");
}

// =====================================================
// LOOP FUNCTION
// =====================================================

void loop() {
  updateGPS();

  float accMagnitude, gyroMagnitude, roll, pitch;

  if (readMotion(accMagnitude, gyroMagnitude, roll, pitch)) {
    float rollChange = angleDifference(roll, baselineRoll);
    float pitchChange = angleDifference(pitch, baselinePitch);

    bool highImpact = accMagnitude > ACC_THRESHOLD;
    bool highRotation = gyroMagnitude > GYRO_THRESHOLD;
    bool abnormalTilt = rollChange > ANGLE_THRESHOLD || pitchChange > ANGLE_THRESHOLD;

    unsigned long currentTime = millis();

    if (currentTime < cancelCooldownUntil) {
      highImpact = false;
      highRotation = false;
      abnormalTilt = false;
    }

    if (highImpact || highRotation) {
      lastImpactTime = currentTime;
      impactDetectedOnce = true;
    }

    if (abnormalTilt) {
      lastTiltTime = currentTime;
      tiltDetectedOnce = true;
    }

    bool impactRecent = impactDetectedOnce && (currentTime - lastImpactTime <= detectionWindow);
    bool tiltRecent = tiltDetectedOnce && (currentTime - lastTiltTime <= detectionWindow);

    if (!accidentConfirmed && !accidentSuspected && impactRecent && tiltRecent) {
      accidentSuspected = true;
      suspectedStartTime = currentTime;

      Serial.println("================================");
      Serial.println("ACCIDENT SUSPECTED");
      Serial.println("Cancel button active");
      Serial.println("================================");

      showOLEDMessage("ACCIDENT SUSPECTED", "Press Cancel", "within 10 sec");
    }

    if (accidentSuspected && !accidentConfirmed) {
      if (cancelButtonPressed()) {
        resetAccidentState();

        alertCancelled = true;
        cancelledMessageTime = currentTime;
        cancelCooldownUntil = currentTime + 3000;

        Serial.println("================================");
        Serial.println("ALERT CANCELLED");
        Serial.println("No Telegram or SMS sent");
        Serial.println("================================");

        showOLEDMessage("ALERT CANCELLED", "No message sent");
      }
    }

    if (accidentSuspected && !accidentConfirmed) {
      if (currentTime - suspectedStartTime >= confirmationDelay) {
        accidentConfirmed = true;

        Serial.println("================================");
        Serial.println("ACCIDENT CONFIRMED");
        Serial.println("Cancel time over");
        Serial.println("Alert process starting");
        Serial.println("================================");

        showOLEDMessage("ACCIDENT CONFIRMED", "Alert Starting...");
      }
    }

    const char* status;

    if (alertCancelled && currentTime - cancelledMessageTime <= 3000) {
      status = "ALERT CANCELLED";
    }
    else if (accidentConfirmed) {
      status = "ACCIDENT CONFIRMED";
    }
    else if (accidentSuspected) {
      status = "ACCIDENT SUSPECTED";
    }
    else if (highImpact || highRotation) {
      status = "IMPACT DETECTED";
    }
    else if (abnormalTilt) {
      status = "TILT DETECTED";
    }
    else {
      status = "NORMAL";
      alertCancelled = false;
    }

    handleBuzzer(status);

    if (accidentConfirmed && !alertsStarted) {
      sendAllEmergencyAlerts();
    }

    if (currentTime - lastDisplayTime >= 300) {
      lastDisplayTime = currentTime;
      updateOLED(accMagnitude, gyroMagnitude, rollChange, pitchChange, status);
    }

    if (currentTime - lastPrintTime >= 500) {
      lastPrintTime = currentTime;

      Serial.print("ACC: ");
      Serial.print(accMagnitude);
      Serial.print("g | GYRO: ");
      Serial.print(gyroMagnitude);
      Serial.print("dps | ROLL CHANGE: ");
      Serial.print(rollChange);
      Serial.print(" | PITCH CHANGE: ");
      Serial.print(pitchChange);
      Serial.print(" | SPEED: ");
      Serial.print(currentSpeed);
      Serial.print(" km/h | GPS: ");
      Serial.print(gpsLocationAvailable ? "LOCKED" : "WAITING");
      Serial.print(" | WiFi: ");
      Serial.print(WiFi.status() == WL_CONNECTED ? "OK" : "NO");
      Serial.print(" | STATUS: ");
      Serial.print(status);
      Serial.print(" | TG: ");
      Serial.print(telegramSent ? "SENT" : (telegramAttempted ? "FAILED" : "NO"));
      Serial.print(" | SMS: ");
      Serial.print(smsSent ? "SENT" : (smsAttempted ? "FAILED" : "NO"));
      Serial.print(" | BUZZER: ");
      Serial.println(alertsCompleted ? "OFF" : "ACTIVE/READY");
    }
  }

  delay(20);
}
