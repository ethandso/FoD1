#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <MFRC522.h>
#include <AccelStepper.h> 

// -----------------------------------------------------------------
// STEPPER MOTOR CONFIGURATION
// -----------------------------------------------------------------

#define MOTOR_TYPE AccelStepper::FULL4WIRE 
#define PIN_IN1 39 
#define PIN_IN2 40
#define PIN_IN3 41 
#define PIN_IN4 42 

AccelStepper stepper(MOTOR_TYPE, PIN_IN1, PIN_IN3, PIN_IN2, PIN_IN4);

const int STEPS_PER_REVOLUTION = 2048; 
const int MOVEMENT_STEPS = 1196; // 4x distance for full door travel
const float MAX_SPEED_STEPS_PER_SEC = 1200.0;
const float MAX_ACCEL_STEPS_PER_SEC2 = 2000.0;
const unsigned long DOOR_OPEN_TIME_MS = 3000;

enum MotorState {
  READY,
  MOVING_FORWARD,
  DOOR_OPEN,
  MOVING_BACKWARD
};

MotorState motorState = READY;
unsigned long doorOpenStartTime = 0;

bool cardWasPresentR1 = false;
bool cardWasPresentR2 = false;

// -----------------------------------------------------------------
// SYSTEM CONFIGURATION
// -----------------------------------------------------------------

#define RST_PIN 16
#define SS_1    4
#define SS_2    5
#define PIN_SCK   6
#define PIN_MISO  15
#define PIN_MOSI  7

MFRC522 r1(SS_1, RST_PIN);
MFRC522 r2(SS_2, RST_PIN);

WebServer server(80);

struct Goat {
  const char* name;
  uint8_t uid[4];
};

Goat goats[] = {
  {"Bron", {0x64,0x3E,0x32,0x03}},
  {"TFrance", {0xB9,0x72,0xA9,0x11}},
  {"BigO", {0xF9,0xEF,0x61,0x12}}
};

const int NUM_GOATS = sizeof(goats) / sizeof(goats[0]);

// Goat location tracking - Simple barn presence system
String goatsInBarn[10];  // Allow up to 10 goats in barn
int barnCount = 0;

void addGoatToBarn(String goatName) {
  // Check if goat is already in barn
  for (int i = 0; i < barnCount; i++) {
    if (goatsInBarn[i] == goatName) {
      Serial.printf("[BARN] %s already in barn (not adding again)\n", goatName.c_str());
      return;  // Already in barn, don't add again
    }
  }
  
  // Add to barn
  if (barnCount < 10) {
    goatsInBarn[barnCount] = goatName;
    barnCount++;
    Serial.printf("[BARN] %s ENTERED barn (total: %d)\n", goatName.c_str(), barnCount);
  } else {
    Serial.println("[BARN] ERROR: Barn is full!");
  }
}

void removeGoatFromBarn(String goatName) {
  // Find and remove goat from barn
  for (int i = 0; i < barnCount; i++) {
    if (goatsInBarn[i] == goatName) {
      // Shift remaining goats down
      for (int j = i; j < barnCount - 1; j++) {
        goatsInBarn[j] = goatsInBarn[j + 1];
      }
      barnCount--;
      Serial.printf("[BARN] %s LEFT barn (total: %d)\n", goatName.c_str(), barnCount);
      return;
    }
  }
  Serial.printf("[BARN] %s was NOT in barn (cannot remove)\n", goatName.c_str());
}

// Timing control - CRITICAL for smooth motor operation
unsigned long lastRfidCheck = 0;
unsigned long lastServerCheck = 0;
const unsigned long RFID_CHECK_INTERVAL = 100;    // Check RFID every 100ms
const unsigned long SERVER_CHECK_INTERVAL = 20;   // Check server every 20ms

// -----------------------------------------------------------------
// HELPER FUNCTIONS
// -----------------------------------------------------------------

String uidToString(MFRC522::Uid &uid) {
  String s = "";
  for (byte i = 0; i < uid.size; i++) {
    if (uid.uidByte[i] < 0x10) s += "0";
    s += String(uid.uidByte[i], HEX);
    if (i < uid.size - 1) s += ":";
  }
  s.toUpperCase();
  return s;
}

const char* getGoatName(MFRC522::Uid &uid) {
  if (uid.size < 4) return nullptr;

  for (int g = 0; g < NUM_GOATS; g++) {
    bool match = true;
    for (int i = 0; i < 4; i++) {
      if (goats[g].uid[i] != uid.uidByte[i]) {
        match = false;
        break;
      }
    }
    if (match) return goats[g].name;
  }
  return nullptr;
}

String timeString() {
  unsigned long sec = millis() / 1000;
  int hh = (sec / 3600) % 24;
  int mm = (sec / 60) % 60;
  int ss = sec % 60;
  char buf[16];
  sprintf(buf, "%02d:%02d:%02d", hh, mm, ss);
  return String(buf);
}

// -----------------------------------------------------------------
// STEPPER MOTOR CONTROL
// -----------------------------------------------------------------

void triggerDoorSequence() {
  if (motorState == READY) {
    stepper.move(MOVEMENT_STEPS);
    motorState = MOVING_FORWARD;
    Serial.println("Door opening...");
  }
}

void runStepper() {
  // This function is no longer used - motor logic moved to loop()
}

// -----------------------------------------------------------------
// RFID SCANNING - HEAVILY THROTTLED
// -----------------------------------------------------------------

void checkReader(MFRC522 &reader, int readerId) {
  bool &cardWasPresent = (readerId == 1) ? cardWasPresentR1 : cardWasPresentR2;

  if (reader.PICC_IsNewCardPresent() && reader.PICC_ReadCardSerial()) {
    
    if (!cardWasPresent) {
      String uidStr = uidToString(reader.uid);
      const char* name = getGoatName(reader.uid);
      String goat = name ? String(name) : "UNKNOWN";

      Serial.printf("Reader %d: %s\n", readerId, goat.c_str());

      // Reader 1 = Enter barn, Reader 2 = Exit barn
      if (readerId == 1) {
        addGoatToBarn(goat);
      } else if (readerId == 2) {
        removeGoatFromBarn(goat);
      }
      
      if (motorState == READY) {
        triggerDoorSequence();
      } else {
        Serial.println("Motor busy - scan ignored");
      }

      cardWasPresent = true;
    }

    reader.PICC_HaltA();
    reader.PCD_StopCrypto1();
    
  } else {
    cardWasPresent = false;
  }
}

void checkRFIDIfTime() {
  if (millis() - lastRfidCheck >= RFID_CHECK_INTERVAL) {
    lastRfidCheck = millis();
    checkReader(r1, 1);
    checkReader(r2, 2);
  }
}

// -----------------------------------------------------------------
// WEBSERVER HTML & HANDLERS
// -----------------------------------------------------------------

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Goat Barn Tracker</title>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { 
      font-family: Arial; 
      margin: 0;
      padding: 20px;
      background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
      min-height: 100vh;
    }
    .container {
      max-width: 600px;
      margin: 0 auto;
      background: white;
      border-radius: 20px;
      padding: 30px;
      box-shadow: 0 20px 60px rgba(0,0,0,0.3);
    }
    h1 { 
      color: #333; 
      text-align: center;
      margin: 0 0 10px 0;
      font-size: 2.5em;
    }
    .subtitle {
      text-align: center;
      color: #666;
      margin-bottom: 30px;
      font-size: 1.1em;
    }
    .barn-header {
      background: #4CAF50;
      color: white;
      padding: 15px;
      border-radius: 10px 10px 0 0;
      font-size: 1.5em;
      font-weight: bold;
      text-align: center;
    }
    .goat-list {
      border: 3px solid #4CAF50;
      border-top: none;
      border-radius: 0 0 10px 10px;
      min-height: 200px;
      background: #f9f9f9;
    }
    .goat-item {
      padding: 20px;
      border-bottom: 1px solid #ddd;
      font-size: 1.3em;
      font-weight: bold;
      color: #333;
      background: white;
      transition: all 0.3s ease;
    }
    .goat-item:last-child {
      border-bottom: none;
      border-radius: 0 0 7px 7px;
    }
    .goat-item:hover {
      background: #e8f5e9;
      transform: translateX(5px);
    }
    .empty-state {
      padding: 60px 20px;
      text-align: center;
      color: #999;
      font-size: 1.2em;
    }
    .count-badge {
      display: inline-block;
      background: white;
      color: #4CAF50;
      padding: 5px 15px;
      border-radius: 20px;
      font-size: 0.8em;
      margin-left: 10px;
      font-weight: bold;
    }
  </style>
</head>
<body>

<div class="container">
  <h1>🐐 Goat Tracker</h1>
  <div class="subtitle">Live Barn Status</div>
  
  <div class="barn-header">
    BARN
    <span class="count-badge" id="count">0</span>
  </div>
  <div class="goat-list" id="goatList"></div>
</div>

<script>
async function loadBarnStatus() {
  try {
    let res = await fetch('/barn');
    let data = await res.json();
    
    let listEl = document.getElementById('goatList');
    let countEl = document.getElementById('count');
    
    countEl.textContent = data.count;
    
    if (data.goats.length === 0) {
      listEl.innerHTML = '<div class="empty-state">No goats in barn</div>';
    } else {
      listEl.innerHTML = data.goats.map(goat => 
        `<div class="goat-item">${goat}</div>`
      ).join('');
    }
  } catch (err) {
    console.error('Failed to load barn status:', err);
  }
}

setInterval(loadBarnStatus, 500);
loadBarnStatus();
</script>

</body>
</html>
)rawliteral";

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleBarn() {
  String json = "{\"count\":" + String(barnCount) + ",\"goats\":[";
  
  for (int i = 0; i < barnCount; i++) {
    json += "\"" + goatsInBarn[i] + "\"";
    if (i < barnCount - 1) json += ",";
  }
  
  json += "]}";
  
  Serial.printf("[WEB] Sending barn status: %d goats\n", barnCount);
  server.send(200, "application/json", json);
}

void checkServerIfTime() {
  if (millis() - lastServerCheck >= SERVER_CHECK_INTERVAL) {
    lastServerCheck = millis();
    server.handleClient();
  }
}

// -----------------------------------------------------------------
// SETUP & LOOP
// -----------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nGoat Tracker Starting...");

  // Stepper motor setup
  stepper.setMaxSpeed(MAX_SPEED_STEPS_PER_SEC);
  stepper.setAcceleration(MAX_ACCEL_STEPS_PER_SEC2);
  stepper.setCurrentPosition(0);
  Serial.println("Stepper motor initialized");
  
  // RFID setup
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, SS_1);
  r1.PCD_Init();
  r2.PCD_Init();
  Serial.println("RFID readers initialized");

  // WiFi AP setup
  WiFi.softAP("GoatTracker", "67676767");
  Serial.print("AP running at: ");
  Serial.println(WiFi.softAPIP());

  // Webserver setup
  server.on("/", handleRoot);
  server.on("/barn", handleBarn);
  server.begin();
  Serial.println("HTTP server started");
  Serial.println("\nReady to track goats!\n");
}

void loop() {
  // EXACT same structure as working minimal test
  stepper.run();

  switch (motorState) {
    case READY:
      // Only check RFID when motor is idle
      checkRFIDIfTime();
      checkServerIfTime();
      break;

    case MOVING_FORWARD:
      if (stepper.distanceToGo() == 0) {
        doorOpenStartTime = millis();
        motorState = DOOR_OPEN;
        Serial.println("Door open - waiting 3 seconds...");
      }
      break;

    case DOOR_OPEN:
      if (millis() - doorOpenStartTime >= DOOR_OPEN_TIME_MS) {
        stepper.move(-MOVEMENT_STEPS);
        motorState = MOVING_BACKWARD;
        Serial.println("Door closing...");
      }
      break;

    case MOVING_BACKWARD:
      if (stepper.distanceToGo() == 0) {
        stepper.setCurrentPosition(0);
        motorState = READY;
        Serial.println("Door closed - ready for next scan");
      }
      break;
  }
  
  delayMicroseconds(100);
}