#include <WiFi.h>

#include <WebServer.h>
#include <SPI.h>
#include <MFRC522.h>
#include <AccelStepper.h> 

// =================================================================
// STEPPER MOTOR - COMPLETELY REBUILT
// =================================================================

// Hardware pins
#define MOTOR_PIN1 39
#define MOTOR_PIN2 40
#define MOTOR_PIN3 41
#define MOTOR_PIN4 42

// Create stepper with FULL4WIRE for maximum torque
AccelStepper motor(AccelStepper::FULL4WIRE, MOTOR_PIN1, MOTOR_PIN3, MOTOR_PIN2, MOTOR_PIN4);

// Motor specifications for 28BYJ-48
#define STEPS_PER_REV 2048
#define DOOR_TRAVEL_STEPS 800

// Performance tuning - these are CRITICAL
#define MOTOR_MAX_SPEED 800.0      // Reduced for reliability
#define MOTOR_ACCELERATION 1500.0  // Gentler acceleration
#define DOOR_HOLD_TIME_MS 3000     // How long door stays open

// Position definitions
#define POS_CLOSED 0
#define POS_OPEN 1400

// Motor state machine
typedef enum {
  MOTOR_IDLE,           // Ready for commands
  MOTOR_OPENING,        // Moving to open position
  MOTOR_OPEN_HOLDING,   // Door is open, waiting
  MOTOR_CLOSING,        // Moving to closed position
  MOTOR_MANUAL_OPEN,    // Manually opened - stays open
  MOTOR_ERROR           // Something went wrong
} MotorState_t;

MotorState_t motorState = MOTOR_IDLE;
unsigned long motorStateStartTime = 0;
unsigned long motorMoveStartTime = 0;
bool manualMode = false;  // Track if door is in manual control mode

// Motor statistics
unsigned long totalMoves = 0;
unsigned long failedMoves = 0;

// =================================================================
// RFID SYSTEM
// =================================================================

#define RST_PIN 16
#define SS_1    4
#define SS_2    5
#define PIN_SCK   6
#define PIN_MISO  15
#define PIN_MOSI  7

MFRC522 rfidReader1(SS_1, RST_PIN);
MFRC522 rfidReader2(SS_2, RST_PIN);

bool lastCardStateR1 = false;
bool lastCardStateR2 = false;

// =================================================================
// WEB SERVER
// =================================================================

WebServer webServer(80);

// =================================================================
// GOAT DATABASE
// =================================================================

typedef struct {
  const char* name;
  uint8_t uid[4];
} GoatRecord_t;

GoatRecord_t knownGoats[] = {
  {"Bron",    {0x64, 0x3E, 0x32, 0x03}},
  {"TFrance", {0xB9, 0x72, 0xA9, 0x11}},
  {"BigO",    {0xF9, 0xEF, 0x61, 0x12}}
};

#define NUM_KNOWN_GOATS (sizeof(knownGoats) / sizeof(knownGoats[0]))
#define MAX_GOATS_IN_BARN 10

String barnOccupants[MAX_GOATS_IN_BARN];
int barnPopulation = 0;

// Thread safety for dual-core operation
SemaphoreHandle_t barnDataMutex;

// =================================================================
// MOTOR CONTROL FUNCTIONS
// =================================================================

void motorInit() {
  motor.setMaxSpeed(MOTOR_MAX_SPEED);
  motor.setAcceleration(MOTOR_ACCELERATION);
  motor.setCurrentPosition(POS_CLOSED);
  motorState = MOTOR_IDLE;
  
  Serial.println("[MOTOR] Initialized");
  Serial.printf("  Max Speed: %.0f steps/sec\n", MOTOR_MAX_SPEED);
  Serial.printf("  Acceleration: %.0f steps/sec²\n", MOTOR_ACCELERATION);
  Serial.printf("  Travel Distance: %d steps\n", DOOR_TRAVEL_STEPS);
}

void motorOpenDoor() {
  if (motorState != MOTOR_IDLE && motorState != MOTOR_MANUAL_OPEN) {
    Serial.println("[MOTOR] Busy - command ignored");
    return;
  }
  
  Serial.println("[MOTOR] ▶ Opening door...");
  motor.moveTo(POS_OPEN);
  motorState = MOTOR_OPENING;
  motorStateStartTime = millis();
  motorMoveStartTime = millis();
  totalMoves++;
}

void motorCloseDoor() {
  if (motorState == MOTOR_IDLE) {
    Serial.println("[MOTOR] Already closed");
    return;
  }
  
  Serial.println("[MOTOR] ◀ Closing door (manual)...");
  motor.moveTo(POS_CLOSED);
  motorState = MOTOR_CLOSING;
  motorMoveStartTime = millis();
  manualMode = false;
}

void motorManualOpen() {
  Serial.println("[MOTOR] Manual OPEN command");
  manualMode = true;
  motorOpenDoor();
}

void motorManualClose() {
  Serial.println("[MOTOR] Manual CLOSE command");
  motorCloseDoor();
}

void motorUpdate() {
  // MUST be called every loop - this is what actually moves the motor
  motor.run();
  
  // State machine
  switch (motorState) {
    
    case MOTOR_IDLE:
      // Nothing to do - waiting for command
      break;
      
    case MOTOR_OPENING:
      if (motor.distanceToGo() == 0) {
        // Reached open position
        unsigned long moveTime = millis() - motorMoveStartTime;
        int finalPos = motor.currentPosition();
        
        Serial.printf("[MOTOR] ✓ Door OPEN (position: %d, time: %lu ms)\n", finalPos, moveTime);
        
        // Check if we reached the target
        if (finalPos != POS_OPEN) {
          Serial.printf("[MOTOR] ⚠ Warning: Expected %d, got %d\n", POS_OPEN, finalPos);
          failedMoves++;
        }
        
        if (manualMode) {
          motorState = MOTOR_MANUAL_OPEN;
          Serial.println("[MOTOR] Manual mode - door stays open");
        } else {
          motorState = MOTOR_OPEN_HOLDING;
          motorStateStartTime = millis();
        }
      }
      break;
      
    case MOTOR_OPEN_HOLDING:
      // Door is open, waiting for timer
      if (millis() - motorStateStartTime >= DOOR_HOLD_TIME_MS) {
        Serial.println("[MOTOR] ◀ Closing door (auto)...");
        motor.moveTo(POS_CLOSED);
        motorState = MOTOR_CLOSING;
        motorMoveStartTime = millis();
      }
      break;
      
    case MOTOR_MANUAL_OPEN:
      // Door is manually held open - do nothing
      break;
      
    case MOTOR_CLOSING:
      if (motor.distanceToGo() == 0) {
        // Reached closed position
        unsigned long moveTime = millis() - motorMoveStartTime;
        int finalPos = motor.currentPosition();
        
        Serial.printf("[MOTOR] ✓ Door CLOSED (position: %d, time: %lu ms)\n", finalPos, moveTime);
        
        // Check if we reached the target
        if (finalPos != POS_CLOSED) {
          Serial.printf("[MOTOR] ⚠ Warning: Expected %d, got %d\n", POS_CLOSED, finalPos);
          failedMoves++;
        }
        
        motorState = MOTOR_IDLE;
        Serial.println("[MOTOR] Ready for next operation");
        Serial.println("─────────────────────────────────────");
      }
      break;
      
    case MOTOR_ERROR:
      // Error state - requires manual reset
      Serial.println("[MOTOR] ERROR STATE - Manual intervention required");
      break;
  }
}

bool motorIsIdle() {
  return (motorState == MOTOR_IDLE || motorState == MOTOR_MANUAL_OPEN);
}

String motorGetStatus() {
  switch (motorState) {
    case MOTOR_IDLE: return "closed";
    case MOTOR_OPENING: return "opening";
    case MOTOR_OPEN_HOLDING: return "open";
    case MOTOR_MANUAL_OPEN: return "manual_open";
    case MOTOR_CLOSING: return "closing";
    case MOTOR_ERROR: return "error";
    default: return "unknown";
  }
}

void motorPrintStats() {
  Serial.printf("[MOTOR STATS] Total: %lu, Failed: %lu, Success Rate: %.1f%%\n",
                totalMoves, failedMoves, 
                totalMoves > 0 ? (100.0 * (totalMoves - failedMoves) / totalMoves) : 100.0);
}

// =================================================================
// GOAT TRACKING FUNCTIONS
// =================================================================

void barnAddGoat(String name) {
  xSemaphoreTake(barnDataMutex, portMAX_DELAY);
  
  // Check if already in barn
  for (int i = 0; i < barnPopulation; i++) {
    if (barnOccupants[i] == name) {
      Serial.printf("[BARN] %s already inside\n", name.c_str());
      xSemaphoreGive(barnDataMutex);
      return;
    }
  }
  
  // Add to barn
  if (barnPopulation < MAX_GOATS_IN_BARN) {
    barnOccupants[barnPopulation] = name;
    barnPopulation++;
    Serial.printf("[BARN] ➜ %s ENTERED (total: %d)\n", name.c_str(), barnPopulation);
  } else {
    Serial.println("[BARN] ✗ FULL - cannot add more goats!");
  }
  
  xSemaphoreGive(barnDataMutex);
}

void barnRemoveGoat(String name) {
  xSemaphoreTake(barnDataMutex, portMAX_DELAY);
  
  // Find and remove
  for (int i = 0; i < barnPopulation; i++) {
    if (barnOccupants[i] == name) {
      // Shift remaining goats
      for (int j = i; j < barnPopulation - 1; j++) {
        barnOccupants[j] = barnOccupants[j + 1];
      }
      barnPopulation--;
      Serial.printf("[BARN] ← %s EXITED (total: %d)\n", name.c_str(), barnPopulation);
      xSemaphoreGive(barnDataMutex);
      return;
    }
  }
  
  Serial.printf("[BARN] %s not found in barn\n", name.c_str());
  xSemaphoreGive(barnDataMutex);
}

// =================================================================
// RFID FUNCTIONS
// =================================================================

String rfidUidToString(MFRC522::Uid &uid) {
  String result = "";
  for (byte i = 0; i < uid.size; i++) {
    if (uid.uidByte[i] < 0x10) result += "0";
    result += String(uid.uidByte[i], HEX);
    if (i < uid.size - 1) result += ":";
  }
  result.toUpperCase();
  return result;
}

const char* rfidIdentifyGoat(MFRC522::Uid &uid) {
  if (uid.size < 4) return nullptr;
  
  for (int i = 0; i < NUM_KNOWN_GOATS; i++) {
    bool match = true;
    for (int j = 0; j < 4; j++) {
      if (knownGoats[i].uid[j] != uid.uidByte[j]) {
        match = false;
        break;
      }
    }
    if (match) return knownGoats[i].name;
  }
  
  return nullptr;
}

void rfidCheckReader(MFRC522 &reader, int readerNumber, bool &lastState) {
  bool cardPresent = reader.PICC_IsNewCardPresent() && reader.PICC_ReadCardSerial();
  
  if (cardPresent && !lastState) {
    // Card just detected
    String uid = rfidUidToString(reader.uid);
    const char* goatName = rfidIdentifyGoat(reader.uid);
    String goat = goatName ? String(goatName) : "UNKNOWN";
    
    Serial.printf("[RFID-%d] Detected: %s (%s)\n", readerNumber, goat.c_str(), uid.c_str());
    
    // Update barn tracking
    if (readerNumber == 1) {
      barnAddGoat(goat);
    } else if (readerNumber == 2) {
      barnRemoveGoat(goat);
    }
    
    // Trigger door (only if not in manual mode)
    if (!manualMode && motorIsIdle()) {
      motorOpenDoor();
    } else {
      Serial.println("[RFID] Motor busy or manual mode - ignoring scan");
    }
    
    reader.PICC_HaltA();
    reader.PCD_StopCrypto1();
    lastState = true;
    
  } else if (!cardPresent) {
    lastState = false;
  }
}

// =================================================================
// WEB SERVER HANDLERS
// =================================================================

const char HTML_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>HerdLink</title>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body { 
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
      background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
      min-height: 100vh;
      padding: 20px;
    }
    .container {
      max-width: 600px;
      margin: 0 auto;
      background: white;
      border-radius: 20px;
      padding: 40px;
      box-shadow: 0 20px 60px rgba(0,0,0,0.3);
    }
    h1 { 
      color: #333;
      text-align: center;
      font-size: 2.5em;
      margin-bottom: 10px;
    }
    .subtitle {
      text-align: center;
      color: #666;
      margin-bottom: 30px;
      font-size: 1.1em;
    }
    .barn-card {
      background: linear-gradient(135deg, #4CAF50 0%, #45a049 100%);
      color: white;
      padding: 20px;
      border-radius: 15px;
      margin-bottom: 20px;
      box-shadow: 0 4px 15px rgba(76, 175, 80, 0.3);
    }
    .barn-title {
      font-size: 1.8em;
      font-weight: bold;
      display: flex;
      justify-content: space-between;
      align-items: center;
    }
    .count-badge {
      background: white;
      color: #4CAF50;
      padding: 8px 20px;
      border-radius: 25px;
      font-size: 0.9em;
      font-weight: bold;
    }
    .door-control {
      background: #f5f5f5;
      padding: 20px;
      border-radius: 15px;
      margin-bottom: 20px;
    }
    .door-control h2 {
      font-size: 1.3em;
      color: #333;
      margin-bottom: 15px;
      text-align: center;
    }
    .door-buttons {
      display: flex;
      gap: 10px;
    }
    .door-btn {
      flex: 1;
      padding: 15px;
      border: none;
      border-radius: 10px;
      font-size: 1.1em;
      font-weight: bold;
      cursor: pointer;
      transition: all 0.3s ease;
    }
    .door-btn:hover {
      transform: translateY(-2px);
      box-shadow: 0 4px 15px rgba(0,0,0,0.2);
    }
    .door-btn.open {
      background: #4CAF50;
      color: white;
    }
    .door-btn.close {
      background: #f44336;
      color: white;
    }
    .door-btn:disabled {
      opacity: 0.5;
      cursor: not-allowed;
      transform: none;
    }
    .door-status {
      text-align: center;
      margin-top: 10px;
      padding: 10px;
      border-radius: 8px;
      font-weight: bold;
    }
    .door-status.closed { background: #ffebee; color: #c62828; }
    .door-status.open { background: #e8f5e9; color: #2e7d32; }
    .door-status.opening { background: #fff3e0; color: #e65100; }
    .door-status.closing { background: #fff3e0; color: #e65100; }
    .door-status.manual_open { background: #e3f2fd; color: #1565c0; }
    .goat-list {
      background: #f9f9f9;
      border-radius: 15px;
      overflow: hidden;
      min-height: 200px;
    }
    .goat-item {
      padding: 20px;
      background: white;
      border-bottom: 1px solid #e0e0e0;
      font-size: 1.3em;
      font-weight: 600;
      color: #333;
      transition: all 0.3s ease;
    }
    .goat-item:hover {
      background: #e8f5e9;
      transform: translateX(10px);
      padding-left: 30px;
    }
    .goat-item:last-child {
      border-bottom: none;
    }
    .empty-state {
      padding: 80px 20px;
      text-align: center;
      color: #999;
      font-size: 1.3em;
    }
    .status-bar {
      margin-top: 20px;
      padding: 15px;
      background: #f5f5f5;
      border-radius: 10px;
      text-align: center;
      font-size: 0.9em;
      color: #666;
    }
    .status-online { color: #4CAF50; font-weight: bold; }
    .status-offline { color: #f44336; font-weight: bold; }
  </style>
</head>
<body>

<div class="container">
  <h1> HerdLink</h1>
  <div class="subtitle">Real-Time Barn Management</div>
  
  <div class="door-control">
    <h2> Manual Door Control</h2>
    <div class="door-buttons">
      <button class="door-btn open" onclick="openDoor()">OPEN</button>
      <button class="door-btn close" onclick="closeDoor()">CLOSE</button>
    </div>
    <div class="door-status" id="doorStatus">Loading...</div>
  </div>
  
  <div class="barn-card">
    <div class="barn-title">
      <span>BARN STATUS</span>
      <span class="count-badge" id="count">0</span>
    </div>
  </div>
  
  <div class="goat-list" id="goatList">
    <div class="empty-state">Loading...</div>
  </div>
  
  <div class="status-bar">
    <span id="status" class="status-online">● Connected</span>
  </div>
</div>

<script>
let failCount = 0;

async function openDoor() {
  try {
    await fetch('/api/door/open', { method: 'POST' });
    update();
  } catch (err) {
    alert('Failed to open door');
  }
}

async function closeDoor() {
  try {
    await fetch('/api/door/close', { method: 'POST' });
    update();
  } catch (err) {
    alert('Failed to close door');
  }
}

function updateDoorStatus(status) {
  const statusEl = document.getElementById('doorStatus');
  statusEl.className = 'door-status ' + status;
  
  const statusText = {
    'closed': '🔒 Door Closed',
    'open': '🔓 Door Open (Auto)',
    'manual_open': '🔓 Door Open (Manual)',
    'opening': '⏳ Opening...',
    'closing': '⏳ Closing...',
    'error': '⚠️ Error'
  };
  
  statusEl.textContent = statusText[status] || status;
}

async function update() {
  try {
    const res = await fetch('/api/barn');
    const data = await res.json();
    
    document.getElementById('count').textContent = data.count;
    document.getElementById('status').textContent = '● Connected';
    document.getElementById('status').className = 'status-online';
    updateDoorStatus(data.doorStatus);
    failCount = 0;
    
    const listEl = document.getElementById('goatList');
    if (data.goats.length === 0) {
      listEl.innerHTML = '<div class="empty-state">🌾 Barn is empty</div>';
    } else {
      listEl.innerHTML = data.goats.map(goat => 
        `<div class="goat-item">🐐 ${goat}</div>`
      ).join('');
    }
  } catch (err) {
    failCount++;
    if (failCount > 3) {
      document.getElementById('status').textContent = '● Connection Lost';
      document.getElementById('status').className = 'status-offline';
    }
  }
}

setInterval(update, 1000);
update();
</script>

</body>
</html>
)rawliteral";

void webHandleRoot() {
  webServer.send_P(200, "text/html", HTML_PAGE);
}

void webHandleAPI() {
  xSemaphoreTake(barnDataMutex, portMAX_DELAY);
  
  String json = "{\"count\":" + String(barnPopulation) + 
                ",\"doorStatus\":\"" + motorGetStatus() + 
                "\",\"goats\":[";
  for (int i = 0; i < barnPopulation; i++) {
    json += "\"" + barnOccupants[i] + "\"";
    if (i < barnPopulation - 1) json += ",";
  }
  json += "]}";
  
  xSemaphoreGive(barnDataMutex);
  
  webServer.send(200, "application/json", json);
}

void webHandleDoorOpen() {
  motorManualOpen();
  webServer.send(200, "text/plain", "OK");
}

void webHandleDoorClose() {
  motorManualClose();
  webServer.send(200, "text/plain", "OK");
}

// =================================================================
// FREERTOS TASKS
// =================================================================

void taskMotorAndRFID(void *param) {
  Serial.println("[TASK] Motor+RFID task started on Core 1");
  
  while (true) {
    // Update motor - HIGHEST PRIORITY
    motorUpdate();
    
    // Check RFID only when motor is idle
    if (motorIsIdle()) {
      rfidCheckReader(rfidReader1, 1, lastCardStateR1);
      rfidCheckReader(rfidReader2, 2, lastCardStateR2);
    }
    
    // Minimal delay for smooth operation
    delayMicroseconds(100);
  }
}

void taskWebServer(void *param) {
  Serial.println("[TASK] WebServer task started on Core 0");
  
  while (true) {
    webServer.handleClient();
    delay(5);  // Check every 5ms
  }
}

// =================================================================
// MAIN SETUP
// =================================================================

void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("\n");
  Serial.println("╔═══════════════════════════════════════════════╗");
  Serial.println("║       GOAT TRACKER PRO - SYSTEM START        ║");
  Serial.println("╚═══════════════════════════════════════════════╝");
  Serial.println();
  
  // Initialize mutex
  barnDataMutex = xSemaphoreCreateMutex();
  Serial.println("[SYSTEM] Mutex created");
  
  // Initialize motor
  motorInit();
  
  // Initialize RFID
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, SS_1);
  rfidReader1.PCD_Init();
  rfidReader2.PCD_Init();
  Serial.println("[RFID] Readers initialized");
  
  // Initialize WiFi
  WiFi.softAP("GoatTracker", "67676767");
  Serial.print("[WIFI] Access Point: ");
  Serial.println(WiFi.softAPIP());
  
  // Initialize web server
  webServer.on("/", webHandleRoot);
  webServer.on("/api/barn", webHandleAPI);
  webServer.on("/api/door/open", HTTP_POST, webHandleDoorOpen);
  webServer.on("/api/door/close", HTTP_POST, webHandleDoorClose);
  webServer.begin();
  Serial.println("[WEB] Server started");
  
  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(taskMotorAndRFID, "Motor+RFID", 10000, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskWebServer, "WebServer", 10000, NULL, 1, NULL, 0);
  
  Serial.println();
  Serial.println("╔═══════════════════════════════════════════════╗");
  Serial.println("║            SYSTEM READY - DUAL CORE           ║");
  Serial.println("║  Core 1: Motor + RFID (High Priority)        ║");
  Serial.println("║  Core 0: WiFi + Web (Low Priority)           ║");
  Serial.println("╚═══════════════════════════════════════════════╝");
  Serial.println();
}

void loop() {
  // Print stats every 30 seconds
  static unsigned long lastStatsTime = 0;
  if (millis() - lastStatsTime > 30000) {
    lastStatsTime = millis();
    motorPrintStats();
  }
  
  delay(1000);
}