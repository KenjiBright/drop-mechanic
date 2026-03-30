#include <Arduino.h>
#include <ESP32Servo.h>
#include <driver/uart.h>

// ==================== Pin & Config ====================
const int SERVO_PINS[] = {5, 26, 18, 25};
Servo servos[4];

#define SBUS_RX 16
#define SBUS_TX 17

// ==================== SBUS ====================
uint16_t channels[16];
bool failsafe = false;
bool frameLost = false;
unsigned long lastFrameTime = 0;

enum SwPos { SW_LOW, SW_MID, SW_HIGH };

SwPos getSwitch(uint16_t val) {
  if (val < 400)  return SW_LOW;
  if (val > 1600) return SW_HIGH;
  return SW_MID;
}

const char* swName(SwPos s) {
  switch (s) {
    case SW_LOW:  return "LOW";
    case SW_MID:  return "MID";
    case SW_HIGH: return "HIGH";
  }
  return "?";
}

void decodeSBUS(const uint8_t *d) {
  channels[0]  = ((d[1]       | d[2]  << 8)                    & 0x07FF);
  channels[1]  = ((d[2]  >> 3 | d[3]  << 5)                    & 0x07FF);
  channels[2]  = ((d[3]  >> 6 | d[4]  << 2 | d[5]  << 10)     & 0x07FF);
  channels[3]  = ((d[5]  >> 1 | d[6]  << 7)                    & 0x07FF);
  channels[4]  = ((d[6]  >> 4 | d[7]  << 4)                    & 0x07FF);
  channels[5]  = ((d[7]  >> 7 | d[8]  << 1 | d[9]  << 9)      & 0x07FF);
  channels[6]  = ((d[9]  >> 2 | d[10] << 6)                    & 0x07FF);
  channels[7]  = ((d[10] >> 5 | d[11] << 3)                    & 0x07FF);
  channels[8]  = ((d[12]      | d[13] << 8)                    & 0x07FF);
  channels[9]  = ((d[13] >> 3 | d[14] << 5)                    & 0x07FF);
  channels[10] = ((d[14] >> 6 | d[15] << 2 | d[16] << 10)     & 0x07FF);
  channels[11] = ((d[16] >> 1 | d[17] << 7)                    & 0x07FF);
  channels[12] = ((d[17] >> 4 | d[18] << 4)                    & 0x07FF);
  channels[13] = ((d[18] >> 7 | d[19] << 1 | d[20] << 9)      & 0x07FF);
  channels[14] = ((d[20] >> 2 | d[21] << 6)                    & 0x07FF);
  channels[15] = ((d[21] >> 5 | d[22] << 3)                    & 0x07FF);
  frameLost = d[23] & 0x04;
  failsafe  = d[23] & 0x08;
}

bool readSBUS() {
  static uint8_t buf[25];
  static uint8_t idx = 0;
  bool newFrame = false;

  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    if (idx == 0 && b != 0x0F) continue;
    buf[idx++] = b;
    if (idx == 25) {
      idx = 0;
      if ((buf[24] & 0xF0) == 0x00) {
        decodeSBUS(buf);
        lastFrameTime = millis();
        newFrame = true;
      }
    }
  }
  return newFrame;
}

// ==================== Tube Control ====================
bool isArmed = false;
bool isOpen[4] = {};
bool isReloadMode = false;
unsigned long dropTimer[4] = {};
const unsigned long DROP_TIMEOUT = 2000;
const unsigned long SEQ_STEP = 1000;

// ==================== Global Servo Lock ====================
// Chi 1 servo duoc phep chay tai bat ky thoi diem nao
bool servoLocked = false;
unsigned long servoLockUntil = 0;
const unsigned long SERVO_TRAVEL_MS = 700; // Thoi gian servo di chuyen xong

void updateServoLock() {
  if (servoLocked && millis() >= servoLockUntil) {
    servoLocked = false;
  }
}

// Sequence state
enum SeqMode { SEQ_IDLE, SEQ_OPENING, SEQ_CLOSING };
SeqMode seqMode = SEQ_IDLE;
int seqIdx = 0;
unsigned long seqStepStart = 0;

// Previous switch states for edge detection
SwPos prevSA = SW_LOW, prevSB = SW_LOW, prevSC = SW_LOW, prevSD = SW_LOW;
bool switchesInitialized = false;

// Tra ve true neu lenh duoc thuc thi, false neu bi khoa
bool setTube(int i, bool open) {
  if (servoLocked) return false; // Servo khac dang chay, tu choi
  // Tube 0,3: open=180, close=0 | Tube 1,2: open=0, close=180
  int angle = (i == 0 || i == 3) ? (open ? 180 : 0) : (open ? 0 : 180);
  servos[i].write(angle);
  isOpen[i] = open;
  servoLocked = true;
  servoLockUntil = millis() + SERVO_TRAVEL_MS;
  return true;
}

void stopSequence() {
  seqMode = SEQ_IDLE;
  seqIdx = 0;
  seqStepStart = 0;
}

void dropTube(int id) {
  if (id < 0 || id > 3) return;
  if (isOpen[id]) return; // Da mo roi
  if (servoLocked) {
    Serial.printf("[BLOCK] Servo dang ban, lenh Drop Tube %d bi TU CHOI\n", id + 1);
    return;
  }
  isReloadMode = false;
  stopSequence();
  setTube(id, true);
  dropTimer[id] = millis();
  Serial.printf("[DROP] Tube %d OPENED (auto-close 2s)\n", id + 1);
}

void startReload() {
  if (servoLocked) {
    Serial.println("[BLOCK] Servo dang ban, lenh Reload bi TU CHOI");
    return;
  }
  isReloadMode = true;
  for (int i = 0; i < 4; i++) dropTimer[i] = 0;
  seqMode = SEQ_OPENING;
  seqIdx = 0;
  seqStepStart = 0;
  Serial.println("[RELOAD] Opening all tubes...");
}

void closeAll() {
  if (servoLocked) {
    Serial.println("[BLOCK] Servo dang ban, lenh CloseAll bi TU CHOI");
    return;
  }
  seqMode = SEQ_CLOSING;
  seqIdx = 0;
  seqStepStart = 0;
  Serial.println("[CLOSE ALL] Closing all tubes...");
}

// ==================== Switch Logic ====================
void processSwitches() {
  SwPos sa = getSwitch(channels[4]); // CH5 = SwA
  SwPos sb = getSwitch(channels[5]); // CH6 = SwB
  SwPos sc = getSwitch(channels[6]); // CH7 = SwC
  SwPos sd = getSwitch(channels[7]); // CH8 = SwD

  // Lần đầu: chỉ ghi nhận trạng thái, không xử lý
  if (!switchesInitialized) {
    prevSA = sa; prevSB = sb; prevSC = sc; prevSD = sd;
    switchesInitialized = true;
    return;
  }

  // --- SA: ARM / DISARM ---
  if (sa != prevSA) {
    if (sa == SW_HIGH) {
      isArmed = true;
      isReloadMode = false;
      stopSequence();
      Serial.println("[SA] >> ARMED");
    } else if (sa == SW_LOW) {
      isArmed = false;
      isReloadMode = false;
      stopSequence();
      Serial.println("[SA] >> DISARMED");
    }
  }

  // Chỉ xử lý khi armed
  if (isArmed) {
    // --- SB: Tube 1 (MID), Tube 3 (HIGH) ---
    if (sb != prevSB && !isReloadMode) {
      if (sb == SW_MID)  dropTube(0); // Tube 1
      if (sb == SW_HIGH) dropTube(2); // Tube 3
    }

    // --- SC: Tube 2 (MID), Tube 4 (HIGH) ---
    if (sc != prevSC && !isReloadMode) {
      if (sc == SW_MID)  dropTube(1); // Tube 2
      if (sc == SW_HIGH) dropTube(3); // Tube 4
    }

    // --- SD: HIGH = Reload, LOW = Close All ---
    if (sd != prevSD) {
      if (sd == SW_HIGH) {
        startReload();
      } else if (sd == SW_LOW && prevSD == SW_HIGH) {
        closeAll();
      }
    }
  }

  prevSA = sa; prevSB = sb; prevSC = sc; prevSD = sd;
}

// ==================== Sequence Runner ====================
void runSequence() {
  if (seqMode == SEQ_IDLE || seqIdx >= 4) return;

  if (seqStepStart == 0 || millis() - seqStepStart >= SEQ_STEP) {
    bool ok = false;
    if (seqMode == SEQ_OPENING) {
      ok = setTube(seqIdx, true);
      if (ok) Serial.printf("[SEQ] Tube %d OPENED\n", seqIdx + 1);
    } else {
      ok = setTube(seqIdx, false);
      if (ok) Serial.printf("[SEQ] Tube %d CLOSED\n", seqIdx + 1);
    }

    // Chi tien hanh buoc tiep theo khi servo hien tai da thuc thi thanh cong
    if (ok) {
      seqIdx++;
      seqStepStart = millis();

      if (seqIdx >= 4) {
        Serial.printf("[SEQ] %s complete\n",
          seqMode == SEQ_OPENING ? "Reload" : "Close-all");
        if (seqMode == SEQ_CLOSING) isReloadMode = false;
        stopSequence();
      }
    }
    // Neu bi khoa (ok=false), khong tang seqIdx -> tu dong thu lai o vong lap sau
  }
}

// ==================== Auto-Close Timer ====================
void checkDropTimers() {
  if (isReloadMode) return;
  for (int i = 0; i < 4; i++) {
    if (isOpen[i] && dropTimer[i] > 0 && millis() - dropTimer[i] > DROP_TIMEOUT) {
      if (setTube(i, false)) {
        dropTimer[i] = 0;
        Serial.printf("[TIMER] Tube %d auto-closed\n", i + 1);
      }
      // Neu bi khoa, dropTimer giu nguyen va se thu lai o vong lap sau
    }
  }
}

// ==================== Status Display ====================
void printStatus() {
  Serial.printf("[%s|%s] Tubes:[%s][%s][%s][%s] SA:%s SB:%s SC:%s SD:%s%s%s\n",
    isArmed ? "ARMED" : "DISARM",
    isReloadMode ? "RELOAD" : "NORMAL",
    isOpen[0] ? "O" : "X", isOpen[1] ? "O" : "X",
    isOpen[2] ? "O" : "X", isOpen[3] ? "O" : "X",
    swName(prevSA), swName(prevSB), swName(prevSC), swName(prevSD),
    failsafe ? " FAILSAFE" : "",
    frameLost ? " FRAMELOST" : "");
}

// ==================== SETUP & LOOP ====================
void setup() {
  Serial.begin(9600);
  delay(500);

  // SBUS init
  Serial1.begin(100000, SERIAL_8E2, SBUS_RX, SBUS_TX);
  uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_RXD_INV | UART_SIGNAL_TXD_INV);

  // Servo init (khong qua lock vi chua co lenh nao truoc do)
  for (int i = 0; i < 4; i++) {
    servos[i].attach(SERVO_PINS[i], 500, 2400);
    int angle = (i == 0 || i == 3) ? 0 : 180;
    servos[i].write(angle);
    isOpen[i] = false;
  }
  delay(800); // Cho tat ca servo ve vi tri closed truoc khi bat dau

  Serial.println("\n================================");
  Serial.println("  TUBE DROP SYSTEM - SBUS RC");
  Serial.println("  RX: GPIO16 | TX: GPIO17");
  Serial.println("================================");
  Serial.println("SA: LOW=Disarm HIGH=Arm");
  Serial.println("SB: MID=Tube1  HIGH=Tube3");
  Serial.println("SC: MID=Tube2  HIGH=Tube4");
  Serial.println("SD: HIGH=Reload LOW=CloseAll");
  Serial.println("Dang cho tin hieu...\n");
}

void loop() {
  static unsigned long lastPrint = 0;

  readSBUS();

  // Kiểm tra mất kết nối
  if (lastFrameTime > 0 && millis() - lastFrameTime > 1000) {
    if (millis() - lastPrint >= 1000) {
      lastPrint = millis();
      Serial.println("[!] MAT KET NOI DIEU KHIEN!");
    }
    return;
  }

  if (lastFrameTime == 0) return; // Chưa nhận frame

  updateServoLock(); // Cap nhat trang thai khoa servo
  processSwitches();
  runSequence();
  checkDropTimers();

  // In trạng thái mỗi 2000ms
  if (millis() - lastPrint >= 2000) {
    lastPrint = millis();
    printStatus();
  }
}