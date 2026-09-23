#include <Arduino.h>
#include <NimBLEDevice.h>

// ESP32-C3 SuperMini. All switch to GND, read with internal pullups.
#define ENC_A_PIN     20
#define ENC_B_PIN     3
#define BTN1_PIN      1
#define BTN2_PIN      4
#define BTN_WHEEL_PIN 0
#define LED_PIN       8  // onboard blue LED, active low. On = connected and write characteristic found.

#define DEBOUNCE_MS 20
#define ENC_STEPS_PER_DETENT 2  // quadrature steps per wheel click; try 2 if one click does nothing

// Test UUIDs set up in nRF Connect's GATT server. Real bulb write UUID:
// a8b3fff1-4834-4051-89d0-3de95cddd318
#define SERVICE_UUID "19B10000-E8F2-537E-4F3C-D1A1D2E45670"
#define WRITE_UUID   "19B10001-E8F2-537E-4F3C-D1A1D2E45670"

// ---- BLE central ----

static NimBLEClient *client = nullptr;
static NimBLERemoteCharacteristic *writeChar = nullptr;
static NimBLEAddress target;
static volatile bool targetFound = false;

class ScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice *dev) override {
    if (!dev->isAdvertisingService(NimBLEUUID(SERVICE_UUID))) return;
    target = dev->getAddress();
    targetFound = true;
    NimBLEDevice::getScan()->stop();  // connect from loop(), not from a callback
  }
} scanCallbacks;

static void connectToTarget() {
  Serial.println("connecting...");
  if (!client->connect(target)) {
    Serial.println("connect failed");
    return;
  }
  writeChar = nullptr;
  for (NimBLERemoteService *svc : client->getServices(true)) {
    writeChar = svc->getCharacteristic(WRITE_UUID);
    if (writeChar) break;
  }
  Serial.println(writeChar ? "connected" : "connected, but no write characteristic");
}

static void maintainConnection() {
  if (client->isConnected()) return;
  if (targetFound) {
    targetFound = false;
    connectToTarget();
  } else if (!NimBLEDevice::getScan()->isScanning()) {
    NimBLEDevice::getScan()->start(5000);
  }
}

// BeeWi frame: 0x55 <cmd> <arg> 0x0D 0x0A
static void sendFrame(uint8_t cmd, uint8_t arg) {
  Serial.printf("send 55 %02X %02X 0D 0A\n", cmd, arg);
  if (!client->isConnected() || !writeChar) return;
  uint8_t frame[] = {0x55, cmd, arg, 0x0D, 0x0A};
  writeChar->writeValue(frame, sizeof(frame), !writeChar->canWriteNoResponse());
}

// ---- Scroll wheel: changes brightness 0..9 ----

// Quadrature decoder: index is (previous AB << 2) | current AB.
// Invalid transitions (bounce) count as 0.
static const int8_t ENC_TABLE[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
static volatile uint8_t encState = 0;
static volatile int32_t encSteps = 0;

static void IRAM_ATTR onEncoderChange() {
  encState = ((encState << 2) | (digitalRead(ENC_A_PIN) << 1) | digitalRead(ENC_B_PIN)) & 0x0F;
  encSteps += ENC_TABLE[encState];
}

static int brightness = 9;

static void pollEncoder() {
  static int32_t pending = 0;
  noInterrupts();
  pending += encSteps;
  encSteps = 0;
  interrupts();

  int clicks = pending / ENC_STEPS_PER_DETENT;
  if (clicks == 0) return;
  pending -= clicks * ENC_STEPS_PER_DETENT;

  int level = constrain(brightness + clicks, 0, 9);
  if (level == brightness) return;
  brightness = level;
  Serial.printf("brightness %d\n", brightness);
  sendFrame(0x12, brightness + 2);  // levels 0..9 are raw 2..11
}

// ---- Buttons: any press toggles power ----

struct Button {
  uint8_t pin;
  bool pressed;
  bool lastRaw;
  uint32_t changedAt;
};

static Button buttons[] = {{BTN1_PIN}, {BTN2_PIN}, {BTN_WHEEL_PIN}};
static bool powerOn = true;

static void pollButtons() {
  uint32_t now = millis();
  for (Button &b : buttons) {
    bool raw = digitalRead(b.pin) == LOW;
    if (raw != b.lastRaw) {
      b.lastRaw = raw;
      b.changedAt = now;
    } else if (raw != b.pressed && now - b.changedAt >= DEBOUNCE_MS) {
      b.pressed = raw;
      if (!raw) continue;
      powerOn = !powerOn;
      Serial.printf("GPIO%d pressed, power %s\n", b.pin, powerOn ? "on" : "off");
      sendFrame(0x10, powerOn);
    }
  }
}

// ---- Main ----

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);  // don't stall when no USB host is listening

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  for (Button &b : buttons) pinMode(b.pin, INPUT_PULLUP);

  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  encState = (digitalRead(ENC_A_PIN) << 1) | digitalRead(ENC_B_PIN);
  attachInterrupt(ENC_A_PIN, onEncoderChange, CHANGE);
  attachInterrupt(ENC_B_PIN, onEncoderChange, CHANGE);

  NimBLEDevice::init("BeeWi Remote");
  NimBLEDevice::getScan()->setScanCallbacks(&scanCallbacks);
  NimBLEDevice::getScan()->setActiveScan(true);
  client = NimBLEDevice::createClient();
  client->setConnectTimeout(5000);
}

void loop() {
  maintainConnection();
  digitalWrite(LED_PIN, client->isConnected() && writeChar ? LOW : HIGH);
  pollEncoder();
  pollButtons();
  delay(1);
}
