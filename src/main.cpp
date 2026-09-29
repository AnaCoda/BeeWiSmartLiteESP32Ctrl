#include <Arduino.h>
#include <NimBLEDevice.h>

#include <algorithm>

#include "beewi_protocol.h"

// My configs on ESP32-C3 SuperMini. All switch to GND, read with internal pullups.
#define ENC_A_PIN     20
#define ENC_B_PIN     3
#define BTN_LEFT_PIN  1
#define BTN_RIGHT_PIN 4
#define BTN_WHEEL_PIN 0
#define LED_PIN       8  // onboard blue LED

#define DEBOUNCE_MS 20
#define ENC_STEPS_PER_DETENT 2  // quadrature steps per wheel click

// BULB1_ADDR (left click) and BULB2_ADDR (right click)
#if __has_include("secrets.h")
#include "secrets.h"
#else
#error "Copy include/secrets.example.h to include/secrets.h and add your bulb addresses"
#endif

#define CONNECT_TIMEOUT_MS 3000
#define LINK_TIMEOUT_10MS  600  // drop a link after 6 s of silence (NimBLE default 2.56 s)
#define RETRY_MIN_MS       5000   // doubles after each failed attempt...
#define RETRY_MAX_MS       60000  // ...up to this, and resets once connected

// ---- Bulbs ----

struct Bulb {
  const char *addr;
  NimBLEClient *client = nullptr;
  NimBLERemoteCharacteristic *writeChar = nullptr;
  bool on = true;
  uint32_t lastAttempt = 0;
  uint32_t retryMs = RETRY_MIN_MS;
};

static Bulb bulbs[] = {{BULB1_ADDR}, {BULB2_ADDR}};

// Levels the scroll wheel sets. Only levels the wheel has set since boot are
// re-sent on reconnect, so a reset doesn't override what the bulbs already had.
static bool wheelWarmth = false;  // wheel click switches between brightness and warmth
static int brightness = beewi::LEVEL_MAX;
static int warmth = 5;  // default
static bool brightnessSet = false;
static bool warmthSet = false;

// Reason codes: 0x208 = signal lost (supervision timeout), 0x213 = bulb closed the
// connection, 0x216 = we closed it, 0x23E = connection never established
// for serial debugging
class ClientCallbacks : public NimBLEClientCallbacks {
  void onDisconnect(NimBLEClient *c, int reason) override {
    Serial.printf("%s: disconnected, reason 0x%X\n", c->getPeerAddress().toString().c_str(), reason);
  }
} clientCallbacks;

static bool isReady(const Bulb &b) { return b.client->isConnected() && b.writeChar; }

static void send(Bulb &b, const beewi::Frame &frame) {
  Serial.printf("%s: send", b.addr);
  for (size_t i = 0; i < frame.len; i++) Serial.printf(" %02X", frame.data[i]);
  Serial.println();
  b.writeChar->writeValue(frame.data, frame.len, !b.writeChar->canWriteNoResponse());
}

// Blocks up to CONNECT_TIMEOUT_MS, so buttons aren't read meanwhile.
static void connectBulb(Bulb &b) {
  b.writeChar = nullptr;
  Serial.printf("%s: connecting...\n", b.addr);
  if (!b.client->connect(NimBLEAddress(b.addr, BLE_ADDR_PUBLIC))) {
    Serial.printf("%s: connect failed\n", b.addr);
    return;
  }

  NimBLERemoteCharacteristic *readChar = nullptr;
  for (NimBLERemoteService *svc : b.client->getServices(true)) {
    if (!b.writeChar) b.writeChar = svc->getCharacteristic(beewi::WRITE_UUID);
    if (!readChar) readChar = svc->getCharacteristic(beewi::READ_UUID);
  }
  if (!b.writeChar) {
    Serial.printf("%s: no write characteristic, disconnecting\n", b.addr);
    b.client->disconnect();
    return;
  }

  // Start from the bulb's real power state so the first toggle does something.
  beewi::Status status;
  if (readChar && readChar->canRead()) {
    NimBLEAttValue value = readChar->readValue();
    if (beewi::parseStatus(value.data(), value.size(), status)) b.on = status.on;
  }
  b.retryMs = RETRY_MIN_MS;
  Serial.printf("%s: connected, %s, RSSI %d dBm\n", b.addr, b.on ? "on" : "off", b.client->getRssi());

  // Catch up on wheel changes made while this bulb was disconnected
  if (warmthSet) send(b, beewi::cmdTemperature(warmth));
  if (brightnessSet) send(b, beewi::cmdBrightness(brightness));
}

static void maintainBulbs() {
  uint32_t now = millis();
  for (Bulb &b : bulbs) {
    if (b.client->isConnected()) continue;
    if (b.lastAttempt != 0 && now - b.lastAttempt < b.retryMs) continue;
    b.lastAttempt = now;
    connectBulb(b);
    if (!isReady(b)) {
      b.retryMs = std::min<uint32_t>(b.retryMs * 2, RETRY_MAX_MS);
      Serial.printf("%s: retrying in %lus\n", b.addr, (unsigned long)(b.retryMs / 1000));
    }
    return; 
  }
}

static void toggle(Bulb &b) {
  if (!isReady(b)) {
    Serial.printf("%s: not connected\n", b.addr);
    return;
  }
  b.on = !b.on;
  send(b, b.on ? beewi::cmdOn() : beewi::cmdOff());
}

// ---- Scroll wheel: brightness or warmth for both bulbs ----

// Quadrature decoder: index is (previous AB << 2) | current AB
// Invalid transitions (bounce) count as 0 (ignore)
static const int8_t ENC_TABLE[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
static volatile uint8_t encState = 0;
static volatile int32_t encSteps = 0;

static void IRAM_ATTR onEncoderChange() {
  encState = ((encState << 2) | (digitalRead(ENC_A_PIN) << 1) | digitalRead(ENC_B_PIN)) & 0x0F;
  encSteps += ENC_TABLE[encState];
}

static void pollEncoder() {
  static int32_t pending = 0;
  noInterrupts();
  pending += encSteps;
  encSteps = 0;
  interrupts();

  int clicks = pending / ENC_STEPS_PER_DETENT;
  if (clicks == 0) return;
  pending -= clicks * ENC_STEPS_PER_DETENT;

  int &value = wheelWarmth ? warmth : brightness;
  int level = constrain(value + clicks, beewi::LEVEL_MIN, beewi::LEVEL_MAX);
  if (level == value) return;
  value = level;
  (wheelWarmth ? warmthSet : brightnessSet) = true;
  Serial.printf("%s %d\n", wheelWarmth ? "warmth" : "brightness", value);
  for (Bulb &b : bulbs) {
    if (!isReady(b)) continue;
    send(b, wheelWarmth ? beewi::cmdTemperature(value) : beewi::cmdBrightness(value));
  }
}

static void onWheelClick() {
  wheelWarmth = !wheelWarmth;
  Serial.printf("wheel adjusts %s\n", wheelWarmth ? "warmth" : "brightness");
}

// ---- Buttons ----

static void onLeft() { toggle(bulbs[0]); }
static void onRight() { toggle(bulbs[1]); }

struct Button {
  uint8_t pin;
  void (*onPress)();
  bool pressed = false;
  bool lastRaw = false;
  uint32_t changedAt = 0;
};

static Button buttons[] = {
    {BTN_LEFT_PIN, onLeft},
    {BTN_RIGHT_PIN, onRight},
    {BTN_WHEEL_PIN, onWheelClick},
};

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
      Serial.printf("GPIO%d pressed\n", b.pin);
      if (b.onPress) b.onPress();
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
  for (Bulb &b : bulbs) {
    b.client = NimBLEDevice::createClient();
    b.client->setClientCallbacks(&clientCallbacks, false);
    b.client->setConnectTimeout(CONNECT_TIMEOUT_MS);
    // Interval 30-50 ms (1.25 ms units), no latency, longer link timeout to try and ride out weak signal (more or less arbitrarily picked)
    b.client->setConnectionParams(24, 40, 0, LINK_TIMEOUT_10MS);
  }
}

void loop() {
  maintainBulbs();

  bool allReady = true;
  for (Bulb &b : bulbs) allReady &= isReady(b);
  digitalWrite(LED_PIN, allReady ? LOW : HIGH);

  pollEncoder();
  pollButtons();
  delay(1);
}
