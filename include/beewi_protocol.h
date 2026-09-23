#pragma once

// BeeWi SmartLite command frames, ported from BeeWiSmartLiteWinCTRL/beewi/protocol.py.
// Each command is 0x55 <cmd> <args...> 0x0D 0x0A.
//
// Protocol (UUIDs + command bytes) reverse-engineered by others; credit to:
// - BeewiPy by delkk0: https://github.com/delkk0/BeewiPy
// - light.beewi by bbo76: https://github.com/bbo76/light.beewi
// - Raspberry Pi forum thread: https://forums.raspberrypi.com/viewtopic.php?t=117729

#include <stddef.h>
#include <stdint.h>

#include <initializer_list>

namespace beewi {

// GATT characteristics on the bulb.
constexpr const char *WRITE_UUID = "a8b3fff1-4834-4051-89d0-3de95cddd318";
constexpr const char *READ_UUID  = "a8b3fff2-4834-4051-89d0-3de95cddd318";

constexpr int LEVEL_MIN = 0;
constexpr int LEVEL_MAX = 9;

struct Frame {
  uint8_t data[7];
  size_t len;
};

namespace detail {

constexpr uint8_t CMD_POWER       = 16;
constexpr uint8_t CMD_TEMPERATURE = 17;
constexpr uint8_t CMD_BRIGHTNESS  = 18;
constexpr uint8_t CMD_COLOR       = 19;
constexpr uint8_t CMD_WHITE       = 20;

// Brightness / temperature levels 0..9 map onto raw bytes 2..11.
constexpr int LEVEL_OFFSET = 2;

inline Frame frame(uint8_t command, std::initializer_list<uint8_t> args) {
  Frame f{};
  f.data[f.len++] = 0x55;
  f.data[f.len++] = command;
  for (uint8_t a : args) f.data[f.len++] = a;
  f.data[f.len++] = 0x0D;
  f.data[f.len++] = 0x0A;
  return f;
}

inline uint8_t level(int value) {
  if (value < LEVEL_MIN) value = LEVEL_MIN;
  if (value > LEVEL_MAX) value = LEVEL_MAX;
  return (uint8_t)(value + LEVEL_OFFSET);
}

}  // namespace detail

// Turn the bulb on.
inline Frame cmdOn() { return detail::frame(detail::CMD_POWER, {1}); }

// Turn the bulb off.
inline Frame cmdOff() { return detail::frame(detail::CMD_POWER, {0}); }

// Set brightness. level is 0 (dimmest) to 9 (brightest).
inline Frame cmdBrightness(int level) {
  return detail::frame(detail::CMD_BRIGHTNESS, {detail::level(level)});
}

// Set white color temperature. level is 0 (warm) to 9 (cool).
inline Frame cmdTemperature(int level) {
  return detail::frame(detail::CMD_TEMPERATURE, {detail::level(level)});
}

// Set an RGB color.
inline Frame cmdColor(uint8_t r, uint8_t g, uint8_t b) {
  return detail::frame(detail::CMD_COLOR, {r, g, b});
}

// Switch to plain white (full RGB white) mode.
inline Frame cmdWhite() { return detail::frame(detail::CMD_WHITE, {255, 255, 255}); }

// Decoded 5-byte status from the read characteristic.
struct Status {
  bool on;
  uint8_t brightnessOrWhite;
  uint8_t r, g, b;
};

// Decode [power, brightness/white, R, G, B]. Returns false if too short.
inline bool parseStatus(const uint8_t *data, size_t len, Status &out) {
  if (data == nullptr || len < 5) return false;
  out = {data[0] != 0, data[1], data[2], data[3], data[4]};
  return true;
}

}  // namespace beewi
