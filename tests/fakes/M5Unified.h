#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <string>

inline uint64_t fakeUs = 0;
inline int fakeAdc = 2048;
inline bool fakeGpioLow = false;
inline uint8_t fakePowerConfig = 8;
struct FakeSerial {
  bool connected = false;
  int capacity = 256;
  std::string output;
  explicit operator bool() const { return connected; }
  void begin(unsigned) {}
  void setTxTimeoutMs(unsigned) {}
  int availableForWrite() const { return capacity; }
  size_t write(const uint8_t* bytes, size_t length) {
    assertWriteFits(length);
    output.append(reinterpret_cast<const char*>(bytes), length);
    return length;
  }
  void assertWriteFits(size_t length) {
    if (length > (size_t)capacity) std::abort();
  }
};
inline FakeSerial Serial;
inline std::function<void()> onUpdate;
inline unsigned long millis() { return (unsigned long)(fakeUs / 1000); }
inline unsigned long micros() { return (unsigned long)++fakeUs; }
inline void delay(unsigned long ms) { fakeUs += ms * 1000; }
inline bool setCpuFrequencyMhz(unsigned) { return true; }
constexpr int INPUT = 0, LOW = 0;
inline void pinMode(int, int) {}
inline int digitalRead(int) { return fakeGpioLow ? 0 : 1; }
inline int analogRead(int) { return fakeAdc; }
inline long map(long x, long lo, long hi, long a, long b) { return (x-lo)*(b-a)/(hi-lo)+a; }
struct String : std::string {
  using std::string::string;
  String(long value) : std::string(std::to_string(value)) {}
  String(const std::string& value) : std::string(value) {}
};
namespace fonts { inline int FreeSans9pt7b, FreeSans12pt7b, FreeSans18pt7b, FreeSansBold18pt7b; }
constexpr int top_left = 0, top_right = 1, top_center = 2;
struct FakeDisplay {
  int brightness = 25;
  bool sleeping = false;
  unsigned draws = 0;
  int width() const { return 240; }
  int height() const { return 135; }
  void setBrightness(int value) { brightness = value; }
  void sleep() { sleeping = true; }
  void wakeup() { sleeping = false; }
  void setRotation(int) {}
  template<class... T> void drawString(T...) { ++draws; }
};
struct LGFX_Sprite : FakeDisplay {
  LGFX_Sprite(FakeDisplay*) {}
  bool createSprite(int, int) { return true; }
  template<class... T> void fillScreen(T...) {}
  template<class... T> void setFont(T...) {}
  template<class... T> void setTextColor(T...) {}
  template<class... T> void setTextDatum(T...) {}
  template<class... T> void drawRightString(T...) {}
  template<class... T> void drawRoundRect(T...) {}
  template<class... T> void fillRect(T...) {}
  template<class... T> void fillRoundRect(T...) {}
  template<class... T> void drawRect(T...) {}
  template<class... T> void pushSprite(T...) {}
  int textWidth(const String& value) { return (int)value.size() * 9; }
};
struct FakeButton {
  bool pressed = false, released = false, down = false, longHeld = false;
  bool wasPressed() const { return pressed; }
  bool wasReleased() const { return released; }
  bool isReleased() const { return !down; }
  bool pressedFor(unsigned long) const { return down && longHeld; }
  void clear() { pressed = released = false; }
};
namespace m5 { struct Power_Class { enum { is_discharging, is_charging }; }; }
struct FakePower {
  int vbus = 0, charging = m5::Power_Class::is_discharging, percent = 75;
  unsigned enables = 0;
  int getVBUSVoltage() { return vbus; }
  int isCharging() { return charging; }
  int getBatteryLevel() { return percent; }
  int getBatteryVoltage() { return 4060; }
  void setBatteryCharge(bool enabled) {
    ++enables;
    if (enabled) fakePowerConfig |= 1;
    else fakePowerConfig &= ~1;
  }
  void setExtOutput(bool enabled) {
    if (enabled) fakePowerConfig |= 8;
    else fakePowerConfig &= ~8;
  }
  void setLed(int) {}
};
struct FakeMic {
  struct Config { unsigned sample_rate = 8000; } cfg;
  bool enabled = false, stuck = false, fail = false;
  double toneHz = 500, amplitude = 1000;
  unsigned starts = 0, records = 0, polls = 0;
  int remaining = 0;
  int16_t* buffer = nullptr;
  size_t length = 0;
  Config config() { return cfg; }
  void config(Config c) { cfg = c; }
  bool begin() { ++starts; enabled = !fail; return enabled; }
  bool isRunning() const { return enabled; }
  void end() { enabled = false; remaining = 0; }
  bool record(int16_t* data, size_t count, double) {
    ++records;
    buffer = data; length = count; remaining = 26;
    return enabled && !fail;
  }
  int isRecording() {
    ++polls;
    if (stuck) return 1;
    if (remaining && --remaining == 0) {
      for (size_t i = 0; i < length; ++i)
        buffer[i] = (int16_t)(amplitude * std::sin(2 * M_PI * toneHz * i / cfg.sample_rate));
    }
    return remaining > 0;
  }
};
struct FakeI2C {
  bool failRead = false, failWrite = false;
  unsigned reads = 0;
  bool readRegister(int address, int reg, uint8_t* dest, size_t length, unsigned) {
    ++reads;
    if (failRead) return false;
    std::memset(dest, 0, length);
    if (reg == 0) dest[0] = 0x24;
    if (address == 0x6E && reg == 0x06) dest[0] = fakePowerConfig;
    return true;
  }
  bool writeRegister8(int, int, int, unsigned) { return !failWrite; }
};
struct FakeM5 {
  struct Config { bool internal_spk = true, internal_rtc = true, output_power = true; };
  bool lastConfigOutputPower = true;
  unsigned updates = 0;
  FakeDisplay Display;
  FakeButton BtnA, BtnB;
  FakePower Power;
  FakeMic Mic;
  FakeI2C In_I2C;
  struct { void end() {} } Speaker;
  Config config() { return {}; }
  void begin(Config cfg) {
    lastConfigOutputPower = cfg.output_power;
    Power.setExtOutput(cfg.output_power);
  }
  void update() {
    ++updates;
    BtnA.clear(); BtnB.clear();
    if (onUpdate) onUpdate();
  }
};
inline FakeM5 M5;
