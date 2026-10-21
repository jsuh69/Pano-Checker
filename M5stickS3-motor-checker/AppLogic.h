#pragma once
#include <stdint.h>

namespace Speed {
constexpr double kRollerDiameterMm = 19.0;
constexpr uint32_t kPulsesPerRevolution = 1;
constexpr uint32_t kSampleWindowUs = 500000;

inline double kmhFromPulses(uint32_t pulses, uint32_t elapsedUs) {
  if (elapsedUs == 0) return 0;
  constexpr double pi = 3.14159265358979323846;
  const double metersPerPulse = pi * kRollerDiameterMm / 1000.0 / kPulsesPerRevolution;
  return pulses * metersPerPulse * 3600000.0 / elapsedUs;
}
} // namespace Speed

enum class FrontAction : uint8_t { None, ShortPress, LongPress };
class FrontButtonGesture {
 public:
  void cancel() { armed_ = false; longDone_ = false; }
  FrontAction update(bool pressed, bool released, bool heldLong) {
    if (pressed) { armed_ = true; longDone_ = false; }
    if (armed_ && heldLong && !longDone_) {
      longDone_ = true;
      return FrontAction::LongPress;
    }
    if (released && armed_) {
      armed_ = false;
      return longDone_ ? FrontAction::None : FrontAction::ShortPress;
    }
    return FrontAction::None;
  }
 private:
  bool armed_ = false;
  bool longDone_ = false;
};

// All times come from the same monotonic clock, including light-sleep time.
class Stopwatch {
 public:
  bool running() const { return running_; }
  uint64_t elapsedMs(uint64_t nowMs) const {
    return accumulatedMs_ + (running_ ? nowMs - startedMs_ : 0);
  }
  void toggle(uint64_t nowMs) {
    if (running_) accumulatedMs_ += nowMs - startedMs_;
    else startedMs_ = nowMs;
    running_ = !running_;
  }
  void reset() { running_ = false; accumulatedMs_ = 0; startedMs_ = 0; }
 private:
  bool running_ = false;
  uint64_t accumulatedMs_ = 0;
  uint64_t startedMs_ = 0;
};

enum class DisplayPower : uint8_t { Bright, Dim, Off };
inline DisplayPower displayPowerFor(uint64_t idleMs, bool usb, bool stopwatch,
                                    bool calibrating) {
  if (usb || calibrating) return DisplayPower::Bright;
  if (stopwatch) return idleMs < 120000 ? DisplayPower::Bright : DisplayPower::Off;
  if (idleMs >= 20000) return DisplayPower::Off;
  return idleMs >= 5000 ? DisplayPower::Dim : DisplayPower::Bright;
}
