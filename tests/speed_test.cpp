#include <cassert>
#include <cmath>
#include <iostream>
#include "../M5stickS3-motor-checker/M5stickS3-motor-checker.ino"

static void nextMode() {
  M5.BtnB.down = M5.BtnB.pressed = true;
  handleButtons();
  M5.BtnB = {};
  handleButtons();
}
static void frontButton(bool longPress) {
  M5.BtnA.down = M5.BtnA.pressed = true;
  handleButtons();
  M5.BtnA.pressed = false;
  if (longPress) {
    delay(1000); M5.BtnA.longHeld = true; handleButtons();
  }
  M5.BtnA.down = false; M5.BtnA.released = true;
  handleButtons(); M5.BtnA = {};
}
static void passage(uint32_t lowUs, uint32_t highUs) {
  fakeDriveSpeedInput(false); fakeUs += lowUs;
  fakeDriveSpeedInput(true); fakeUs += highUs;
}
int main() {
  setup(); batteryManager.update();
  nextMode(); nextMode(); nextMode();
  assert(sSpeedCaptureEnabled && fakeInterruptMode == CHANGE);
  fakeDriveSpeedInput(true); resetSpeedMeasurement();
  // One opening split by a short HIGH gap, even with a long LOW on either side.
  passage(3000, 500); passage(4000, 1100); processSpeed();
  assert(sSpeedRawCount == 2 && sSpeedPulseCount == 1);
  processSpeed(); assert(sSpeedPulseCount == 1);
  // Main loop need not run between passages: ISR completes a stable prior HIGH.
  resetSpeedMeasurement();
  passage(100, 2000); passage(100, 2000); processSpeed();
  assert(sSpeedPulseCount == 2);
  // A high gap just below/above the threshold merges/separates.
  resetSpeedMeasurement();
  passage(100, 998); passage(100, 1100); processSpeed();
  assert(sSpeedPulseCount == 1);
  resetSpeedMeasurement();
  passage(100, 1000); passage(100, 1100); processSpeed();
  assert(sSpeedPulseCount == 2);
  for (uint32_t lowUs : {5U, 60U, 120U, 1000U}) {
    resetSpeedMeasurement();
    for (int w = 0; w < 2; ++w) {
      for (int i = 0; i < 70; ++i) passage(lowUs, 7150 - lowUs);
      processSpeed();
      assert(sSpeedLastAcceptedCount == 70 && sSpeedLastRawCount == 70);
      assert(std::abs(sCurrentSpeedKmh - 30.02) < 0.1);
    }
  }
  // Peak is held at stop and lower speed, and updated only for higher speed.
  sMaximumSpeedKmh = 0; resetSpeedMeasurement();
  for (int i = 0; i < 70; ++i) passage(60, 7090);
  processSpeed(); const double peak = sMaximumSpeedKmh;
  assert(peak > 29 && peak == sCurrentSpeedKmh);
  delay(500); processSpeed();
  assert(sCurrentSpeedKmh == 0 && sMaximumSpeedKmh == peak);
  for (int i = 0; i < 35; ++i) passage(60, 14240);
  processSpeed(); assert(sCurrentSpeedKmh < peak && sMaximumSpeedKmh == peak);
  for (int i = 0; i < 100; ++i) passage(60, 4950);
  processSpeed(); assert(sMaximumSpeedKmh > peak);

  // Timestamp wrap and reset of pending passages.
  fakeUs = (1ULL << 32) - 500;
  resetSpeedMeasurement(); passage(5, 1100); processSpeed();
  assert(sSpeedPulseCount == 1);
  passage(100, 100); resetSpeedMeasurement();
  delay(500); processSpeed(); assert(sSpeedPulseCount == 0 && sCurrentSpeedKmh == 0);
  // Raw 20 transitions from ten split openings become ten accepted passages.
  resetSpeedMeasurement();
  for (int i = 0; i < 10; ++i) { passage(2000, 300); passage(2000, 2000); }
  delay(500); processSpeed();
  assert(sSpeedTotalRaw == 20 && sSpeedTotalAccepted == 10);
  // No diagnostics even with a serial monitor connected.
  M5.Power.vbus = 5000; batteryManager.begin(); batteryManager.update(); wakeDisplay();
  Serial.connected = true; Serial.output.clear();
  delay(1100);
  for (int i = 0; i < 100; ++i) loop();
  assert(Serial.output.empty());
  Serial.connected = false;
  sStopwatch.toggle(monotonicMs());
  frontButton(false);
  assert(sSpeedPulseCount == 0 && sStopwatch.running() && sMaximumSpeedKmh == 0);
  sMaximumSpeedKmh = 42;
  passage(100, 100); frontButton(true);
  assert(sMaximumSpeedKmh == 0);
  delay(500); processSpeed(); assert(sSpeedPulseCount == 0);
  nextMode(); assert(!sSpeedCaptureEnabled && !fakeSpeedInterrupt);
  nextMode(); nextMode(); nextMode();
  M5.Power.vbus = 0; batteryManager.begin(); batteryManager.update();
  noteActivity();
  sMaximumSpeedKmh = 42;
  delay(5000); updatePowerSaveState(false);
  delay(15000); updatePowerSaveState(false);
  assert(sDisplayBlanked && !sSpeedCaptureEnabled);
  wakeDisplay(); assert(sSpeedCaptureEnabled && sMaximumSpeedKmh == 42);
  M5.Power.vbus = 5000; batteryManager.begin(); batteryManager.update();
  delay(120000); updatePowerSaveState(false); assert(!sDisplayBlanked);
  const auto records = M5.Mic.records, reads = M5.In_I2C.reads, adc = fakeAdcReads;
  for (int i = 0; i < 100; ++i) loop();
  assert(M5.Mic.records == records && M5.In_I2C.reads == reads && fakeAdcReads == adc);
  std::cout << "PASS: LOW passage merging, stable HIGH threshold, delayed loop, narrow LOW, wrap, reset, serial, sleep and sensor isolation\n";
}
