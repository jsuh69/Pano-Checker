// Execute the production sketch with deterministic board/time/capture fakes.
#include <cassert>
#include <iostream>
#include "../M5stickS3-motor-checker/M5stickS3-motor-checker.ino"

static void pressFront(bool held = false) {
  M5.BtnA = {}; M5.BtnA.down = M5.BtnA.pressed = true;
  handleButtons();
  M5.BtnA.pressed = false;
  if (held) {
    delay(1000); M5.BtnA.longHeld = true; handleButtons();
  }
  delay(50); M5.BtnA.down = false; M5.BtnA.released = true;
  handleButtons(); M5.BtnA = {};
}
static void pressMode() {
  M5.BtnB.down = M5.BtnB.pressed = true; handleButtons();
  M5.BtnB = {}; handleButtons();
}
int main() {
  setup();
  batteryManager.update();
  assert(!M5.lastConfigOutputPower && !(fakePowerConfig & 8));
  assert(!sMicEnabled && !sBmiEnabled);
  double hz; bool detected;
  // Fake fills samples only after 26 completion checks: catches premature FFT.
  assert(processAudioAndFFT(hz, detected) && !detected);
  assert(M5.Mic.polls >= 26 && !sMicEnabled && !M5.Mic.enabled);
  delay(500);
  assert(processAudioAndFFT(hz, detected) && detected && std::abs(hz - 500) < 1);
  // Silence resets consecutive evidence; out-of-band tones cannot be RPM.
  M5.Mic.amplitude = 0;
  assert(processAudioAndFFT(hz, detected) && !detected);
  M5.Mic.amplitude = 1000; M5.Mic.toneHz = 1000;
  assert(processAudioAndFFT(hz, detected) && !detected);
  M5.Mic.toneHz = 500;
  M5.Mic.stuck = true;
  const auto timeoutStart = monotonicMs();
  assert(!processAudioAndFFT(hz, detected));
  assert(monotonicMs() - timeoutStart <= 510 && !sMicEnabled);
  M5.Mic.stuck = false;
  // Mode switch during capture must abort the old frame and cancel calibration.
  startCalibration();
  bool switched = false;
  onUpdate = [&] {
    if (!switched) { M5.BtnB.pressed = M5.BtnB.down = true; switched = true; }
    else M5.BtnB.down = false;
  };
  assert(!processAudioAndFFT(hz, detected));
  onUpdate = {}; M5.BtnB = {}; handleButtons();
  assert(sCurrentMode == AppMode::ModeMag && !sMicEnabled);
  pressMode();
  assert(sCurrentMode == AppMode::ModeStopwatch && !sIsCalibrating);
  assert(!sHasTrackedPeak && !sSignalContinuous);
  pressFront();
  assert(sStopwatch.running());
  delay(1000); pressFront();
  assert(!sStopwatch.running() && sStopwatch.elapsedMs(monotonicMs()) == 1050);
  pressFront();
  assert(sStopwatch.running());
  const auto elapsedBefore = sStopwatch.elapsedMs(monotonicMs());
  // Two minutes stays bright. Sleep retains stopwatch, first press only wakes.
  sLastActivityMs = monotonicMs();
  delay(119999); updatePowerSaveState(false);
  assert(!sDisplayBlanked && sBrightness == PowerSave::kBrightnessActive);
  delay(1); updatePowerSaveState(false);
  assert(sDisplayBlanked && !sMicEnabled && !sBmiEnabled);
  sleepWhileBlanked();
  assert(sleepCalls == 1 && sStopwatch.elapsedMs(monotonicMs()) == elapsedBefore + 122000);
  wakeCause = ESP_SLEEP_WAKEUP_GPIO;
  sleepWhileBlanked();
  assert(!sDisplayBlanked);
  pressFront(true); // Entire wake gesture is consumed.
  assert(sStopwatch.running());
  pressFront(true);
  assert(!sStopwatch.running() && sStopwatch.elapsedMs(monotonicMs()) == 0);
  // USB attachment wakes the device and suppresses all idle power stages.
  delay(120000); updatePowerSaveState(false); assert(sDisplayBlanked);
  M5.Power.vbus = 5000;
  batteryManager.update();
  assert(batteryManager.hasExternalPower());
  updatePowerSaveState(false); assert(!sDisplayBlanked);
  delay(240000); updatePowerSaveState(false);
  assert(!sDisplayBlanked && sBrightness == PowerSave::kBrightnessActive);
  // Removal requires 3 observations and then restores battery policy.
  M5.Power.vbus = 0;
  for (int i = 0; i < 3; ++i) { delay(2000); batteryManager.update(); }
  assert(!batteryManager.hasExternalPower());
  noteActivity(); pressMode();
  assert(sCurrentMode == AppMode::ModePano);
  delay(5000); updatePowerSaveState(false);
  assert(sBrightness == PowerSave::kBrightnessDim);
  updatePowerSaveState(true);
  assert(sBrightness == PowerSave::kBrightnessActive);
  // Calibration resets magnetic filter; missing IMU fails without bogus FFT.
  pressMode(); sLastInstGauss = sCurrentGauss = 500;
  bool active;
  processMagnetic(active); delay(1000); processMagnetic(active);
  assert(!sIsCalibrating && sCurrentGauss == 0);
  pressMode(); pressMode();
  sSensLevel = SensLevel::Vib; syncPanoSensorPower();
  M5.In_I2C.failRead = true;
  assert(!processAudioAndFFT(hz, detected));
  M5.In_I2C.failRead = false;
  // A mode switch inside the blocking vibration sampler also aborts its frame.
  switched = false;
  onUpdate = [&] {
    if (!switched) { M5.BtnB.pressed = M5.BtnB.down = true; switched = true; }
    else M5.BtnB.down = false;
  };
  assert(!processAudioAndFFT(hz, detected));
  onUpdate = {}; M5.BtnB = {}; handleButtons();
  assert(sCurrentMode == AppMode::ModeMag && !sBmiEnabled);
  pressMode();
  const auto recordsBefore = M5.Mic.records;
  const auto readsBefore = M5.In_I2C.reads;
  const auto updatesBefore = M5.updates;
  for (int i = 0; i < 100; ++i) loop();
  assert(M5.Mic.records == recordsBefore && M5.In_I2C.reads == readsBefore);
  assert(M5.updates - updatesBefore <= 51); // 100 stopped loops are 2.5s, not 100 polls.
  // Diagnostics are absent without a monitor, and fit small USB TX FIFOs.
  Serial.connected = true; Serial.capacity = 64;
  const auto diagnosticReads = M5.In_I2C.reads;
  logPowerDiagnostics();
  logPowerDiagnostics();
  logPowerDiagnostics();
  assert(M5.In_I2C.reads == diagnosticReads + 3);
  assert(Serial.output.find("en=1 boost=0 imu=0 mic=0 watch=0") != std::string::npos);
  const auto firstLogSize = Serial.output.size();
  logPowerDiagnostics();
  assert(Serial.output.size() == firstLogSize && M5.In_I2C.reads == diagnosticReads + 3);
  M5.In_I2C.failRead = true; delay(30000);
  logPowerDiagnostics(); logPowerDiagnostics(); logPowerDiagnostics();
  assert(Serial.output.find("chg=-1 en=-1 boost=-1 imu=-1") != std::string::npos);
  M5.In_I2C.failRead = false; Serial.capacity = 0; delay(30000);
  const auto congestedReads = M5.In_I2C.reads;
  logPowerDiagnostics();
  assert(M5.In_I2C.reads == congestedReads);
  Serial.connected = false;
  logPowerDiagnostics();
  // Calibration must not bypass the 500ms cap on actual sketch loop execution.
  pressMode(); sSensLevel = SensLevel::High; syncPanoSensorPower();
  startCalibration();
  sLastFuncExecutionMs = millis();
  const auto calibratedRecords = M5.Mic.records;
  const auto analysisStart = monotonicMs();
  while (monotonicMs() - analysisStart < 2000) loop();
  assert(M5.Mic.records - calibratedRecords >= 3 && M5.Mic.records - calibratedRecords <= 4);
  std::cout << "PASS: production FFT/capture timeout/cancellation, mode/buttons, stopwatch, sleep/USB, magnetic calibration and IMU failure\n";
}
