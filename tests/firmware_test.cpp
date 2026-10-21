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
  assert(processAudioAndFFT(hz, detected) && detected && std::abs(hz - 500) < 1);
  assert(M5.Mic.polls >= 26 && !sMicEnabled && !M5.Mic.enabled);
  delay(500);
  assert(processAudioAndFFT(hz, detected) && detected && std::abs(hz - 500) < 1);
  // Every microphone sensitivity detects the first frame and tracks changing RPM.
  for (SensLevel sensitivity : {SensLevel::Low, SensLevel::Mid, SensLevel::High, SensLevel::Auto}) {
    sSensLevel = sensitivity; sAutoIsVib = false;
    resetDetection();
    for (double tone : {300.0, 320.0, 420.0, 467.0, 470.0, 530.0, 580.0, 620.0, 650.0}) {
      M5.Mic.toneHz = tone;
      assert(processAudioAndFFT(hz, detected) && detected && std::abs(hz - tone) < 3);
    }
    M5.Mic.amplitude = 0;
    assert(processAudioAndFFT(hz, detected) && !detected);
    M5.Mic.amplitude = 1000; M5.Mic.toneHz = 500;
    assert(processAudioAndFFT(hz, detected) && detected);
  }
  // Weak PCM previously failed LOW/MID: both now detect it on the first frame.
  M5.Mic.amplitude = 130; M5.Mic.toneHz = 500;
  for (SensLevel sensitivity : {SensLevel::Low, SensLevel::Mid, SensLevel::High}) {
    sSensLevel = sensitivity;
    assert(processAudioAndFFT(hz, detected) && detected && std::abs(hz - 500) < 1);
  }
  M5.Mic.amplitude = 0;
  assert(processAudioAndFFT(hz, detected) && !detected);
  M5.Mic.amplitude = 1000;
  resetDetection();
  // A bright clean tone above the old 4-million magnitude cap remains valid.
  sSensLevel = SensLevel::High; M5.Mic.amplitude = 25000; M5.Mic.toneHz = 500;
  assert(processAudioAndFFT(hz, detected) && detected && std::abs(hz - 500) < 1);
  // Discarded startup frame cannot drive the FFT (fake supplies wrong startup tone).
  M5.Mic.amplitude = 1000; M5.Mic.toneHz = 450; M5.Mic.startupToneHz = 320;
  assert(processAudioAndFFT(hz, detected) && detected && std::abs(hz - 450) < 3);
  M5.Mic.startupToneHz = 0;
  M5.Mic.toneHz = 467; M5.Mic.amplitude = 1000;
  assert(processAudioAndFFT(hz, detected) && detected && std::abs(hz - 467) < 1);
  std::cout << "467Hz synthetic PCM -> " << hz << "Hz\n";
  // Loud sub-band tones must not become bogus 300Hz readings at a cropped edge.
  sSensLevel = SensLevel::High; M5.Mic.amplitude = 12000;
  for (double tone : {100.0, 200.0, 280.0, 290.0, 670.0, 700.0}) {
    M5.Mic.toneHz = tone;
    assert(processAudioAndFFT(hz, detected) && !detected);
  }
  M5.Mic.amplitude = 1000;
  // The expanded microphone band still rejects tones clearly below 300Hz.
  sSensLevel = SensLevel::High; M5.Mic.toneHz = 250;
  assert(processAudioAndFFT(hz, detected) && !detected);
  M5.Mic.toneHz = 500;
  sSensLevel = SensLevel::High;
  // Silence and out-of-band tones cannot be RPM.
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
  assert(!sSignalContinuous);
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
  assert(sCurrentMode == AppMode::ModeSpeed);
  pressMode();
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
  // Magnetic samples at 10Hz; rise approaches the new field within ~300ms.
  fakeAdc = (int)sMagBaseline + 100;
  const auto magneticStart = monotonicMs();
  const auto magneticAdcBefore = fakeAdcReads;
  while (monotonicMs() - magneticStart < 400) loop();
  assert(sCurrentGauss > 87.2 * 0.95 && sCurrentGauss <= 87.2);
  assert(fakeAdcReads - magneticAdcBefore >= 96 && fakeAdcReads - magneticAdcBefore <= 160);
  fakeAdc = (int)sMagBaseline;
  for (int i = 0; i < 20; ++i) { delay(100); processMagnetic(active); }
  assert(sCurrentGauss < 0.1);

  pressMode(); pressMode(); pressMode();
  sSensLevel = SensLevel::Vib; syncPanoSensorPower();
  M5.In_I2C.failRead = true;
  assert(!processAudioAndFFT(hz, detected));
  M5.In_I2C.failRead = false;
  assert(M5.In_I2C.vibPowerConfig == 2);
  M5.In_I2C.vibInjectDelay = true; M5.In_I2C.vibDataReads = 0;
  assert(processAudioAndFFT(hz, detected) && !sVibTimingRetry && !M5.In_I2C.vibFifoEnabled);
  M5.In_I2C.vibFifoOverflow = true;
  assert(processAudioAndFFT(hz, detected) && !detected && sVibTimingRetry && !M5.In_I2C.vibFifoEnabled);
  M5.In_I2C.vibFifoOverflow = false;
  M5.In_I2C.vibInjectDelay = false;
  // Signed dominant-axis FFT preserves horizontal fundamentals instead of doubling.
  for (int axis : {0, 1, 2}) {
    for (double tone : {200.0, 350.0, 467.0, 600.0, 700.0}) {
      M5.In_I2C.vibAxis = axis; M5.In_I2C.vibToneHz = tone;
      resetDetection();
      assert(processAudioAndFFT(hz, detected) && !detected);
      delay(500);
      assert(processAudioAndFFT(hz, detected) && detected && std::abs(hz - tone) < 3);
    }
  }
  M5.In_I2C.vibAmplitude = 0;
  assert(processAudioAndFFT(hz, detected) && !detected);
  M5.In_I2C.vibAmplitude = 1000; M5.In_I2C.vibToneHz = 467;
  M5.In_I2C.vibStuck = true;
  const auto vibTimeout = monotonicMs();
  assert(!processAudioAndFFT(hz, detected) && monotonicMs() - vibTimeout < 110);
  M5.In_I2C.vibStuck = false;
  // A mode switch inside the blocking vibration sampler also aborts its frame.
  switched = false;
  onUpdate = [&] {
    if (!switched) { M5.BtnB.pressed = M5.BtnB.down = true; switched = true; }
    else M5.BtnB.down = false;
  };
  assert(!processAudioAndFFT(hz, detected));
  onUpdate = {}; M5.BtnB = {}; handleButtons();
  assert(sCurrentMode == AppMode::ModeMag && !sBmiEnabled && M5.In_I2C.vibPowerConfig == 3);
  pressMode();
  const auto recordsBefore = M5.Mic.records;
  const auto readsBefore = M5.In_I2C.reads;
  const auto updatesBefore = M5.updates;
  for (int i = 0; i < 100; ++i) loop();
  assert(M5.Mic.records == recordsBefore && M5.In_I2C.reads == readsBefore);
  assert(M5.updates - updatesBefore <= 51); // 100 stopped loops are 2.5s, not 100 polls.
  // A connected monitor must not trigger periodic logs or diagnostic bus reads.
  Serial.connected = true; Serial.output.clear(); delay(30000);
  const auto diagnosticReads = M5.In_I2C.reads;
  for (int i = 0; i < 100; ++i) loop();
  assert(Serial.output.empty() && M5.In_I2C.reads == diagnosticReads);
  Serial.connected = false;
  // Calibration must not bypass the 500ms cap on actual sketch loop execution.
  pressMode(); pressMode(); sSensLevel = SensLevel::High; syncPanoSensorPower();
  startCalibration();
  sLastFuncExecutionMs = millis();
  const auto calibratedRecords = M5.Mic.records;
  const auto analysisStart = monotonicMs();
  while (monotonicMs() - analysisStart < 2000) loop();
  assert(M5.Mic.records - calibratedRecords >= 6 && M5.Mic.records - calibratedRecords <= 8);
  std::cout << "PASS: production FFT/capture timeout/cancellation, mode/buttons, stopwatch, sleep/USB, magnetic calibration and IMU failure\n";
}
