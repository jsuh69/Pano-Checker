#include <M5Unified.h>
#include <arduinoFFT.h>
#include <cmath>
#include "AppLogic.h"
#include <esp_timer.h>
#include <esp_sleep.h>
#include <driver/gpio.h>
#if defined(ESP32)
#include <WiFi.h>
#include <esp_bt.h>
#include <soc/soc_caps.h>
#endif

// ==========================================
// 비동기 타이머 관리를 위한 전역 변수 (상단 전역 구역에 배치)
// ==========================================
static unsigned long sLastButtonCheckMs = 0;   // 버튼 체크용 타이머
static unsigned long sLastFuncExecutionMs = 0; // 센서/FFT/디스플레이 기능용 타이머

// ==========================================
// Constants & Settings
// ==========================================
namespace App {
constexpr char kVersion[] = "0.9.1";
} // namespace App

namespace Audio {
// 8 kHz / 1024 samples preserves the previous 7.8125 Hz FFT resolution while
// roughly halving microphone transfer and FFT work.  It still covers the
// 400–600 Hz motor band with ample Nyquist margin.
constexpr double kSamplingFrequency = 8000;
constexpr uint16_t kSamples = 1024;
constexpr uint16_t kVibSamples = 512;
constexpr int kFftFirstUsableBin = 1;
constexpr int kFftLastUsableBin = (kSamples / 2) - 2;
constexpr double kMinValidRpmHz = 50.0;
constexpr double kFftPeakRejectAbove = 4000000.0;

// The motor tone is expected near 400–600 Hz.  The wider reference band is
// used only to estimate local background noise around that tone.
constexpr int kMicTargetBandLoHz = 400;
constexpr int kMicTargetBandHiHz = 600;
constexpr int kMicReferenceBandLoHz = 350;
constexpr int kMicReferenceBandHiHz = 650;
constexpr double kMicMinimumPeakMagnitude = 25000.0;
constexpr double kTrackedFrequencyToleranceHz = 18.0;
constexpr unsigned long kAnalysisIntervalMs = 500;
} // namespace Audio

namespace PowerSave {
constexpr uint8_t kBrightnessActive = 25;
constexpr uint8_t kBrightnessDim = 3;
constexpr unsigned long kUiRefreshActiveMs = 500;
constexpr unsigned long kUiRefreshIdleMs = 1000;
constexpr uint8_t kCpuFreqActiveMhz = 80;
constexpr unsigned long kStoppedButtonPollMs = 50;
constexpr unsigned long kStoppedLoopDelayMs = 25;
constexpr uint64_t kSleepPollUs = 2000000; // Check USB/charger every two seconds.
// StickS3 front and upper buttons (M5Unified board pin mapping).
constexpr gpio_num_t kFrontButton = GPIO_NUM_11;
constexpr gpio_num_t kModeButton = GPIO_NUM_12;
} // namespace PowerSave

namespace BatConfig {
constexpr int16_t kVbusPresentMinMv = 4200;
} // namespace BatConfig

namespace Diagnostics {
constexpr bool kPowerLogEnabled = true;
constexpr unsigned long kPowerLogIntervalMs = 30000;
} // namespace Diagnostics

namespace UI {
constexpr uint16_t COLOR_BG = 0x0000;
constexpr uint16_t COLOR_TEXT = 0xFFFF;
constexpr uint16_t COLOR_BAT_CHARGING = 0xFFE0; // 충전 중: 노란색
constexpr uint16_t COLOR_BAT_GOOD = 0x07E0;     // 방전 중 양호: 초록색
constexpr uint16_t COLOR_BAT_LOW = 0xF800;      // 방전 중 부족: 빨간색
constexpr uint16_t COLOR_SENS_LABEL = 0xFD20;
constexpr uint16_t COLOR_RPM_VALUE = 0xFDA0;
constexpr uint16_t COLOR_MAG_VALUE = 0x07FF; 
constexpr uint16_t COLOR_READY = 0x7BEF;
} // namespace UI

// ==========================================
// Global State & Objects
// ==========================================
// ESP32-S3 has a single-precision FPU. Avoid software double arithmetic in FFT.
float vReal[Audio::kSamples];
float vImag[Audio::kSamples];
ArduinoFFT<float> FFT(vReal, vImag, Audio::kSamples, Audio::kSamplingFrequency);
static float sMicWindow[Audio::kSamples / 2];
static float sVibWindow[Audio::kVibSamples / 2];

static void prepareWindows() {
  for (int i = 0; i < Audio::kSamples / 2; ++i)
    sMicWindow[i] = 0.54f - 0.46f * cosf(2.0f * (float)M_PI * i / (Audio::kSamples - 1));
  for (int i = 0; i < Audio::kVibSamples / 2; ++i)
    sVibWindow[i] = 0.54f - 0.46f * cosf(2.0f * (float)M_PI * i / (Audio::kVibSamples - 1));
}

LGFX_Sprite spr(&M5.Display);

double lastHz = 0;
bool sForceUiRefresh = false;

enum class AppMode : uint8_t { ModePano, ModeMag, ModeStopwatch };
static AppMode sCurrentMode = AppMode::ModePano;

constexpr int kMagPin = 1;
static double sMagBaseline = -1; 
static double sCurrentGauss = 0; 
static char sCurrentPole = ' ';  

static bool sDisplayBlanked = false;
static uint64_t sLastActivityMs = 0;
static uint8_t sBrightness = PowerSave::kBrightnessActive;
static uint32_t sMeasurementGeneration = 0;
static bool sConsumeButtonsUntilRelease = false;
static bool sSensorError = false;
// Assume enabled until setup has explicitly powered down the IMU.
static bool sBmiEnabled = true;
static bool sBmiPresent = false;
static bool sHasTrackedPeak = false;
static double sTrackedPeakHz = 0;
static uint8_t sTrackedPeakFrames = 0;
static unsigned long sSignalContinuousStartMs = 0;
static bool sSignalContinuous = false;
static double sLastInstGauss = 0;
static double sCurrentHz = 0;
static bool sDetected = false;
static bool sRpmActive = false;
static bool sMagActive = false;
static Stopwatch sStopwatch;
static FrontButtonGesture sFrontGesture;
static uint64_t sFrontPressedMs = 0;
static uint64_t monotonicMs() { return esp_timer_get_time() / 1000ULL; }

static void resetDetection() {
  sHasTrackedPeak = false;
  sTrackedPeakHz = 0;
  sTrackedPeakFrames = 0;
  sSignalContinuous = false;
  sSignalContinuousStartMs = 0;
  sCurrentHz = 0;
  sDetected = sRpmActive = sMagActive = false;
  lastHz = 0;
}
static bool sMicEnabled = false;

enum class SensLevel : uint8_t { Low = 0, Mid, High, Vib, Auto };
static SensLevel sSensLevel = SensLevel::High;
static bool sAutoIsVib = false;

static double sAutoThresholdMic = 200000.0;
static double sAutoThresholdUpperMic = Audio::kFftPeakRejectAbove;
static double sAutoThresholdVib = 5000.0;
static double sAutoThresholdUpperVib = Audio::kFftPeakRejectAbove;

static bool sIsCalibrating = false;
static unsigned long sCalibrateStartMs = 0;
static double sCalibrateSum = 0;
static int sCalibrateCount = 0;

// =======================================================
// 배터리 매니저 클래스 (UI 리프레시 연동형 고속 검사)
// =======================================================
class BatteryManager {
private:
  int stableBatPct = 0;
  bool uiCharging = false;
  bool pendingCharging = false;
  bool isInitialized = false;
  uint32_t lastCheckMs = 0;
  uint32_t lastChargerEnableMs = 0;
  uint8_t chargingConfirmations = 0;
  int percentSamples[3] = { 0, 0, 0 };
  uint8_t percentSampleCount = 0;
  uint8_t nextPercentSample = 0;

  static constexpr uint32_t kCheckIntervalMs = 2000;
  static constexpr uint32_t kChargerEnableRefreshMs = 10000;
  static constexpr uint8_t kChargingConfirmationsNeeded = 3;

public:
  bool externalPowerOrCharging() {
    // On StickS3, isCharging() is only the CHG_STAT pin: it turns off during
    // charge termination even while USB is still connected.  VIN is the
    // authoritative indication that the USB cable is supplying the device.
    const int16_t vbusMv = M5.Power.getVBUSVoltage();
    if (vbusMv >= BatConfig::kVbusPresentMinMv) return true;
    return M5.Power.isCharging() == m5::Power_Class::is_charging;
  }

  void begin() {
    isInitialized = false;
    stableBatPct = 0;
    lastCheckMs = 0;
    lastChargerEnableMs = 0;
    chargingConfirmations = 0;
    percentSampleCount = 0;
    nextPercentSample = 0;
  }

  void update() {
    uint32_t currentMs = millis();

    if (!isInitialized) {
      uiCharging = externalPowerOrCharging();
      pendingCharging = uiCharging;
      stableBatPct = M5.Power.getBatteryLevel(); 
      if (stableBatPct < 0) stableBatPct = 0;
      if (stableBatPct > 100) stableBatPct = 100;
      percentSamples[0] = stableBatPct;
      percentSamples[1] = stableBatPct;
      percentSamples[2] = stableBatPct;
      percentSampleCount = 3;
      isInitialized = true;
      lastCheckMs = currentMs;
      lastChargerEnableMs = currentMs;
      sForceUiRefresh = true;
      return;
    }

    if (currentMs - lastCheckMs >= kCheckIntervalMs) {
      lastCheckMs = currentMs;
      
      const bool sampledCharging = externalPowerOrCharging();

      // Keep the PM1 charger-enable bit asserted while external USB power is
      // present. This restores charging if a power-state transition cleared it.
      if (sampledCharging && currentMs - lastChargerEnableMs >= kChargerEnableRefreshMs) {
        M5.Power.setBatteryCharge(true);
        lastChargerEnableMs = currentMs;
      }
      // USB presence takes effect on the first positive reading so the display
      // stays on. Debounce removal to tolerate charge-status transients.
      if (sampledCharging && !uiCharging) {
        uiCharging = true;
        sForceUiRefresh = true;
      }
      if (sampledCharging != pendingCharging) {
        pendingCharging = sampledCharging;
        chargingConfirmations = 1;
      } else if (chargingConfirmations < 255) {
        ++chargingConfirmations;
      }

      // The PMIC charging-status pin can briefly change while USB power is
      // applied. Confirm cable removal with three matching samples.
      if (chargingConfirmations >= kChargingConfirmationsNeeded
          && pendingCharging != uiCharging) {
        uiCharging = pendingCharging;
        sForceUiRefresh = true;
      }

      const int rawPct = M5.Power.getBatteryLevel();
      if (rawPct < 0 || rawPct > 100) {
        return;
      }

      // A 3-sample median filters one-off voltage noise without concealing a
      // real charge or discharge.  Unlike the earlier direction lock, a drop
      // is always reflected even when a USB cable remains connected.
      percentSamples[nextPercentSample] = rawPct;
      nextPercentSample = (nextPercentSample + 1) % 3;
      if (percentSampleCount < 3) ++percentSampleCount;
      if (percentSampleCount == 3) {
        int a = percentSamples[0];
        int b = percentSamples[1];
        int c = percentSamples[2];
        if (a > b) { const int t = a; a = b; b = t; }
        if (b > c) { const int t = b; b = c; c = t; }
        if (a > b) { const int t = a; a = b; b = t; }
        if (b != stableBatPct) {
          stableBatPct = b;
          sForceUiRefresh = true;
        }
      }
    }
  }

  int getBatteryPercent() const { return stableBatPct; }
  bool hasExternalPower() const { return uiCharging; }
};

BatteryManager batteryManager; 

static void logPowerDiagnostics() {
  static unsigned long lastLogMs = 0;
  static char line[192];
  static size_t pending = 0, offset = 0;
  // No extra sensor/bus work when a serial monitor is not attached.
  if (!Diagnostics::kPowerLogEnabled || !Serial) {
    pending = offset = 0;
    return;
  }
  const unsigned long now = millis();
  if (pending == 0 && now - lastLogMs >= Diagnostics::kPowerLogIntervalMs) {
    lastLogMs = now;
    if (Serial.availableForWrite() <= 0) return;
    uint8_t powerConfig = 0, gpioInputs = 0, imuPower = 0;
    const bool configOk = M5.In_I2C.readRegister(0x6E, 0x06, &powerConfig, 1, 100000);
    const bool gpioOk = M5.In_I2C.readRegister(0x6E, 0x12, &gpioInputs, 1, 100000);
    const bool imuOk = sBmiPresent && M5.In_I2C.readRegister(0x68, 0x7D, &imuPower, 1, 400000);
    const int length = snprintf(line, sizeof(line),
        "PWR usb=%u vin=%d bat=%d pct=%d chg=%d en=%d boost=%d imu=%d mic=%u watch=%u\n",
        (unsigned)batteryManager.hasExternalPower(), (int)M5.Power.getVBUSVoltage(),
        (int)M5.Power.getBatteryVoltage(), batteryManager.getBatteryPercent(),
        gpioOk ? ((gpioInputs & 1) ? 0 : 1) : -1,
        configOk ? ((powerConfig & 1) ? 1 : 0) : -1,
        configOk ? ((powerConfig & 8) ? 1 : 0) : -1,
        imuOk ? (int)imuPower : -1, (unsigned)M5.Mic.isRunning(),
        (unsigned)sStopwatch.running());
    if (length > 0 && length < (int)sizeof(line)) {
      pending = (size_t)length;
      offset = 0;
    }
  }
  // Drain only currently available TX space, including on 64-byte USB FIFOs.
  // Never wait for the monitor to consume a message.
  const int available = Serial.availableForWrite();
  if (pending > 0 && available > 0) {
    const size_t chunk = pending < (size_t)available ? pending : (size_t)available;
    const size_t written = Serial.write(reinterpret_cast<const uint8_t*>(line + offset), chunk);
    pending -= written;
    offset += written;
  }
}

// ==========================================
// Helper Functions
// ==========================================
void setBmi270Power(bool enable) {
  if (enable == sBmiEnabled) return;
  if ((enable && !sBmiPresent)
      || !M5.In_I2C.writeRegister8(0x68, 0x7D, enable ? 0x04 : 0x00, 400000)) {
    sSensorError = true;
    sForceUiRefresh = true;
    return;
  }
  sBmiEnabled = enable;
}

static void applyBrightness(uint8_t brightness) {
  if (sBrightness == brightness) return;
  sBrightness = brightness;
  M5.Display.setBrightness(brightness);
}

static void noteActivity() {
  sLastActivityMs = monotonicMs();
  if (!sDisplayBlanked) applyBrightness(PowerSave::kBrightnessActive);
}

static void setMicrophonePower(bool enable) {
  if (enable == sMicEnabled) return;

  if (enable) {
    if (!M5.Mic.begin()) {
      sSensorError = true;
      sForceUiRefresh = true;
      return;
    }
  } else {
    M5.Mic.end();
  }
  sMicEnabled = enable;
}

static bool isVibrationSensorSelected() {
  return sSensLevel == SensLevel::Vib
      || (sSensLevel == SensLevel::Auto && sAutoIsVib)
      || (sIsCalibrating && sAutoIsVib);
}

// The BMI270 and microphone should never both run in Pano mode. HIGH/LOW/MID
// use airborne audio; only VIB needs the IMU.
static void syncPanoSensorPower() {
  const bool useVibrationSensor = isVibrationSensorSelected();
  setBmi270Power(useVibrationSensor);
  // Microphone runs only while capturing a frame, not between analyses.
  if (useVibrationSensor) setMicrophonePower(false);
}

static void cycleSensitivity() {
  ++sMeasurementGeneration;
  sIsCalibrating = false;
  sSensorError = false;
  resetDetection();
  switch (sSensLevel) {
    case SensLevel::Low:     sSensLevel = SensLevel::Mid; break;
    case SensLevel::Mid:     sSensLevel = SensLevel::High; break;
    case SensLevel::High:    sSensLevel = SensLevel::Vib; break;
    case SensLevel::Vib:     sSensLevel = SensLevel::Low; break;
    case SensLevel::Auto:    sSensLevel = SensLevel::Low; break;
  }
  if (sCurrentMode == AppMode::ModePano && !sDisplayBlanked) {
    syncPanoSensorPower();
  }
}

static double sensitivityThreshold() {
  switch (sSensLevel) {
    // Microphone sensitivity is determined primarily by peak-to-noise ratio,
    // not a fixed room-dependent FFT magnitude.
    case SensLevel::Low:
    case SensLevel::Mid:
    case SensLevel::High: return Audio::kMicMinimumPeakMagnitude;
    case SensLevel::Vib: return 5000.0;
    case SensLevel::Auto: return sAutoIsVib ? sAutoThresholdVib : sAutoThresholdMic;
  }
  return Audio::kMicMinimumPeakMagnitude;
}

static double sensitivityRequiredSnrDb() {
  switch (sSensLevel) {
    case SensLevel::Low:  return 12.0;
    case SensLevel::Mid:  return 9.0;
    case SensLevel::High: return 6.0;
    case SensLevel::Auto: return 8.0;
    case SensLevel::Vib:  return 0.0;
  }
  return 9.0;
}

static uint8_t sensitivityRequiredFrames() {
  switch (sSensLevel) {
    // At two analysis frames per second, these accumulate 1–2 seconds of
    // frequency-consistent evidence before reporting a motor tone.
    case SensLevel::Low:  return 4;
    case SensLevel::Mid:  return 3;
    case SensLevel::High: return 2;
    case SensLevel::Auto: return 3;
    case SensLevel::Vib:  return 1;
  }
  return 3;
}

static double sensitivityThresholdUpper() {
  if (sSensLevel == SensLevel::Auto) {
    return sAutoIsVib ? sAutoThresholdUpperVib : sAutoThresholdUpperMic;
  }
  return Audio::kFftPeakRejectAbove;
}

static const char *sensitivityLabel() {
  switch (sSensLevel) {
    case SensLevel::Low: return "LOW";
    case SensLevel::Mid: return "MID";
    case SensLevel::High: return "HIGH";
    case SensLevel::Vib: return "VIB";
    case SensLevel::Auto: return "AUTO";
  }
  return "";
}

static void disableUnusedBluetooth() {
#if defined(ESP32) && SOC_BT_SUPPORTED
  if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE) {
    (void)esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);
  }
#endif
}

String formatNumber(long n) {
  char buf[32];
  char out[64];
  snprintf(buf, sizeof(buf), "%ld", n);
  int len = strlen(buf);
  int outIdx = 0;
  for (int i = 0; i < len; i++) {
    out[outIdx++] = buf[i];
    if ((len - 1 - i) % 3 == 0 && i != len - 1) {
      out[outIdx++] = ',';
    }
  }
  out[outIdx] = '\0';
  return String(out);
}

// ==========================================
// Core Logic Functions
// ==========================================
void startCalibration() {
  ++sMeasurementGeneration;
  resetDetection();
  sSensorError = false;
  sIsCalibrating = true;
  sCalibrateStartMs = millis();
  sCalibrateSum = 0;
  sCalibrateCount = 0;
  
  if (sCurrentMode == AppMode::ModePano) {
    sAutoIsVib = (sSensLevel == SensLevel::Vib || (sSensLevel == SensLevel::Auto && sAutoIsVib));
  }
}

static void wakeDisplay() {
  sDisplayBlanked = false;
  M5.Display.wakeup();
  applyBrightness(PowerSave::kBrightnessActive);
  resetDetection();
  if (sCurrentMode == AppMode::ModePano) syncPanoSensorPower();
  noteActivity();
  sForceUiRefresh = true;
}

static void changeMode() {
  ++sMeasurementGeneration;
  sIsCalibrating = false;
  sCalibrateSum = 0;
  sCalibrateCount = 0;
  resetDetection();
  sSensorError = false;
  setMicrophonePower(false);
  setBmi270Power(false);
  switch (sCurrentMode) {
    case AppMode::ModePano: sCurrentMode = AppMode::ModeMag; break;
    case AppMode::ModeMag: sCurrentMode = AppMode::ModeStopwatch; break;
    case AppMode::ModeStopwatch: sCurrentMode = AppMode::ModePano; break;
  }
  if (sCurrentMode == AppMode::ModeMag) startCalibration();
  if (sCurrentMode == AppMode::ModePano) syncPanoSensorPower();
  // A held front button must never act on the newly selected mode.
  sConsumeButtonsUntilRelease = true;
  sFrontGesture.cancel();
  noteActivity();
  sForceUiRefresh = true;
}

void handleButtons() {
  if (sConsumeButtonsUntilRelease) {
    sFrontGesture.cancel();
    if (M5.BtnA.isReleased() && M5.BtnB.isReleased()) {
      sConsumeButtonsUntilRelease = false;
    }
    return;
  }
  if (sDisplayBlanked && (M5.BtnA.wasPressed() || M5.BtnB.wasPressed())) {
    wakeDisplay();
    sConsumeButtonsUntilRelease = true;
    return;
  }
  if (M5.BtnA.wasPressed() || M5.BtnB.wasPressed()) noteActivity();
  if (M5.BtnB.wasPressed()) {
    changeMode();
    return;
  }
  if (M5.BtnA.wasPressed()) sFrontPressedMs = monotonicMs();
  const FrontAction action = sFrontGesture.update(M5.BtnA.wasPressed(),
      M5.BtnA.wasReleased(), M5.BtnA.pressedFor(1000));
  if (action == FrontAction::LongPress) {
    if (sCurrentMode == AppMode::ModeStopwatch) sStopwatch.reset();
    else startCalibration();
    noteActivity();
    sForceUiRefresh = true;
  } else if (action == FrontAction::ShortPress) {
    if (sCurrentMode == AppMode::ModePano) cycleSensitivity();
    else if (sCurrentMode == AppMode::ModeMag) startCalibration();
    else sStopwatch.toggle(sFrontPressedMs); // Timestamp the press, not its release.
    noteActivity();
    sForceUiRefresh = true;
  }
}

bool processMagnetic(bool &magActive) {
  magActive = false;
  if (sIsCalibrating) {
    if (millis() - sCalibrateStartMs < 1000) {
      long subSum = 0;
      for(int i = 0; i < 32; i++) {
        subSum += analogRead(kMagPin);
      }
      sCalibrateSum += (subSum / 32.0);
      sCalibrateCount++;
      return true;
    } else {
      sIsCalibrating = false;
      if (sCalibrateCount > 0) {
        sMagBaseline = sCalibrateSum / sCalibrateCount;
        sLastInstGauss = sCurrentGauss = 0;
        sCurrentPole = ' ';
      }
      sForceUiRefresh = true;
    }
  }

  if (sMagBaseline < 0) return true; 

  long rawSum = 0;
  for(int i = 0; i < 32; i++) {
    rawSum += analogRead(kMagPin);
  }
  double rawVal = rawSum / 32.0; 
  
  sCurrentPole = (rawVal >= sMagBaseline) ? 'S' : 'N';
  double diffADC = fabs(rawVal - sMagBaseline);
  
  if (diffADC < 10.0) {
    diffADC = 0.0; 
    sCurrentPole = ' '; 
  }
  
  double instGauss = diffADC * 0.872;
  double filteredGauss = (sLastInstGauss * 0.75) + (instGauss * 0.25);
  sLastInstGauss = filteredGauss;
  sCurrentGauss = filteredGauss;

  magActive = (sCurrentGauss > 20.0); 
  return true;
}

bool processAudioAndFFT(double &outHz, bool &outDetected) {
  outHz = 0;
  outDetected = false;
  static int16_t rawBuffer[Audio::kSamples];
  static double sVibActualFs = 1600.0;
  const uint32_t generation = sMeasurementGeneration;

  bool useVib = (sSensLevel == SensLevel::Vib || (sSensLevel == SensLevel::Auto && sAutoIsVib) || (sIsCalibrating && sAutoIsVib));

  if (useVib) {
    if (!sBmiPresent || !sBmiEnabled) return false;
    unsigned long start_time = micros();
    unsigned long next_sample = start_time;
    float vibMean = 0;
    int actualSamples = 0;

    for (int i = 0; i < Audio::kVibSamples; i++) {
        while (micros() - next_sample < 625) { }
        next_sample += 625;
        
        uint8_t bytes[6];
        if (!M5.In_I2C.readRegister(0x68, 0x0C, bytes, sizeof(bytes), 400000)) {
          return false; // Discard incomplete frames instead of analysing bad data.
        }
        int16_t ax = (int16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
        int16_t ay = (int16_t)((uint16_t)bytes[2] | ((uint16_t)bytes[3] << 8));
        int16_t az = (int16_t)((uint16_t)bytes[4] | ((uint16_t)bytes[5] << 8));
        float mag = sqrtf((float)ax * ax + (float)ay * ay + (float)az * az);
        vReal[actualSamples++] = mag;
        vibMean += mag;

        if (i > 0 && i % 128 == 0) {
            M5.update();
            handleButtons();
            if (generation != sMeasurementGeneration) return false;
            next_sample = micros(); 
        }
    }
    
    unsigned long end_time = micros();
    const unsigned long elapsedUs = end_time - start_time;
    if (actualSamples != Audio::kVibSamples || elapsedUs == 0) {
      return false;
    }
    vibMean /= actualSamples;
    sVibActualFs = (actualSamples * 1000000.0) / (double)elapsedUs;
    
    for (int i = 0; i < Audio::kSamples; i++) {
       if (i < actualSamples) {
           const int wi = i < actualSamples / 2 ? i : actualSamples - 1 - i;
           vReal[i] = (vReal[i] - vibMean) * sVibWindow[wi];
       } else {
           vReal[i] = 0; 
       }
       vImag[i] = 0;
    }
  } else {
    setMicrophonePower(true);
    if (!sMicEnabled || !M5.Mic.record(rawBuffer, Audio::kSamples, Audio::kSamplingFrequency)) {
      setMicrophonePower(false);
      return false;
    }
    // record() queues capture. Do not touch rawBuffer until the producer finishes.
    const uint64_t captureStartedMs = monotonicMs();
    while (M5.Mic.isRecording()) {
      delay(5); // Yield instead of spinning while the I2S task captures samples.
      M5.update();
      handleButtons();
      if (generation != sMeasurementGeneration || monotonicMs() - captureStartedMs > 500) {
        setMicrophonePower(false); // end() also joins/cancels the writer.
        return false;
      }
    }
    setMicrophonePower(false);
    float mean = 0;
    for (int i = 0; i < Audio::kSamples; i++) mean += rawBuffer[i];
    mean /= Audio::kSamples;
    for (int i = 0; i < Audio::kSamples; i++) {
      const int wi = i < Audio::kSamples / 2 ? i : Audio::kSamples - 1 - i;
      vReal[i] = ((float)rawBuffer[i] - mean) * sMicWindow[wi];
      vImag[i] = 0;
    }
  }

  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();

  double currentFs = useVib ? sVibActualFs : Audio::kSamplingFrequency;
  if (currentFs <= 0) return false;
  const int searchBandLoHz = useVib ? 150 : Audio::kMicTargetBandLoHz;
  const int searchBandHiHz = useVib ? 750 : Audio::kMicTargetBandHiHz;
  int binLo = (searchBandLoHz * Audio::kSamples + (int)currentFs - 1) / (int)currentFs;
  int binHi = (searchBandHiHz * Audio::kSamples) / (int)currentFs;
  if (binLo < Audio::kFftFirstUsableBin) binLo = Audio::kFftFirstUsableBin;
  if (binHi > Audio::kFftLastUsableBin) binHi = Audio::kFftLastUsableBin;
  if (binLo > binHi) return false;

  double maxVal = 0;
  int peakIndex = 0;
  for (int i = binLo; i <= binHi; i++) {
    if (vReal[i] > maxVal) {
      maxVal = vReal[i];
      peakIndex = i;
    }
  }

  // Use a robust local noise floor, not the absolute loudness of the room.
  // The fixed array is sufficient for the 350–650 Hz microphone reference
  // band at 8 kHz / 1024 samples (about 39 FFT bins).
  double peakToNoiseDb = 0.0;
  if (!useVib && peakIndex > 0) {
    int referenceBinLo = (Audio::kMicReferenceBandLoHz * Audio::kSamples
                         + (int)currentFs - 1) / (int)currentFs;
    int referenceBinHi = (Audio::kMicReferenceBandHiHz * Audio::kSamples)
                        / (int)currentFs;
    if (referenceBinLo < Audio::kFftFirstUsableBin) referenceBinLo = Audio::kFftFirstUsableBin;
    if (referenceBinHi > Audio::kFftLastUsableBin) referenceBinHi = Audio::kFftLastUsableBin;

    double noiseBins[64];
    int noiseCount = 0;
    for (int i = referenceBinLo; i <= referenceBinHi && noiseCount < 64; ++i) {
      // Do not let the main lobe of the candidate tone raise its own noise floor.
      if (i < peakIndex - 1 || i > peakIndex + 1) {
        noiseBins[noiseCount++] = vReal[i];
      }
    }
    for (int i = 1; i < noiseCount; ++i) {
      double value = noiseBins[i];
      int j = i - 1;
      while (j >= 0 && noiseBins[j] > value) {
        noiseBins[j + 1] = noiseBins[j];
        --j;
      }
      noiseBins[j + 1] = value;
    }
    if (noiseCount > 0) {
      double noiseFloor = noiseBins[noiseCount / 2];
      if (noiseFloor < 1.0) noiseFloor = 1.0;
      peakToNoiseDb = 20.0 * log10(maxVal / noiseFloor);
    }
  }

  if (sIsCalibrating) {
    if (millis() - sCalibrateStartMs < 1500) {
      if (maxVal > 0) {
        sCalibrateSum += maxVal;
        sCalibrateCount++;
      }
      return true;
    } else {
      sIsCalibrating = false;
      if (sCalibrateCount > 0) {
        double avg = sCalibrateSum / sCalibrateCount;
        if (sAutoIsVib) {
           sAutoThresholdVib = avg * 0.5;
           sAutoThresholdUpperVib = avg * 3.0;
        } else {
           sAutoThresholdMic = avg * 0.5;
           sAutoThresholdUpperMic = avg * 3.0;
        }
      }
      sSensLevel = SensLevel::Auto;
      sForceUiRefresh = true;
    }
  }

  if (maxVal > sensitivityThresholdUpper()) {
    maxVal = 0;
    peakIndex = 0;
  }

  double peakHz = 0.0;
  if (peakIndex >= binLo && peakIndex <= binHi
      && peakIndex > 0 && peakIndex < (Audio::kSamples / 2) - 1) {
    double y0 = vReal[peakIndex - 1];
    double y1 = vReal[peakIndex];
    double y2 = vReal[peakIndex + 1];
    double denom = y0 - 2.0 * y1 + y2;
    if (fabs(denom) > 1e-12) {
      double center = 0.5 * (y0 - y2) / denom;
      if (center < -0.5) center = -0.5;
      if (center > 0.5) center = 0.5;
      peakHz = (peakIndex + center) * (currentFs / Audio::kSamples);
    } else {
      peakHz = (double)peakIndex * (currentFs / Audio::kSamples);
    }
  }

  bool isInstantDetected = (maxVal >= sensitivityThreshold());
  if (!useVib) {
    isInstantDetected = isInstantDetected
                      && peakHz >= Audio::kMicTargetBandLoHz
                      && peakHz <= Audio::kMicTargetBandHiHz
                      && peakToNoiseDb >= sensitivityRequiredSnrDb();


    if (isInstantDetected) {
      if (sHasTrackedPeak && fabs(peakHz - sTrackedPeakHz) <= Audio::kTrackedFrequencyToleranceHz) {
        if (sTrackedPeakFrames < 255) ++sTrackedPeakFrames;
        // This running average accumulates consecutive spectra, following a
        // genuine motor speed change without accepting a sudden tonal noise.
        sTrackedPeakHz = sTrackedPeakHz * 0.7 + peakHz * 0.3;
      } else {
        sHasTrackedPeak = true;
        sTrackedPeakHz = peakHz;
        sTrackedPeakFrames = 1;
      }
      outDetected = sTrackedPeakFrames >= sensitivityRequiredFrames();
      if (outDetected) outHz = sTrackedPeakHz;
    } else {
      sHasTrackedPeak = false;
      sTrackedPeakFrames = 0;
    }
  } else {
    // Preserve the vibration-sensor behaviour: its spectral characteristics
    // are different from the airborne microphone signal.
    if (isInstantDetected) {
      if (!sSignalContinuous) {
        sSignalContinuous = true;
        sSignalContinuousStartMs = millis();
      }
      outDetected = (millis() - sSignalContinuousStartMs >= 300);
    } else {
      sSignalContinuous = false;
      outDetected = false;
    }
    if (outDetected) outHz = peakHz;
  }
  return true;
}

void updatePowerSaveState(bool active) {
  if (active) noteActivity();
  const DisplayPower desired = displayPowerFor(monotonicMs() - sLastActivityMs,
      batteryManager.hasExternalPower(), sCurrentMode == AppMode::ModeStopwatch, sIsCalibrating);
  if (desired != DisplayPower::Off) {
    if (sDisplayBlanked) wakeDisplay();
    applyBrightness(desired == DisplayPower::Bright
        ? PowerSave::kBrightnessActive : PowerSave::kBrightnessDim);
    return;
  }
  if (sDisplayBlanked) return;
  ++sMeasurementGeneration;
  resetDetection();
  setMicrophonePower(false);
  setBmi270Power(false);
  sDisplayBlanked = true;
  applyBrightness(0);
  M5.Display.sleep();
}

static void sleepWhileBlanked() {
  // No low-frequency polling loop: light sleep preserves RAM and stopwatch time.
  // USB detection/charge maintenance still run on the periodic timer wakeups.
  setBmi270Power(false); // Retry a failed power-down on timer wakeups.
  if (digitalRead(PowerSave::kFrontButton) == LOW || digitalRead(PowerSave::kModeButton) == LOW) {
    wakeDisplay();
    sConsumeButtonsUntilRelease = true;
    return;
  }
  const bool gpioReady = gpio_wakeup_enable(PowerSave::kFrontButton, GPIO_INTR_LOW_LEVEL) == ESP_OK
      && gpio_wakeup_enable(PowerSave::kModeButton, GPIO_INTR_LOW_LEVEL) == ESP_OK
      && esp_sleep_enable_gpio_wakeup() == ESP_OK;
  if (!gpioReady) {
    gpio_wakeup_disable(PowerSave::kFrontButton);
    gpio_wakeup_disable(PowerSave::kModeButton);
    delay(30); // Keep button polling functional if wake source setup fails.
    return;
  }
  if (esp_sleep_enable_timer_wakeup(PowerSave::kSleepPollUs) != ESP_OK) {
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
    gpio_wakeup_disable(PowerSave::kFrontButton);
    gpio_wakeup_disable(PowerSave::kModeButton);
    delay(30);
    return;
  }
  const esp_err_t result = esp_light_sleep_start();
  const bool buttonWake = result == ESP_OK && esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_GPIO;
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
  gpio_wakeup_disable(PowerSave::kFrontButton);
  gpio_wakeup_disable(PowerSave::kModeButton);
  if (buttonWake) {
    wakeDisplay();
    sConsumeButtonsUntilRelease = true;
  }
  if (result != ESP_OK) delay(30);
}

// ==========================================
// 디스플레이 렌더링 제어 (색상 조건 교정본)
// ==========================================
void updateDisplay(double currentHz, bool isDetected, bool rpmActive, bool magActive) {
  if (sDisplayBlanked) return; 

  static unsigned long lastUpdate = 0;
  bool isAnyDetected = (sCurrentMode == AppMode::ModePano) ? isDetected : magActive;
  const unsigned long uiPeriod = sCurrentMode == AppMode::ModeStopwatch && sStopwatch.running()
      ? 100 : (isAnyDetected ? PowerSave::kUiRefreshActiveMs : PowerSave::kUiRefreshIdleMs);
  // A stationary screen is redrawn only on state/value changes.
  const bool changing = (sCurrentMode == AppMode::ModeStopwatch && sStopwatch.running())
      || isAnyDetected || sIsCalibrating;

  if (sForceUiRefresh || (changing && millis() - lastUpdate >= uiPeriod)) {
    sForceUiRefresh = false;
    lastUpdate = millis();

    spr.fillScreen(UI::COLOR_BG);
    spr.setTextColor(UI::COLOR_TEXT);
    spr.setFont(&fonts::FreeSans9pt7b);
    spr.setTextDatum(top_left);
    const char* modeLabel = sCurrentMode == AppMode::ModePano ? "PANO"
        : (sCurrentMode == AppMode::ModeMag ? "MAGNETIC" : "STOPWATCH");
    spr.drawString(modeLabel, 5, 7);

    // ── Battery status: stable number plus a charging battery icon ──
    spr.setFont(&fonts::FreeSans9pt7b);
    const String batteryText = String(batteryManager.getBatteryPercent()) + "%";
    const int batteryTextRight = spr.width() - 5;
    const int batteryTextWidth = spr.textWidth(batteryText);
    spr.setTextColor(UI::COLOR_TEXT);
    spr.drawRightString(batteryText, batteryTextRight, 12);

    if (batteryManager.hasExternalPower()) {
      // 16 x 22 px plug-in-battery icon, following the supplied reference.
      const int iconX = batteryTextRight - batteryTextWidth - 20;
      const int iconY = 7; // three pixels lower than the previous battery icon
      const uint16_t iconColor = UI::COLOR_TEXT;
      spr.drawRoundRect(iconX, iconY, 16, 22, 4, iconColor); // battery outline

      // Plug prongs, body, and cable stem.
      spr.fillRect(iconX + 5, iconY + 6, 2, 5, iconColor);
      spr.fillRect(iconX + 10, iconY + 6, 2, 5, iconColor);
      spr.fillRect(iconX + 4, iconY + 10, 9, 2, iconColor);
      spr.fillRoundRect(iconX + 4, iconY + 12, 9, 6, 2, iconColor);
      spr.fillRect(iconX + 7, iconY + 17, 3, 4, iconColor);
    }

    if (sCurrentMode == AppMode::ModePano) {
      spr.setTextColor(UI::COLOR_SENS_LABEL);
      spr.setFont(&fonts::FreeSans9pt7b);
      spr.setTextDatum(top_right);
      spr.drawString(sensitivityLabel(), spr.width() - 5, 30);
      if (lastHz > 0) {
        char hzBuf[24];
        snprintf(hzBuf, sizeof(hzBuf), "%.1f Hz", lastHz);
        spr.setTextDatum(top_left);
        spr.setTextColor(UI::COLOR_TEXT);
        spr.drawString(hzBuf, 5, 30);
      }
    }

    spr.setTextDatum(top_center);
    
    if (sSensorError) {
      spr.setTextColor(UI::COLOR_BAT_LOW);
      spr.setFont(&fonts::FreeSans12pt7b);
      spr.drawString("SENSOR ERROR", spr.width() / 2, 65);
    }
    else if (sIsCalibrating) {
      if (sCurrentMode == AppMode::ModeMag) {
        spr.setTextColor(UI::COLOR_MAG_VALUE);
      } else {
        spr.setTextColor(UI::COLOR_SENS_LABEL);
      }
      spr.setFont(&fonts::FreeSans12pt7b);
      spr.drawString("CALIBRATING", spr.width() / 2, 65);
    } 
    else if (sCurrentMode == AppMode::ModeStopwatch) {
      const uint64_t elapsed = sStopwatch.elapsedMs(monotonicMs());
      const unsigned long hours = (unsigned long)(elapsed / 3600000ULL);
      char timeBuf[32];
      snprintf(timeBuf, sizeof(timeBuf), "%02lu:%02u:%02u.%02u", hours,
          (unsigned)((elapsed / 60000) % 60), (unsigned)((elapsed / 1000) % 60),
          (unsigned)((elapsed / 10) % 100));
      spr.setFont(&fonts::FreeSans18pt7b);
      spr.setTextColor(UI::COLOR_MAG_VALUE);
      spr.drawString(timeBuf, spr.width() / 2, 43);
      spr.setFont(&fonts::FreeSans9pt7b);
      spr.setTextColor(UI::COLOR_TEXT);
      spr.drawString(sStopwatch.running() ? "RUNNING" : "STOPPED", spr.width() / 2, 89);
      spr.drawString("Hold front: RESET", spr.width() / 2, 112);
    }
    else if (sCurrentMode == AppMode::ModePano) {
      if (rpmActive) {
        long rpm = (long)(currentHz * 60);
        spr.setFont(&fonts::FreeSansBold18pt7b);
        spr.setTextColor(UI::COLOR_RPM_VALUE);
        spr.drawString(formatNumber(rpm), spr.width() / 2, 52);

        spr.setFont(&fonts::FreeSans12pt7b);
        spr.drawString("RPM", spr.width() / 2, 95);
      } else {
        spr.setTextColor(UI::COLOR_READY);
        spr.setFont(&fonts::FreeSans12pt7b);
        spr.drawString("READY", spr.width() / 2, 65);
      }
    } 
    else if (sCurrentMode == AppMode::ModeMag) {
      spr.setFont(&fonts::FreeSansBold18pt7b);
      spr.setTextColor(UI::COLOR_MAG_VALUE);
      
      char magBuf[32];
      if (sCurrentPole == 'S' || sCurrentPole == 'N') {
        snprintf(magBuf, sizeof(magBuf), "%c %.1f G", sCurrentPole, sCurrentGauss);
      } else {
        snprintf(magBuf, sizeof(magBuf), "%.1f G", sCurrentGauss);
      }
      spr.drawString(magBuf, spr.width() / 2, 52);

      spr.setFont(&fonts::FreeSans9pt7b);
      spr.setTextColor(UI::COLOR_TEXT);
      spr.drawString("Magnetic Force", spr.width() / 2, 95);
      
      int barWidth = map((long)sCurrentGauss, 0, 1000, 0, spr.width() - 20);
      if (barWidth > spr.width() - 20) barWidth = spr.width() - 20;
      if (barWidth < 0) barWidth = 0;
      
      spr.drawRect(10, 115, spr.width() - 20, 15, UI::COLOR_TEXT);
      spr.fillRect(10, 115, barWidth, 15, UI::COLOR_MAG_VALUE);
    }

    spr.pushSprite(0, 0);
  }
}

// ==========================================
// Setup & Loop
// ==========================================
void setup() {
  auto cfg = M5.config();
  // No feature uses external 5V: do not leave its boost converter enabled.
  cfg.output_power = false;
  cfg.internal_spk = false;
  cfg.internal_rtc = false;
  M5.begin(cfg);
  M5.Power.setExtOutput(false);
  M5.Speaker.end();
  M5.Power.setLed(0);

  // Explicitly keep the StickS3 charger enabled while the firmware is active.
  // Charge current and voltage are board-hardware controlled, so do not apply
  // values intended for a different PMIC.
  M5.Power.setBatteryCharge(true);

  if (Diagnostics::kPowerLogEnabled) {
    Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
    Serial.setTxTimeoutMs(0);
#endif
  }

  delay(200);
  
#if defined(ESP32)
  WiFi.mode(WIFI_OFF);         
  disableUnusedBluetooth();    
#endif

  setCpuFrequencyMhz(PowerSave::kCpuFreqActiveMhz); 
  M5.Display.setRotation(1);
  M5.Display.setBrightness(PowerSave::kBrightnessActive);
  if (!spr.createSprite(M5.Display.width(), M5.Display.height())) {
    M5.Display.drawString("Display allocation failed", 0, 0);
  }

  batteryManager.begin(); // 정돈된 배터리 매니저 시작

  // Keep board-supplied I2S pin/codec settings; only change the sample rate.
  auto mic_cfg = M5.Mic.config();
  mic_cfg.sample_rate = Audio::kSamplingFrequency;
  M5.Mic.config(mic_cfg);
  M5.Mic.end();
  sMicEnabled = false;

  // Use M5Unified's internal I2C owner rather than reinitialising the same bus
  // through Wire/Wire1 (the microphone codec and PMIC also share it).
  uint8_t chipId = 0;
  sBmiPresent = M5.In_I2C.readRegister(0x68, 0x00, &chipId, 1, 400000) && chipId == 0x24;
  if (sBmiPresent) {
    const bool configured = M5.In_I2C.writeRegister8(0x68, 0x40, 0xAC, 400000);
    sBmiPresent = configured;
  }
  setBmi270Power(false);

  pinMode(kMagPin, INPUT);

  prepareWindows();
  noteActivity();
  sForceUiRefresh = true;
}

// ==========================================
// Main loop: responsive buttons, capped sensor work, independent screen updates
// ==========================================
void loop() {
  const unsigned long now = millis();
  const bool stopwatchStopped = sCurrentMode == AppMode::ModeStopwatch && !sStopwatch.running();
  const unsigned long buttonPollMs = stopwatchStopped ? PowerSave::kStoppedButtonPollMs : 10;
  if (now - sLastButtonCheckMs >= buttonPollMs) {
    sLastButtonCheckMs = now;
    M5.update();
    handleButtons();
  }
  batteryManager.update();
  logPowerDiagnostics();
  static bool previousUsb = false;
  const bool usb = batteryManager.hasExternalPower();
  if (usb != previousUsb) {
    previousUsb = usb;
    noteActivity();
  }
  updatePowerSaveState(false);
  if (sDisplayBlanked) {
    sleepWhileBlanked();
    return;
  }

  // Never increase FFT frequency during calibration; stopwatch needs no sensors.
  if (sCurrentMode != AppMode::ModeStopwatch && now - sLastFuncExecutionMs >= Audio::kAnalysisIntervalMs) {
    sLastFuncExecutionMs = now;
    const uint32_t generation = sMeasurementGeneration;
    const bool oldDetected = sDetected;
    const bool oldMagActive = sMagActive;
    const bool oldError = sSensorError;
    const double oldGauss = sCurrentGauss;
    const char oldPole = sCurrentPole;
    double hz = 0;
    bool detected = false;
    bool magActive = false;
    bool success = true;
    if (sCurrentMode == AppMode::ModePano) {
      success = processAudioAndFFT(hz, detected);
    } else {
      processMagnetic(magActive);
    }
    if (generation == sMeasurementGeneration) {
      sSensorError = !success;
      if (!success) resetDetection();
      sCurrentHz = hz;
      sDetected = detected;
      sRpmActive = detected && hz > Audio::kMinValidRpmHz;
      sMagActive = magActive;
      if (detected) lastHz = hz;
      if (oldDetected != detected || oldMagActive != magActive || oldError != sSensorError
          || fabs(oldGauss - sCurrentGauss) >= 0.1 || oldPole != sCurrentPole) sForceUiRefresh = true;
      // A failed acquisition must not leave calibration keeping the device awake.
      if (!success && sIsCalibrating && millis() - sCalibrateStartMs >= 2000) {
        sIsCalibrating = false;
        sForceUiRefresh = true;
      }
      updatePowerSaveState(sRpmActive || sMagActive);
    }
  }
  const bool stoppedNow = sCurrentMode == AppMode::ModeStopwatch && !sStopwatch.running();
  if (!stoppedNow || sForceUiRefresh)
    updateDisplay(sCurrentHz, sDetected, sRpmActive, sMagActive);
  // Re-evaluate after button handling: starting the watch restores fast polling.
  delay(stoppedNow ? PowerSave::kStoppedLoopDelayMs : 10);
}
