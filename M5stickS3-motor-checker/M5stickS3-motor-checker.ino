#include <M5Unified.h>
#include <arduinoFFT.h>
#include <cmath>
#include <Wire.h>
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
constexpr char kVersion[] = "0.8.0";
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
constexpr uint8_t kBrightnessFixed = 25;
constexpr unsigned long kDisplayBlankAfterReadyMs = 20000; 
constexpr unsigned long kUiRefreshActiveMs = 500;
constexpr unsigned long kUiRefreshIdleMs = 700;
constexpr uint8_t kCpuFreqActiveMhz = 80; 
} // namespace PowerSave

namespace BatConfig {
constexpr int16_t kVbusPresentMinMv = 4200;
constexpr int32_t kVoltMaxUnplugged = 4150; 
constexpr int32_t kVoltMaxCharging  = 4230; 
constexpr int32_t kVoltMin          = 3400; 
} // namespace BatConfig

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
double vReal[Audio::kSamples];
double vImag[Audio::kSamples];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, Audio::kSamples, Audio::kSamplingFrequency);

LGFX_Sprite spr(&M5.Display);

double lastHz = 0;
bool sForceUiRefresh = false;

enum class AppMode : uint8_t { ModePano, ModeMag };
static AppMode sCurrentMode = AppMode::ModePano;

constexpr int kMagPin = 1;
static double sMagBaseline = -1; 
static double sCurrentGauss = 0; 
static char sCurrentPole = ' ';  

static bool sDisplayBlanked = false;
static unsigned long sReadySinceMs = 0;
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

  static constexpr uint32_t kCheckIntervalMs = 1000;
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
      if (sampledCharging != pendingCharging) {
        pendingCharging = sampledCharging;
        chargingConfirmations = 1;
      } else if (chargingConfirmations < 255) {
        ++chargingConfirmations;
      }

      // The PMIC charging-status pin can briefly change while USB power is
      // applied.  Only update the bolt after three matching samples.
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
  bool isCharging() const { return uiCharging; } 
};

BatteryManager batteryManager; 

// ==========================================
// Helper Functions
// ==========================================
void setBmi270Power(bool enable) {
  Wire1.beginTransmission(0x68);
  Wire1.write(0x7D); 
  Wire1.write(enable ? 0x04 : 0x00); 
  Wire1.endTransmission();
}

static void applyFixedBrightness() {
  M5.Display.setBrightness(PowerSave::kBrightnessFixed);
}

static void setMicrophonePower(bool enable) {
  if (enable == sMicEnabled) return;

  if (enable) {
    M5.Mic.begin();
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
  setMicrophonePower(!useVibrationSensor);
}

static void cycleSensitivity() {
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
  if (sSensLevel == SensLevel::Auto) {
    return sAutoIsVib ? sAutoThresholdVib : sAutoThresholdMic;
  }
  switch (sSensLevel) {
    // Microphone sensitivity is determined primarily by peak-to-noise ratio,
    // not a fixed room-dependent FFT magnitude.
    case SensLevel::Low:
    case SensLevel::Mid:
    case SensLevel::High: return Audio::kMicMinimumPeakMagnitude;
    case SensLevel::Vib: return 5000.0;
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
  if (sSensLevel == SensLevel::Auto) return "AUTO";
  switch (sSensLevel) {
    case SensLevel::Low: return "LOW";
    case SensLevel::Mid: return "MID";
    case SensLevel::High: return "HIGH";
    case SensLevel::Vib: return "VIB";
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
  sIsCalibrating = true;
  sCalibrateStartMs = millis();
  sCalibrateSum = 0;
  sCalibrateCount = 0;
  
  if (sCurrentMode == AppMode::ModePano) {
    sAutoIsVib = (sSensLevel == SensLevel::Vib || (sSensLevel == SensLevel::Auto && sAutoIsVib));
  }
}

void handleButtons() {
  if (sDisplayBlanked && (M5.BtnA.wasPressed() || M5.BtnB.wasPressed())) {
    // ⚡ 깨어날 때 가장 먼저 CPU 속도부터 정상화합니다.
    setCpuFrequencyMhz(PowerSave::kCpuFreqActiveMhz); 
    
    sDisplayBlanked = false;
    
    M5.Display.wakeup(); 
    applyFixedBrightness();
    
    if (sCurrentMode == AppMode::ModePano) {
      syncPanoSensorPower();
    } else {
      setMicrophonePower(false);
    }
    sReadySinceMs = millis();
    sForceUiRefresh = true;
    return;
  }

  // 기존 BtnA, BtnB 제어 로직 유지
  if (M5.BtnA.wasPressed()) {
    sCurrentMode = (sCurrentMode == AppMode::ModePano) ? AppMode::ModeMag : AppMode::ModePano;
    if (sCurrentMode == AppMode::ModeMag) {
      setBmi270Power(false); 
      setMicrophonePower(false);
      startCalibration();
    } else {
      syncPanoSensorPower();
    }
    sReadySinceMs = millis();
    sForceUiRefresh = true;
  }

  static bool sHasCalibratedThisPress = false;

  if (M5.BtnB.wasPressed()) {
    sHasCalibratedThisPress = false;
    if (sCurrentMode == AppMode::ModeMag) {
      sHasCalibratedThisPress = true;
      startCalibration();
      sReadySinceMs = millis();
      sForceUiRefresh = true;
    }
  }

  if (sCurrentMode == AppMode::ModePano && M5.BtnB.pressedFor(1000) && !sHasCalibratedThisPress) {
    sHasCalibratedThisPress = true;
    startCalibration();
    sReadySinceMs = millis();
    sForceUiRefresh = true;
  }

  if (M5.BtnB.wasReleased()) {
    if (!sHasCalibratedThisPress) {
      if (sCurrentMode == AppMode::ModePano) {
        cycleSensitivity();
      }
      sReadySinceMs = millis();
      sForceUiRefresh = true;
    }
  }
}

static double sLastInstGauss = 0.0;

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

  bool useVib = (sSensLevel == SensLevel::Vib || (sSensLevel == SensLevel::Auto && sAutoIsVib) || (sIsCalibrating && sAutoIsVib));

  if (useVib) {
    unsigned long start_time = micros();
    unsigned long next_sample = start_time;
    double vibMean = 0;
    int actualSamples = 0;

    for (int i = 0; i < Audio::kVibSamples; i++) {
        while (micros() - next_sample < 625) { }
        next_sample += 625;
        
        Wire1.beginTransmission(0x68);
        Wire1.write(0x0C);
        if (Wire1.endTransmission(false) != 0) {
          break; 
        }
        if (Wire1.requestFrom(0x68, 6) == 6) {
          int16_t ax = Wire1.read() | (Wire1.read() << 8);
          int16_t ay = Wire1.read() | (Wire1.read() << 8);
          int16_t az = Wire1.read() | (Wire1.read() << 8);
          double mag = sqrt((double)ax * ax + (double)ay * ay + (double)az * az);
          // A three-axis magnitude can exceed int16_t; retain its full value.
          vReal[actualSamples++] = mag;
          vibMean += mag;
        }

        if (i > 0 && i % 128 == 0) {
            M5.update();
            handleButtons();
            next_sample = micros(); 
        }
    }
    
    unsigned long end_time = micros();
    const unsigned long elapsedUs = end_time - start_time;
    if (actualSamples < 2 || elapsedUs == 0) {
      return false;
    }
    vibMean /= actualSamples;
    sVibActualFs = (actualSamples * 1000000.0) / (double)elapsedUs;
    
    for (int i = 0; i < Audio::kSamples; i++) {
       if (i < actualSamples) {
           double val = vReal[i] - vibMean;
           double ratio = (double)i / (actualSamples - 1);
           double hamming = 0.54 - 0.46 * cos(2.0 * M_PI * ratio);
           vReal[i] = val * hamming;
       } else {
           vReal[i] = 0; 
       }
       vImag[i] = 0;
    }
  } else {
    if (!M5.Mic.record(rawBuffer, Audio::kSamples, Audio::kSamplingFrequency)) {
      return false;
    }
    double mean = 0;
    for (int i = 0; i < Audio::kSamples; i++) mean += rawBuffer[i];
    mean /= Audio::kSamples;
    for (int i = 0; i < Audio::kSamples; i++) {
      vReal[i] = (double)rawBuffer[i] - mean;
      vImag[i] = 0;
    }
    FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
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
  // band at 16 kHz / 2048 samples (about 39 FFT bins).
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

    static bool sHasTrackedPeak = false;
    static double sTrackedPeakHz = 0.0;
    static uint8_t sTrackedPeakFrames = 0;

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
    static unsigned long sSignalContinuousStartMs = 0;
    static bool sSignalContinuous = false;
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
  if (active) {
    sReadySinceMs = 0;
  } else {
    if (sReadySinceMs == 0) sReadySinceMs = millis();
      
    if (!sDisplayBlanked && (millis() - sReadySinceMs >= PowerSave::kDisplayBlankAfterReadyMs)) {
      sDisplayBlanked = true;
      
      // 1. 센서 최소 전력 모드 유도
      setBmi270Power(false); 
      setMicrophonePower(false);

      // 2. 디스플레이를 완벽하게 물리적 슬립으로 전환 (소모 전류 극소화)
      M5.Display.setBrightness(0);
      M5.Display.sleep(); 

      // ⚡ [핵심 치트키] CPU 클록을 10MHz로 떨어뜨려 내부 소모 전력을 제로에 가깝게 만듭니다.
      // 이렇게 하면 VBUS 전압이 4.9V 이상으로 복구되어 충전 IC가 최고 속도로 배터리를 채웁니다.
      setCpuFrequencyMhz(10); 

      Serial.println("💤 초절전 하드웨어 충전 모드 진입 (CPU 10MHz 제한)");
    }
  }
}

// ==========================================
// 디스플레이 렌더링 제어 (색상 조건 교정본)
// ==========================================
void updateDisplay(double currentHz, bool isDetected, bool rpmActive, bool magActive) {
  if (sDisplayBlanked) return; 

  static unsigned long lastUpdate = 0;
  bool isAnyDetected = (sCurrentMode == AppMode::ModePano) ? isDetected : magActive;
  const unsigned long uiPeriod = isAnyDetected ? PowerSave::kUiRefreshActiveMs : PowerSave::kUiRefreshIdleMs;

  if (sForceUiRefresh || millis() - lastUpdate >= uiPeriod) {
    sForceUiRefresh = false;
    lastUpdate = millis();

    spr.fillScreen(UI::COLOR_BG);
    spr.setFont(&fonts::FreeSans12pt7b);
    spr.setTextColor(UI::COLOR_TEXT);
    spr.setCursor(5, 12);

    if (sCurrentMode == AppMode::ModePano) {
      double displayHz = isDetected ? currentHz : lastHz;
      if (displayHz > 0) {
        spr.printf("%.1f Hz", displayHz);
      }
    }

    // ── Battery status: stable number plus a charging battery icon ──
    spr.setFont(&fonts::FreeSans9pt7b);
    const String batteryText = String(batteryManager.getBatteryPercent()) + "%";
    const int batteryTextRight = spr.width() - 5;
    const int batteryTextWidth = spr.textWidth(batteryText);
    spr.setTextColor(UI::COLOR_TEXT);
    spr.drawRightString(batteryText, batteryTextRight, 12);

    if (batteryManager.isCharging()) {
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
    }

    spr.setTextDatum(top_center);
    
    if (sIsCalibrating) {
      if (sCurrentMode == AppMode::ModeMag) {
        spr.setTextColor(UI::COLOR_MAG_VALUE);
      } else {
        spr.setTextColor(UI::COLOR_SENS_LABEL);
      }
      spr.setFont(&fonts::FreeSans12pt7b);
      spr.drawString("CALIBRATING", spr.width() / 2, 65);
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
  M5.begin(cfg);

  // Explicitly keep the StickS3 charger enabled while the firmware is active.
  // Charge current and voltage are board-hardware controlled, so do not apply
  // values intended for a different PMIC.
  M5.Power.setBatteryCharge(true);

  delay(200);
  
#if defined(ESP32)
  WiFi.mode(WIFI_OFF);         
  disableUnusedBluetooth();    
#endif

  setCpuFrequencyMhz(PowerSave::kCpuFreqActiveMhz); 
  M5.Display.setRotation(1);
  applyFixedBrightness();

  spr.createSprite(M5.Display.width(), M5.Display.height());

  auto pin_num_sda = M5.getPin(m5::pin_name_t::in_i2c_sda);
  auto pin_num_scl = M5.getPin(m5::pin_name_t::in_i2c_scl);
  Wire.end();
  Wire.begin(pin_num_sda, pin_num_scl, 100000U);

  batteryManager.begin(); // 정돈된 배터리 매니저 시작

  // 이하 마이크 및 IMU(Wire1) 설정 코드는 그대로 유지...
  auto mic_cfg = M5.Mic.config();
  mic_cfg.sample_rate = Audio::kSamplingFrequency;
  mic_cfg.pin_data_in = 16;
  mic_cfg.pin_bck = 17;
  mic_cfg.pin_ws = 15;
  M5.Mic.config(mic_cfg);
  M5.Mic.begin();
  sMicEnabled = true;

  Wire1.begin(47, 48, 400000); 
  Wire1.setClock(400000);      
  Wire1.setTimeOut(10);        
  Wire1.beginTransmission(0x68);
  Wire1.write(0x7D); 
  Wire1.write(0x04); 
  Wire1.endTransmission();
  delay(10); 
  Wire1.beginTransmission(0x68);
  Wire1.write(0x40); 
  Wire1.write(0xAC); 
  Wire1.endTransmission();
  // Default HIGH mode only uses the microphone; leave the IMU powered down.
  syncPanoSensorPower();
  
  pinMode(kMagPin, INPUT);

  sForceUiRefresh = true;
}

// ==========================================
// 메인 루프 (버튼 50ms / 기능 500ms 완전 분리)
// ==========================================
void loop() {
  unsigned long currentMs = millis();

  // [파트 1] 버튼 체크 및 배터리 업데이트
  // 💡 CPU가 10MHz일 때는 millis() 속도 체감이 다르므로 슬립 시에는 매 루프마다 체크하도록 오픈합니다.
  if (sDisplayBlanked || (currentMs - sLastButtonCheckMs >= 50)) {
    sLastButtonCheckMs = currentMs;

    M5.update();
    handleButtons();         
    batteryManager.update(); 
  }

  // [파트 2] 슬립 모드(암전 상태)에서의 대기
  if (sDisplayBlanked) {
    // CPU 속도가 10MHz로 극도로 낮아진 상태이므로, 
    // 여기서는 delay를 길게 주지 않고 아주 미세하게만 쉬어주어야 버튼 반응성이 유지됩니다.
    delay(30); 
    return;
  }

  // [파트 3] 깨어있는 상태에서의 센서링 및 디스플레이 제어 (초당 2회)
  unsigned long executionInterval = Audio::kAnalysisIntervalMs;
  if (sIsCalibrating) executionInterval = 1;   

  if (currentMs - sLastFuncExecutionMs >= executionInterval) {
    sLastFuncExecutionMs = currentMs; 

    double currentHz = 0;
    bool isDetected = false;
    bool rpmActive = false;
    bool magActive = false;

    if (sCurrentMode == AppMode::ModePano) {
      if (processAudioAndFFT(currentHz, isDetected)) {
        if (isDetected) lastHz = currentHz;
        rpmActive = isDetected && currentHz > Audio::kMinValidRpmHz;
      }
    } else if (sCurrentMode == AppMode::ModeMag) {
      processMagnetic(magActive);
    }

    updatePowerSaveState(rpmActive || magActive);
    updateDisplay(currentHz, isDetected, rpmActive, magActive);
  }

  delay(50); 
}
