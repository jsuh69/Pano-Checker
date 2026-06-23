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
namespace Audio {
constexpr double kSamplingFrequency = 16000;
constexpr uint16_t kSamples = 2048;
constexpr int kFftBandBinLo = (150 * (int)kSamples + 15999) / 16000;
constexpr int kFftBandBinHi = (750 * (int)kSamples) / 16000;
constexpr double kMinValidRpmHz = 50.0;
constexpr double kFftPeakRejectAbove = 4000000.0;
} // namespace Audio

namespace PowerSave {
constexpr uint8_t kBrightnessFixed = 35; 
constexpr unsigned long kDisplayBlankAfterReadyMs = 20000; 
constexpr unsigned long kUiRefreshActiveMs = 500;
constexpr unsigned long kUiRefreshIdleMs = 280;
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
static bool sShouldGoToSleep = false; 

enum class SensLevel : uint8_t { Low = 0, Mid, High, Vib, Auto };
static SensLevel sSensLevel = SensLevel::High;
static bool sAutoIsVib = false;

static double sAutoThresholdMic = 400000.0;
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
  bool isInitialized = false;
  uint32_t lastCheckMs = 0;
  static constexpr uint32_t kCheckIntervalMs = 2000;

public:
  bool externalPowerOrCharging() {
    // 💡 [핵심 교정] Plus2 전용 공식 M5Unified 함수를 사용합니다.
    // getType()이 아닌, isCharging() 함수가 내부적으로 GPIO38 전압 변동을 읽어 판정합니다.
    // 만약 라이브러리 상에서 오차가 있을 것에 대비해 두 가지 크로스 체크를 수행합니다.
    
    auto chg_stat = M5.Power.isCharging();
    int mv = M5.Power.getBatteryVoltage(); // 현재 배터리 밀리볼트(mV) 취득

    // 1) M5Unified 드라이버 레벨에서 충전 상태로 인지하거나
    if (chg_stat == m5::Power_Class::is_charging) {
      return true;
    }

    // 2) [하드웨어 직결 보완] USB가 꽂히면 충전 IC(TP4057)의 전류 밀어내기로 인해 
    // 배터리 잔량이 76% 수준이더라도 전압 계측치가 일시적으로 평소 방전 전압보다 상승합니다.
    // 일반적으로 USB 구동 시 측정 전압이 대략 3,850mV를 초과하여 유지됩니다.
    if (mv > 3850) {
      return true;
    }

    return false;
  }

  void begin() {
    isInitialized = false;
    stableBatPct = 0;
    lastCheckMs = 0;
  }

  void update() {
    uint32_t currentMs = millis();

    if (!isInitialized) {
      uiCharging = externalPowerOrCharging();
      stableBatPct = M5.Power.getBatteryLevel(); 
      if (stableBatPct > 100) stableBatPct = 100;
      isInitialized = true;
      lastCheckMs = currentMs;
      sForceUiRefresh = true;
      return;
    }

    if (currentMs - lastCheckMs >= kCheckIntervalMs) {
      lastCheckMs = currentMs;
      
      bool newCharging = externalPowerOrCharging();
      if (newCharging != uiCharging) {
        uiCharging = newCharging;
        sForceUiRefresh = true;
      }

      int rawPct = M5.Power.getBatteryLevel();
      if (rawPct >= 0 && rawPct <= 100 && rawPct != stableBatPct) {
        stableBatPct = rawPct;
        sForceUiRefresh = true; 
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

static void cycleSensitivity() {
  switch (sSensLevel) {
    case SensLevel::Low:     sSensLevel = SensLevel::Mid; break;
    case SensLevel::Mid:     sSensLevel = SensLevel::High; break;
    case SensLevel::High:    sSensLevel = SensLevel::Vib; break;
    case SensLevel::Vib:     sSensLevel = SensLevel::Low; break;
    case SensLevel::Auto:    sSensLevel = SensLevel::Low; break;
  }
}

static double sensitivityThreshold() {
  if (sSensLevel == SensLevel::Auto) {
    return sAutoIsVib ? sAutoThresholdVib : sAutoThresholdMic;
  }
  switch (sSensLevel) {
    case SensLevel::Low: return 900000.0;
    case SensLevel::Mid: return 400000.0;
    case SensLevel::High: return 50000.0;
    case SensLevel::Vib: return 5000.0;
  }
  return 900000.0;
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
      setBmi270Power(true);
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
      startCalibration();
    } else {
      setBmi270Power(true);
    }
    sReadySinceMs = millis();
    sForceUiRefresh = true;
  }

  static bool sHasCalibratedThisPress = false;
  static bool sWokeUpThisPressB = false;

  if (M5.BtnB.wasPressed()) {
    sHasCalibratedThisPress = false;
    sWokeUpThisPressB = false;
    if (sCurrentMode == AppMode::ModeMag) {
      sHasCalibratedThisPress = true;
      startCalibration();
      sReadySinceMs = millis();
      sForceUiRefresh = true;
    }
  }

  if (!sWokeUpThisPressB) {
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

    for (int i = 0; i < 512; i++) {
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
          rawBuffer[actualSamples++] = (int16_t)mag;
          vibMean += mag;
        }

        if (i > 0 && i % 128 == 0) {
            M5.update();
            handleButtons();
            next_sample = micros(); 
        }
    }
    
    unsigned long end_time = micros();
    if (actualSamples > 0) {
        vibMean /= actualSamples;
        sVibActualFs = (actualSamples * 1000000.0) / (double)(end_time - start_time);
    }
    
    for (int i = 0; i < Audio::kSamples; i++) {
       if (i < actualSamples) {
           double val = (double)rawBuffer[i] - vibMean;
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
  int binLo = (150 * Audio::kSamples + (int)currentFs - 1) / (int)currentFs;
  int binHi = (750 * Audio::kSamples) / (int)currentFs;

  double maxVal = 0;
  int peakIndex = 0;
  for (int i = binLo; i <= binHi; i++) {
    if (vReal[i] > maxVal) {
      maxVal = vReal[i];
      peakIndex = i;
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

  bool isInstantDetected = (maxVal >= sensitivityThreshold());
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

  if (outDetected && peakIndex >= binLo && peakIndex <= binHi) {
    double y0 = vReal[peakIndex - 1];
    double y1 = vReal[peakIndex];
    double y2 = vReal[peakIndex + 1];
    double denom = y0 - 2.0 * y1 + y2;
    if (fabs(denom) > 1e-12) {
      double center = 0.5 * (y0 - y2) / denom;
      outHz = (peakIndex + center) * (currentFs / Audio::kSamples);
    } else {
      outHz = (double)peakIndex * (currentFs / Audio::kSamples);
    }
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

    // ── 배터리 잔량 표출 제어 블록 ──
    uint16_t batColor;
    if (batteryManager.isCharging()) {
      // 1) 충전 중인 경우: 잔량 수치변화 추이를 파악하도록 수치는 표기하되 '노란색' 고정
      batColor = UI::COLOR_BAT_CHARGING;
    } else {
      // 2) 충전기가 빠진 경우: 잔량에 따라 안전(초록색) / 부족(빨간색) 분기 적용
      batColor = (batteryManager.getBatteryPercent() > 20) ? UI::COLOR_BAT_GOOD : UI::COLOR_BAT_LOW;
    }

    spr.setTextColor(batColor);
    spr.setFont(&fonts::FreeSans9pt7b);
    spr.drawRightString(String(batteryManager.getBatteryPercent()) + "%", spr.width() - 5, 12);

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

  // ❌ [기존 오동작 유발 코드 전면 삭제]
  // M5.Power.setChargeCurrent(600); -> 삭제
  // M5.Power.setChargeVoltage(4200); -> 삭제
  // M5.Power.setBatteryCharge(true); -> 삭제
  // #if defined(LGFX_M5STACK_M5STIC_C) || true 블록 전체 -> 삭제

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
  
  pinMode(kMagPin, INPUT);

  sForceUiRefresh = true;
}

// ==========================================
// 메인 루프 (버튼 50ms / 기능 300ms 완전 분리)
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

  // [파트 3] 깨어있는 상태에서의 센서링 및 디스플레이 제어 (300ms 주기) - 기존 코드 동일
  unsigned long executionInterval = 300; 
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