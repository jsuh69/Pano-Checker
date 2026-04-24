#include <M5Unified.h>
#include <arduinoFFT.h>
#include <cmath>
#if defined(ESP32)
#include <WiFi.h>
#include <esp_bt.h>
#include <soc/soc_caps.h>
#endif

// ==========================================
// Constants & Settings
// ==========================================
namespace Audio {
    constexpr double kSamplingFrequency = 16000;
    constexpr uint16_t kSamples = 2048;
    /** Fs=16k, N=2048 → 빈 k ≈ k·Fs/N Hz. 150~750Hz → 빈 20..96. */
    constexpr int kFftBandBinLo = (150 * (int)kSamples + 15999) / 16000;
    constexpr int kFftBandBinHi = (750 * (int)kSamples) / 16000;
    constexpr double kMinValidRpmHz = 50.0;
    /** FFT 피크가 이 값을 넘으면 포화·폭음 등으로 보고 검출·Hz/RPM 출력 안 함. */
    constexpr double kFftPeakRejectAbove = 4000000.0;
}

namespace PowerSave {
    /** BtnA 순환: 1=가장 어두움 … 4=밝음. 디폴트=가장 어두움(인덱스 0). */
    constexpr uint8_t kBrightnessLevels[4] = {35, 105, 175, 235};
    constexpr uint8_t kBrightnessDefaultIndex = 0;
    /** READY 유지 시 이 시간 후 백라이트 0(RPM 표시·버튼 시 다시 켜짐). */
    constexpr unsigned long kDisplayBlankAfterReadyMs = 20000;
    /** 신호 검출 중 Hz/RPM 표시 갱신(FFT는 마이크 버퍼마다 수행, 여기서는 LCD만 스로틀). */
    constexpr unsigned long kUiRefreshActiveMs = 500;
    constexpr unsigned long kUiRefreshIdleMs = 280;
    /** 240=최대 성능, 160/80=절전. FFT 여유 없으면 200 또는 240. */
    constexpr uint8_t kCpuFreqMhz = 160;
}

namespace BatConfig {
    /** 충전 경로를 끈 뒤 정밀 %를 읽는 주기. (사용자 요청에 따라 2분으로 조정) */
    constexpr unsigned long kPrecisionSamplePeriodMs = 2UL * 60UL * 1000UL;
    /** 10초는 배터리 소모가 크므로 즉각적인 IR drop만 피하도록 3초로 단축. */
    constexpr unsigned long kOpenCircuitSettleMs = 3000;
    constexpr unsigned long kBootOpenCircuitSettleMs = kOpenCircuitSettleMs;
    constexpr unsigned long kUnpluggedSamplePeriodMs = 10000;
    constexpr unsigned long kPostUnplugRelaxMs = 5000;
    constexpr int16_t kVbusPresentMinMv = 4200;
}

namespace UI {
    constexpr uint16_t COLOR_BG           = 0x0000;
    constexpr uint16_t COLOR_TEXT         = 0xFFFF;
    constexpr uint16_t COLOR_BAT_CHARGING = 0xFFE0;
    constexpr uint16_t COLOR_BAT_GOOD     = 0x07E0;
    constexpr uint16_t COLOR_BAT_LOW      = 0xF800;
    constexpr uint16_t COLOR_SENS_LABEL   = 0xFD20; // kSensOrangeRgb565
    constexpr uint16_t COLOR_RPM_VALUE    = 0xFDA0;
    constexpr uint16_t COLOR_READY        = 0x7BEF;
}

// ==========================================
// Global State & Objects
// ==========================================
double vReal[Audio::kSamples];
double vImag[Audio::kSamples];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, Audio::kSamples, Audio::kSamplingFrequency);

LGFX_Sprite spr(&M5.Display);

double lastHz = 0;
bool sForceUiRefresh = false;

// Display & Power state
static uint8_t sBrightnessStep = PowerSave::kBrightnessDefaultIndex;
static bool sDisplayBlanked = false;
static unsigned long sReadySinceMs = 0;

// Sensitivity state
enum class SensLevel : uint8_t { Low = 0, Mid, High };
static SensLevel sSensLevel = SensLevel::High;
static const double kSensThresholds[] = {900000.0, 500000.0, 100000.0};
static const char* const kSensLabels[] = {"LOW", "MID", "HIGH"};


// ==========================================
// Classes
// ==========================================
class BatteryManager {
private:
    enum class Phase : uint8_t { Idle, OpenCircuitRest };
    Phase gaugePhase = Phase::Idle;
    int stableBatPct = 0;
    bool uiCharging = false;
    unsigned long openCircuitStartMs = 0;
    unsigned long lastChargingPrecisionMs = 0;
    unsigned long lastUnpluggedSampleMs = 0;
    bool hadExternalPower = false;
    unsigned long postUnplugSampleDueMs = 0;

    int clampBatteryPercent(int v) {
        if (v < 0) return 0;
        if (v > 100) return 100;
        return v;
    }

    bool externalPowerOrCharging() {
        typedef decltype(M5.Power.isCharging()) chg_t;
        if (M5.Power.isCharging() == chg_t::is_charging) return true;
        const int16_t vbusMv = M5.Power.getVBUSVoltage();
        if (vbusMv < 0) return false;
        return vbusMv >= BatConfig::kVbusPresentMinMv;
    }

public:
    void begin() {
        if (externalPowerOrCharging()) {
            M5.Power.setBatteryCharge(false);
            delay(BatConfig::kBootOpenCircuitSettleMs);
            stableBatPct = clampBatteryPercent(M5.Power.getBatteryLevel());
            M5.Power.setBatteryCharge(true);
            uiCharging = true;
        } else {
            stableBatPct = clampBatteryPercent(M5.Power.getBatteryLevel());
            uiCharging = false;
        }
        lastChargingPrecisionMs = millis();
        lastUnpluggedSampleMs = millis();
        postUnplugSampleDueMs = 0;
        hadExternalPower = externalPowerOrCharging();
    }

    void update() {
        const unsigned long now = millis();
        const bool onExternalPower = externalPowerOrCharging();

        if (gaugePhase == Phase::OpenCircuitRest) {
            if (!onExternalPower) {
                M5.Power.setBatteryCharge(true);
                gaugePhase = Phase::Idle;
                lastChargingPrecisionMs = now;
                uiCharging = false;
                postUnplugSampleDueMs = now + BatConfig::kPostUnplugRelaxMs;
                hadExternalPower = onExternalPower;
                return;
            }
            uiCharging = true;
            if (now - openCircuitStartMs >= BatConfig::kOpenCircuitSettleMs) {
                stableBatPct = clampBatteryPercent(M5.Power.getBatteryLevel());
                M5.Power.setBatteryCharge(true);
                gaugePhase = Phase::Idle;
                lastChargingPrecisionMs = now;
            }
            hadExternalPower = onExternalPower;
            return;
        }

        if (onExternalPower) {
            postUnplugSampleDueMs = 0;
            uiCharging = true;
            
            static unsigned long lastChargeKeepAliveMs = 0;
            if (now - lastChargeKeepAliveMs >= 10000) {
                M5.Power.setBatteryCharge(true);
                lastChargeKeepAliveMs = now;
            }

            if (now - lastChargingPrecisionMs >= BatConfig::kPrecisionSamplePeriodMs) {
                M5.Power.setBatteryCharge(false);
                gaugePhase = Phase::OpenCircuitRest;
                openCircuitStartMs = now;
            }
        } else {
            uiCharging = false;
            gaugePhase = Phase::Idle;
            if (hadExternalPower && !onExternalPower) {
                postUnplugSampleDueMs = now + BatConfig::kPostUnplugRelaxMs;
            }
            if (postUnplugSampleDueMs != 0) {
                if (now >= postUnplugSampleDueMs) {
                    stableBatPct = clampBatteryPercent(M5.Power.getBatteryLevel());
                    postUnplugSampleDueMs = 0;
                    lastUnpluggedSampleMs = now;
                }
            } else if (now - lastUnpluggedSampleMs >= BatConfig::kUnpluggedSamplePeriodMs) {
                stableBatPct = clampBatteryPercent(M5.Power.getBatteryLevel());
                lastUnpluggedSampleMs = now;
            }
        }
        hadExternalPower = onExternalPower;
    }

    int getBatteryPercent() const { return stableBatPct; }
    bool isCharging() const { return uiCharging; }
};

BatteryManager batteryManager;

// ==========================================
// Helper Functions
// ==========================================
static void applyBrightnessStep() {
    M5.Display.setBrightness(PowerSave::kBrightnessLevels[sBrightnessStep]);
}

static void cycleBrightness() {
    sBrightnessStep = (uint8_t)((sBrightnessStep + 1) % 4);
    applyBrightnessStep();
}

static void cycleSensitivity() {
    switch (sSensLevel) {
        case SensLevel::High: sSensLevel = SensLevel::Mid; break;
        case SensLevel::Mid:  sSensLevel = SensLevel::Low; break;
        case SensLevel::Low:  sSensLevel = SensLevel::High; break;
    }
}

static double sensitivityThreshold() { return kSensThresholds[(uint8_t)sSensLevel]; }
static const char* sensitivityLabel() { return kSensLabels[(uint8_t)sSensLevel]; }

static void disableUnusedBluetooth() {
#if defined(ESP32) && SOC_BT_SUPPORTED
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE) {
        (void)esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);
    }
#endif
}

// 메모리 파편화 방지를 위해 String 덧붙이기 대신 char 버퍼 조작으로 변경
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
void handleButtons() {
    if (M5.BtnA.wasPressed()) {
        if (sDisplayBlanked) {
            sDisplayBlanked = false;
            applyBrightnessStep();
            M5.Mic.begin(); // 마이크 다시 시작
        } else {
            cycleBrightness();
        }
        sReadySinceMs = millis();
        sForceUiRefresh = true;
    }
    if (M5.BtnB.wasPressed()) {
        if (sDisplayBlanked) {
            sDisplayBlanked = false;
            applyBrightnessStep();
            M5.Mic.begin(); // 마이크 다시 시작
        } else {
            cycleSensitivity();
        }
        sReadySinceMs = millis();
        sForceUiRefresh = true;
    }
}

bool processAudioAndFFT(double &outHz, bool &outDetected) {
    outHz = 0;
    outDetected = false;
    static int16_t rawBuffer[Audio::kSamples];
    
    if (!M5.Mic.record(rawBuffer, Audio::kSamples, Audio::kSamplingFrequency)) {
        return false;
    }

    double mean = 0;
    for (int i = 0; i < Audio::kSamples; i++) {
        vReal[i] = (double)rawBuffer[i];
        vImag[i] = 0;
        mean += vReal[i];
    }
    mean /= Audio::kSamples;
    for (int i = 0; i < Audio::kSamples; i++) vReal[i] -= mean;

    FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(FFT_FORWARD);
    FFT.complexToMagnitude();

    double maxVal = 0;
    int peakIndex = 0;
    for (int i = Audio::kFftBandBinLo; i <= Audio::kFftBandBinHi; i++) {
        if (vReal[i] > maxVal) {
            maxVal = vReal[i];
            peakIndex = i;
        }
    }
    if (maxVal > Audio::kFftPeakRejectAbove) {
        maxVal = 0;
        peakIndex = 0;
    }

    outDetected = (maxVal >= sensitivityThreshold());

    if (peakIndex >= Audio::kFftBandBinLo && peakIndex <= Audio::kFftBandBinHi) {
        double y0 = vReal[peakIndex - 1];
        double y1 = vReal[peakIndex];
        double y2 = vReal[peakIndex + 1];
        double denom = y0 - 2.0 * y1 + y2;
        if (fabs(denom) > 1e-12) {
            double center = 0.5 * (y0 - y2) / denom;
            outHz = (peakIndex + center) * (Audio::kSamplingFrequency / Audio::kSamples);
        } else {
            outHz = (double)peakIndex * (Audio::kSamplingFrequency / Audio::kSamples);
        }
    }
    return true;
}

void updatePowerSaveState(bool rpmActive) {
    if (rpmActive) {
        sReadySinceMs = 0;
    } else {
        if (sReadySinceMs == 0) sReadySinceMs = millis();
        if (!sDisplayBlanked && (millis() - sReadySinceMs >= PowerSave::kDisplayBlankAfterReadyMs)) {
            M5.Display.setBrightness(0);
            sDisplayBlanked = true;
            M5.Mic.end(); // 화면이 꺼질 때 마이크도 완전히 중지
        }
    }
}

void updateDisplay(double currentHz, bool isDetected, bool rpmActive) {
    static unsigned long lastUpdate = 0;
    const unsigned long uiPeriod = isDetected ? PowerSave::kUiRefreshActiveMs : PowerSave::kUiRefreshIdleMs;
    
    if (sForceUiRefresh || millis() - lastUpdate >= uiPeriod) {
        const bool drawToPanel = (!sDisplayBlanked || rpmActive);
        if (drawToPanel) {
            sForceUiRefresh = false;
            lastUpdate = millis();

            spr.fillScreen(UI::COLOR_BG);

            spr.setFont(&fonts::FreeSans12pt7b);
            spr.setTextColor(UI::COLOR_TEXT);
            spr.setCursor(5, 12);

            double displayHz = isDetected ? currentHz : lastHz;
            if (displayHz > 0) spr.printf("%.1f Hz", displayHz);

            uint16_t batColor = batteryManager.isCharging() ? UI::COLOR_BAT_CHARGING : 
                                (batteryManager.getBatteryPercent() > 20 ? UI::COLOR_BAT_GOOD : UI::COLOR_BAT_LOW);
            spr.setTextColor(batColor);
            spr.setFont(&fonts::FreeSans9pt7b);
            spr.drawRightString(String(batteryManager.getBatteryPercent()) + "%", spr.width() - 5, 12);

            spr.setTextColor(UI::COLOR_SENS_LABEL);
            spr.setFont(&fonts::FreeSans9pt7b);
            spr.setTextDatum(top_right);
            spr.drawString(sensitivityLabel(), spr.width() - 5, 30);

            spr.setTextDatum(top_center);
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

            spr.pushSprite(0, 0);
        } else {
            sForceUiRefresh = false;
            lastUpdate = millis();
        }
    }
}

// ==========================================
// Setup & Loop
// ==========================================
void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
#if defined(ESP32)
    WiFi.mode(WIFI_OFF);
    disableUnusedBluetooth();
#endif
    setCpuFrequencyMhz(PowerSave::kCpuFreqMhz);
    M5.Display.setRotation(1);
    applyBrightnessStep();

    spr.createSprite(M5.Display.width(), M5.Display.height());

    batteryManager.begin();

    auto mic_cfg = M5.Mic.config();
    mic_cfg.sample_rate = Audio::kSamplingFrequency;
    mic_cfg.pin_data_in = 16;
    mic_cfg.pin_bck = 17;
    mic_cfg.pin_ws = 15;
    M5.Mic.config(mic_cfg);
    M5.Mic.begin();
    
    // Serial.begin(115200);
}

void loop() {
    M5.update();
    handleButtons();
    batteryManager.update();

    if (sDisplayBlanked) {
        // 화면이 꺼져있을 때는 오디오/FFT 처리를 생략하고 CPU 낭비를 줄임
        delay(100);
        return;
    }

    double currentHz = 0;
    bool isDetected = false;

    if (processAudioAndFFT(currentHz, isDetected)) {
        if (isDetected) lastHz = currentHz;
        
        bool rpmActive = isDetected && currentHz > Audio::kMinValidRpmHz;
        updatePowerSaveState(rpmActive);
        updateDisplay(currentHz, isDetected, rpmActive);
    }
}
