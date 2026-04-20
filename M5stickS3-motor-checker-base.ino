#include <M5Unified.h>
#include <arduinoFFT.h>

const double samplingFrequency = 16000; 
const uint16_t samples = 2048; 
double vReal[samples];
double vImag[samples];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency);

// [추가] 스프라이트 객체 생성
M5Canvas spr(&M5.Display);

int displayBattery = 0;      
unsigned long lastChargeTime = 0;
bool stableCharging = false;
double lastHz = 0; 
const double SENSITIVITY_THRESHOLD = 150000.0;

// --- 전역 변수 설정 ---
unsigned long lastPrecisionCheck = 0;
const unsigned long checkInterval = 30000; 
unsigned long chargingStepTime = 0;        
int chargingStep = 0;                     
int stableBatteryLevel = -1;
bool isLogicCharging = false; // [추가] 화면 표시용 충전 상태 플래그

String formatNumber(long n) {
    String s = String(n);
    int len = s.length();
    if (len <= 3) return s;
    String res = "";
    int count = 0;
    for (int i = len - 1; i >= 0; i--) { res = s[i] + res; count++; if (count % 3 == 0 && i != 0) res = "," + res; }
    return res;
}

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    M5.Display.setRotation(1);
    
    // [추가] 스프라이트 메모리 할당 (화면 전체 크기)
    spr.createSprite(M5.Display.width(), M5.Display.height());
    
    // --- [추가] 초기 배터리 정밀 측정 ---
    if (M5.Power.isCharging()) {
        M5.Power.setBatteryCharge(false); // 일단 충전 중단
        delay(10000); // 전원을 켠 직후이므로 확실히 안정화 대기
        stableBatteryLevel = M5.Power.getBatteryLevel();
        if (stableBatteryLevel > 100) stableBatteryLevel = 100;
        M5.Power.setBatteryCharge(true);  // 다시 충전 시작
        isLogicCharging = true;
    } else {
        stableBatteryLevel = M5.Power.getBatteryLevel();
        isLogicCharging = false;
    }
    displayBattery = stableBatteryLevel;
    lastPrecisionCheck = millis(); // 1분 타이머 시작

    auto mic_cfg = M5.Mic.config();
    mic_cfg.sample_rate = 16000;
    mic_cfg.pin_data_in = 16;
    mic_cfg.pin_bck     = 17;
    mic_cfg.pin_ws      = 15;
    M5.Mic.config(mic_cfg);
    M5.Mic.begin();
    M5.Power.setBatteryCharge(true);
    Serial.begin(115200);
}

void loop() {
    int16_t rawBuffer[samples];
    //M5.Power.setBatteryCharge(true);
    M5.update();
    bool hardwareCharging = M5.Power.isCharging();  
    hardwareCharging = M5.Power.isCharging();    
  

    // 1. 초기값 설정 (부팅 후 딱 한 번만 실행)
    if (stableBatteryLevel == -1) {
        stableBatteryLevel = M5.Power.getBatteryLevel();
        if (stableBatteryLevel > 100) stableBatteryLevel = 100;
           lastPrecisionCheck = millis(); // 여기서부터 1분 카운트 시작
    }

    Serial.print("hardwareCharging : ");
    Serial.println(hardwareCharging, 0);
    Serial.print("chargingStep : ");
    Serial.println(chargingStep, 0);

    // 2. [충전 모드 로직] 케이블이 꽂혀 있거나 측정 중일 때
    if (hardwareCharging || chargingStep == 1) {
        isLogicCharging = true; // 무조건 노란색 유지
        

        // --- 핵심: 평상시(Step 0)에는 getBatteryLevel()을 호출하지 않고 기존 값 유지 ---
        if (chargingStep == 0) {
            if (millis() - lastPrecisionCheck > checkInterval) {
                // 1분이 되는 순간 측정 프로세스 시작
                M5.Power.setBatteryCharge(false); 
                chargingStepTime = millis();
                chargingStep = 1; 
            }
        }
        // Step 1: 10초 대기 후 딱 한 번만 측정하고 복구
        else if (chargingStep == 1) {
            if (millis() - chargingStepTime > 10000) {
                int currentReal = M5.Power.getBatteryLevel();
                if (currentReal > 100) currentReal = 100;
            
                stableBatteryLevel = currentReal; // 여기서만 숫자가 바뀜

                M5.Power.setBatteryCharge(true); 
                M5.Power.setBatteryCharge(true); 
                lastPrecisionCheck = millis(); // 1분 타이머 리셋
                chargingStep = 0; 
            }
        }
    } 
    // 3. [방전 모드 로직] 케이블이 완전히 뽑혔을 때만 실시간 업데이트
    else {
        
        if (M5.Power.isCharging() || M5.Power.isCharging()) {
         isLogicCharging = true;
        } else {
         isLogicCharging = false;
        }
        chargingStep = 0;
    
        // 1초마다 숫자가 바뀌는 것을 방지하기 위해 비충전 시에도 약간의 텀을 둡니다.
        static unsigned long lastNormalUpdate = millis();
        if (millis() - lastNormalUpdate > 10000) { // 비충전 시eh 60초마다 갱신
            int currentBat = M5.Power.getBatteryLevel();
            if (currentBat > 100) currentBat = 100;
            stableBatteryLevel = currentBat;
            lastNormalUpdate = millis();
        }
    }

    // 최종 화면 출력 변수 할당
    displayBattery = stableBatteryLevel;
    stableCharging = isLogicCharging;
    Serial.print("isLogicCharging : ");
    Serial.println(isLogicCharging, 0);


    // 2. FFT 분석
    if (M5.Mic.record(rawBuffer, samples, 16000)) {
        double mean = 0;
        for (int i = 0; i < samples; i++) {
            vReal[i] = (double)rawBuffer[i];
            vImag[i] = 0;
            mean += vReal[i];
        }
        mean /= samples;
        for (int i = 0; i < samples; i++) { vReal[i] -= mean; }

        FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
        FFT.compute(FFT_FORWARD);
        FFT.complexToMagnitude();
        
        double maxVal = 0;
        int peakIndex = 0;
        for (int i = 13; i < 128; i++) { 
            if (vReal[i] > maxVal) { maxVal = vReal[i]; peakIndex = i; }
        }
        
        double currentHz = 0;
        bool isDetected = (maxVal >= SENSITIVITY_THRESHOLD);

        if (peakIndex > 13 && peakIndex < 127) {
            double y0 = vReal[peakIndex - 1];
            double y1 = vReal[peakIndex];
            double y2 = vReal[peakIndex + 1];
            double center = 0.5 * (y0 - y2) / (y0 - 2 * y1 + y2);
            currentHz = (peakIndex + center) * (samplingFrequency / samples);
            if (isDetected) lastHz = currentHz;
        }

        static unsigned long lastUpdate = 0;
        if (millis() - lastUpdate > 100) { // 반응성을 위해 0.1초로 단축
            lastUpdate = millis();

            // [변경] 모든 그리기 명령을 M5.Display 대신 spr(스프라이트)로 수행
            spr.fillScreen(0x0000); 
            
            // 상단 정보
            spr.setFont(&fonts::FreeSans12pt7b);
            spr.setTextColor(0xFFFF); 
            spr.setCursor(5, 12); 
            
            double displayHz = isDetected ? currentHz : lastHz;
            if (displayHz > 0) {
                spr.printf("%.1f Hz", displayHz);
            }

            uint16_t batColor =  stableCharging ? 0xFFE0 : (displayBattery > 20 ? 0x07E0 : 0xF800);
            spr.setTextColor(batColor);
            spr.setFont(&fonts::FreeSans9pt7b);
            spr.drawRightString(String(displayBattery) + "%", spr.width() - 5, 12);

            // 중앙 RPM
            spr.setTextDatum(top_center);
            if (isDetected && currentHz > 50) {
                long rpm = (long)(currentHz * 60);
                spr.setFont(&fonts::FreeSansBold18pt7b);
                spr.setTextColor(0xFDA0); 
                spr.drawString(formatNumber(rpm), spr.width() / 2, 52); 
                
                spr.setFont(&fonts::FreeSans12pt7b);
                spr.drawString("RPM", spr.width() / 2, 95);
            } else {
                spr.setTextColor(0x7BEF); 
                spr.setFont(&fonts::FreeSans12pt7b);
                spr.drawString("READY", spr.width() / 2, 65);
            }

            // [핵심] 완성된 스프라이트를 한 번에 실제 화면으로 전송
            spr.pushSprite(0, 0); 
        }
    }
}