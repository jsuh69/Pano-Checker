#include <Arduino.h>
#include <arduinoFFT.h>
#include <driver/i2s.h>

// 1. 설정 (하드웨어에 맞게 수정)
#define I2S_WS 4    // LRCLK
#define I2S_SD 5    // DOUT
#define I2S_SCK 6   // BCLK
#define I2S_PORT I2S_NUM_0

const uint16_t N = 1024;          // 샘플 개수 (2의 거듭제곱)
const double SR = 44100;          // 샘플링 주파수
double vReal[N];
double vImag[N];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, N, SR);

void setupI2S() {
i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 44100,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, // 반드시 32비트로 설정
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,  // 마이크 L/R 핀 연결 확인
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 128,
    .use_apll = false
};

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
}

void setup() {
  Serial.begin(115200);
  setupI2S();
  Serial.println("RPM Meter Ready...");
}

void loop() {
  // 2. I2S 데이터 읽기
  size_t bytes_read;
  int32_t raw_buffer[N];
  
 i2s_read(I2S_PORT, (char *)raw_buffer, sizeof(raw_buffer), &bytes_read, portMAX_DELAY);

for (int i = 0; i < N; i++) {
    // 중요: 대부분의 I2S 마이크는 상위 24비트에 데이터가 있습니다.
    // 데이터를 우측으로 14~16비트 밀어서 크기를 조정합니다.
    vReal[i] = (double)(raw_buffer[i] >> 14); 
    vImag[i] = 0.0;
}

  // 3. FFT 처리
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();

  // 4. 주파수 및 RPM 계산 (핵심 루프)
  double max_mag = 0;
  double peak_freq = 0;

  // 저주파 노이즈 제거를 위해 i=5 (약 200Hz 이상)부터 탐색
  for (int i = 5; i < (N / 2); i++) {
    double freq = (i * SR) / N;

    Serial.print("freq: ");
    Serial.println(freq, 1);
    
    // 타겟 주파수 범위 (200Hz ~ 900Hz) 내에서만 피크 탐색
    if (freq >= 200 && freq <= 900) {
      if (vReal[i] > max_mag) {
        max_mag = vReal[i];
        peak_freq = freq;
      }
    }
  }

  // 5. 결과 출력
  if (max_mag > 1) { // 일정 크기 이상의 신호가 있을 때만 출력 (임계값은 조정 필요)
    double rpm = peak_freq * 60;
    Serial.printf("Peak Freq: %.1f Hz | Mag: %.1f | RPM: %.0f\n", peak_freq, max_mag, rpm);
  } else {
    Serial.println("Signal too weak... [WAITING]");
    Serial.print("max_mag: ");
    Serial.println(max_mag, 0);
  }

  delay(100); // 디스플레이 갱신 속도 조절
}