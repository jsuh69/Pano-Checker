# 스톱워치 STOPPED + USB 연결 시 전력 점검

분석일: 2026-10-20. 소스 정적 분석이며, 실제 전류 및 PMIC 레지스터를 장치에서 읽은 결과는 아니다. 이 점검에서는 펌웨어를 수정하지 않았다.

현재 로컬 설치는 M5Unified 0.2.26, M5Stack 보드 패키지 3.3.10으로 이전 빌드 기록(0.2.25/3.3.9) 이후 변경되어 있다. 현재 설치 소스를 기준으로 확인했다.

## 확인된 실행 상태

- STOPWATCH 모드는 loop의 측정 분기를 건너뛴다. FFT, 진동 수집, 홀 센서 analogRead를 실행하지 않는다.
- 모드 전환 시 마이크를 end하고 BMI270 PWR_CTRL(0x7D)을 0으로 쓴다. 정상 통신 시 가속도·자이로 등은 정지된다. 센서 전원 레일 자체 차단은 아니다.
- 스피커는 M5Unified StickS3 초기화에서 PM1 GPIO3를 Low로 설정하며, 앱도 internal_spk=false를 지정한다. Wi-Fi는 off, Bluetooth는 시작하지 않는다.
- STOPPED 화면은 매 루프 다시 그리지 않는다. 상태 또는 배터리 표시가 바뀔 때 강제 갱신된다.
- CPU는 80 MHz를 유지하고 약 10 ms마다 M5.update/버튼 처리/전력 정책을 확인한다. delay(10)은 FreeRTOS에 양보하는 동작이며 명시적인 light sleep이 아니다. USB 연결 시 정책상 화면 off와 light sleep 진입을 막는다.
- 배터리/USB 판정은 2초마다, 충전 enable 재설정은 외부 전원 감지 중 10초마다 실행된다.

## 중요한 누락: 외부 5V 출력

앱 setup은 `auto cfg = M5.config()` 뒤 internal_spk/internal_rtc만 바꾸고 output_power는 지정하지 않는다. 현재 M5Unified config_t의 output_power 기본값은 true이며, M5Unified::_begin에서 `Power.setExtOutput(cfg.output_power)`를 호출한다. StickS3의 이 API는 PM1 PWR_CFG(0x06)의 BOOST_EN(bit3)을 설정한다.

따라서 현재 설치 라이브러리 기준으로 외부 5V 승압 출력을 켜는 초기화 경로가 존재한다. 앱은 이후 이를 끄지 않는다. 실제 장치에서도 켜져 있는지는 `M5.Power.getExtOutput()` 또는 PM1 레지스터 readback으로 확인해야 한다. 승압 회로와 외부 회로는 측정/FFT를 멈추는 것만으로 꺼지지 않는다.

외부 5V 전원을 필요로 하지 않는 현재 용도에서는 우선 다음 설정을 검토한다.

```cpp
auto cfg = M5.config();
cfg.output_power = false;  // M5.begin(cfg) 전에 지정
// ...
M5.begin(cfg);
M5.Power.setExtOutput(false); // 명시적으로 비활성화하고 readback 확인
```

외부 홀 센서는 README 기준 3.3V 전원을 사용하므로 5V 출력과 별개다. 3.3V에 연결된 센서는 측정을 하지 않아도 자체 소비전류가 남는다. 실제 외부 배선은 별도 확인이 필요하다.

## 하드웨어 전원과 충전 표시의 한계

공식 회로도에는 MEMS 마이크 전원, 코덱/오디오 레일, IMU 전원, LCD 및 백라이트 회로가 별도로 존재한다. Mic.end는 I2S 태스크와 코덱을 절전 처리하지만 MEMS 마이크의 전원 공급 자체를 제거하는 API가 아니다. LCD를 유지하면서 공유 전원 레일을 임의로 끄면 화면 또는 다른 기능에 영향을 줄 수 있으므로 레일 단위 조치는 회로와 함께 검증해야 한다.

USB 아이콘은 `VBUS >= 4200mV || CHG_STAT`로 정한 외부 전원 감지 결과다. 실제 충전 전류를 나타내지 않는다. 앱은 충전을 enable하지만 실제 상태 readback이나 충전 전류는 기록하지 않는다. 이 보드에서 M5Unified getBatteryCurrent는 전용 측정을 구현하지 않아 기본값 0으로 빠지므로 순충전 확인에 사용할 수 없다.

현재 설치 라이브러리의 잔량 계산은 `int((VBAT_mV-3300)*100/800)`을 0–100으로 제한한다. 95%는 약 4060mV에 해당하며, 100% 표시가 실제 완충·정확한 잔여 용량을 뜻하지 않는다. 표시 하락만으로 소비량이나 충전 실패를 확정할 수 없다. 다만 지속적으로 하락한다면 충전 전원 경로와 순전류를 확인해야 한다.

## 우선순위

1. 외부 5V 승압 출력을 명시적으로 끄고 실제 BOOST_EN readback으로 확인한다.
2. USB 감지/실제 CHG_STAT/CHG_EN을 구분하고 VBUS·VBAT 추이를 함께 기록한다. BMI270 0x7D, 마이크 isRunning, 외부 5V 상태도 저주기로 기록한다.
3. STOPPED 버튼 폴링 주기를 20–50 ms 등으로 늘리는 방안을 검토한다. USB 연결 시 화면 유지·슬립 금지 요구는 유지한다. 폴링 감소만으로 CPU가 sleep 상태가 되는 것은 아니다.
4. 동일 케이블/전원에서 기존 대비 전류를 측정한다. 충전기가 공급하는 전류만으로 배터리 순충전을 판단하지 않으며 필요하면 배터리 경로 측정을 병행한다.

## 근거

- 앱: `M5stickS3-motor-checker.ino`의 setup(), changeMode(), loop(), BatteryManager, updateDisplay().
- 로컬 M5Unified: M5Unified.hpp의 config_t::output_power, M5Unified.inl의 _begin 및 StickS3 초기화, Power_Class.inl 및 power/M5PM1_Class.inl의 외부 출력·충전·잔량 구현.
- [M5Stack StickS3 공식 문서](https://docs.m5stack.com/en/core/StickS3)
- [공식 회로도](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/1207/K150_Stick_S3_PRJ_V0.6_20251111_2025_11_17_16_10_24.pdf)

## v0.9.1 반영

위 내용은 수정 전 원인 분석이다. 후속 소스 수정에서 `cfg.output_power=false` 및 기동 직후 `setExtOutput(false)`를 적용했다. STOPPED 상태 버튼 확인은 50ms, 메인 루프 대기는 25ms로 조정했고, 화면 변경이 없으면 화면 갱신 호출도 생략한다. 스톱워치 실행 시에는 다시 10ms 버튼 확인으로 돌아간다. USB 연결 중 밝기 유지·슬립 금지 및 배터리 사용 중 스톱워치 2분 화면 유지 정책은 그대로다.

시리얼 모니터가 연결된 경우에만 30초마다 PMIC PWR_CFG/CHG_STAT, BMI270 PWR_CTRL, VIN/VBAT, 마이크 실행 여부를 로그에 남긴다. 이 진단용 읽기는 센서 수집/FFT를 실행하지 않는다. 읽기 실패는 -1로 구분하며, 송신은 사용 가능한 공간까지만 나누어 수행한다. 전류/충전속도 개선량은 아직 실기기로 측정하지 않았다.

### 사용자 제공 v0.9.1 로그

후속 사용자가 제공한 연속 5회 로그에서는 VIN 약 5202–5208mV, VBAT 4100→4124mV, `chg=1 en=1 boost=0 imu=0 mic=0 watch=0`을 확인했다. 외부 출력과 미사용 센서의 기대 상태 및 배터리 전압 상승은 확인되지만, 충전 전류·소비전류·완충 여부의 실측을 대체하지는 않는다.
