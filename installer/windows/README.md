# Pano-Checker Windows 설치 프로그램

Windows 10/11 x64, .NET Framework 4.8용 WinForms 실행파일입니다.
수동 COM 선택과 설치 동작은 유지하고 **M5StickS3 찾기** 버튼을 추가했습니다.

## 검색 동작

- Windows PnP 이름에서 StickS3 제품명을 확인하거나, M5Stack 보드 정의의 USB VID 303A / PID 1001을 후보로 사용합니다.
- 후보 포트에만 동봉된 esptool 5.2.0으로 `read-mac` 명령을 실행합니다. `--chip auto`, `--no-stub`, 제한된 연결 횟수와 12초 타임아웃을 사용합니다. 펌웨어 쓰기/삭제/eFuse 변경 명령은 실행하지 않습니다.
- 종료 코드 0과 ESP32-S3 칩 응답이 모두 확인되어야 성공 후보로 인정합니다.
- 확인된 후보가 하나면 자동 선택합니다. 없으면 찾지 못했다는 메시지, 여러 개면 포트 목록과 수동 선택 안내를 표시합니다. 검색이 실패해도 기존 수동 선택은 유지합니다.
- 검색과 설치는 동시에 실행할 수 없습니다. 검색 중 UI는 응답하며 후보별 결과를 로그에 표시합니다. 시간 초과 시 해당 조회 프로세스 트리만 종료합니다.

**한계:** 이 USB ID와 칩은 다른 ESP32-S3 보드도 사용합니다. 따라서 기종 확정이 아닌 연결 후보 탐색입니다. 여러 보드를 함께 연결한 경우 수동 확인이 필요합니다. WMI가 사용 불가능하거나 다른 USB ID를 사용한 펌웨어/드라이버는 검색에서 누락될 수 있으므로 수동 선택을 유지합니다. 검색 시 후보 보드가 재시작될 수 있습니다.

## Windows에서 다시 빌드

`Build-Windows.ps1`을 실행하거나 .NET Framework C# 컴파일러로 아래 3개 소스를 함께 빌드하세요.

- Installer.cs
- DetectionPolicy.cs
- DeviceDiscovery.cs

참조: System.Management.dll. 생성된 ESP32-Installer.exe와 ESP32-Installer.exe.config를 패키지 루트에 놓습니다. firmware.txt, firmware, tools는 같은 구조로 유지하세요.

`Test-Windows.ps1`은 Windows에서 판별 정책과 설정 파서 회귀 테스트를 실행합니다. 실제 WMI/USB 연결 검사는 포함하지 않습니다.

## 검증 범위

- Windows .NET Framework 4.8 참조 어셈블리로 /target:winexe /platform:anycpu 빌드 통과.
- DetectionPolicyTests.cs: USB 후보 필터, 중복 제거, 다중 후보, 미발견, 잘못된 칩/실패 응답, 포트 인자 안전성 및 실제 Package 설정 파서 검사 통과.
- Mac에서 Windows GUI, WMI, 실제 USB 포트 연결/타임아웃은 실행하지 않았습니다. Windows PC와 M5StickS3에서 검색 및 쓰기/부팅을 확인하세요.
