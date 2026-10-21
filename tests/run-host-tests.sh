#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd "$(dirname "$0")/.." && pwd)"
fft_src="${1:-$HOME/Documents/Arduino/libraries/arduinoFFT/src}"
if [[ ! -f "$fft_src/arduinoFFT.cpp" ]]; then
  printf 'arduinoFFT/src 경로를 첫 번째 인자로 지정하세요.\n' >&2
  exit 2
fi
build_dir="$(mktemp -d "${TMPDIR:-/tmp}/pano-tests.XXXXXX")"
compiler="${CXX:-c++}"
"$compiler" -std=c++11 -Wall -Wextra -Werror \
  "$project_dir/tests/logic_test.cpp" -o "$build_dir/logic-test"
"$build_dir/logic-test"
"$compiler" -std=c++17 -Wall -Wextra -Werror -Wno-unused-function \
  -I "$project_dir/tests/fakes" -I "$fft_src" \
  "$project_dir/tests/firmware_test.cpp" "$fft_src/arduinoFFT.cpp" \
  -o "$build_dir/firmware-test"
"$build_dir/firmware-test"
"$compiler" -std=c++17 -Wall -Wextra -Werror -Wno-unused-function \
  -I "$project_dir/tests/fakes" -I "$fft_src" \
  "$project_dir/tests/speed_test.cpp" "$fft_src/arduinoFFT.cpp" \
  -o "$build_dir/speed-test"
"$build_dir/speed-test"
printf 'Test binaries: %s\n' "$build_dir"
