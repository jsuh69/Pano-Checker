#include "../M5stickS3-motor-checker/AppLogic.h"
#include <assert.h>
#include <stdio.h>
#include <initializer_list>
#include <cmath>

int main() {
  // 19mm diameter roller; one rising edge is one turn.
  assert(std::abs(Speed::kmhFromPulses(1, 1000000) - 0.2148849375) < 1e-9);
  assert(std::abs(Speed::kmhFromPulses(50, 500000) - 21.48849375) < 1e-7);
  assert(std::abs(Speed::kmhFromPulses(50, 600000) - 17.90707813) < 1e-7);
  assert(Speed::kmhFromPulses(0, 500000) == 0);
  assert(Speed::kmhFromPulses(100, 0) == 0);
  FrontButtonGesture button;
  assert(button.update(false, true, false) == FrontAction::None);
  assert(button.update(true, false, false) == FrontAction::None);
  assert(button.update(false, true, false) == FrontAction::ShortPress);
  button.update(true, false, false);
  assert(button.update(false, false, true) == FrontAction::LongPress);
  assert(button.update(false, false, true) == FrontAction::None);
  assert(button.update(false, true, false) == FrontAction::None);
  button.update(true, false, false);
  button.cancel(); // Wakeup and mode changes consume the entire gesture.
  assert(button.update(false, false, true) == FrontAction::None);
  assert(button.update(false, true, false) == FrontAction::None);
  Stopwatch watch;
  assert(!watch.running() && watch.elapsedMs(100) == 0);
  watch.toggle(100);
  assert(watch.running() && watch.elapsedMs(123556) == 123456);
  // Sleep and mode changes do not need loop ticks to keep elapsed time.
  assert(watch.elapsedMs(3600100) == 3600000);
  watch.toggle(3600100);
  assert(!watch.running() && watch.elapsedMs(9000000) == 3600000);
  watch.toggle(9000000);
  assert(watch.elapsedMs(9000500) == 3600500);
  watch.reset();
  assert(!watch.running() && watch.elapsedMs(9001000) == 0);
  // Continue beyond 32-bit millisecond rollover without losing elapsed time.
  const uint64_t late = (1ULL << 32) - 100;
  watch.toggle(late);
  assert(watch.elapsedMs(late + 250) == 250);
  watch.toggle(late + 250);
  watch.reset();

  assert(displayPowerFor(4999, false, false, false) == DisplayPower::Bright);
  assert(displayPowerFor(5000, false, false, false) == DisplayPower::Dim);
  assert(displayPowerFor(19999, false, false, false) == DisplayPower::Dim);
  assert(displayPowerFor(20000, false, false, false) == DisplayPower::Off);
  assert(displayPowerFor(119999, false, true, false) == DisplayPower::Bright);
  assert(displayPowerFor(120000, false, true, false) == DisplayPower::Off);
  for (uint64_t idle : {0ULL, 5000ULL, 20000ULL, 120000ULL, 86400000ULL}) {
    assert(displayPowerFor(idle, true, false, false) == DisplayPower::Bright);
    assert(displayPowerFor(idle, true, true, false) == DisplayPower::Bright);
    assert(displayPowerFor(idle, false, false, true) == DisplayPower::Bright);
  }
  puts("PASS: button gestures/cancellation, stopwatch start/stop/resume/reset, sleep continuity, rollover, power policy boundaries and USB override");
}
