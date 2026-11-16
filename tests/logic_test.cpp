#include "../M5stickS3-motor-checker/AppLogic.h"
#include "../M5stickS3-motor-checker/PanoTrend.h"
#include <assert.h>
#include <stdio.h>
#include <initializer_list>
#include <cmath>

int main() {
  PanoTrend speed(true);
  speed.reset(0);
  speed.observe(100, 30, true); speed.observe(600, 0, true);
  assert(speed.hasPoint(119) && speed.minimumPoint(119) == 0);
  uint16_t speedAt;
  assert(speed.orderedPoint(119, 2, speedAt) == 0 && speedAt == 600);
  assert(speed.low() == 0 && speed.high() >= 30);
  speed.observe(1100, 0, true);
  assert(speed.hasPoint(119) && speed.point(119) == 0);
  speed.observe(2100, 0, false);
  assert(!speed.hasPoint(119));
  speed.reset(3000); speed.observe(3100, 27.5, true);
  assert(speed.high() - speed.low() == 6);
  speed.observe(124000, 0, true);
  assert(!speed.hasPoint(118) && speed.hasPoint(119));

  // Falling then rising extrema must keep their real time order.
  PanoTrend ordered;
  ordered.reset(0);
  ordered.observe(20, 634, true); ordered.observe(180, 400, true);
  uint16_t offset;
  assert(ordered.orderedPoint(119, 0, offset) == 634 && offset == 20);
  assert(ordered.orderedPoint(119, 1, offset) == 400 && offset == 180);
  ordered.observe(1020, 410, true); ordered.observe(1180, 620, true);
  assert(ordered.orderedPoint(119, 0, offset) == 410 && offset == 20);
  assert(ordered.orderedPoint(119, 1, offset) == 620 && offset == 180);
  // Current reading inside the extrema must still become the graph endpoint.
  PanoTrend endpoint;
  endpoint.reset(0);
  endpoint.observe(20, 500, true);
  endpoint.observe(120, 475, true);
  endpoint.observe(220, 485, true);
  assert(endpoint.orderedPoint(119, 0, offset) == 500 && offset == 20);
  assert(endpoint.orderedPoint(119, 1, offset) == 475 && offset == 120);
  assert(endpoint.orderedPoint(119, 2, offset) == 485 && offset == 220);
  endpoint.observe(320, 485, true);
  assert(endpoint.orderedPoint(119, 2, offset) == 485 && offset == 320);
  endpoint.observe(1020, 634, true); // Scale expansion keeps old data in Hz.
  assert(endpoint.orderedPoint(118, 2, offset) == 485 && offset == 320);
  assert(endpoint.high() > 634);
  assert(endpoint.orderedPoint(119, 2, offset) == 634 && offset == 20);
  endpoint.observe(2020, 0, false);
  assert(endpoint.orderedPoint(119, 2, offset) == 0);
  endpoint.observe(140020, 0, false);
  for (unsigned i = 0; i < PanoTrend::kPoints; ++i)
    assert(endpoint.orderedPoint(i, 2, offset) == 0);
  endpoint.reset(160000);
  assert(endpoint.orderedPoint(119, 2, offset) == 0);
  PanoTrend trend;
  trend.reset(20);
  trend.observe(20, 460, true);
  const double initialLow = trend.low(), initialHigh = trend.high();
  trend.observe(120, 480, true);
  assert(trend.point(119) == 480 && trend.maximum() == 480);
  assert(trend.minimumPoint(119) == 460);
  trend.observe(1020, 0, false);
  assert(trend.point(118) == 480 && trend.point(119) == 0);
  trend.observe(2020, 520, true);
  assert(trend.high() > initialHigh && trend.low() == initialLow);
  trend.observe(2120, 450, true);
  assert(trend.high() > initialHigh && trend.maximum() == 520);
  assert(trend.minimumPoint(119) == 450);
  assert(trend.point(119) == 520); // Lower values cannot erase the bucket peak.
  trend.observe(140020, 0, false);
  for (unsigned i = 0; i < PanoTrend::kPoints; ++i) assert(trend.point(i) == 0 && trend.minimumPoint(i) == 0);
  assert(trend.maximum() == 520);
  trend.reset(160020);
  assert(trend.maximum() == 0 && trend.point(119) == 0);

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
