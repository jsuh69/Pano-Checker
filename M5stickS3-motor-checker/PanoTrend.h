#pragma once
#include <stdint.h>
#include <cmath>

// Presentation-only history. Never feeds back into sensor/FFT decisions.
class PanoTrend {
 public:
  explicit PanoTrend(bool speed = false) : speed_(speed) {}
  static constexpr unsigned kPoints = 120;
  static constexpr uint64_t kBucketMs = 1000;
  void reset(uint64_t now) {
    origin_ = now; bucket_ = 0;
    maximum_ = 0; low_ = 0; high_ = 0;
    for (unsigned i = 0; i < kPoints; ++i) {
      values_[i] = minima_[i] = latest_[i] = 0; occupied_[i] = false;
    }
  }
  void observe(uint64_t now, double hz, bool valid) {
    const uint64_t next = (now - origin_) / kBucketMs;
    if (next != bucket_) {
      const uint64_t steps = next - bucket_;
      if (steps >= kPoints) {
        for (unsigned i = 0; i < kPoints; ++i) {
          values_[i] = minima_[i] = latest_[i] = 0; occupied_[i] = false;
        }
      } else {
        for (uint64_t i = bucket_ + 1; i <= next; ++i) {
          const unsigned slot = i % kPoints;
          values_[slot] = minima_[slot] = latest_[slot] = 0; occupied_[slot] = false;
        }
      }
      bucket_ = next;
    }
    if (!valid || !std::isfinite(hz) || (speed_ ? hz < 0 : hz <= 0)) return;
    if (hz > maximum_) maximum_ = hz;
    const double step = speed_ ? 1.0 : 10.0;
    const double margin = speed_ ? 0.5 : 5.0;
    if (high_ == 0) {
      low_ = std::floor((hz - (speed_ ? 2.5 : 25.0)) / step) * step;
      if (low_ < 0) low_ = 0;
      high_ = low_ + (speed_ ? 6.0 : 60.0);
    }
    if (hz < low_ + margin) low_ = std::floor((hz - step) / step) * step;
    if (low_ < 0) low_ = 0;
    if (hz > high_ - margin) high_ = std::ceil((hz + step) / step) * step;
    // Preserve both extremes so brief slowdowns are not hidden by the peak.
    const unsigned slot = bucket_ % kPoints;
    const uint16_t offset = (uint16_t)((now - origin_) % kBucketMs);
    latest_[slot] = (float)hz; lastAt_[slot] = offset;
    if (!occupied_[slot] || hz < minima_[slot]) {
      minima_[slot] = (float)hz; minAt_[slot] = offset;
    }
    if (!occupied_[slot] || hz > values_[slot]) {
      values_[slot] = (float)hz; maxAt_[slot] = offset;
    }
    occupied_[slot] = true;
  }
  bool hasPoint(unsigned x) const {
    const uint64_t age = kPoints - 1 - x;
    return bucket_ >= age && occupied_[(bucket_ - age) % kPoints];
  }
  float point(unsigned x) const {
    const uint64_t age = kPoints - 1 - x;
    return bucket_ < age ? 0 : values_[(bucket_ - age) % kPoints];
  }
  float minimumPoint(unsigned x) const {
    const uint64_t age = kPoints - 1 - x;
    return bucket_ < age ? 0 : minima_[(bucket_ - age) % kPoints];
  }
  // Extrema in acquisition order, followed by the latest accepted measurement.
  float orderedPoint(unsigned x, unsigned order, uint16_t &offset) const {
    const uint64_t age = kPoints - 1 - x;
    if (bucket_ < age || !hasPoint(x)) { offset = 0; return 0; }
    const unsigned slot = (bucket_ - age) % kPoints;
    if (order == 2) { offset = lastAt_[slot]; return latest_[slot]; }
    const bool minimum = (minAt_[slot] <= maxAt_[slot]) == (order == 0);
    offset = minimum ? minAt_[slot] : maxAt_[slot];
    return minimum ? minima_[slot] : values_[slot];
  }
  double maximum() const { return maximum_; }
  double low() const { return low_; }
  double high() const { return high_ == 0 ? 100 : high_; }
 private:
  bool speed_ = false;
  bool occupied_[kPoints] = {};
  float values_[kPoints] = {};
  float minima_[kPoints] = {};
  float latest_[kPoints] = {};
  uint16_t minAt_[kPoints] = {}, maxAt_[kPoints] = {}, lastAt_[kPoints] = {};
  uint64_t origin_ = 0, bucket_ = 0;
  double maximum_ = 0, low_ = 0, high_ = 0;
};
