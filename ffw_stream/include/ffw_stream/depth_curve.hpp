// depth_curve.hpp -- nonlinear 8-bit depth code shared by the D405 streamer
// (encode) and the UDP receiver (decode). Both sides MUST use the same Params.
//
// Code layout (one byte per pixel):
//   0              raw Z16 == 0, i.e. "no valid depth" from the sensor
//   1..3           guard band: never sent, so MJPEG ringing around holes does
//                  not turn an invalid pixel into a near-range depth. The
//                  decoder treats code < kInvalidBelow as invalid and
//                  kInvalidBelow..kFirstValid as min_m.
//   4..255         depth in [min_m, max_m] (values outside are clamped)
//
// The valid codes follow an exponential step profile: the depth step between
// neighbouring codes grows as exp((d - min_m) / lambda), so the close range is
// near-uniformly fine and resolution falls off quickly with distance (smooth,
// no abrupt knee):
//
//   frac(d) = (1 - exp(-(d - min_m) / lambda)) / (1 - exp(-(max_m - min_m) / lambda))
//   code    = kFirstValid + round(frac * (255 - kFirstValid))
//
// lambda is solved so that frac(knee_m) == knee_frac. Defaults: 7 cm .. 1 m
// with 80% of the codes below 30 cm (lambda ~= 14.3 cm): step ~0.6 mm at 7 cm,
// 0.7 mm at 10 cm, 1.4 mm at 20 cm, 2.8 mm at 30 cm, 11 mm at 50 cm, but
// ~9 cm at 80 cm and ~37 cm at 1 m (far range is coarse by design).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace ffw_stream {
namespace depth_curve {

constexpr int kInvalid = 0;
constexpr int kInvalidBelow = 2;  // decoder: code < 2 -> invalid
constexpr int kFirstValid = 4;    // encoder: first code used for valid depth
constexpr int kLastValid = 255;

struct Params {
  double min_m = 0.07;
  double max_m = 1.0;
  double knee_m = 0.30;
  double knee_frac = 0.80;  // 80% of the codes below knee_m
};

class Curve {
public:
  explicit Curve(const Params &p = Params()) : p_(p) {
    if (!(p_.max_m > p_.min_m)) p_.max_m = p_.min_m + 0.01;
    p_.knee_m = std::clamp(p_.knee_m, p_.min_m + 1e-4, p_.max_m - 1e-4);
    // frac_at_knee(lambda) falls monotonically from 1 (lambda->0) to the
    // linear fraction (lambda->inf). knee_frac <= linear means "no stretch".
    const double lin = (p_.knee_m - p_.min_m) / (p_.max_m - p_.min_m);
    const double want = std::clamp(p_.knee_frac, lin + 1e-6, 1.0 - 1e-6);
    double lo = 1e-5, hi = 1e3;  // bisect lambda in log space
    for (int i = 0; i < 200; ++i) {
      const double mid = std::sqrt(lo * hi);
      if (frac_with_lambda(p_.knee_m, mid) > want) lo = mid; else hi = mid;
    }
    lambda_ = std::sqrt(lo * hi);
    span_ = -std::expm1(-(p_.max_m - p_.min_m) / lambda_);  // 1 - exp(-range/lambda)
  }

  const Params &params() const { return p_; }
  double lambda() const { return lambda_; }

  // Meters -> code. d <= 0 or non-finite -> kInvalid.
  uint8_t encode(double d) const {
    if (!(d > 0.0) || !std::isfinite(d)) return kInvalid;
    d = std::clamp(d, p_.min_m, p_.max_m);
    const double f = -std::expm1(-(d - p_.min_m) / lambda_) / span_;
    const double c = kFirstValid + f * (kLastValid - kFirstValid);
    return static_cast<uint8_t>(std::clamp(static_cast<int>(std::lround(c)), kFirstValid, kLastValid));
  }

  // Code -> meters. Invalid codes -> 0.
  double decode(int code) const {
    if (code < kInvalidBelow) return 0.0;
    code = std::clamp(code, kFirstValid, kLastValid);
    const double f = double(code - kFirstValid) / double(kLastValid - kFirstValid);
    return p_.min_m - lambda_ * std::log1p(-f * span_);
  }

  // LUT over raw Z16 units (65536 entries) for the per-pixel encode.
  std::vector<uint8_t> z16_lut(double depth_scale_m) const {
    std::vector<uint8_t> lut(65536);
    lut[0] = kInvalid;
    for (int r = 1; r < 65536; ++r) lut[r] = encode(r * depth_scale_m);
    return lut;
  }

  // 256-entry decode table (meters, 0 = invalid).
  std::vector<float> decode_lut() const {
    std::vector<float> lut(256);
    for (int c = 0; c < 256; ++c) lut[c] = static_cast<float>(decode(c));
    return lut;
  }

private:
  double frac_with_lambda(double d, double l) const {
    return std::expm1(-(d - p_.min_m) / l) / std::expm1(-(p_.max_m - p_.min_m) / l);
  }

  Params p_;
  double lambda_ = 0.1;
  double span_ = 1.0;
};

}  // namespace depth_curve
}  // namespace ffw_stream
