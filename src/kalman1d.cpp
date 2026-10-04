#include "auto_aim/kalman1d.h"

namespace auto_aim {

Kalman1DFilter::Kalman1DFilter(double sa, double r)
    : sa_(sa),
      r_(r),
      pos_(0.0),
      vel_(0.0),
      p00_(kInitVar),
      p01_(0.0),
      p11_(kInitVar),
      initialized_(false) {}

void Kalman1DFilter::Predict(double dt) {
  pos_ += vel_ * dt;
  const double dt2 = dt * dt;
  const double sa2 = sa_ * sa_;
  const double q00 = sa2 * dt2 * dt2 / 4.0;
  const double q01 = sa2 * dt2 * dt / 2.0;
  const double q11 = sa2 * dt2;
  const double p00 = p00_ + 2.0 * dt * p01_ + dt2 * p11_ + q00;
  const double p01 = p01_ + dt * p11_ + q01;
  const double p11 = p11_ + q11;
  p00_ = p00;
  p01_ = p01;
  p11_ = p11;
}

void Kalman1DFilter::Update(double meas) {
  if (!initialized_) {
    pos_ = meas;
    vel_ = 0.0;
    p00_ = kInitVar;
    p01_ = 0.0;
    p11_ = kInitVar;
    initialized_ = true;
    return;
  }
  const double s = p00_ + r_;
  const double k0 = p00_ / s;
  const double k1 = p01_ / s;
  const double innov = meas - pos_;
  pos_ += k0 * innov;
  vel_ += k1 * innov;
  const double p00 = (1.0 - k0) * p00_;
  const double p01 = (1.0 - k0) * p01_;
  const double p11 = -k1 * p01_ + p11_;
  p00_ = p00;
  p01_ = p01;
  p11_ = p11;
}

double Kalman1DFilter::PredictPos(double t) const { return pos_ + vel_ * t; }

void Kalman1DFilter::Reset() {
  pos_ = 0.0;
  vel_ = 0.0;
  p00_ = kInitVar;
  p01_ = 0.0;
  p11_ = kInitVar;
  initialized_ = false;
}

}  // namespace auto_aim