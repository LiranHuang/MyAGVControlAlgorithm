#pragma once
namespace agv {
class PID {
  double kp_, ki_, kd_;
  double prev_{0}, integ_{0};
  double out_min_{-1e9}, out_max_{1e9};
public:
  PID(double kp, double ki, double kd) : kp_(kp), ki_(ki), kd_(kd) {}
  void setOutputLimit(double mn, double mx) { out_min_ = mn; out_max_ = mx; }
  double step(double target, double current) {
    double e = target - current;
    integ_ += e;
    double deriv = e - prev_;
    prev_ = e;
    double u = kp_*e + ki_*integ_ + kd_*deriv;
    if(u < out_min_) u = out_min_;
    if(u > out_max_) u = out_max_;
    return u;
  }
};
} // namespace agv
