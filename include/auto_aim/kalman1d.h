#ifndef AUTO_AIM_KALMAN1D_H_
#define AUTO_AIM_KALMAN1D_H_

namespace auto_aim {

// 单轴匀速卡尔曼滤波，过程噪声用标准"白噪声加加速度"模型。
class Kalman1DFilter {
 public:
  static constexpr double kKalmanSa = 350.0;  // 加速度噪声标准差（px/s²）
  static constexpr double kObsNoise = 6.0;    // 观测噪声方差
  static constexpr double kInitVar = 1.0;     // 初始协方差

  Kalman1DFilter(double sa = kKalmanSa, double r = kObsNoise);

  void Predict(double dt);
  void Update(double meas);
  double PredictPos(double t) const;
  void Reset();

  double pos() const { return pos_; }
  double vel() const { return vel_; }
  bool initialized() const { return initialized_; }
  void set_pos(double pos) { pos_ = pos; }

 private:
  double sa_;
  double r_;
  double pos_;
  double vel_;
  double p00_;
  double p01_;
  double p11_;
  bool initialized_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM_KALMAN1D_H_