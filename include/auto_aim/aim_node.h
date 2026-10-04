#ifndef AUTO_AIM_AIM_NODE_H_
#define AUTO_AIM_AIM_NODE_H_

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "auto_aim/kalman1d.h"
#include "auto_aim/line_guard.h"
#include "auto_aim/plate_detector.h"
#include "auto_aim/plate_tracker.h"
#include "auto_aim/serial_turret.h"

namespace auto_aim {

// 自动瞄准节点：订阅图像 → 判色/检测/跟踪/选靶 → 转向+开火。
class AimNode : public rclcpp::Node {
 public:
  explicit AimNode(const std::string& slave);

  // ---- 回合/生命周期常量 ----
  static constexpr double kFirstFrameTimeout = 21600.0;
  static constexpr double kImageStopTimeout = 3.0;
  static constexpr double kRescanInterval = 1.0;
  static constexpr double kReopenInterval = 20.0;  // 同路径强制重开串口的周期
  static constexpr double kWaitHintInterval = 30.0;

  // ---- 弹道/炮台常量 ----
  static constexpr double kBulletSpeedPxS = 600.0;
  static constexpr double kGunPivotXRatio = 0.493;  // 炮口 x（1152*0.493）
  static constexpr double kGunPivotYRatio = 0.94;   // 炮口 y（648*0.94）

  // ---- 开火门常量 ----
  static constexpr double kCompLead = -0.05;   // 相对 flight 的固定提前量
  static constexpr double kFireInterval = 0.12;
  static constexpr double kAngleGateDeg = 2.0;
  static constexpr double kFlightAimMax = 1.6;
  static constexpr double kStaticSpeed = 30.0;
  static constexpr int kFireDrainSleepMs = 20;  // 开火后等游戏读线程吞字节

  // ---- 瞄准/卡尔曼常量 ----
  static constexpr double kJumpResetDist = 120.0;
  static constexpr double kInheritDist = 120.0;
  static constexpr double kKalmanDtMax = 1.0;
  static constexpr double kObsSpeedMinDt = 0.01;
  static constexpr double kReverseProductGate = -50.0;
  static constexpr double kReverseMinObsSpeed = 30.0;
  static constexpr int kReverseCountGate = 2;
  static constexpr double kHoldAfterSelect = 0.12;
  static constexpr double kHoldAfterJump = 0.3;
  static constexpr double kHoldAfterReverse = 0.5;

  // ---- 焦点锁/选靶常量 ----
  static constexpr int kLockLostFrames = 10;
  static constexpr int kLockShotsMax = 12;
  static constexpr double kLockMatchRadius = 135.0;
  static constexpr double kBlacklistRadius = 70.0;
  static constexpr double kBlacklistMaxAge = 2.0;
  static constexpr double kAltDelay = 0.3;
  static constexpr double kAltDropDelay = 0.8;
  static constexpr double kAltMinDist = 90.0;
  static constexpr double kTargetAngleWeight = 0.01;
  static constexpr double kTargetLeaveWeight = 0.25;
  static constexpr double kTargetLeaveMax = 3.0;
  static constexpr double kTargetSpeedWeight = 0.008;
  static constexpr double kTargetSpeedMin = 5.0;
  static constexpr double kTrackMatchTol = 45.0;

 private:
  enum class Phase { kWaiting, kRunning };

  struct ShotMark {
    double x;
    double y;
    double t;
  };

  void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void TimerTick();
  void ProcessFrame(double now, double img_t);
  double TargetScore(double cx, double cy, double gx, double gy, int w, int h,
                     PlateKind enemy, double bullet_v) const;
  bool InBlacklist(double x, double y) const;

  // ---- 子部件 ----
  SerialTurret serial_;
  PlateDetector detector_;
  PlateTracker plate_trk_;
  LineGuard line_guard_;
  Kalman1DFilter kfx_;
  Kalman1DFilter kfy_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ---- 最新帧（单线程执行器，无需加锁）----
  cv::Mat latest_img_;
  double latest_t_ = 0.0;
  bool has_frame_ = false;

  // ---- 回合状态 ----
  Phase phase_ = Phase::kWaiting;
  bool done_ = false;
  double wait_start_ = 0.0;
  double last_rescan_ = 0.0;
  double last_reopen_ = 0.0;
  double last_hint_ = 0.0;
  std::string slave_;

  // ---- 作战状态 ----
  std::optional<PlateKind> self_color_;
  double last_fire_ = 0.0;
  std::optional<double> last_angle_;
  std::optional<double> last_t_;     // 卡尔曼上一帧图像时刻
  std::optional<double> prev_tx_;    // 上帧观测位置（折返方向检测）
  std::optional<double> prev_ty_;
  int reverse_count_ = 0;
  double fire_holduntil_ = 0.0;
  bool raw_in_ = true;
  double last_img_t_ = -1.0;  // 已处理图像时间戳（去重）
  double tx_ = 0.0;           // 上一帧锁定位（换锁继承判定用）
  double ty_ = 0.0;
  std::optional<std::pair<double, double>> locked_pl_;
  int lock_lost_ = 0;
  int lock_shots_ = 0;
  std::vector<ShotMark> shot_marks_;
  std::optional<double> block_since_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM_AIM_NODE_H_