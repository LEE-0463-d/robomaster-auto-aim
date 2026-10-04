#ifndef AUTO_AIM_PLATE_TRACKER_H_
#define AUTO_AIM_PLATE_TRACKER_H_

#include <vector>

#include "auto_aim/plate_detector.h"

namespace auto_aim {

// 单块装甲板的航迹。
struct Track {
  double cx = 0.0;
  double cy = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double pos_t = 0.0;
  PlateKind kind = PlateKind::kNone;
  double seen_t = 0.0;
  int hits = 0;
  int id = 0;
};

// 装甲板跟踪：帧间近邻匹配 + 滑行预测。
class PlateTracker {
 public:
  static constexpr double kMaxAge = 0.5;          // 0.5s 未见则移除
  static constexpr double kMatchRadius = 70.0;    // 帧间匹配半径（px）
  static constexpr double kVelocityEma = 0.5;     // 速度 EMA 系数
  static constexpr double kMaxPlateSpeed = 400.0; // 板速上限（px/s）
  static constexpr double kMinVelDt = 1e-3;       // 更新速度的最小 dt

  PlateTracker(double max_age = kMaxAge, double match_r = kMatchRadius);

  void Update(const std::vector<DetectedPlate>& plates, double now);

  const std::vector<Track>& tracks() const { return tracks_; }

 private:
  double max_age_;
  double match_r_;
  std::vector<Track> tracks_;
  int next_id_ = 0;
};

}  // namespace auto_aim

#endif  // AUTO_AIM_PLATE_TRACKER_H_