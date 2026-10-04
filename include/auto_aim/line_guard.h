#ifndef AUTO_AIM_LINE_GUARD_H_
#define AUTO_AIM_LINE_GUARD_H_

#include <vector>

#include "auto_aim/plate_detector.h"

namespace auto_aim {

struct Track;

// 火线时间外推检查（相对运动法）：判断弹道在飞行途中是否被己方/中性板拦截。
class LineGuard {
 public:
  static constexpr double kMarginFriend = 60.0;   // 己方板拦截边距（px）
  static constexpr double kMarginNeutral = 30.0;  // 中性板拦截边距（px）
  static constexpr double kScreenWidth = 1152.0;
  static constexpr double kScreenHeight = 648.0;
  static constexpr double kOverFlyTime = 0.35;      // 无出屏方向时的检查余量（s）
  static constexpr double kMinBulletSpeed = 1.0;
  static constexpr double kEpsDist = 1e-3;
  static constexpr double kEpsVel = 1e-6;
  static constexpr double kEpsRelSpeed2 = 1e-6;

  LineGuard() = default;

  // 返回挡住弹道的第一块板航迹指针，无则 nullptr。
  const Track* FirstBlocker(double gx, double gy, double tx, double ty,
                            const std::vector<Track>& tracks,
                            PlateKind self_color, double bullet_v,
                            double margin = kMarginFriend,
                            double none_margin = kMarginNeutral) const;

  bool LineBlocked(double gx, double gy, double tx, double ty,
                   const std::vector<Track>& tracks, PlateKind self_color,
                   double bullet_v, double margin = kMarginFriend,
                   double none_margin = kMarginNeutral) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM_LINE_GUARD_H_