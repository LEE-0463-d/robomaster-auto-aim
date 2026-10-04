#include "auto_aim/line_guard.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "auto_aim/plate_tracker.h"

namespace auto_aim {

const Track* LineGuard::FirstBlocker(double gx, double gy, double tx,
                                     double ty,
                                     const std::vector<Track>& tracks,
                                     PlateKind self_color, double bullet_v,
                                     double margin,
                                     double none_margin) const {
  const double dx = tx - gx;
  const double dy = ty - gy;
  const double length = std::hypot(dx, dy);
  if (length < kEpsDist || bullet_v <= kMinBulletSpeed) {
    return nullptr;
  }
  const double ux = dx / length * bullet_v;
  const double uy = dy / length * bullet_v;

  // 检查窗口：子弹沿弹道飞出屏幕的完整时间。
  std::vector<double> t_edges;
  if (std::fabs(ux) > kEpsVel) {
    t_edges.push_back(ux < 0.0 ? (0.0 - gx) / ux
                               : (kScreenWidth - gx) / ux);
  }
  if (std::fabs(uy) > kEpsVel) {
    t_edges.push_back(uy < 0.0 ? (0.0 - gy) / uy
                               : (kScreenHeight - gy) / uy);
  }
  double window_t = length / bullet_v + kOverFlyTime;
  bool has_positive = false;
  double min_positive = 0.0;
  for (const double t : t_edges) {
    if (t > 0.0 && (!has_positive || t < min_positive)) {
      min_positive = t;
      has_positive = true;
    }
  }
  if (has_positive) {
    window_t = min_positive;
  }

  for (const Track& tr : tracks) {
    const double cx = tr.cx;
    const double cy = tr.cy;
    const double vx = tr.vx;
    const double vy = tr.vy;
    double m;
    bool has_reverse;
    if (tr.kind == PlateKind::kNone) {
      m = none_margin;
      has_reverse = false;
    } else if (tr.kind == self_color) {
      m = margin;
      has_reverse = true;  // 己方板额外考虑折返
    } else {
      continue;  // 敌板不拦
    }
    const double rx = gx - cx;
    const double ry = gy - cy;

    // 板此刻已在弹道上时相对运动法会漏判，这里单独查一次垂距。
    const double nx = dx / length;
    const double ny = dy / length;
    const double perp = std::fabs(nx * (cy - gy) - ny * (cx - gx));
    const double along = nx * (cx - gx) + ny * (cy - gy);
    if (along > 0.0 && perp < m) {
      return &tr;
    }

    const int sign_count = has_reverse ? 2 : 1;
    for (int si = 0; si < sign_count; ++si) {
      const double sign = (si == 0) ? 1.0 : -1.0;
      const double sx = sign * vx;
      const double sy = sign * vy;
      const double wx = ux - sx;
      const double wy = uy - sy;
      const double w2 = wx * wx + wy * wy;
      double t = 0.0;
      if (w2 >= kEpsRelSpeed2) {
        t = -(rx * wx + ry * wy) / w2;
      }
      t = std::clamp(t, 0.0, window_t);
      if (std::hypot(rx + wx * t, ry + wy * t) < m) {
        return &tr;
      }
    }
  }
  return nullptr;
}

bool LineGuard::LineBlocked(double gx, double gy, double tx, double ty,
                            const std::vector<Track>& tracks,
                            PlateKind self_color, double bullet_v,
                            double margin, double none_margin) const {
  return FirstBlocker(gx, gy, tx, ty, tracks, self_color, bullet_v, margin,
                      none_margin) != nullptr;
}

}  // namespace auto_aim