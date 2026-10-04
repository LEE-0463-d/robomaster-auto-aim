#include "auto_aim/plate_tracker.h"

#include <cmath>
#include <cstddef>

namespace auto_aim {

PlateTracker::PlateTracker(double max_age, double match_r)
    : max_age_(max_age), match_r_(match_r) {}

void PlateTracker::Update(const std::vector<DetectedPlate>& plates,
                          double now) {
  const std::size_t n_det = plates.size();
  std::vector<bool> used(n_det, false);
  for (Track& tr : tracks_) {
    const double dt = now - tr.pos_t;
    const double px = tr.cx + tr.vx * dt;  // 滑行预测位置
    const double py = tr.cy + tr.vy * dt;
    double bd = match_r_;
    std::size_t best_i = n_det;
    for (std::size_t i = 0; i < n_det; ++i) {
      if (used[i]) {
        continue;
      }
      const double d = std::hypot(plates[i].cx - px, plates[i].cy - py);
      if (d < bd) {
        bd = d;
        best_i = i;
      }
    }
    if (best_i != n_det) {
      used[best_i] = true;
      const DetectedPlate& np = plates[best_i];
      if (dt > kMinVelDt) {
        tr.vx = kVelocityEma * tr.vx + kVelocityEma * (np.cx - tr.cx) / dt;
        tr.vy = kVelocityEma * tr.vy + kVelocityEma * (np.cy - tr.cy) / dt;
        const double sp = std::hypot(tr.vx, tr.vy);
        if (sp > kMaxPlateSpeed) {
          tr.vx *= kMaxPlateSpeed / sp;
          tr.vy *= kMaxPlateSpeed / sp;
        }
      }
      tr.cx = np.cx;
      tr.cy = np.cy;
      tr.pos_t = now;
      tr.kind = np.kind;
      tr.seen_t = now;
    } else {
      tr.cx = px;  // 未匹配：沿速度滑行
      tr.cy = py;
      tr.pos_t = now;
    }
  }
  for (std::size_t i = 0; i < n_det; ++i) {
    if (!used[i]) {
      Track t;
      t.cx = plates[i].cx;
      t.cy = plates[i].cy;
      t.vx = 0.0;
      t.vy = 0.0;
      t.pos_t = now;
      t.kind = plates[i].kind;
      t.seen_t = now;
      t.hits = 0;
      t.id = ++next_id_;
      tracks_.push_back(t);
    }
  }
  auto it = tracks_.begin();
  while (it != tracks_.end()) {
    if (now - it->seen_t >= max_age_) {
      it = tracks_.erase(it);
    } else {
      ++it;
    }
  }
}

}  // namespace auto_aim