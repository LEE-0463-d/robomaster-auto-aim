#include "auto_aim/aim_node.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <thread>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace auto_aim {

namespace {

constexpr double kPi = 3.14159265358979323846;

double NowSeconds() {
  using Clock = std::chrono::steady_clock;
  static const Clock::time_point kStart = Clock::now();
  return std::chrono::duration<double>(Clock::now() - kStart).count();
}

}  // namespace

AimNode::AimNode(const std::string& slave) : Node("auto_aim"), slave_(slave) {
  // 订阅 QoS 与发布端一致（RELIABLE），用默认值即可。
  sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", 10,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        ImageCallback(msg);
      });
  timer_ = create_wall_timer(std::chrono::milliseconds(5),
                             [this]() { TimerTick(); });

  wait_start_ = NowSeconds();
  last_rescan_ = wait_start_;
  last_hint_ = wait_start_;

  RCLCPP_INFO(get_logger(), "SLAVE=%s", slave_.c_str());
  if (serial_.Open(slave_)) {
    RCLCPP_INFO(get_logger(),
                "已连上串口，等待对局开始（请点击「开始游戏」）");
  } else {
    RCLCPP_ERROR(get_logger(), "打开串口失败：%s", slave_.c_str());
  }
}

void AimNode::ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
  // 图像可能是 rgb8 或 bgr8 编码，统一转成 BGR 再处理。
  if (msg->width <= 0 || msg->height <= 0 || msg->data.empty()) {
    return;
  }
  if (msg->encoding == "bgr8") {
    latest_img_ = cv::Mat(msg->height, msg->width, CV_8UC3,
                          msg->data.data(), msg->step)
                      .clone();
  } else if (msg->encoding == "rgb8") {
    const cv::Mat rgb(msg->height, msg->width, CV_8UC3, msg->data.data(),
                      msg->step);
    cv::cvtColor(rgb, latest_img_, cv::COLOR_RGB2BGR);
  } else {
    return;
  }
  latest_t_ = NowSeconds();
  has_frame_ = true;
}

void AimNode::TimerTick() {
  if (done_) {
    return;
  }
  const double now = NowSeconds();
  if (phase_ == Phase::kWaiting) {
    if (!has_frame_) {
      if (now - wait_start_ > kFirstFrameTimeout) {
        RCLCPP_ERROR(get_logger(), "等待超时：21600 秒未收到图像");
        serial_.Close();
        done_ = true;
        rclcpp::shutdown();
        return;
      }
      if (now - last_rescan_ > kRescanInterval) {
        last_rescan_ = now;
        const std::string slave2 = SerialTurret::FindSlave();
        // 同路径也定期重开：pts 编号会被复用，路径相同不代表设备相同。
        const bool same = (slave2 == slave_);
        if (!slave2.empty() &&
            (!same || now - last_reopen_ > kReopenInterval)) {
          if (serial_.Open(slave2)) {
            if (!same) {
              RCLCPP_INFO(get_logger(), "等待中检测到新串口 %s，切换...",
                          slave2.c_str());
            }
            slave_ = slave2;
            last_reopen_ = now;
          }
        }
      }
      if (now - last_hint_ > kWaitHintInterval) {
        last_hint_ = now;
        RCLCPP_INFO(get_logger(), "  ...仍在等待图像");
      }
      return;
    }
    // 开赛后游戏会为本局新建串口，再切一次。
    const std::string slave2 = SerialTurret::FindSlave();
    if (!slave2.empty() && slave2 != slave_) {
      if (serial_.Open(slave2)) {
        RCLCPP_INFO(get_logger(), "检测到更新的一局串口 %s，切换...",
                    slave2.c_str());
        slave_ = slave2;
      }
    }
    phase_ = Phase::kRunning;
    RCLCPP_INFO(get_logger(), "收到图像，对局进行中");
    RCLCPP_INFO(get_logger(), "自动瞄准已启动，持续控制炮台...");
  }

  if (!has_frame_ || now - latest_t_ > kImageStopTimeout) {
    RCLCPP_INFO(get_logger(), "图像停止，对局结束");
    serial_.Close();
    done_ = true;
    rclcpp::shutdown();
    return;
  }
  if (latest_t_ == last_img_t_) {
    return;  // 去重：仅在新帧到来时处理一次
  }
  last_img_t_ = latest_t_;
  ProcessFrame(now, latest_t_);
}

void AimNode::ProcessFrame(double now, double img_t) {
  if (latest_img_.empty()) {
    return;
  }
  const cv::Mat& img = latest_img_;

  // 己方颜色判定：只在未知时检测，已定后不再更新。
  if (!self_color_.has_value()) {
    const std::optional<PlateKind> c = detector_.DetectSelf(img);
    if (c.has_value()) {
      self_color_ = c;
      RCLCPP_INFO(get_logger(), "己方=%s → 打击 %s",
                  (*c == PlateKind::kRed) ? "RED" : "BLUE",
                  (*c == PlateKind::kRed) ? "BLUE" : "RED");
    }
    return;
  }

  const PlateKind enemy =
      (*self_color_ == PlateKind::kRed) ? PlateKind::kBlue : PlateKind::kRed;
  const int w = img.cols;
  const int h = img.rows;
  const double gx = w * kGunPivotXRatio;
  const double gy = h * kGunPivotYRatio;

  const std::vector<DetectedPlate> plates = detector_.DetectPlates(img);
  plate_trk_.Update(plates, img_t);
  const double bullet_v = kBulletSpeedPxS;

  // ---- 焦点锁：锁定一块敌板打到消失 ----
  shot_marks_.erase(
      std::remove_if(shot_marks_.begin(), shot_marks_.end(),
                     [now](const ShotMark& m) {
                       return now - m.t >= kBlacklistMaxAge;
                     }),
      shot_marks_.end());

  if (locked_pl_.has_value()) {
    double best_d = kLockMatchRadius;
    bool found = false;
    std::pair<double, double> nxt;
    for (const DetectedPlate& p : plates) {
      if (p.kind != enemy) {
        continue;
      }
      const double d =
          std::hypot(p.cx - locked_pl_->first, p.cy - locked_pl_->second);
      if (d < best_d) {
        best_d = d;
        nxt = std::make_pair(p.cx, p.cy);
        found = true;
      }
    }
    if (found) {
      locked_pl_ = nxt;
      lock_lost_ = 0;
    } else {
      ++lock_lost_;
      if (lock_lost_ > kLockLostFrames) {
        if (lock_shots_ >= kLockShotsMax) {
          shot_marks_.push_back(
              ShotMark{locked_pl_->first, locked_pl_->second, now});
        }
        locked_pl_.reset();
        lock_shots_ = 0;
      } else {
        return;  // 检测闪烁宽限：本帧不动作
      }
    }
  }

  if (!locked_pl_.has_value()) {
    std::vector<std::pair<double, double>> cands;
    for (const DetectedPlate& p : plates) {
      if (p.kind != enemy) {
        continue;
      }
      if (InBlacklist(p.cx, p.cy)) {
        continue;
      }
      cands.emplace_back(p.cx, p.cy);
    }
    if (cands.empty()) {
      return;  // 场上无有效敌板
    }
    std::vector<std::pair<double, double>> clear;
    for (const auto& c : cands) {
      if (!line_guard_.LineBlocked(gx, gy, c.first, c.second,
                                   plate_trk_.tracks(), *self_color_,
                                   bullet_v)) {
        clear.push_back(c);
      }
    }
    const std::vector<std::pair<double, double>>& pool =
        clear.empty() ? cands : clear;
    std::size_t best_i = 0;
    double best_score = TargetScore(pool[0].first, pool[0].second, gx, gy, w,
                                    h, enemy, bullet_v);
    for (std::size_t i = 1; i < pool.size(); ++i) {
      const double s = TargetScore(pool[i].first, pool[i].second, gx, gy, w, h,
                                   enemy, bullet_v);
      if (s < best_score) {
        best_score = s;
        best_i = i;
      }
    }
    locked_pl_ = pool[best_i];
    lock_lost_ = 0;
    lock_shots_ = 0;
    // 换锁时目标离旧滤波器位置很近则继承速度估计，否则重置。
    const double ox = kfx_.pos();
    const double oy = kfy_.pos();
    if (kfx_.initialized() && kfy_.initialized() &&
        std::hypot(tx_ - ox, ty_ - oy) < kInheritDist) {
      kfx_.set_pos(tx_);
      kfy_.set_pos(ty_);
    } else {
      kfx_.Reset();
      kfy_.Reset();
    }
    prev_tx_.reset();
    prev_ty_.reset();
    fire_holduntil_ = now + kHoldAfterSelect;
  }
  tx_ = locked_pl_->first;
  ty_ = locked_pl_->second;

  // ---- 瞄准：卡尔曼预测 + 观测校正 + 超前外推 ----
  double px;
  double py;
  if (!last_t_.has_value()) {
    last_t_ = img_t;
    kfx_.Update(tx_);
    kfy_.Update(ty_);
    px = tx_;
    py = ty_;
  } else {
    const double dt = std::clamp(img_t - *last_t_, 0.0, kKalmanDtMax);
    last_t_ = img_t;
    const double jump = std::hypot(tx_ - kfx_.pos(), ty_ - kfy_.pos());
    if (jump > kJumpResetDist) {
      kfx_.Reset();
      kfy_.Reset();
      kfx_.Update(tx_);
      kfy_.Update(ty_);
      prev_tx_.reset();
      prev_ty_.reset();
      reverse_count_ = 0;
      px = tx_;
      py = ty_;
      fire_holduntil_ = now + kHoldAfterJump;
    } else {
      kfx_.Predict(dt);
      kfy_.Predict(dt);
      kfx_.Update(tx_);
      kfy_.Update(ty_);
      // 折返方向检测：观测速度与卡尔曼估计速度方向相反且量级够大。
      if (prev_tx_.has_value() && dt > kObsSpeedMinDt) {
        const double obs_vx = (tx_ - *prev_tx_) / dt;
        const double obs_vy = (ty_ - *prev_ty_) / dt;
        const bool rev_x = obs_vx * kfx_.vel() < kReverseProductGate &&
                           std::fabs(obs_vx) > kReverseMinObsSpeed;
        const bool rev_y = obs_vy * kfy_.vel() < kReverseProductGate &&
                           std::fabs(obs_vy) > kReverseMinObsSpeed;
        if (rev_x || rev_y) {
          ++reverse_count_;
          if (reverse_count_ >= kReverseCountGate) {
            fire_holduntil_ = now + kHoldAfterReverse;
          }
        } else {
          reverse_count_ = std::max(0, reverse_count_ - 1);
        }
      }
      prev_tx_ = tx_;
      prev_ty_ = ty_;
      const double dist = std::hypot(kfx_.pos() - gx, kfy_.pos() - gy);
      const double flight = dist / bullet_v;
      const double comp = flight + kCompLead;
      if (std::hypot(kfx_.vel(), kfy_.vel()) < kStaticSpeed) {
        px = tx_;
        py = ty_;
        raw_in_ = true;
      } else {
        px = tx_ + kfx_.vel() * comp;
        py = ty_ + kfy_.vel() * comp;
        raw_in_ = (0.0 <= px && px <= static_cast<double>(w)) &&
                  (0.0 <= py && py <= static_cast<double>(h));
        px = std::clamp(px, 0.0, static_cast<double>(w));
        py = std::clamp(py, 0.0, static_cast<double>(h));
      }
    }
  }
  double ang = std::atan2(gy - py, px - gx) * 180.0 / kPi;
  ang = std::clamp(ang, -180.0, 180.0);

  // ---- 火线检查与遮挡处理 ----
  const bool line_ok = !line_guard_.LineBlocked(
      gx, gy, px, py, plate_trk_.tracks(), *self_color_, bullet_v);
  if (!line_ok) {
    if (!block_since_.has_value()) {
      block_since_ = now;
    }
    if (now - *block_since_ > kAltDelay) {
      std::optional<std::pair<double, double>> alt;
      for (const DetectedPlate& p : plates) {
        if (p.kind != enemy) {
          continue;
        }
        if (std::hypot(p.cx - tx_, p.cy - ty_) < kAltMinDist) {
          continue;
        }
        if (InBlacklist(p.cx, p.cy)) {
          continue;
        }
        if (line_guard_.LineBlocked(gx, gy, p.cx, p.cy, plate_trk_.tracks(),
                                    *self_color_, bullet_v)) {
          continue;
        }
        alt = std::make_pair(p.cx, p.cy);
        break;
      }
      if (alt.has_value()) {
        locked_pl_ = *alt;
        lock_lost_ = 0;
        lock_shots_ = 0;
        block_since_.reset();
        kfx_.Reset();
        kfy_.Reset();
        prev_tx_.reset();
        prev_ty_.reset();
        fire_holduntil_ = now + kHoldAfterSelect;
        return;  // 换畅通目标，本帧不发
      }
    }
    if (now - *block_since_ > kAltDropDelay) {
      locked_pl_.reset();
      lock_shots_ = 0;
      block_since_.reset();
      return;  // 持续被挡且无替代：弃锁，本帧不发
    }
  } else {
    block_since_.reset();
  }

  // ---- 开火门 ----
  const double flight_aim =
      std::hypot(px - gx, py - gy) / std::max(bullet_v, 1.0);
  const bool fire_due =
      last_angle_.has_value() && raw_in_ && line_ok &&
      flight_aim <= kFlightAimMax &&
      std::fabs(ang - *last_angle_) < kAngleGateDeg &&
      now - last_fire_ > kFireInterval && now >= fire_holduntil_;
  if (fire_due) {
    serial_.SendFire();
    ++lock_shots_;
    last_fire_ = now;
    // 等游戏读线程吞下开火字节，再发下一包。
    std::this_thread::sleep_for(std::chrono::milliseconds(kFireDrainSleepMs));
  } else {
    serial_.SendTurn(static_cast<float>(ang));
  }
  last_angle_ = ang;
}

double AimNode::TargetScore(double cx, double cy, double gx, double gy, int w,
                            int h, PlateKind enemy, double bullet_v) const {
  const double d = std::hypot(cx - gx, cy - gy);
  const double flight = d / std::max(bullet_v, 1.0);
  const double ta = std::atan2(gy - cy, cx - gx) * 180.0 / kPi;
  double adiff =
      last_angle_.has_value() ? std::fabs(ta - *last_angle_) : 0.0;
  if (adiff > 180.0) {
    adiff = 360.0 - adiff;
  }
  double t_leave = kTargetLeaveMax;
  double sp = 0.0;
  for (const Track& tr : plate_trk_.tracks()) {
    if (tr.kind != enemy) {
      continue;
    }
    if (std::fabs(tr.cx - cx) < kTrackMatchTol &&
        std::fabs(tr.cy - cy) < kTrackMatchTol) {
      const double vx = tr.vx;
      const double vy = tr.vy;
      sp = std::hypot(vx, vy);
      if (sp > kTargetSpeedMin) {
        std::vector<double> edges;
        if (vx < 0.0) {
          edges.push_back(-cx / vx);
        } else if (vx > 0.0) {
          edges.push_back((w - cx) / vx);
        }
        if (vy < 0.0) {
          edges.push_back(-cy / vy);
        } else if (vy > 0.0) {
          edges.push_back((h - cy) / vy);
        }
        double min_pe = 0.0;
        bool has_pe = false;
        for (const double e : edges) {
          if (e > 0.0 && (!has_pe || e < min_pe)) {
            min_pe = e;
            has_pe = true;
          }
        }
        if (has_pe) {
          t_leave = std::max(0.0, std::min(min_pe, kTargetLeaveMax));
        }
      }
      break;
    }
  }
  return flight + adiff * kTargetAngleWeight +
         (kTargetLeaveMax - t_leave) * kTargetLeaveWeight +
         sp * kTargetSpeedWeight;
}

bool AimNode::InBlacklist(double x, double y) const {
  const double r2 = kBlacklistRadius * kBlacklistRadius;
  for (const ShotMark& m : shot_marks_) {
    const double dx = x - m.x;
    const double dy = y - m.y;
    if (dx * dx + dy * dy < r2) {
      return true;
    }
  }
  return false;
}

}  // namespace auto_aim