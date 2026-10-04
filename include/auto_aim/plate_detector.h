#ifndef AUTO_AIM_PLATE_DETECTOR_H_
#define AUTO_AIM_PLATE_DETECTOR_H_

#include <cstdint>
#include <optional>
#include <vector>

#include <opencv2/core.hpp>

namespace auto_aim {

// 板条颜色分类。
enum class PlateKind {
  kNone = 0,  // 未激活（米白）
  kRed = 1,   // 红条带
  kBlue = 2,  // 蓝条带
};

// 一块检出的装甲板：中心点 (cx, cy) 与条带分类。
struct DetectedPlate {
  double cx = 0.0;
  double cy = 0.0;
  PlateKind kind = PlateKind::kNone;
};

// 板检测器：黑色板体找矩形，两端条带分类红/蓝/未激活；己方颜色判定。
class PlateDetector {
 public:
  // ---- 己方颜色判定阈值 ----
  static constexpr int kSelfMinPixels = 50;
  static constexpr double kSelfRoiTopRatio = 0.8;
  static constexpr double kSelfRoiLeftRatio = 0.35;
  static constexpr double kSelfRoiRightRatio = 0.65;

  // ---- 板体检测阈值 ----
  static constexpr int kPlateMaxValue = 70;   // V < 70 判为黑色板体
  static constexpr int kMorphKernelSize = 9;
  static constexpr int kMinContourArea = 800;
  static constexpr int kMinPlateWidth = 30;
  static constexpr int kMinPlateHeight = 15;
  static constexpr int kMaxPlateWidth = 200;
  static constexpr double kMinAspectRatio = 1.1;
  static constexpr double kMaxAspectRatio = 4.5;

  // ---- 条带采样与分类阈值 ----
  static constexpr double kStripWidthRatio = 0.25;
  static constexpr int kMinStripWidth = 4;
  static constexpr int kStripYPad = 3;
  static constexpr int kStripXPad = 2;
  static constexpr int kMinStripPixels = 15;
  static constexpr double kStripDominanceRatio = 1.5;

  // 判定己方颜色（画面下 20%、水平中央 35%-65% ROI）。未知返回 nullopt。
  std::optional<PlateKind> DetectSelf(const cv::Mat& bgr,
                                      int min_px = kSelfMinPixels) const;

  // 检测整板，返回 [(cx, cy, kind)]。
  std::vector<DetectedPlate> DetectPlates(const cv::Mat& bgr) const;

 private:
  // 按己方/敌板颜色（宽松阈值）统计 BGR 图掩膜像素。
  int CountColorMask(const cv::Mat& bgr, PlateKind color) const;
  // 按条带严格阈值统计 HSV 图掩膜像素。
  int CountStripMask(const cv::Mat& hsv, PlateKind color) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM_PLATE_DETECTOR_H_