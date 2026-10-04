#include "auto_aim/plate_detector.h"

#include <algorithm>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace auto_aim {

namespace {

// 己方颜色判定用的 HSV 阈值。
const cv::Scalar kRedH1Low(0, 100, 30);
const cv::Scalar kRedH1High(12, 255, 255);
const cv::Scalar kRedH2Low(160, 100, 30);
const cv::Scalar kRedH2High(180, 255, 255);
const cv::Scalar kBlueHueLow(98, 100, 30);
const cv::Scalar kBlueHueHigh(118, 255, 255);

// 条带分类用的严格阈值（排除未激活的浅色条带）。
const cv::Scalar kStripRed1Low(0, 120, 120);
const cv::Scalar kStripRed1High(12, 255, 255);
const cv::Scalar kStripRed2Low(160, 120, 120);
const cv::Scalar kStripRed2High(180, 255, 255);
const cv::Scalar kStripBlueLow(98, 120, 120);
const cv::Scalar kStripBlueHigh(118, 255, 255);

}  // namespace

std::optional<PlateKind> PlateDetector::DetectSelf(const cv::Mat& bgr,
                                                   int min_px) const {
  const int h = bgr.rows;
  const int w = bgr.cols;
  const int y0 = static_cast<int>(h * kSelfRoiTopRatio);
  const int x0 = static_cast<int>(w * kSelfRoiLeftRatio);
  const int x1 = static_cast<int>(w * kSelfRoiRightRatio);
  if (y0 < 0 || y0 >= h || x0 < 0 || x0 >= x1 || x1 > w) {
    return std::nullopt;
  }
  const cv::Mat roi = bgr(cv::Rect(x0, y0, x1 - x0, h - y0));
  const int r = CountColorMask(roi, PlateKind::kRed);
  const int b = CountColorMask(roi, PlateKind::kBlue);
  if (r < min_px && b < min_px) {
    return std::nullopt;
  }
  return (r >= b) ? PlateKind::kRed : PlateKind::kBlue;
}

std::vector<DetectedPlate> PlateDetector::DetectPlates(
    const cv::Mat& bgr) const {
  std::vector<DetectedPlate> out;
  const int h = bgr.rows;
  const int w = bgr.cols;
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
  std::vector<cv::Mat> channels;
  cv::split(hsv, channels);
  cv::Mat black;
  cv::compare(channels[2], cv::Scalar(kPlateMaxValue), black, cv::CMP_LT);
  const cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size(kMorphKernelSize, kMorphKernelSize));
  cv::morphologyEx(black, black, cv::MORPH_CLOSE, kernel);
  cv::morphologyEx(black, black, cv::MORPH_OPEN, kernel);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(black, contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);
  for (const auto& c : contours) {
    if (cv::contourArea(c) < kMinContourArea) {
      continue;
    }
    const cv::Rect br = cv::boundingRect(c);
    const int bw = br.width;
    const int bh = br.height;
    if (bw < kMinPlateWidth || bh < kMinPlateHeight || bw > kMaxPlateWidth) {
      continue;
    }
    const double ar = static_cast<double>(bw) / bh;
    if (ar < kMinAspectRatio || ar > kMaxAspectRatio) {
      continue;
    }
    const int zw =
        std::max(kMinStripWidth, static_cast<int>(bw * kStripWidthRatio));
    const int y0 = std::max(0, br.y - kStripYPad);
    const int y1 = std::min(h, br.y + bh + kStripYPad);
    const int zx0 = std::max(0, br.x - kStripXPad);
    const int zx1 =
        std::min(std::max(0, w - zw), br.x + bw - zw + kStripXPad);
    int red_n = 0;
    int blue_n = 0;
    for (const int zx : {zx0, zx1}) {
      const int zx_end = std::min(w, zx + zw);
      if (zx_end <= zx || y1 <= y0) {
        continue;  // 采样区域无效
      }
      const cv::Mat z = hsv(cv::Range(y0, y1), cv::Range(zx, zx_end));
      red_n += CountStripMask(z, PlateKind::kRed);
      blue_n += CountStripMask(z, PlateKind::kBlue);
    }
    PlateKind kind = PlateKind::kNone;
    if (red_n >= kMinStripPixels &&
        red_n > blue_n * kStripDominanceRatio) {
      kind = PlateKind::kRed;
    } else if (blue_n >= kMinStripPixels &&
               blue_n > red_n * kStripDominanceRatio) {
      kind = PlateKind::kBlue;
    }
    DetectedPlate p;
    p.cx = br.x + bw / 2.0;
    p.cy = br.y + bh / 2.0;
    p.kind = kind;
    out.push_back(p);
  }
  return out;
}

int PlateDetector::CountColorMask(const cv::Mat& bgr, PlateKind color) const {
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
  if (color == PlateKind::kRed) {
    cv::Mat m1;
    cv::Mat m2;
    cv::inRange(hsv, kRedH1Low, kRedH1High, m1);
    cv::inRange(hsv, kRedH2Low, kRedH2High, m2);
    return cv::countNonZero(m1 | m2);
  }
  cv::Mat m;
  cv::inRange(hsv, kBlueHueLow, kBlueHueHigh, m);
  return cv::countNonZero(m);
}

int PlateDetector::CountStripMask(const cv::Mat& hsv, PlateKind color) const {
  if (color == PlateKind::kRed) {
    cv::Mat m1;
    cv::Mat m2;
    cv::inRange(hsv, kStripRed1Low, kStripRed1High, m1);
    cv::inRange(hsv, kStripRed2Low, kStripRed2High, m2);
    return cv::countNonZero(m1 | m2);
  }
  cv::Mat m;
  cv::inRange(hsv, kStripBlueLow, kStripBlueHigh, m);
  return cv::countNonZero(m);
}

}  // namespace auto_aim