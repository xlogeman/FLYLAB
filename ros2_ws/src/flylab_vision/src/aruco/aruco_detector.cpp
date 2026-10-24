// Copyright 2026 flylab
#include "aruco/aruco_detector.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <opencv2/calib3d.hpp>

namespace flylab_vision
{

namespace
{

// OpenCV 4.7+ 推荐用 ArucoDetector 类, 旧版 (Ubuntu 22.04 为 4.5.4) 只有全局函数
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#define FLYLAB_HAS_ARUCO_DETECTOR_CLASS 1
#endif

// 字典名 -> OpenCV 预定义字典枚举 (与 Python 版 make_dictionary 覆盖一致)
const std::map<std::string, int> & dict_table()
{
  static const std::map<std::string, int> table = {
    {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
    {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
    {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
    {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
    {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
    {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
    {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
    {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
    {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
    {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
    {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
    {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
    {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
    {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
    {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
    {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
    {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL},
    {"DICT_APRILTAG_16h5", cv::aruco::DICT_APRILTAG_16h5},
    {"DICT_APRILTAG_25h9", cv::aruco::DICT_APRILTAG_25h9},
    {"DICT_APRILTAG_36h10", cv::aruco::DICT_APRILTAG_36h10},
    {"DICT_APRILTAG_36h11", cv::aruco::DICT_APRILTAG_36h11},
  };
  return table;
}

cv::Mat intrinsics_from_fov(int w, int h, double fov_deg)
{
  const double fx = w / (2.0 * std::tan(fov_deg * CV_PI / 360.0));
  return (cv::Mat_<double>(3, 3) <<
          fx, 0.0, w / 2.0,
          0.0, fx, h / 2.0,
          0.0, 0.0, 1.0);
}

cv::Mat no_distortion()
{
  return cv::Mat::zeros(1, 5, CV_64F);
}

// 兼容两种 API 的角点检测
void detect_markers(
  const cv::Mat & bgr, const cv::Ptr<cv::aruco::Dictionary> & dictionary,
  const cv::Ptr<cv::aruco::DetectorParameters> & params,
  std::vector<std::vector<cv::Point2f>> & corners, std::vector<int> & ids)
{
#ifdef FLYLAB_HAS_ARUCO_DETECTOR_CLASS
  cv::aruco::ArucoDetector detector(dictionary, *params);
  detector.detectMarkers(bgr, corners, ids);
#else
  cv::aruco::detectMarkers(bgr, dictionary, corners, ids, params);
#endif
}

std::string num(double v, int prec = 1)
{
  std::ostringstream s;
  s << std::fixed << std::setprecision(prec) << v;
  return s.str();
}

}  // namespace

cv::Ptr<cv::aruco::Dictionary> make_dictionary(const std::string & name)
{
  const auto & table = dict_table();
  auto it = table.find(name);
  if (it == table.end()) {
    std::string all;
    for (const auto & kv : table) {
      all += kv.first + " ";
    }
    throw std::invalid_argument("未知字典 " + name + ", 可选: " + all);
  }
  return cv::aruco::getPredefinedDictionary(it->second);
}

void load_calib(const std::string & path, cv::Mat & K, cv::Mat & D)
{
  cv::FileStorage fs(path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    throw std::runtime_error("无法打开标定文件: " + path);
  }
  fs["camera_matrix"] >> K;
  fs["distortion_coefficients"] >> D;
  fs.release();
  if (K.empty()) {
    throw std::runtime_error(path + " 中未找到 camera_matrix");
  }
  if (D.empty()) {
    D = no_distortion();
  }
}

ArucoDetector::ArucoDetector(const DetectorConfig & config)
: config_(config)
{
  dictionary_ = make_dictionary(config_.dict_name);
  params_ = cv::aruco::DetectorParameters::create();
  params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
}

cv::Mat ArucoDetector::K_for(const cv::Mat & bgr) const
{
  if (!config_.K.empty()) {
    return config_.K;
  }
  return intrinsics_from_fov(bgr.cols, bgr.rows, config_.fov_deg);
}

std::vector<Detection> ArucoDetector::detect(const cv::Mat & bgr)
{
  std::vector<Detection> results;
  if (bgr.empty()) {
    return results;
  }
  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<int> ids;
  detect_markers(bgr, dictionary_, params_, corners, ids);
  if (ids.empty()) {
    return results;
  }

  const int w = bgr.cols, h = bgr.rows;
  const cv::Mat K = K_for(bgr);
  const cv::Mat D = config_.D.empty() ? no_distortion() : config_.D;
  const double half = config_.marker_length / 2.0;
  // IPPE_SQUARE: 物点顺序须与角点顺序 (左上/右上/右下/左下) 匹配,
  // marker 坐标系中心为原点, z 轴垂直码面朝外
  const cv::Mat objp = (cv::Mat_<cv::Point3f>(4, 1) <<
                        cv::Point3f(-half, half, 0), cv::Point3f(half, half, 0),
                        cv::Point3f(half, -half, 0), cv::Point3f(-half, -half, 0));

  for (size_t i = 0; i < ids.size(); ++i) {
    Detection d;
    d.id = ids[i];
    d.corners = corners[i];
    for (const auto & p : d.corners) {
      d.center_px += p;
    }
    d.center_px /= static_cast<float>(d.corners.size());
    d.center_norm = cv::Point2f(d.center_px.x / w, d.center_px.y / h);

    cv::Point2f lo(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    cv::Point2f hi(-lo.x, -lo.y);
    for (const auto & p : d.corners) {
      lo.x = std::min(lo.x, p.x);
      lo.y = std::min(lo.y, p.y);
      hi.x = std::max(hi.x, p.x);
      hi.y = std::max(hi.y, p.y);
    }
    d.bbox_wh = hi - lo;

    // 位姿解算: 等价 OpenCV legacy estimatePoseSingleMarkers
    cv::Mat corners_mat(4, 1, CV_32FC2);
    for (int k = 0; k < 4; ++k) {
      corners_mat.at<cv::Vec2f>(k) = cv::Vec2f(d.corners[k].x, d.corners[k].y);
    }
    cv::Mat rvec, tvec;
    if (cv::solvePnP(
        objp, corners_mat, K, D, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE))
    {
      d.rvec = cv::Vec3d(rvec);
      d.tvec = cv::Vec3d(tvec);
      d.has_pose = true;
      // 位姿轴端点投影到像素 (供可视化画 RGB 轴)
      const double a = config_.axis_len_m;
      cv::Mat pts(4, 1, CV_64FC3);
      pts.at<cv::Vec3d>(0) = {0, 0, 0};
      pts.at<cv::Vec3d>(1) = {a, 0, 0};
      pts.at<cv::Vec3d>(2) = {0, a, 0};
      pts.at<cv::Vec3d>(3) = {0, 0, a};
      cv::Mat proj;
      cv::projectPoints(pts, d.rvec, d.tvec, K, D, proj);
      for (int k = 0; k < 4; ++k) {
        d.axis_pts.emplace_back(
          static_cast<float>(proj.at<cv::Vec2d>(k)[0]),
          static_cast<float>(proj.at<cv::Vec2d>(k)[1]));
      }
    }

    results.push_back(d);
  }
  return results;
}

std::string to_annotations_json(
  const std::vector<Detection> & results, const std::string & algo)
{
  std::ostringstream os;
  os << "{\"algo\":\"" << algo << "\",\"items\":[";
  for (size_t i = 0; i < results.size(); ++i) {
    const auto & d = results[i];
    std::string label = "id=" + std::to_string(d.id);
    if (d.has_pose) {
      label += " z=" + num(d.tvec[2], 2) + "m";
    }
    float bx0 = std::numeric_limits<float>::max(), by0 = bx0;
    float bx1 = -bx0, by1 = -bx0;
    for (const auto & p : d.corners) {
      bx0 = std::min(bx0, p.x);
      by0 = std::min(by0, p.y);
      bx1 = std::max(bx1, p.x);
      by1 = std::max(by1, p.y);
    }
    // 字号与码框成比例: 文字像素高 = 外接框高 × 0.12, 夹取 [4, 20] 防退化
    const float box_h = std::max(by1 - by0, 1.0f);
    const float text_px = std::min(std::max(0.12f * box_h, 4.0f), 20.0f);
    if (i > 0) {
      os << ",";
    }
    // 四边形轮廓 (绿, 闭合): 沿角点连线, 画的是码的真实形状而非外接框
    os << "{\"type\":\"polyline\",\"points\":[";
    for (size_t k = 0; k < d.corners.size(); ++k) {
      os << (k ? "," : "") << "[" << num(d.corners[k].x) << ","
         << num(d.corners[k].y) << "]";
    }
    os << "],\"close\":true,\"color\":[0,255,0]}";
    // 首角点 (红): 标出码的正方向起点
    os << ",{\"type\":\"point\",\"point\":[" << num(d.corners[0].x) << ","
       << num(d.corners[0].y) << "],\"color\":[255,0,0]}";
    // 中心点 (红)
    os << ",{\"type\":\"point\",\"point\":[" << num(d.center_px.x) << ","
       << num(d.center_px.y) << "],\"color\":[255,0,0]}";
    // id 与距离 (黄, 挂在外接框左上角); px = 文字像素高, 随码框大小等比缩放
    os << ",{\"type\":\"text\",\"pos\":[" << num(bx0) << ","
       << num(by0 - 4.0f) << "],\"text\":\"" << label << "\","
       << "\"px\":" << num(text_px) << ",\"color\":[0,255,255]}";
    // 位姿轴 (x红/y绿/z蓝, OpenCV BGR 顺序): 3 条从原点出发的线段
    if (d.has_pose && d.axis_pts.size() == 4) {
      const cv::Point2f & o = d.axis_pts[0];
      const char * colors[3] = {"[0,0,255]", "[0,255,0]", "[255,0,0]"};
      for (int k = 1; k < 4; ++k) {
        os << ",{\"type\":\"polyline\",\"points\":[[" << num(o.x) << ","
           << num(o.y) << "],[" << num(d.axis_pts[k].x) << ","
           << num(d.axis_pts[k].y) << "]],\"color\":" << colors[k - 1] << "}";
      }
    }
  }
  os << "]}";
  return os.str();
}

}  // namespace flylab_vision
