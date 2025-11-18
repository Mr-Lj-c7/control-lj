
#include "trajectory_analyzer.h"

namespace control {
TrajectoryAnalyzer::TrajectoryAnalyzer(
    const common_msg::ADCTrajectory *planning_published_trajectory) {
    header_time_ = planning_published_trajectory->header.timestamp_sec;
    seq_num_ = planning_published_trajectory->header.sequence_num;

    for (int i = 0; i < planning_published_trajectory->trajectory_point.size();
        ++i) {
        trajectory_points_.push_back(
        planning_published_trajectory->trajectory_point[i]);
        auto &path_point = trajectory_points_[i].path_point;
        if (path_point.z == 0.0) {
        path_point.z = 0.0;
        }
    }
}

void TrajectoryAnalyzer::ToTrajectoryFrame(const double x, const double y,
                                           const double theta, const double v,
                                           const common_msg::PathPoint &ref_point,
                                           double *ptr_s, double *ptr_s_dot,
                                           double *ptr_d,
                                           double *ptr_d_dot) const {
  double dx = x - ref_point.x;
  double dy = y - ref_point.y;

  double cos_ref_theta = std::cos(ref_point.theta);
  double sin_ref_theta = std::sin(ref_point.theta);

  // the sin of diff angle between vector (cos_ref_theta, sin_ref_theta) and
  // (dx, dy)
  double cross_rd_nd = cos_ref_theta * dy - sin_ref_theta * dx;
  *ptr_d = cross_rd_nd;

  // the cos of diff angle between vector (cos_ref_theta, sin_ref_theta) and
  // (dx, dy)
  double dot_rd_nd = dx * cos_ref_theta + dy * sin_ref_theta;
  *ptr_s = ref_point.s + dot_rd_nd;

  double delta_theta = theta - ref_point.theta;
  double cos_delta_theta = std::cos(delta_theta);
  double sin_delta_theta = std::sin(delta_theta);

  *ptr_d_dot = v * sin_delta_theta;
  // 曲率修正因子
  double one_minus_kappa_r_d = 1 - ref_point.kappa * (*ptr_d);
  if (one_minus_kappa_r_d <= 0.0) {
    std::cerr << "TrajectoryAnalyzer::ToTrajectoryFrame "
              "found fatal reference and actual difference. "
              "Control output might be unstable:"
           << " ref_point.kappa:" << ref_point.kappa
           << " ref_point.x:" << ref_point.x
           << " ref_point.y:" << ref_point.y << " car x:" << x
           << " car y:" << y << " *ptr_d:" << *ptr_d
           << " one_minus_kappa_r_d:" << one_minus_kappa_r_d;
    // currently set to a small value to avoid control crash.
    one_minus_kappa_r_d = 0.01;
  }

  *ptr_s_dot = v * cos_delta_theta / one_minus_kappa_r_d;
}

common_msg::TrajectoryPoint TrajectoryAnalyzer::QueryNearestPointByPosition(
    const double x, const double y) 
{
    double d_min = PointDistanceSquare(trajectory_points_.front(), x, y);  // 起点
    size_t index_min = 0;
    for (size_t i = 0; i < trajectory_points_.size(); ++i) {
        double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
        if (d_temp < d_min) {
            d_min = d_temp;
            index_min = i;
        }
    }
    return trajectory_points_[index_min];
}

double TrajectoryAnalyzer::PointDistanceSquare(
    const common_msg::TrajectoryPoint &point, 
    const double x, const double y)
{
    const double dx = point.path_point.x - x;
    const double dy = point.path_point.y - y;
    return dx * dx + dy * dy;
}

common_msg::TrajectoryPoint TrajectoryAnalyzer::QueryNearestPointByAbsoluteTime(
    const double t) const
{
    return QueryNearestPointByRelativeTime(t - header_time_);
}

common_msg::TrajectoryPoint TrajectoryAnalyzer::QueryNearestPointByRelativeTime(
    const double t) const
{
    auto func_comp = [](const common_msg::TrajectoryPoint &point, 
                        const double relative_time) {
        return point.relative_time < relative_time;
    };
    auto it_low = std::lower_bound(trajectory_points_.begin(), 
                                    trajectory_points_.end(), t, func_comp);
    if (it_low == trajectory_points_.begin()) {
        return trajectory_points_.front();
    }
    if (it_low == trajectory_points_.end()) {
        return trajectory_points_.back();
    }
    if (FLAGS_query_forward_time_point_only) {
        return *it_low;
    } else {
        auto it_lower = it_low - 1; 
        if (it_low->relative_time - t < t - it_lower->relative_time) {
            return *it_low;
        }
        return *it_lower;        
    }   
}

common_msg::PathPoint TrajectoryAnalyzer::TrajectoryPointToPathPoint(
    const common_msg::TrajectoryPoint &point) {
    if (point.path_point.x != 0.0 ||
        point.path_point.y != 0.0 ||
        point.path_point.z != 0.0) {
        return point.path_point;
    } else { 
        return common_msg::PathPoint();
    }
    
}

common_msg::PathPoint TrajectoryAnalyzer::FindMinDistancePoint(
    const common_msg::TrajectoryPoint &p0,
    const common_msg::TrajectoryPoint &p1,
    const double x,
    const double y) {
  // given the fact that the discretized trajectory is dense enough,
  // we assume linear trajectory between consecutive trajectory points.
  auto dist_square = [&p0, &p1, &x, &y](const double s) {
    double px = common::lerp(p0.path_point.x, p0.path_point.s,
                                   p1.path_point.x, p1.path_point.s, s);
    double py = common::lerp(p0.path_point.y, p0.path_point.s,
                                   p1.path_point.y, p1.path_point.s, s);
    double dx = px - x;
    double dy = py - y;
    return dx * dx + dy * dy;
  };

  common_msg::PathPoint p = p0.path_point;
  double s = common::GoldenSectionSearch(dist_square, p0.path_point.s,
                                               p1.path_point.s);
  p.s = s;
  p.x = (common::lerp(p0.path_point.x, p0.path_point.s,
                             p1.path_point.x, p1.path_point.s, s));
  p.y = (common::lerp(p0.path_point.y, p0.path_point.s,
                             p1.path_point.y, p1.path_point.s, s));
  p.theta = (common::slerp(p0.path_point.theta, p0.path_point.s,
                            p1.path_point.theta, p1.path_point.s,
                            s));
  // approximate the curvature at the intermediate point
  p.kappa = (common::lerp(p0.path_point.kappa, p0.path_point.s,
                                 p1.path_point.kappa, p1.path_point.s,
                                 s));
  return p;
}

common_msg::PathPoint TrajectoryAnalyzer::QueryMatchedPathPoint(
    const double x, const double y) {
    // 轨迹点不为空
    // assert(trajectory_points_.size() > 0 && "trajectory_points_ is empty");
    if (!trajectory_points_.size()) {
        std::cerr << "trajectory_points_ is empty" << std::endl;
    }
    double d_min = PointDistanceSquare(trajectory_points_.front(), x, y);
    size_t index_min = 0;

    // 距离最近点
    for (size_t i = 0; i < trajectory_points_.size(); ++i) {
        double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
        if (d_temp < d_min) {
            d_min = d_temp;
            index_min = i;
        }
    }
    // 边界点
    size_t index_start = index_min == 0 ? index_min : index_min - 1;
    size_t index_end = index_min + 1 ==
     trajectory_points_.size() ? index_min : index_min + 1;
    const double kEpsilon = 1e-3;  // 最小弧长
    if (index_start == index_end ||
        (std::fabs(trajectory_points_[index_start].path_point.s - 
        trajectory_points_[index_end].path_point.s)) <= kEpsilon) {
        return TrajectoryPointToPathPoint(trajectory_points_[index_start]);
    }
    // 插值计算给定点(x,y)最近的位置
    return FindMinDistancePoint(trajectory_points_[index_start],
                              trajectory_points_[index_end], x, y); 
}

common::Vec2d TrajectoryAnalyzer::ComputeCOMPosition(
    const double rear_to_com_distance, const common_msg::PathPoint &path_point) const {
  // Initialize the vector for coordinate transformation of the position
  // reference point
  Eigen::Vector3d v;
  const double cos_heading = std::cos(path_point.theta);
  const double sin_heading = std::sin(path_point.theta);
  v << rear_to_com_distance * cos_heading, rear_to_com_distance * sin_heading,
      0.0;
  // Original position reference point at center of rear-axis
  Eigen::Vector3d pos_vec(path_point.x, path_point.y, path_point.z);
  // Transform original position with vector v
  Eigen::Vector3d com_pos_3d = v + pos_vec;
  // Return transfromed x and y
  return common::Vec2d(com_pos_3d[0], com_pos_3d[1]);
}

void TrajectoryAnalyzer::TrajectoryTransformToCOM(
    const double rear_to_com_distance) {
//   CHECK_GT(trajectory_points_.size(), 0U);
  for (size_t i = 0; i < trajectory_points_.size(); ++i) {
    auto com = ComputeCOMPosition(rear_to_com_distance,
                                  trajectory_points_[i].path_point);
    trajectory_points_[i].path_point.x = com.x();
    trajectory_points_[i].path_point.y = com.y();
  }
}
    
} // control
