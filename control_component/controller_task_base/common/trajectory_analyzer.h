#ifndef TRAJECTROY_ANALYZER_H
#define TRAJECTORY_ANALYZER_H 

#include <vector>
#include <string>

#include <iostream>
#include <algorithm>
#include <cassert>

#include "common_msg/basic_msgs/pnc_point_msg.h"
#include "common_msg/planning_msgs/planning_msg.h"
#include "control_component/controller_task_base/common/dependency_injector.h"


namespace control {

class TrajectoryAnalyzer {
public:
    TrajectoryAnalyzer() = default;
    TrajectoryAnalyzer(
      const common_msg::ADCTrajectory *planning_published_trajectory);
    ~TrajectoryAnalyzer() = default;

    // 基于位置距离寻找最近点
    common_msg::TrajectoryPoint QueryNearestPointByPosition(
      const double x, const double y);
    double PointDistanceSquare(
      const common_msg::TrajectoryPoint &point, 
      const double x, const double y);

    // 基于相对时间寻找最近点
    common_msg::TrajectoryPoint QueryNearestPointByAbsoluteTime(
      const double t) const;
    common_msg::TrajectoryPoint QueryNearestPointByRelativeTime(
      const double t) const;

    // 寻找匹配点
    common_msg::PathPoint TrajectoryPointToPathPoint(
      const common_msg::TrajectoryPoint &point); 
    common_msg::PathPoint QueryMatchedPathPoint(
      const double x, const double y);
    common_msg::PathPoint FindMinDistancePoint(
      const common_msg::TrajectoryPoint &p0,
      const common_msg::TrajectoryPoint &p1,
      const double x,
      const double y);
    void ToTrajectoryFrame(const double x, const double y,
      const double theta, const double v,
      const common_msg::PathPoint &ref_point,
      double *ptr_s, double *ptr_s_dot,
      double *ptr_d,
      double *ptr_d_dot) const;

protected:
    std::vector<common_msg::TrajectoryPoint> trajectory_points_;
    double header_time_ = 0.0;
    bool FLAGS_query_forward_time_point_only = false;
    unsigned int seq_num_ = 0;

};
} // control

#endif