#include "interpolation_2d.h"

namespace {const double kDoubleEpsilon = 1e-6;}

namespace control {
namespace common {
bool Interpolation2D::CheckMap() const { 
    double keysize = xyz_.begin()->second.size();  // yz_table大小
    auto itr_ = xyz_.begin();
    for ( ; itr_ != xyz_.end(); itr_++) {
        if (keysize != itr_->second.size()) {
            std::cerr << "calibration map dimension is not equal" 
                    << "\n" << "first value: " << keysize
                    << "\n" << "second value: " << itr_->second.size()
                    << std::endl;
        }
        // 标定表中加速度包含正负值
        int pos_cout = 0;
        int nag_cout = 0;
        auto inner_itr_ = itr_->second.begin();
        for ( ; inner_itr_ != itr_->second.end(); inner_itr_++) {
            if (inner_itr_->first > 0) pos_cout++;
            else if (inner_itr_->first < 0) nag_cout++;
        }
        if (pos_cout == 0) {
            std::cerr << "calibration map does not contain positive values"
                    << std::endl;
            return false;
        }
        if (nag_cout == 0) {
            std::cerr << "calibration map does not contain negative values"
                    << std::endl;
            return false;
        }
    }
    return true;
}   

bool Interpolation2D::Init(const DataType &xyz) { 
    if (xyz.empty()) {
        std::cerr << "calibration map is empty" << std::endl;
        return false;
    }
    // 按顺序写入键值 speed acceleration command
    for (const auto &item : xyz) {
        xyz_[std::get<0>(item)][std::get<1>(item)] = std::get<2>(item);
    }
    if (!CheckMap()) {
        std::cerr << "calibration map is not valid" << std::endl;
        return false;
    }
    return true;
}

// 线性插值计算
double Interpolation2D::InterpolateVaule(
    const double value_before,
    const double dist_before,
    const double value_after,
    const double dist_after) const { 
    // 边界处理
    if (dist_before < kDoubleEpsilon) {
        return value_before;
    }
    if (dist_after < kDoubleEpsilon)
    {
        return value_after;
    }
    // 距离比计算权重值
    double value_gap = value_after - value_before;
    double value_buff = value_gap * dist_before / (dist_before + dist_after);
    return value_before + value_buff;
}

double Interpolation2D::InterpolateYz(
    const std::map<double, double> &yz_table, 
    const double &y) const {
        if (yz_table.empty()) {
            std::cerr << "Unable to interpolateYz because yz_table is empty" << std::endl;
            return y;
        }
        // 边界处理
        double max_y = yz_table.rbegin()->first;
        double min_y = yz_table.begin()->first;
        if (y >= max_y - kDoubleEpsilon) {
            return yz_table.rbegin()->second;
        }
        if (y <= min_y + kDoubleEpsilon) {
            return yz_table.begin()->second;
        }
        // y的包围点
        auto itr_after = yz_table.lower_bound(y);
        auto itr_before = itr_after;
        if (itr_before != yz_table.begin()) {
            --itr_before;
        }
        double y_before = itr_before->first;
        double z_before = itr_before->second;
        double y_after = itr_after->first;
        double z_after = itr_after->second;

        double y_diff_dist = y_after - y_before;
        double z_diff_dist = z_after - z_before;
        // 插值计算z值
        return InterpolateVaule(z_before, y_diff_dist, z_after, y_diff_dist);
    }

double Interpolation2D::Interpolate(const KeyType &xy) const { 
    double max_x = xyz_.rbegin()->first;
    double min_x = xyz_.begin()->first;
    // 边界处理
    if (xy.first >= max_x - kDoubleEpsilon) {
        return InterpolateYz(xyz_.rbegin()->second, xy.second);
    }
    if (xy.first <= min_x + kDoubleEpsilon) {
        return InterpolateYz(xyz_.begin()->second, xy.second);
    }
    // x包围点
    auto itr_after = xyz_.lower_bound(xy.first);
    auto itr_before = itr_after;
    if (itr_before != xyz_.begin()) {
        --itr_before;
    }
    double x_before = itr_before->first;
    double z_before = InterpolateYz(itr_before->second, xy.second);
    double x_after = itr_after->first;
    double z_after = InterpolateYz(itr_after->second, xy.second);

    double x_diff_dist = x_after - x_before;
    double z_diff_dist = z_after - z_before;

    return InterpolateVaule(z_before, x_diff_dist, z_after, x_diff_dist);
}

}  // control
}  // common