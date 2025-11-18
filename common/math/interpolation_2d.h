#ifndef INTERPOLATION_2D_H
#define INTERPOLATION_2D_H 

#include <iostream>
#include <vector>   
#include <map>   
#include <memory>   
#include <tuple>   
#include <utility>  
#include "common/configs/config_gflags.h"

namespace control {
namespace common {
// 双线性插值
class Interpolation2D {
public:
    typedef std::vector<std::tuple<double, double, double>> DataType;  // 三维数据 (speed, acceleration, command)
    typedef std::pair<double, double> KeyType;  // 组合键 KeyType.first, KeyType.second

    Interpolation2D() = default;
    ~Interpolation2D() = default;

    bool Init(const DataType &xyz);
    bool CheckMap() const;
    double Interpolate(const KeyType &xy) const;  // (speed, acceleration)

private:
    // yz插值
    double InterpolateYz(const std::map<double, double> &yz_table, 
        const double &y) const;

    // 距离的比作为权重值，插值计算
    double InterpolateVaule(const double value_before, 
                            const double dist_before, 
                            const double value_after, 
                            const double dist_after) const;
    // 嵌套键值对，第一个 double 键 → 第二个 double 键 → double 值
    std::map<double, std::map<double, double>> xyz_;

};
} // namespace control
} // namespace common


#endif