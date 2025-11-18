#ifndef INTERPOLATION_1D_H
#define INTERPOLATION_1D_H 

#include "bspline_1d.h"
#include <eigen3/Eigen/Core>
#include <eigen3/unsupported/Eigen/Splines>

namespace control {
namespace common {

// B样条插值器 
class Interpolation1D {
public:
    typedef std::vector<std::pair<double, double>> DataType;
    
    Interpolation1D() = default;
    ~Interpolation1D() = default;

    bool Init(const DataType & xy);
    // 插值计算x在[x_max, x_min]，超出范围返回边界值x_start或x_end
    double Interpolate(double x) const;

protected:

private:
    // 对输入x进行缩放至[0, 1]
    double ScaledValue(double x) const;

    Eigen::RowVectorXd ScaledValues(Eigen::VectorXd const& x_vec) const;
    // 边界值
    double x_min_ = 0.0;
    double x_max_ = 0.0;
    double y_start_ = 0.0;
    double y_end_ = 0.0;

    // 一维点样条插值器
    std::unique_ptr<Eigen::Spline<double, 1>> spline_;
};

} // control
} // common
#endif