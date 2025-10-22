#ifndef INTERPOLATION_1D_H
#define INTERPOLATION_1D_H 

#include "bspline_1d.h"

namespace control {
namespace common {

class Interpolation1D {
public:
    typedef std::vector<std::pair<double, double>> DataType;
    Interpolation1D() = default;
    ~Interpolation1D() = default;

    bool Init(const DataType & xy);

    double Interpolate(double x) const;

private:
    double ScaleValue(double x) const;

    // 边界值
    double x_min_ = 0.0;
    double x_max_ = 0.0;
    double y_start_ = 0.0;
    double y_end_ = 0.0;

    // B样条插值器
    std::unique_ptr<BSpline1D> bspline_;
};

} // control
} // common
#endif