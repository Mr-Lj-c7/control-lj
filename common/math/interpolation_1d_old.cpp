#include "interpolation_1d.h"

namespace {
    const double kDoubleEpsilon = 1e-6;
}

namespace control {
namespace common {

    bool Interpolation1D::Init(const DataType & xy) {
        if (xy.empty()) {
            std::cerr << "[Interpolation1D]: input is empty!" << std::endl;
            return false;
        }
        // B样条插值器初始化
        bspline_.reset(new BSpline1D());
        if (!bspline_->Init(xy)) {
            std::cerr << "[Interpolation1D]: init BSpline1D failed!" << std::endl;
            return false;
        }
        x_min_ = bspline_->XMin();
        x_max_ = bspline_->XMax();
        y_start_ = bspline_->YStart();
        y_end_ = bspline_->YEnd();
        
        return true;
    };

    double Interpolation1D::Interpolate(double x) const {
        if (!bspline_) {
            std::cerr << "[Interpolation1D]: bspline is null!" << std::endl;
            return NAN;
        }
        return bspline_->Interpolate(x);
    };


    double Interpolation1D::ScaleValue(double x) const {
        if (x_max_ - x_min_ < kDoubleEpsilon) {
            return x_min_;
        }
        return (x - x_min_) / (x_max_ - x_min_);
    };

} // control
} // common
