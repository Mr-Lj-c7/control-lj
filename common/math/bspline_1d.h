#ifndef BSPLINE_1D_H
#define BSpline_1D_H 

#include <vector>
#include <utility>
#include <memory>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace control {
namespace common {
class BSpline1D { 
public:
    typedef std::vector<std::pair<double, double>> DataType;

    BSpline1D() = default;
    ~BSpline1D() = default;
    
    // 数据预处理
    bool Init(const DataType & xy);

    // 插值计算
    double Interpolate(double x) const;

    // 边界值API
    double XMin() const;
    double XMax() const;
    double YStart() const;
    double YEnd() const;

private:
    // 对输入值x进行缩放
    double ScaleValue(double x) const;

    // 控制点计算
    void ComputeKnots(const std::vector<std::pair<double, double>> &xy);

    // 基函数
    double BasedFunction(int p, int k, double t, 
        const std::vector<double> &knots) const;

    // 查找knot区间
    int FindKnotSpan(int n, int p, double u, 
        const std::vector<double> &knots) const;

    double x_min_ = 0.0;
    double x_max_ = 0.0;
    double y_start_ = 0.0;
    double y_end_ = 0.0;

    // 样条曲线控制点
    std::vector<std::pair<double, double>> control_points_;

    // 样条曲线节点
    std::vector<double> knots_;

    // B样条阶数
    int k_ = 3;
};

}  // common
}  // control

#endif  