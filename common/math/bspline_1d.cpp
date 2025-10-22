#include "bspline_1d.h"

namespace { const double kDoubleEpsilon = 1e-6; }

namespace control {
namespace common {

    // 数据预处理
    bool BSpline1D::Init(const DataType & xy) {
        if (xy.empty()) {
            std::cerr << "[BSpline1D]: Empty input for B-Spline interpolation." 
                      << std::endl;
            return false;
        }
        // 预处理，排序
        auto xy_(xy);
        std::sort(xy_.begin(), xy_.end());  // 升序

        x_min_ = xy_.front().first;
        x_max_ = xy_.back().first;
        y_start_ = xy_.front().second;
        y_end_ = xy_.back().second;

        // 控制点， 节点
        ComputeKnots(xy_);
        return true;
    };

    // 对输入值x进行缩放
    double BSpline1D::ScaleValue(double x) const {
        if (std::fabs(x_max_ - x_min_) < kDoubleEpsilon) {
            return x_min_;
        }
        return (x - x_min_) / (x_max_ - x_min_);  // 映射到[0, 1]
    };

    // 查找knot区间,已知特定节点域，优化计算
    int BSpline1D::FindKnotSpan(int n, int p, double u, 
        const std::vector<double> &knots) const {
             // 特殊情况处理
        if (u >= knots[n + 1]) return n;
        if (u <= knots[p]) return p;
        
        // 二分查找
        int low = p;
        int high = n + 1;
        int mid = (low + high) / 2;
        
        while (u < knots[mid] || u >= knots[mid + 1]) {
            if (u < knots[mid]) {
            high = mid;
            } else {
            low = mid;
            }
            mid = (low + high) / 2;
        }
        return mid;
    };

    // 计算节点向量
    void BSpline1D::ComputeKnots(
        const std::vector<std::pair<double, double>> &xy) {
        int n = static_cast<int>(xy.size()) - 1;  // 控制点数减1
        k_ = std::min(3, n);  // 3次B样条曲线

        // 以所有数据点作为控制点（这里如果数据较大可以对初始数据做减法）
        control_points_.clear();
        control_points_.reserve(xy.size());
        for (const auto &xy_pair : xy) {
            control_points_.push_back(xy_pair);
        }

        // 计算节点向量（非均匀）
        int m = n + k_ + 1;  // 节点大小
        knots_.resize(m+1);
        for (size_t i = 0; i <= k_; ++i) {  // 前K_+1个节点为0
            knots_[i] = 0.0;
        }
        int num_interal = n - k_;
        for (size_t i = 1; i <= num_interal; ++i) {  // 中间节点均匀分布
            knots_[k_+i] = static_cast<double>(i) / (num_interal+1);
        }
        for (size_t i = 0; i <= k_; ++i) {  // 后K_+1个节点为1
            knots_[n+1+i] = 1.0;
        }
    };

    // 基函数
    double BSpline1D::BasedFunction(int p, int k, double t, 
        const std::vector<double> &knots) const {
        // k == 0 零阶基函数
        if (k == 0) {
            return (t >= knots[p] && t < knots[p+1]) ? 1.0 : 0.0;
        }
        // 每一个控制点权重值（基函数）与低一阶前、后端控制点的基函数相关
        double denominator_1 = knots[p+k] - knots[p];
        double denominator_2 = knots[p+k+1] - knots[p+1];
        double numerator_1 = t - knots[p];
        double numerator_2 = knots[p+k+1] - t;
        double coeff_1 = 0.0;
        double coeff_2 = 0.0;

        if (denominator_1 > kDoubleEpsilon) {
            coeff_1 = numerator_1 / denominator_1;
        }
        if (denominator_2 > kDoubleEpsilon) {
            coeff_2 = numerator_2 / denominator_2;
        }

        return coeff_1 * BasedFunction(p, k-1, t, knots) + 
               coeff_2 * BasedFunction(p+1, k-1, t, knots);
    };

        
    // 插值计算
    double BSpline1D::Interpolate(double x) const {
        if (x <= x_min_) {
            return y_start_;
        }
        if (x >= x_max_) {
            return y_end_;
        }
        double t = ScaleValue(x);
        // 计算B样条基函数
        double result = 0.0;
        int n = static_cast<int>(control_points_.size()) - 1;

        for (int i = 0; i <= n; ++i) {
            result += control_points_[i].second * 
                BasedFunction(i, k_, t, knots_);
        }

        return result;
    };

    
    // 边界值API
    double BSpline1D::XMin() const { return x_min_; };

    double BSpline1D::XMax() const { return x_max_; };

    double BSpline1D::YStart() const { return y_start_; };

    double BSpline1D::YEnd() const { return y_end_; };


}  // common
}  // control

