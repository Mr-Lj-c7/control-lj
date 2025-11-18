#include "linear_quadratic_regulator.h"

namespace control {
namespace common {

using Matrix = Eigen::MatrixXd;

/**
 * @brief 求解离散时间线性二次调节器问题
 * @param[in] A 系统动态矩阵（状态转移矩阵）
 * @param[in] B 控制输入矩阵
 * @param[in] Q 状态代价矩阵（衡量系统状态的权重）
 * @param[in] R 控制输入代价矩阵（衡量控制输出的权重）
 * @param[in] M 状态和控制输入之间的交叉项矩阵（空矩阵）
 * @param[in] tolerrance 数值求解Riccati方程的容差（最小求解精度）
 * @param[in] max_num_iterations 数值求解Riccati方程的最大迭代次数
 * @param[out] ptr_K 求解的最优反馈控制的系数矩阵K, u=-k*x
 * @param[out] iterate_num 数值求解Riccati方程的输出迭代次数
 * @param[out] result_diff 数值求解Riccati方程的输出精度
 */
void SolveLQRProblem(const Matrix &A, const Matrix &B,
    const Matrix &Q, const Matrix &R, 
    const Matrix &M, const double tolerance,
    const std::uint32_t max_num_iterations, Matrix *ptr_K,
    std::uint32_t *iterate_num, double *result_diff) {
    if (A.rows() != A.cols() || B.rows() != A.rows() || Q.rows() != Q.cols() ||
        Q.rows() != A.rows() || R.rows() != R.cols() || R.rows() != B.cols() || 
        M.rows() != Q.rows() || M.cols() != R.cols()) {
        std::cerr << "[SolveLQRProblem]: one or more matrices have wrong size." 
                  << std::endl;
        return;
    }
    Matrix AT = A.transpose();
    Matrix BT = B.transpose();
    Matrix MT = M.transpose();
    Matrix P = Q;
    std::uint32_t num_iteration = 0;
    double diff = std::numeric_limits<double>::max();  // 最大double数值
    // Raccatic 方程求解
    while (num_iteration++ < max_num_iterations && diff > tolerance) {
        Matrix P_next = (AT * P * A) 
                        - (AT * P * B + M) 
                        * (R + BT * P * B).inverse() 
                        * (BT * P * A + MT) + Q;
        diff = std::fabs((P_next - P).maxCoeff());  // 矩阵元素最大值
        P = P_next;
    }
    if (num_iteration >= max_num_iterations) {
        std::cerr << 
          "[SolveLQRProblem]: LQR Solver cannot converge to a solution, last consecutive result diff is: " 
                  << diff << std::endl;
    } else {
        std::cerr << 
          "[SolveLQRProblem]: LQR Solver converged to a solution, last consecutive result diff is: " 
                 << diff << std::endl;
    }
    *ptr_K = (R + BT * P * B).inverse() * (BT * P * A + MT);
    *iterate_num = num_iteration;
    *result_diff = diff;
};

/**
 * @brief 求解离散时间线性二次调节器问题
 * @param[in] A 系统动态矩阵（状态转移矩阵）
 * @param[in] B 控制输入矩阵
 * @param[in] Q 状态代价矩阵（衡量系统状态的权重）
 * @param[in] R 控制输入代价矩阵（衡量控制输出的权重）
 * @param[in] tolerrance 数值求解Riccati方程的容差（最小求解精度）
 * @param[in] max_num_iterations 数值求解Riccati方程的最大迭代次数
 * @param[out] ptr_K 求解的最优反馈控制的系数矩阵K, u=-k*x
 * @param[out] iterate_num 数值求解Riccati方程的输出迭代次数
 * @param[out] result_diff 数值求解Riccati方程的输出精度
 */
void SolveLQRProblem(const Matrix &A, const Matrix &B,
    const Matrix &Q, const Matrix &R,
    const double tolerance, const std::uint32_t max_num_iterations, 
    Matrix *ptr_K,
    std::uint32_t *iterate_num, double *result_diff) {
    // create M as zero matrix of the right size:
    // M.rows() == Q.rows() && M.cols() == R.cols()
    Matrix M = Matrix::Zero(Q.rows(), R.cols());
    SolveLQRProblem(A, B, Q, R, M, tolerance, 
                    max_num_iterations, ptr_K, 
                    iterate_num, result_diff);
};

} // control
} // common
