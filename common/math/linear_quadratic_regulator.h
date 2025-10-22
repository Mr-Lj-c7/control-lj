#ifndef LINEAR_QUADRATIC_REGULATOR_H
#define LINEAR_QUADRATIC_REGULATOR_H 

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cstdint>
#include <string>
#include <iostream>

namespace control{
namespace common{

class LinearQuadraticRegularizer
{
private:
    /* data */
public:
    LinearQuadraticRegularizer() = default;
    ~LinearQuadraticRegularizer() = default;
};

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
void SolveLQRProblem(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                     const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R, 
                     const Eigen::MatrixXd &M, const double tolerance,
                     const std::uint32_t max_num_iterations, Eigen::MatrixXd *ptr_K,
                     std::uint32_t *iterate_num, double *result_diff);

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
void SolveLQRProblem(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                     const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
                     const double tolerance, const std::uint32_t max_num_iterations, 
                     Eigen::MatrixXd *ptr_K,
                     std::uint32_t *iterate_num, double *result_diff);

}
}

#endif