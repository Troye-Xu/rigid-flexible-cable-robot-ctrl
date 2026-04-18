// data_types.hpp
#ifndef CAPL_DATA_TYPES_HPP
#define CAPL_DATA_TYPES_HPP

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <functional>

namespace capl {

// 演示数据结构
struct Demonstration {
    std::vector<Eigen::VectorXd> x;  // 状态序列
    std::vector<Eigen::VectorXd> u;  // 控制输入序列
    std::vector<double> t;           // 时间戳
    double dt;                       // 采样时间
};

// 约束参数
struct ConstraintParams {
    Eigen::MatrixXd w_A;  // 约束矩阵参数
    Eigen::MatrixXd w_B;  // 任务向量参数
    int s;                // 约束维度
};

// 零空间策略参数
struct NullspacePolicy {
    std::vector<Eigen::VectorXd> centers;  // 高斯核中心
    std::vector<Eigen::MatrixXd> w_pi;     // 局部模型参数
    Eigen::VectorXd D;                     // 高斯核方差
    int m;                                 // 局部模型数量
};

// 任务空间定义
struct TaskSpace {
    Eigen::Matrix4d T_task;  // 任务坐标系
    std::function<Eigen::VectorXd(const Eigen::VectorXd&)> constraint_function;  // 约束函数
    std::function<Eigen::MatrixXd(const Eigen::VectorXd&)> constraint_jacobian;  // 约束雅可比
};

// 配置参数
struct Config {
    int num_local_models = 5;         // 局部模型数量
    double gaussian_variance = 0.1;   // 高斯核方差
    double force_gain = 0.1;          // 力反馈增益
    double position_gain = 1.0;       // 位置增益
    double damping_ratio = 0.7;       // 阻尼比
    bool use_force_feedback = true;   // 是否使用力反馈
};

}  // namespace capl

#endif // CAPL_DATA_TYPES_HPP    