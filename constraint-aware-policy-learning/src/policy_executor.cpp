#include "capl/policy_executor.hpp"

namespace capl {

PolicyExecutor::PolicyExecutor(const TaskSpace& task_space, const Config& config)
    : task_space_(task_space), config_(config) {}
//对应公式25，26
Eigen::VectorXd PolicyExecutor::execute(
    const Eigen::VectorXd& current_state,
    const NullspacePolicy& policy,//零空间参数
    const Eigen::VectorXd& force_sensor,
    double dt,
    const ConstraintParams& params,//任务空间参数
    const Eigen::MatrixXd& Phi_A_t,
    const Eigen::MatrixXd& Phi_B_t) const {
    
    int q = current_state.size();
    
    // 1. 计算当前约束矩阵 A(t)
    Eigen::MatrixXd A_t = params.w_A * Phi_A_t;//公式15
    
    // 2. 计算约束雅可比矩阵 J_c，根据末端雅可比计算
    Eigen::MatrixXd J_c = computeConstraintJacobian(current_state);
    
    // 3. 计算零空间投影矩阵 N = I - J_c^† J_c
    Eigen::MatrixXd J_c_dag = J_c.transpose() * (J_c * J_c.transpose()).inverse();
    Eigen::MatrixXd N = Eigen::MatrixXd::Identity(q, q) - J_c_dag * J_c;
    
    // 4. 计算零空间策略输出 π(x)
    Eigen::VectorXd pi = computeNullspacePolicy(current_state, policy);
    
    // 5. 计算任务空间控制输入 u_t = J_c^† * [b_o - A_o*q̇]
    Eigen::VectorXd b_o = params.w_B * Phi_B_t.col(0);
    Eigen::VectorXd u_t = J_c_dag * (b_o - A_t * current_state);//论文这里没有加控制率
    
    // 6. 计算力反馈控制分量
    Eigen::VectorXd force_error = force_sensor;  // 假设力传感器直接测量误差
    Eigen::VectorXd u_f = -config_.force_gain * force_error;
    
    // 7. 整合控制输入: u = u_t + N*π + u_f，N*pi是加权求和部分
    Eigen::VectorXd control_input = u_t + N * pi + u_f;
    
    return control_input;
}

Eigen::MatrixXd PolicyExecutor::computeConstraintJacobian(const Eigen::VectorXd& state) const {
    // 如果任务空间定义了约束雅可比函数，则使用它
    if (task_space_.constraint_jacobian) {
        return task_space_.constraint_jacobian(state);
    }
    
    // 否则返回默认雅可比矩阵（单位矩阵）
    return Eigen::MatrixXd::Identity(state.size(), state.size());
}



//求零空间策略
Eigen::VectorXd PolicyExecutor::computeNullspacePolicy(
    const Eigen::VectorXd& state,
    const NullspacePolicy& policy) const {
    
    int q = state.size();
    Eigen::VectorXd output = Eigen::VectorXd::Zero(q);
    double weight_sum = 0.0;
    
    // 对所有局部模型进行加权求和
    for (int m = 0; m < policy.m; ++m) {
        double weight = gaussianKernel(state, policy.centers[m], policy.D[m]);
        output += weight * (policy.w_pi[m] * state);
        weight_sum += weight;
    }
    
    // 归一化
    if (weight_sum > 0) {
        output /= weight_sum;
    }
    
    return output;
}

//高斯核权重
double PolicyExecutor::gaussianKernel(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& center,
    double variance) const {
    
    return std::exp(-0.5 * (x - center).squaredNorm() / (variance * variance));
}

Eigen::VectorXd PolicyExecutor::computeImpedanceControl(
    const Eigen::VectorXd& state,
    const Eigen::VectorXd& target,
    const Eigen::VectorXd& force_error) const {
    
    // 简单阻抗控制器: F = Kp(x_d - x) + Kd(ẋ_d - ẋ) + F_ext
    Eigen::VectorXd position_error = target - state;
    Eigen::VectorXd control_force = config_.position_gain * position_error + force_error;
    
    return control_force;
}

}  // namespace capl    