// policy_executor.hpp
#ifndef CAPL_POLICY_EXECUTOR_HPP
#define CAPL_POLICY_EXECUTOR_HPP

#include "capl/data_types.hpp"

namespace capl {

class PolicyExecutor {
public:
    PolicyExecutor(const TaskSpace& task_space, const Config& config = Config());
    
    Eigen::VectorXd execute(
        const Eigen::VectorXd& current_state,
        const NullspacePolicy& policy,
        const Eigen::VectorXd& force_sensor,
        double dt,
        const ConstraintParams& params,
        const Eigen::MatrixXd& Phi_A_t,
        const Eigen::MatrixXd& Phi_B_t) const;

private:
    TaskSpace task_space_;
    Config config_;
    
    // 计算约束雅可比矩阵
    Eigen::MatrixXd computeConstraintJacobian(const Eigen::VectorXd& state) const;
    
    // 计算零空间策略输出
    Eigen::VectorXd computeNullspacePolicy(
        const Eigen::VectorXd& state,
        const NullspacePolicy& policy) const;
    
    // 计算阻抗控制器输出
    Eigen::VectorXd computeImpedanceControl(
        const Eigen::VectorXd& state,
        const Eigen::VectorXd& target,
        const Eigen::VectorXd& force_error) const;
};

}  // namespace capl

#endif // CAPL_POLICY_EXECUTOR_HPP    