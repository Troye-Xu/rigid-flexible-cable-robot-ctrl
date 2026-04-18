// constraint_estimator.hpp
#ifndef CAPL_CONSTRAINT_ESTIMATOR_HPP
#define CAPL_CONSTRAINT_ESTIMATOR_HPP

#include "capl/data_types.hpp"

namespace capl {

class ConstraintEstimator {
public:
    ConstraintParams estimate(
        const Demonstration& demo, 
        int s, 
        const std::vector<Eigen::MatrixXd>& Phi_A,
        const std::vector<Eigen::MatrixXd>& Phi_B) const;

private:
    // 构建回归矩阵 H = [Phi_A(t)*u(t); -Phi_B(t)]
    Eigen::MatrixXd buildRegressionMatrix(
        const Demonstration& demo,
        const std::vector<Eigen::MatrixXd>& Phi_A,
        const std::vector<Eigen::MatrixXd>& Phi_B) const;

    // 构建 Q 矩阵（用于广义特征值问题）
    Eigen::MatrixXd buildQMatrix(int rows) const;
};

}  // namespace capl

#endif // CAPL_CONSTRAINT_ESTIMATOR_HPP    