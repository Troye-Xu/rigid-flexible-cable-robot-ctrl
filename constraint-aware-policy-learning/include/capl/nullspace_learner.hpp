// nullspace_learner.hpp
#ifndef CAPL_NULLSPACE_LEARNER_HPP
#define CAPL_NULLSPACE_LEARNER_HPP

#include "capl/data_types.hpp"

namespace capl {

class NullspaceLearner {
public:
    NullspacePolicy learn(
        const Demonstration& demo,
        const ConstraintParams& params,
        const std::vector<Eigen::MatrixXd>& Phi_A,
        const std::vector<Eigen::MatrixXd>& Phi_B,
        const Config& config = Config()) const;

private:
    // 计算约束矩阵 A(t)
    Eigen::MatrixXd computeConstraintMatrix(
        const ConstraintParams& params,
        const Eigen::MatrixXd& Phi_A_t) const;
    
    // 计算约束零空间投影矩阵 N(t)
    Eigen::MatrixXd computeNullspaceProjection(
        const Eigen::MatrixXd& A_t) const;
    
    // k-means 聚类算法
    std::vector<Eigen::VectorXd> kMeans(
        const std::vector<Eigen::VectorXd>& points,
        int k,
        int max_iterations = 100) const;
    
    // 高斯核函数
    double gaussianKernel(
        const Eigen::VectorXd& x,
        const Eigen::VectorXd& center,
        double variance) const;
    
    // 局部加权回归
    Eigen::MatrixXd locallyWeightedRegression(
        const std::vector<Eigen::VectorXd>& X,//x(t)
        const std::vector<Eigen::VectorXd>& Y,//u_null​(t)
        const Eigen::VectorXd& center,
        double variance) const;//高斯核方差（带宽参数）参数设置选择手动，设置或自适应计算，kmeans聚类中心
};

}  // namespace capl

#endif // CAPL_NULLSPACE_LEARNER_HPP    