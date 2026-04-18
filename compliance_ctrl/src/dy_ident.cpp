// BaseParametersIdentification.cpp
#include "BaseParametersIdentification.h"
#include <iostream>

using namespace Eigen;

BaseParametersIdentifier::BaseParametersIdentifier(int base_params_num)
    : base_params_num_(base_params_num), data_count_(0) {}

void BaseParametersIdentifier::AddSample(
    const VectorXd& Y_row,
    const VectorXd& torque) {
    
    // 应用转换矩阵：Φ = Y * T
    VectorXd phi_row = Y_row.transpose() * T_;
    
    // 初始化存储
    if(data_count_ == 0) {
        Phi_ = MatrixXd::Zero(1000, base_params_num_);
        Tau_ = VectorXd::Zero(1000 * 7);
    }
    
    Phi_.row(data_count_) = phi_row;
    Tau_.segment(data_count_*7, 7) = torque;
    data_count_++;
}

void BaseParametersIdentifier::SetTransformationMatrix(const MatrixXd& T) {
    T_ = T;
}

bool BaseParametersIdentifier::IdentifyBaseParameters(VectorXd& base_params) {
    if(T_.size() == 0) {
        std::cerr << "Transformation matrix not initialized!" << std::endl;
        return false;
    }
    
    MatrixXd Phi = Phi_.topRows(data_count_);
    VectorXd Tau = Tau_.head(data_count_*7);
    
    // 求解：Φ^TΦ * π = Φ^Tτ
    base_params = (Phi.transpose() * Phi).ldlt().solve(Phi.transpose() * Tau);
    
    return true;
}