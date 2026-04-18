// DragTeaching.cpp
#include "DragTeaching.h"
#include <iostream>
#include <cmath>

using namespace Eigen;

DragTeaching::DragTeaching(const RobotParameters& params)
    : params_(params), force_threshold_(5.0),
      stiffness_(100.0), damping_(10.0) {
    if(params_.mass.size() != DOF || 
       params_.inertia.size() != DOF ||
       params_.com_pos.size() != DOF) {
        throw std::invalid_argument("Invalid parameters size");
    }
}

void DragTeaching::UpdateJointStates(
    const std::vector<double>& positions,
    const std::vector<double>& velocities,
    const std::vector<double>& torques) {
    
    if(positions.size() != DOF ||
       velocities.size() != DOF ||
       torques.size() != DOF) {
        throw std::invalid_argument("Invalid input size");
    }
    
    current_positions_ = positions;
    current_velocities_ = velocities;
    measured_torques_ = torques;
}

bool DragTeaching::CalculateTargetJoints(
    std::vector<double>& target_positions) {
    
    // 1. 计算动力学预测力矩
    VectorXd predicted_torques = ComputeInverseDynamics();
    
    // 2. 计算外力矩差异
    VectorXd tau_ext(DOF);
    for(size_t i=0; i<DOF; ++i){
        tau_ext(i) = measured_torques_[i] - predicted_torques(i);
    }
    
    // 3. 判断是否达到拖拽阈值
    if(tau_ext.norm() < force_threshold_){
        return false;
    }
    
    // 4. 计算雅可比矩阵
    MatrixXd J = ComputeJacobian();
    
    // 5. 将力矩转换为末端力
    MatrixXd J_pinv = J.completeOrthogonalDecomposition().pseudoInverse();
    Vector3d F_end = J_pinv.topRows(3) * tau_ext;
    
    // 6. 计算期望位移
    Vector3d delta_pos = (F_end - damping_*Vector3d::Zero()) / stiffness_;
    
    // 7. 求解逆运动学
    std::vector<double> new_positions;
    if(SolveInverseKinematics(delta_pos, new_positions)){
        target_positions = new_positions;
        return true;
    }
    return false;
}




VectorXd DragTeaching::ComputeInverseDynamics() {
    // 实现牛顿欧拉递推动力学计算
    VectorXd torques(DOF);
    // ... 详细动力学实现 ...
    return torques;
}

MatrixXd DragTeaching::ComputeJacobian() const {
    // 实现雅可比矩阵计算
    MatrixXd J(6, DOF);
    // ... 详细雅可比计算 ...
    return J;
}

bool DragTeaching::SolveInverseKinematics(
    const Vector3d& delta_position,
    std::vector<double>& new_positions) {
    
    // 数值逆运动学求解
    const double eps = 1e-6;
    const int max_iter = 100;
    
    std::vector<double> q = current_positions_;
    MatrixXd J;
    Vector3d error = delta_position;
    
    for(int iter=0; iter<max_iter; ++iter){
        J = ComputeJacobian().topRows(3);
        
        MatrixXd J_pinv = J.completeOrthogonalDecomposition().pseudoInverse();
        VectorXd dq = J_pinv * error;
        
        // 更新关节角度
        for(int i=0; i<DOF; ++i){
            q[i] += dq(i);
        }
        
        // 计算新位置误差
        // ... 需要正运动学计算 ...
        Vector3d new_pos; 
        error = delta_position - new_pos;
        
        if(error.norm() < eps){
            new_positions = q;
            return true;
        }
    }
    return false;
}

void DragTeaching::SetControlParams(double force_threshold,
                                   double stiffness,
                                   double damping) {
    force_threshold_ = force_threshold;
    stiffness_ = stiffness;
    damping_ = damping;
}