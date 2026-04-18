// DragTeaching.h
#ifndef DragTeaching_H
#define DragTeaching_H
#pragma once
#include <Eigen/Dense>
#include <vector>

class DragTeaching {
public:
    // 机械臂参数结构体
    struct RobotParameters {
        std::vector<double> mass;
        std::vector<Eigen::Matrix3d> inertia;
        std::vector<double> friction;
        std::vector<Eigen::Vector3d> com_pos;
    };

    DragTeaching(const RobotParameters& params);
    
    // 更新关节状态
    void UpdateJointStates(const std::vector<double>& positions,
                          const std::vector<double>& velocities,
                          const std::vector<double>& torques);
    
    // 检测拖拽并计算目标关节角度
    bool CalculateTargetJoints(std::vector<double>& target_positions);
    
    // 设置控制参数
    void SetControlParams(double force_threshold, 
                         double stiffness,
                         double damping);

private:
    // 动力学计算
    Eigen::VectorXd ComputeInverseDynamics();
    
    // 计算雅可比矩阵
    Eigen::MatrixXd ComputeJacobian() const;
    
    // 逆运动学求解
    bool SolveInverseKinematics(const Eigen::Vector3d& delta_position,
                               std::vector<double>& new_positions);

    RobotParameters params_;
    std::vector<double> current_positions_;
    std::vector<double> current_velocities_;
    std::vector<double> measured_torques_;
    
    double force_threshold_;
    double stiffness_;
    double damping_;
    
    const size_t DOF = 7;  // 七轴机械臂
};