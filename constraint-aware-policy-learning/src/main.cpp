#include "capl/constraint_aware_learning.hpp"
#include <iostream>

int main() {
    // =========================================
    // 1. 准备演示数据
    // =========================================
    capl::Demonstration demo;
    int q = 7;  // 控制维度（如7自由度机械臂）
    int T = 1000;  // 演示长度
    demo.dt = 0.01;  // 采样时间
    demo.x.resize(T);
    demo.u.resize(T);
    demo.t.resize(T);
    
    // 生成模拟演示数据（实际应用中应从文件或ROS话题加载）
    for (int t = 0; t < T; ++t) {
        demo.x[t] = Eigen::VectorXd::Random(q);  // 随机状态
        demo.u[t] = Eigen::VectorXd::Random(q);  // 随机控制输入
        demo.t[t] = t * demo.dt;
    }
    
    // =========================================
    // 2. 定义特征函数（Phi_A和Phi_B）
    // =========================================
    std::vector<Eigen::MatrixXd> Phi_A(T);
    std::vector<Eigen::MatrixXd> Phi_B(T);
    
    // 示例：定义Phi_A为状态的函数
    for (int t = 0; t < T; ++t) {
        // 构建Phi_A(t)：3xq矩阵（3维约束）
        Phi_A[t] = Eigen::MatrixXd::Zero(3, q);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < q; ++j) {
                Phi_A[t](i, j) = demo.x[t](j);  // 简化示例
            }
        }
        
        // 构建Phi_B(t)：3x1向量
        Phi_B[t] = Eigen::MatrixXd::Zero(3, 1);
        Phi_B[t](0, 0) = 1.0;
        Phi_B[t](1, 0) = demo.x[t](0);
        Phi_B[t](2, 0) = demo.x[t](1);
    }
    
    // =========================================
    // 3. 约束估计
    // =========================================
    capl::ConstraintEstimator estimator;
    int s = 3;  // 约束维度
    capl::ConstraintParams params = estimator.estimate(demo, s, Phi_A, Phi_B);
    
    // =========================================
    // 4. 零空间策略学习
    // =========================================
    capl::NullspaceLearner learner;
    capl::Config config;
    config.num_local_models = 5;  // 局部模型数量
    config.gaussian_variance = 0.1;  // 高斯核方差
    capl::NullspacePolicy policy = learner.learn(demo, params, Phi_A, Phi_B, config);
    
    // =========================================
    // 5. 定义任务空间
    // =========================================
    capl::TaskSpace task_space;
    task_space.T_task = Eigen::Matrix4d::Identity();  // 任务坐标系
    
    // 定义约束函数（例如：平面约束）
    task_space.constraint_function = [](const Eigen::VectorXd& x) {
        Eigen::VectorXd constraint(1);
        constraint(0) = x(2) - 0.5;  // z坐标保持在0.5米
        return constraint;
    };
    
    // 定义约束雅可比
    task_space.constraint_jacobian = [](const Eigen::VectorXd& x) {
        Eigen::MatrixXd jacobian(1, x.size());
        jacobian.setZero();
        jacobian(0, 2) = 1.0;  // 对z坐标的偏导数
        return jacobian;
    };
    
    // =========================================
    // 6. 策略执行
    // =========================================
    capl::PolicyExecutor executor(task_space, config);
    Eigen::VectorXd current_state = demo.x[0];  // 初始状态
    Eigen::VectorXd force_sensor = Eigen::VectorXd::Zero(6);  // 力传感器数据
    
    // 模拟执行
    std::vector<Eigen::VectorXd> trajectory;
    trajectory.push_back(current_state);
    
    for (int t = 0; t < 500; ++t) {
        // 计算控制输入
        Eigen::VectorXd control_input = executor.execute(
            current_state, policy, force_sensor, demo.dt, params, Phi_A[0], Phi_B[0]);
        
        // 更新状态（简化动力学模型）
        current_state += control_input * demo.dt;
        
        // 模拟力反馈（根据约束函数）
        force_sensor.head(3) = task_space.constraint_function(current_state);
        
        // 记录轨迹
        trajectory.push_back(current_state);
    }
    
    // =========================================
    // 7. 结果可视化
    // =========================================
    capl::Utils::visualizeTrajectory(trajectory);
    
    std::cout << "约束感知策略学习完成！" << std::endl;
    
    return 0;
}    