#ifndef IMPEDANCE_H
#define IMPEDANCE_H

#include <Eigen/Dense>
#include <vector>

class IMPEDANCE {
public:
    IMPEDANCE();
    ~IMPEDANCE();

    Eigen::VectorXf Trans_fun(const Eigen::VectorXf& Input_force, 
                              const Eigen::VectorXf& Md, 
                              const Eigen::VectorXf& Bd, 
                              const Eigen::VectorXf& Kd, 
                              float tc, 
                              float dt);

private:
    Eigen::VectorXf g_trans_output_last_;  // 上一次的输出结果
    Eigen::VectorXf g_trans_output_last2_; // 上上次的输出结果
    Eigen::VectorXf g_trans_input_last_;   // 上一次的输入结果
    Eigen::VectorXf g_trans_input_last2_;  // 上上次的输入结果
};

#endif  // IMPEDANCE_H