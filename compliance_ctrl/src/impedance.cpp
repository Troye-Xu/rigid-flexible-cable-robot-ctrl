#include "ros/ros.h"
#include <compliance_ctrl/impedance.h>
#include <Eigen/Dense>

IMPEDANCE::IMPEDANCE() 
    : g_trans_output_last_(Eigen::VectorXf::Zero(6)), 
      g_trans_output_last2_(Eigen::VectorXf::Zero(6)), 
      g_trans_input_last_(Eigen::VectorXf::Zero(6)), 
      g_trans_input_last2_(Eigen::VectorXf::Zero(6)) 
{
    // 构造函数体为空，因为所有初始化都在初始化列表中完成
}
IMPEDANCE::~IMPEDANCE() {
    // 析构函数体为空
}

Eigen::VectorXf IMPEDANCE::Trans_fun(const Eigen::VectorXf& Input_force, 
                                     const Eigen::VectorXf& Md, 
                                     const Eigen::VectorXf& Bd, 
                                     const Eigen::VectorXf& Kd, 
                                     float tc, 
                                     float dt) {
    // 计算中间变量
    Eigen::VectorXf w1 = 4 * Md + 2 * Bd * dt + Kd * dt * dt;
    Eigen::VectorXf w2 = 2 * Kd * dt * dt - 8 * Md;
    Eigen::VectorXf w3 = Kd * dt * dt + 4 * Md - 2 * Bd * dt;

    // 特殊情况处理
    if (tc == 0) {
        g_trans_output_last_.setZero();
        g_trans_output_last2_.setZero();
        g_trans_input_last_ = Input_force;
        g_trans_input_last2_ = g_trans_input_last_;
    }

    // 计算当前输出
    Eigen::VectorXf tem = ((Input_force * dt * dt) + (2 * dt * dt * g_trans_input_last_) + (dt * dt * g_trans_input_last2_) - (w2.cwiseProduct(g_trans_output_last_)) - (w3.cwiseProduct(g_trans_output_last2_))).cwiseQuotient(w1);
    Eigen::VectorXf delta_x = tem;

    // 更新历史数据
    g_trans_output_last2_ = g_trans_output_last_;
    g_trans_output_last_ = tem;
    g_trans_input_last2_ = g_trans_input_last_;
    g_trans_input_last_ = Input_force; 

    return delta_x;
}