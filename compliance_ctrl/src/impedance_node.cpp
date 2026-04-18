#include "ros/ros.h"
#include <iostream>
#include <Eigen/Dense>
#include <compliance_ctrl/impedance.h>
#include "jxb_ctrl/jxb_math.h"
#include "jxb_ctrl/kine.h"
#include "jxb_ctrl/Interp5rdPoly.h"
#include "FT_api.h" 


int main(int argc, char *argv[]) {

    /*机械臂初始化*/
    int JNT_NUM = 7;
    float DH[JNT_NUM][4];
    float JntCurrent[7] ={0};
    float JntTarget[7] ={0};
    float Jaco_0[6][7];
    float J0_pinv[7][6];
    //float Jaco_rel_inv[7][6];
    float T0e[4][4];
    float d1 = 0.3025,d3 = 0.465, d5 = 0.440,d7 = 0.06;
    //theta alpha a d
 	DH[0][0] = -M_PI/2; DH[0][1] = M_PI/2;  DH[0][2] = 0; DH[0][3] = d1;
    DH[1][0] = M_PI;    DH[1][1] = M_PI/2;  DH[1][2] = 0; DH[1][3] = 0;
    DH[2][0] = 0;       DH[2][1] = -M_PI/2; DH[2][2] = 0; DH[2][3] = d3;
    DH[3][0] = 0;       DH[3][1] = M_PI/2;  DH[3][2] = 0; DH[3][3] = 0;
    DH[4][0] = 0;       DH[4][1] = -M_PI/2; DH[4][2] = 0; DH[4][3] = d5;
    DH[5][0] = 0;       DH[5][1] = M_PI/2;  DH[5][2] = 0; DH[5][3] = 0;
    DH[6][0] = -M_PI/2;   DH[6][1] = 0;     DH[6][2] = 0; DH[6][3] = d7;
   // 初始化限幅
   const float MAX_JOINT_SPEED[7] = {0.8, 0.4, 0.8, 0.8, 0.8, 0.8, 0.8};  // 关节速度最大值
    const float MAX_JOINT_POS_CHANGE = 0.02;  // 单次循环关节位置变化最大值
    const float JOINT_POS_LIMITS[7][2] = {  // 关节位置上下限
        {-90.0/180.0*M_PI, 90.0/180.0*M_PI},
        {-90.0/180.0*M_PI, 90.0/180.0*M_PI},
        {-160.0/180.0*M_PI, 160.0/180.0*M_PI},
        {-100.0/180.0*M_PI, 100.0/180.0*M_PI},
        {-150.0/180.0*M_PI, 150.0/180.0*M_PI},
        {-90.0/180.0*M_PI, 90.0/180.0*M_PI},
        {-90.0/180.0*M_PI, 90.0/180.0*M_PI}
    };
       /*拖拽参数初始化*/
       ros::init(argc, argv, "drag_node");
       ros::NodeHandle nh; 
       Eigen::VectorXf Input_force(6);
       Eigen::VectorXf Md(6);
       Eigen::VectorXf Bd(6);
       Eigen::VectorXf Kd(6);
       float limit_force = 2.0;
       float tc = 0.0;
       float dt = 0.01;
       float M = 0.25;
       float K = 20.0;
       //初始化质量、阻尼、刚度
       Md << M, M, M, M/10.0, M/10.0, M/10.0;
       Bd << 50.0f, 50.0f, 50.0f, 5.0f, 5.0f, 5.0f;
       Kd << K, K, K, K/10.0, K/10.0, K/10.0;
       Bd = 2.0 * Md.cwiseSqrt().cwiseProduct(Kd.cwiseSqrt());
       Bd *= 1.2;  // 稍过阻尼（抑制残余振荡）
       IMPEDANCE impedance;
        //初始化标志位
        bool is_dragging = false;  // 拖拽模式标志位
        bool exit_dragging = false;  // 拖拽模式标志位
/*          JntCurrent[1] = -15/180.0*M_PI;
        JntCurrent[3] = 60/180.0*M_PI;
        JntCurrent[5] = -45/180.0*M_PI;   */
        JntCurrent[1] = 30/180.0*M_PI;
        JntCurrent[3] = 60/180.0*M_PI;
        JntCurrent[5] = 90/180.0*M_PI;  
 
  for(int i=0; i<100;i++)
    {
        //std::cout << "Input_force: " << Input_force.transpose() << std::endl;
        Input_force << 5.0, 5.0, 5.0, 0.0, 0.0, 0.0;
    //判断是否达到拖拽阈值，获取初始末端力
    float force_magnitude = Input_force.norm();  // 计算力的模长
    if (force_magnitude > limit_force && !is_dragging) {
            is_dragging = true;
        }

    if (is_dragging) 
    {
        //进入导纳控制模式
        //更新关节位置(弧度制)
   
 /*        JntCurrent[2] = 0.0;
        JntCurrent[4] = 0.0;
        JntCurrent[6] = 0.0; */
        for(int i =0;i<7;i++)
        {
        std::cout<<JntCurrent[i]/M_PI*180<<" ";
        }
        std::cout<<std::endl;
        //计算末端位姿矩阵
        forward_kine(JntCurrent, DH, T0e);
        //求雅可比矩阵
        Jaco_0_DH(JntCurrent, DH, Jaco_0);
        //Jaco_0转化为矩阵
        Eigen::MatrixXf J0(6,7);
        Eigen::MatrixXf J_pinv_0(7,6);
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 7; j++) {
                J0(i, j) = Jaco_0[i][j];
            }
        }
        //std::cout<<J0<<std::endl;
        Eigen:: MatrixXf Jaco_0_pinv = pinv(J0);
       /*  Rbt_PInvMtrx67(Jaco_0, J0_pinv);
        for (int i = 0; i < 7; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                J_pinv_0(i,j) = J0_pinv[i][j];
            }
            
        } */
        Eigen::Matrix3f R_base_end = get_end_effector_rotation(JntCurrent, DH);
        Eigen::MatrixXf R_full(6,6);
        R_full.setZero();
        R_full.block<3,3>(0,0) = R_base_end; // 线速度/位移部分
        R_full.block<3,3>(3,3) = R_base_end; // 角速度/旋转部分
        Input_force = R_full * Input_force; 
        //std::cout<<Input_force.transpose()<<std::endl;
        //std::cout << "Matrix J0_pinv:\n" << Jaco_0_pinv << "\n\n";
        Eigen::VectorXf delta_x = impedance.Trans_fun(Input_force, Md, Bd, Kd, tc, dt);
        std::cout <<"Delta_x (Eigen::VectorXf): " << delta_x.transpose() << std::endl;  
            //坐标系转换
  /*       Eigen::Matrix3f R_base_end = get_end_effector_rotation(JntCurrent, DH);
        Eigen::MatrixXf R_full(6,6);
        R_full.setZero();
        R_full.block<3,3>(0,0) = R_base_end; // 线速度/位移部分
        R_full.block<3,3>(3,3) = R_base_end; // 角速度/旋转部分
        delta_x = R_full * delta_x; */

        //std::cout <<"Delta_x (Eigen::VectorXf): " << delta_x.transpose() << std::endl; 

        Eigen::VectorXf Joint_vel = Jaco_0_pinv * delta_x;
       std::cout <<"Joint_vel: " << Joint_vel.transpose() << std::endl;  
/*         Joint_vel = J_pinv_0 * delta_x;
        std::cout <<"J_pinv_0: " << Joint_vel.transpose() << std::endl;   */


        float Joint_vel_array[7];
        for (int i = 0; i < 7; i++) {
        Joint_vel_array[i] = Joint_vel(i);
        }
        //速度限幅
         for (int i = 0; i < 7; i++) {
            Joint_vel_array[i] = std::max(std::min(Joint_vel_array[i], MAX_JOINT_SPEED[i]), -MAX_JOINT_SPEED[i]);
           // std::cout<<  Joint_vel_array[i]<<" ";
        } 
        //std::cout<<std::endl;
        //关节变化量限幅
        for (int i = 0; i < 7; i++) {
            float pos_change = Joint_vel_array[i] * dt;
            JntTarget[i] = JntCurrent[i] + pos_change;
        // 关节位置限幅
            JntTarget[i] = std::max(std::min(JntTarget[i], JOINT_POS_LIMITS[i][1]), JOINT_POS_LIMITS[i][0]);
        }
        for(int i = 0;i<7;i++)
        {
                JntCurrent[i] = JntTarget[i];
        }
        //更新时间
        tc += dt;
    }
    else
    {
    //保持机械臂当前位置，静止
    ROS_INFO("Force below threshold, exiting drag mode.");
    //break;
    }
    }
    /*退出拖拽*/
    cleanup_shared_memory();
    std::cout<<Bd<<std::endl;
    return 0;
}