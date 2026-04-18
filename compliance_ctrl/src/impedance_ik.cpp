#include "ros/ros.h"
#include <iostream>
#include <Eigen/Dense>
#include <compliance_ctrl/impedance.h>
#include "jxb_ctrl/jxb_math.h"
#include "jxb_ctrl/kine.h"
#include "jxb_ctrl/Interp5rdPoly.h"
#include "FT_api.h" 


int main(int argc, char *argv[]) 
{

    /*机械臂初始化*/
    int JNT_NUM = 7;
    float DH[JNT_NUM][4];
    float JntCurrent[7] ={0};
    float JntTarget[7] ={0};
    float Jaco_0[6][7];
    float J0_pinv[7][6];
    float Jaco_rel_inv[7][6];
    float T0e[4][4];
    float T0n[4][4];
    float R0e[3][3];
    float R0n[3][3];
    float R0e_1[3][3];
    float Eulder_xyz[3][3];
    float Eulder_xyz_inv[3][3];
    float P0e[3];
    float theta0e[3];
    float delta_w[3];
    float Euler_w[3];
    float X0e[6];
    float d1 = 0.3025,d3 = 0.465, d5 = 0.440,d7 = 0.06;
    float dx[6];
    //theta alpha a d
 	DH[0][0] = -M_PI/2; DH[0][1] = M_PI/2;  DH[0][2] = 0; DH[0][3] = d1;
    DH[1][0] = M_PI;    DH[1][1] = M_PI/2;  DH[1][2] = 0; DH[1][3] = 0;
    DH[2][0] = 0;       DH[2][1] = -M_PI/2; DH[2][2] = 0; DH[2][3] = d3;
    DH[3][0] = 0;       DH[3][1] = M_PI/2;  DH[3][2] = 0; DH[3][3] = 0;
    DH[4][0] = 0;       DH[4][1] = -M_PI/2; DH[4][2] = 0; DH[4][3] = d5;
    DH[5][0] = 0;       DH[5][1] = M_PI/2;  DH[5][2] = 0; DH[5][3] = 0;
    DH[6][0] = -M_PI/2;   DH[6][1] = 0;     DH[6][2] = 0; DH[6][3] = d7;
   // 初始化限幅
   const float MAX_JOINT_SPEED = 0.4;  // 关节速度最大值
    const float MAX_JOINT_POS_CHANGE = 0.02;  // 单次循环关节位置变化最大值
    const float JOINT_POS_LIMITS[7][2] = {  // 关节位置上下限
        {-90.0/180.0*M_PI, 90.0/180.0*M_PI},
        {-80.0/180.0*M_PI, 80.0/180.0*M_PI},
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
       float limit_force = 1.0;
       float tc = 0.0;
       float dt = 0.01;
       //初始化质量、阻尼、刚度
       Md << 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f;
       Bd <<20.0f, 20.0f, 20.0f, 2.0f, 2.0f, 2.0f;
       Kd << 50.0f, 50.0f, 50.0f, 5.0f, 5.0f, 5.0f;
       IMPEDANCE impedance;
        //初始化标志位
        bool is_dragging = false;  // 拖拽模式标志位
        bool exit_dragging = false;  // 拖拽模式标志位
         JntCurrent[1] = -15/180.0*M_PI;
        JntCurrent[3] = 60/180.0*M_PI;
        JntCurrent[5] = -45/180.0*M_PI; 

        /*计算初始末端位姿*/
    forward_kine(JntCurrent, DH, T0e);
  for(int i=0; i<1000;i++)
    {
        //std::cout << "Input_force: " << Input_force.transpose() << std::endl;
        Input_force << 0.0, 10.0, 0.0, 0.0, 0.0, 0.0;
    //判断是否达到拖拽阈值，获取初始末端力
    float force_magnitude = Input_force.norm();  // 计算力的模长
    if (force_magnitude > limit_force && !is_dragging) {
            is_dragging = true;
        }

    if (is_dragging) 
    {
        //进入导纳控制模式
        //更新关节位置(弧度制)
        forward_kine(JntCurrent, DH, T0e);
        
       /*  for(int i =0;i<7;i++)
        {
        std::cout<<JntCurrent[i]/M_PI*180<<" ";
        }
        std::cout<<std::endl; */

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
        Eigen::VectorXf delta_x = impedance.Trans_fun(Input_force, Md, Bd, Kd, tc, dt);
       for (int i = 0; i < 6; i++)
       {
        dx[i] = delta_x(i);
       }
        /*更新姿态*/
        /*方式一：小角度xyz欧拉角处理*/
        nfZyxEuler(dx, T0n);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                R0e[i][j] = T0e[i][j];
                R0n[i][j] = T0n[i][j];
            }
        }
        Rbt_MulMtrx(3, 3, 3, R0n[0], R0e[0], R0e_1[0]);
        for (int i = 0; i < 3; i++)
        {
            T0e[i][3] = T0e[i][3] + T0n[i][3];
             for (int j = 0; j < 3; j++)
            {
                T0e[i][j] = R0e_1[i][j];
            } 
        }   

        /*方式二：欧拉角求解*/
/*         JEulder_xyz_0(dx, Eulder_xyz);
        Rbt_InvMtrx(Eulder_xyz[0], Eulder_xyz_inv[0], 3);
        trans2oula(T0e, P0e, theta0e);
        Rbt_MulMtrx(3, 3, 1, Eulder_xyz_inv[0], delta_w, Euler_w);
        for (int i = 0; i < 3; i++)
        {
            X0e[i] = dx[i] + T0e[i][3];
            X0e[i+3] = theta0e[i] + Euler_w[i];
        }
        nfZyxEuler(X0e, T0e);   */

        //std::cout <<"Delta_x (Eigen::VectorXf): " << delta_x.transpose() << std::endl;  
        Eigen::VectorXf Joint_vel = Jaco_0_pinv * delta_x;
        float Joint_vel_array[7];
        for (int i = 0; i < 7; i++) {
        Joint_vel_array[i] = Joint_vel(i);
        }
        //速度限幅
        for (int i = 0; i < 7; i++) {
            Joint_vel_array[i] = std::max(std::min(Joint_vel_array[i], MAX_JOINT_SPEED), -MAX_JOINT_SPEED);
        }
        //关节变化量限幅
        for (int i = 0; i < 7; i++) {
            float pos_change = Joint_vel_array[i] * dt;
            //pos_change = std::max(std::min(pos_change, MAX_JOINT_POS_CHANGE), -MAX_JOINT_POS_CHANGE);
            JntTarget[i] = JntCurrent[i] + pos_change;
        // 关节位置限幅
            JntTarget[i] = std::max(std::min(JntTarget[i], JOINT_POS_LIMITS[i][1]), JOINT_POS_LIMITS[i][0]);
        }
        ikine_Pos_Level(DH, T0e, 0.0, JntCurrent);
   /*      for(int i = 0;i<7;i++)
        {
                JntCurrent[i] = JntTarget[i];
        }   */

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
    return 0;
}