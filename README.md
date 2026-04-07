# rigid-flexible-cable-robot-ctrl
Control software for a 7-DOF rigid-flexible hybrid cable-driven robot based on ROS Noetic and STM32F407, implementing integrated functions of inverse kinematics, PID closed-loop control, trajectory planning, and visual recognition
# 刚柔混合绳驱异构机器人运动学求解与实验控制软件
面向实验室自主研发的7自由度刚柔混合绳驱异构机器人，开发的一体化专用控制软件。基于ROS Noetic搭建上位机运动规划与解算框架，STM32F407实现下位机电机驱动与闭环控制，解决传统串口调试操作繁琐、ROS方案集成度低的痛点，实现机器人全流程运动控制与实验验证。

## 项目亮点
1. **软硬件协同控制**：上位机实现7自由度冗余机械臂运动学逆解、轨迹规划，下位机实现多电机CAN总线驱动、PID闭环控制，系统端到端控制延迟<10ms；
2. **核心运动学算法**：基于臂形角参数化方法实现冗余机械臂位置级逆运动学求解，内置奇异位形校验、关节安全角度限位，解算成功率100%；
3. **高精度闭环控制**：设计增量式PID+前馈PID双控制方案，有效抑制刚柔混合结构的机械抖动、绳松弛问题，关节定位精度±0.05rad；
4. **全功能模块化设计**：解耦系统自检、运动学解算、PID控制、轨迹规划、视觉识别、串口调试模块，支持功能扩展与算法迭代；
5. **完整安全机制**：强制通信自检流程、电机使能保护、异常状态锁定、急停复位功能，保障设备与人身安全。

## 技术栈
- **上位机**：Ubuntu 20.04 LTS、ROS Noetic、C/C++、Eigen矩阵库、运动学逆解、五次多项式轨迹规划
- **下位机**：STM32F407、C语言、HAL库、CAN总线/串口通信、DMA中断、增量式PID控制
- **硬件适配**：达妙伺服电机、大疆M2006电机、CH340 USB转串口、绝对值编码器
- **工程工具**：Git、Makefile、串口调试助手

## 运行环境配置
### 上位机环境
1. 安装Ubuntu 20.04 LTS + ROS Noetic桌面完整版；
2. 安装依赖库：`sudo apt-get install libeigen3-dev ros-noetic-serial`；
3. 配置CH340串口驱动，添加串口权限：`sudo usermod -aG dialout $USER`；
4. 克隆仓库后，进入ros_ws目录，执行`catkin_make`编译ROS功能包；
5. 编译完成后执行`source devel/setup.zsh`加载环境变量。

### 下位机环境
1. 安装arm-none-eabi-gcc交叉编译工具链；
2. 进入STM32_Project目录，执行`make`编译工程，生成hex/bin固件；
3. 通过ST-Link或串口将固件烧录到STM32F407开发板。

## 核心功能与使用指令
### 1. 系统初始化与自检
```bash
# 加载ROS环境
source devel/setup.zsh
# 执行串口与编码器状态自检（控制前强制流程）
rosrun motor_ctrl encoder_check
# 系统复位，恢复关节初始位置与控制参数
rosrun motor_ctrl exp_wu_init
