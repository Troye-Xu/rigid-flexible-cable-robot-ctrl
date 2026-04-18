from time import sleep
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.client import Client
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
import math
import numpy as np
import rbdl
import matplotlib.pyplot as plt # 绘制方程曲线图
import sympy # 求解方程
import pybullet as p
import keyboard

#动力学参数辨识计算预测力矩（在不考虑末端力的情况下，机械臂所需要的关节力矩）
#实际力矩由拖拽过程中，电机反馈的实时力矩
#力矩作差可以求出末端力/关节控制量

# 牛顿欧拉递推函数
def rne2(I, Mr, fc, fv, Jm, R, P, qd, qdd):
    z0 = np.array([0, 0, 1])
    g = np.array([0, 0, -9.8])

    omega = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    alpha = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    a = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    M = np.zeros(6) # 最小惯性参数矩阵中没有M，所以直接设为0

    omega[0,:] = z0*qd[0]
    alpha[0,:] = np.cross(omega[0], np.dot(z0, qd[0])) + np.dot(z0, qdd[0])
    a[0,:] = np.dot(-g, np.transpose(R[0]))  

    for i in [1,2,3,4,5]:
        omega[i] = np.dot(omega[i-1], np.linalg.inv(np.transpose(R[i]))) + z0*qd[i]
        alpha[i] = np.dot(alpha[i-1], np.linalg.inv(np.transpose(R[i]))) + z0*qdd[i] + np.cross(omega[i], z0*qd[i])
        a[i] = np.dot((a[i-1] + np.cross(alpha[i-1], P[i]) + np.cross(omega[i-1], np.cross(omega[i-1], P[i]))), np.linalg.inv(np.transpose(R[i])))

    f = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    n = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])

    f[5] = np.dot(M[5], a[5]) + np.cross(alpha[5], Mr[5]) + np.cross(omega[5], np.cross(omega[5], Mr[5]))
    n[5] = np.dot(I[5], alpha[5]) + np.cross(omega[5], np.dot(I[5], omega[5])) + np.cross(Mr[5], a[5])

    for i in [4,3,2,1,0]:
        f[i] = np.dot(M[i], a[i]) + np.cross(alpha[i], Mr[i]) + np.cross(omega[i], np.cross(omega[i], Mr[i])) + np.dot(R[i+1], f[i+1])
        n[i] = np.dot(I[i], alpha[i]) + np.cross(omega[i], np.dot(I[i], omega[i])) + np.cross(Mr[i], a[i]) + np.cross(P[i+1], np.dot(R[i+1], f[i+1])) + np.dot(R[i+1], n[i+1])

    t = np.zeros(6)
    for i in range(6):
        t[i] = np.dot(n[i], z0)-np.dot(fc[i], np.sign(qd[i]))-np.dot(fv[i], qd[i])-np.dot(Jm[i], qdd[i])
    return t

# 直接用MDH参数求齐次变换矩阵
def Transform_MDH(q):
    MDH_d = [0.22, 0.0, 0.0, 0.42, 0.0, 0.155]
    MDH_a = [0.0, 0.0, 0.38, 0.0, 0.0, 0.0]
    MDH_alpha = [0.0, 1.571, 3.142, -1.571, 1.571, -1.571]
    MDH_theta = [q[0], 1.571+q[1], -1.571+q[2], q[3], q[4], q[5]]

    T = [np.eye(4) for i in range(6)]
    R = [np.eye(3) for i in range(6)]
    P = [np.array([0, 0, 0]) for i in range(6)]

    for i in range(6):
        c_theta = math.cos(MDH_theta[i])
        s_theta = math.sin(MDH_theta[i])
        c_alpha = math.cos(MDH_alpha[i])
        s_alpha = math.sin(MDH_alpha[i])
        T[i][0,0] = c_theta
        T[i][0,1] = -s_theta
        T[i][0,2] = 0
        T[i][0,3] = MDH_a[i]
        T[i][1,0] = s_theta*c_alpha
        T[i][1,1] = c_theta*c_alpha
        T[i][1,2] = -s_alpha
        T[i][1,3] = -MDH_d[i]*s_alpha
        T[i][2,0] = s_theta*s_alpha
        T[i][2,1] = c_theta*s_alpha
        T[i][2,2] = c_alpha
        T[i][2,3] = MDH_d[i]*c_alpha
        T[i][3,0] = 0
        T[i][3,1] = 0
        T[i][3,2] = 0
        T[i][3,3] = 1
        R[i] = T[i][0:3, 0:3]
        P[i] = T[i][0:3, 3]
        # print(T[i])
        # print('\n')
    return T, R, P

def Pwm_to_inertia(Pwm):
    I = [np.eye(3) for i in range(6)]
    Mr = [np.zeros(3) for i in range(6)]
    fc = np.zeros(6)
    fv = np.zeros(6)
    Jm = np.zeros(6)

    # 构建惯量矩阵
    I[0] = [[0, 0, 0], [0, 0, 0], [0, 0, Pwm[0]]]
    I[1] = [[Pwm[3], Pwm[4], Pwm[5]], [Pwm[4], 0, Pwm[6]], [Pwm[5], Pwm[6], Pwm[7]]]
    I[2] = [[Pwm[12], Pwm[13], Pwm[14]], [Pwm[13], 0, Pwm[15]], [Pwm[14], Pwm[15], Pwm[16]]]
    I[3] = [[Pwm[22], Pwm[23], Pwm[24]], [Pwm[23], 0.0, Pwm[25]], [Pwm[24], Pwm[25], Pwm[26]]]
    I[4] = [[Pwm[32], Pwm[33], Pwm[34]], [Pwm[33], 0.0, Pwm[35]], [Pwm[34], Pwm[35], Pwm[36]]]
    I[5] = [[Pwm[42], Pwm[43], Pwm[44]], [Pwm[43], 0.0, Pwm[45]], [Pwm[44], Pwm[45], Pwm[46]]]

    # 构建质量矩矩阵
    Mr[0] = [0.0, 0.0, 0.0]
    Mr[1] = [Pwm[8], Pwm[9], 0.0]
    Mr[2] = [Pwm[17], Pwm[18], 0.0]
    Mr[3] = [Pwm[27], Pwm[28], 0.0]
    Mr[4] = [Pwm[37], Pwm[38], 0.0]
    Mr[5] = [Pwm[47], Pwm[48], 0.0]

    # 构建fc， fv， Jm
    fc = [Pwm[1], Pwm[10], Pwm[19], Pwm[29], Pwm[39], Pwm[49]]
    fv = [Pwm[2], Pwm[11], Pwm[20], Pwm[30], Pwm[40], Pwm[50]]
    Jm = [0.0, 0.0, Pwm[21], Pwm[31], Pwm[41], Pwm[51]]

    return I, Mr, fc, fv, Jm

# 机器人模型类
class SimRobot:
    def __init__(self, urdfFileName, basePosition=[0,0,0], baseRPY=[0,0,0], jointPositions=None, useFixedBase=True, verbose=True):

        self.id = p.loadURDF(fileName=urdfFileName,
                                    basePosition=basePosition,
                                    baseOrientation=p.getQuaternionFromEuler(baseRPY),
                                    useFixedBase=useFixedBase)

        if verbose:
            print('*' * 100 + '\nPyBullet Robot Info ' + '\u2193 '*20 + '\n' + '*' * 100)
            print('robot ID:              ', self.id)
            # print('robot name:            ', self.getRobotName())
            # print('robot total mass:      ', self.getTotalMass())
            # print('base link name:        ', self.getBaseName())
            # print('num of joints:         ', self.getNumJoints())
            # print('num of actuated joints:', self.getNumActuatedJoints())
            # print('joint names:           ', len(self.getJointNames()), self.getJointNames())
            # print('joint indexes:         ', len(self.getJointIndexes()), self.getJointIndexes())
            # print('actuated joint names:  ', len(self.getActuatedJointNames()), self.getActuatedJointNames())
            # print('actuated joint indexes:', len(self.getActuatedJointIndexes()), self.getActuatedJointIndexes())
            # print('link names:            ', len(self.getLinkNames()), self.getLinkNames())
            # print('link indexes:          ', len(self.getLinkIndexes()), self.getLinkIndexes())
            # print('joint dampings:        ', self.getJointDampings())
            # print('joint frictions:       ', self.getJointFrictions())
            print('*' * 100 + '\nPyBullet Robot Info ' + '\u2191 '*20 + '\n' + '*' * 100)

            # 获取关节信息
            self.num_joints = p.getNumJoints(self.id)
            self.joint_ids = []
            for i in range(self.num_joints):
                joint_info = p.getJointInfo(self.id, i)
                joint_type = joint_info[2]
                if joint_type == p.JOINT_REVOLUTE:
                    self.joint_ids.append(joint_info[0])
                    p.resetJointState(self.id, i, 0.0)

class TestClient(Node):

    def __init__(self):
        super().__init__('test_client')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/position_trajectory_controller/follow_joint_trajectory')
        self.joint_names = ["elfin_joint1", "elfin_joint2", "elfin_joint3", "elfin_joint4", "elfin_joint5", "elfin_joint6"]
        # self.joint_state_sub = self.create_subscription(JointState,"/joint_states", self.joint_state_cb,10)
        

        self.vel_factors = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0] 
        self.acc_factors = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        self.curr_positions = [0, 0, 0, 0, 0, 0]
        self.has_state = False

    # 回零位函数
    def go_home(self):
        Home = FollowJointTrajectory.Goal()
        Home.trajectory.header = Header()
        Home.trajectory.joint_names = self.joint_names

        home = JointTrajectoryPoint()
        home.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        home.time_from_start.sec = 10
        Home.trajectory.points.append(home)

        self._action_client.wait_for_server()
        self.future = self._action_client.send_goal_async(Home)
        rclpy.spin_until_future_complete(self, self.future)

    # 使用关节位置作为指令
    def position_control(self, q_init, q_end):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.header = Header()
        goal.trajectory.joint_names = self.joint_names

        start = JointTrajectoryPoint()
        start.positions = q_init
        start.time_from_start.nanosec = 0
        goal.trajectory.points.append(start)

        end = JointTrajectoryPoint()
        end.positions = q_end
        end.time_from_start.nanosec = 100000000
        goal.trajectory.points.append(end)

        self._action_client.wait_for_server()
        self.future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, self.future)

    # 使机械臂到达初始构型
    def go_init_position(self, pos_init):
        Init = FollowJointTrajectory.Goal()
        Init.trajectory.header = Header()
        Init.trajectory.joint_names = self.joint_names

        home = JointTrajectoryPoint()
        home.positions = pos_init
        home.time_from_start.sec = 30
        Init.trajectory.points.append(home)

        self._action_client.wait_for_server()
        self.future = self._action_client.send_goal_async(Init)
        rclpy.spin_until_future_complete(self, self.future)

    # 检测任务是否完成
    def is_goal_finished(self):
        job_success = False
        if self.future.done():
            job_success = True
        else:
            job_success = False
        
        return job_success

# 记录实验数据
class TestSubscriber(Node):

    def __init__(self, robot, I, Mr, fc, fv, Jm, fp):
        super().__init__('test_Subscriber')
        self.joint_names = ["elfin_joint1", "elfin_joint2", "elfin_joint3", "elfin_joint4", "elfin_joint5", "elfin_joint6"]
        self.joint_states_sub = self.create_subscription(JointState,"/joint_states", self.joint_states,10)
        self.robot = robot
        self.I = I
        self.Mr = Mr
        self.fc = fc
        self.fv = fv
        self.Jm = Jm
        self.fp = fp
        self.collision = False
        self.pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.acc = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.T_actual = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.T_yuce = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.torque_error = np.zeros(6)
        self.torque_threshold = [32, 66, 30, 10, 8, 8]
        print("hi")

    def joint_states(self, state: JointState):
        print("wow")
        pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        acc = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        T_actual = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        T_yuce = np.zeros(6)

        for i in range(len(self.joint_names)):
            for j in range(len(state.name)):
                if self.joint_names[i] == state.name[j]:
                    pos[i] = state.position[j]
                    vel[i] = state.velocity[j]
                    T_actual[i] = state.effort[j]
                    break
        
        # 加速度滤波
        dt = 0.1
        if hasattr(self, 'prev_vel'):
            if hasattr(self, 'prev_vel_2'):
                for i in range(6):
                    vel[i] = (vel[i] + self.prev_vel[i] + self.prev_vel_2[i])/3
                    acc[i] = ((vel[i] - self.prev_vel[i]) / dt + (self.prev_vel[i] - self.prev_vel_2[i]) / dt + (vel[i] - self.prev_vel_2[i]) / (dt * 2)) / 3
            else:
                for i in range(6):
                    vel[i] = (vel[i] + self.prev_vel[i])/2
                    acc[i] = (vel[i] - self.prev_vel[i]) / dt
            self.prev_vel_2 = self.prev_vel
            self.prev_vel = vel
        else:
            self.prev_vel = vel
            self.prev_vel_2 = self.prev_vel

        # 获取相邻关节的齐次变换矩阵
        [T, R, P] = Transform_MDH(pos)

        for i in range(6):
            vel[i] = -vel[i]
            acc[i] = -acc[i]

        T_yuce = rne2(self.I, self.Mr, self.fc, self.fv, self.Jm, R, P, vel, acc)
        for i in range(6):
            self.torque_error[i] = T_yuce[i]-T_actual[i]

        self.pos = pos
        self.vel = vel 
        self.acc = acc
        self.T_actual = T_actual
        self.T_yuce = T_yuce

    # 返回关节信息
    def param(self):
        return self.pos, self.vel, self.acc, self.T_actual, self.T_yuce

    def detect(self):
        self.collision = False
        for i in range(6):
            if abs(self.torque_error[i])>self.torque_threshold[i]:
                self.collision = True
        #         print("Ops!")
        #         print(self.collision)
        #         print(str(self.torque_error))
                
        return self.collision

def main(args=None):
    rclpy.init(args=args)
    global g_collision # 运动过程中是否发生碰撞的标志为
    global g_job_success # 是否完成预定运动，目前是30s，关节一旋转3rad

    g_collision = False
    g_job_success = False

    file_path = "/home/fei/baichuan_ws/src/baichuan-moying-description/urdf/elfin5.urdf"
    fp = open(file="/home/fei/baichuan_ws/src/baichuan-moying-elfin-drivers/moying_elfin_hardware/scripts/shijiao_byend.txt",mode="a+")

    # 加载机械臂的urdf文件
    elfin5 = rbdl.loadModel('/home/fei/baichuan_ws/src/baichuan-moying-description/urdf/elfin5.urdf')
    # physcicsClient = p.connect(p.GUI)
    # elfin5_pybullet = SimRobot(file_path)
    
    Pwm = [
        7.14684273405354, -14.3749027627186, -31.9093506972696,
        -1.87730806617133, -0.736793732673288, 3.30811976586476, 1.90193999661596, 37.4456764149143, 5.51207896976902, -1.82020391638184, -16.7607997662530, -16.4258781011924,
        1.08507412556632, -0.977934260152434, -0.539714611014304, 1.99584824220336, -24.1326097659742, 1.14699218863913, 0.0594298061229971, -4.47741299307103, 1.14651799938551, -55.9018541406288,
        -0.289011551956923, -0.649286101395709, 0.180090554793942, -0.968496508139869, -0.537023279019607, 0.302676195808783, 0.0156823919992025, -3.54758189060547, -5.41818583339489, -1.05560240519674,
         -0.255063486940759, -0.535470942717123, 0.0649319892953272, -0.0880573201813887, -1.18445717639398, 0.200851353044823, -0.268825035953812, -0.323542989023054, -5.96127604049606, -6.82290937325223, 
         1.21785060908784, 0.503954052169576, -1.01558109822440, -0.234789226965282, 0.0565105535176195, -0.0112641074390819, -0.180522355958483, -1.95231642150384, -0.850070762791495, -0.658355388496904
         ]
    [I, Mr, fc, fv, Jm] = Pwm_to_inertia(Pwm)

    # 初始化机械臂
    action_client = TestClient()
    Subscriber = TestSubscriber(elfin5, I, Mr, fc, fv, Jm, fp)

    # 定义关节角度的最大增量
    MAX_JOINT_INCREMENT = 1
    MAX_end_INCREMENT = 1

    # 定义刚度、阻尼、质量系数
    M = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    B = np.array([50.0, 50.0, 50.0, 5.0, 5.0, 5.0])
    K = np.array([1000, 1000, 1000, 100, 100, 100])

    # 定义示教的初始位置
    joint_init = [9.68687647e-01, 1.32917246e-01, 1.37100461e+00, 3.17544435e-02, 5.92377126e-01, 1.13960700e-08]
    # end_point_init = np.array([0.21837327, 0.32383671, 0.63620559]) #这是用pybullet作规划时的末端位置
    end_point_init = np.array([[0.27435751, 0.40380339, 0.69624581]]) #这是用rbdl作规划时的末端位置


    # 关节位置初始化
    joint_pos = joint_init
    joint_pos_next = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # 拖动示教
    print("拖动示教开始! 输入'y'结束示教")
    while True:
        if keyboard.is_pressed("y"):  # 检测是否按下了 "end" 键
            fp.close
            break  # 结束循环

        rclpy.spin_once(Subscriber)
        joint_pos_state, joint_vel_state, joint_acc_state, torque_actual, torque_predict = Subscriber.param()

        move = Subscriber.detect()

        if move == True:

            # 计算当前的末端位置
            end = rbdl.CalcBodyToBaseCoordinates(elfin5, np.array(joint_pos), 6, np.array([0.0, 0.0, 0.0]))  # 仿真试一下rbdl准不准

            # 记录示教的关节角度
            for i in range(6):
                fp.write(str(joint_pos[i]))
                fp.write(' ')
            fp.write('\n')
            # for i in range(6):
            #     fp.write(str(joint_vel[i]))
            #     fp.write(' ')
            # fp.write('\n')
            # for i in range(6):
            #     fp.write(str(joint_acc[i]))
            #     fp.write(' ')
            # fp.write('\n')

            # 计算关节力矩增量
            joint_torque_increment = np.array([(torque_predict[i] - torque_actual[i]) for i in range(6)])

            # 计算力雅可比矩阵
            Jacobian = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
            rbdl.CalcPointJacobian(elfin5, np.array(joint_pos), 6, np.array([0.0, 0.0, 0.0]), Jacobian)
            Jacobian_T = np.transpose(Jacobian)

            # 计算末端力的误差
            Jacobian_mul = np.dot(np.linalg.inv(np.dot(Jacobian, Jacobian_T)), Jacobian)
            F_end = np.dot(Jacobian_mul, joint_torque_increment) #若求逆解失败，咋办

            # 计算末端位置增量，在末端限位
            end_error = [((F_end[i]-M[i]*joint_acc_state[i]-B[i]*joint_vel_state[i])/K[i]) for i in range(3)]
            end_error_increment = [np.clip(end_error[i] / MAX_end_INCREMENT, -0.002, 0.002) for i in range(3)]

            # 求解新的末端位置
            end_next = np.array([[(end[i] + end_error_increment[i]) for i in range(3)]])

            # 更新机械臂的关节角度
            lala = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            symbol = rbdl.InverseKinematics(elfin5, np.array(joint_pos), np.array([6]), np.array([[0.0, 0.0, 0.0]]), end_next, lala)

            for i in range(6):
                joint_pos_next[i] = lala[i]

            for i in range(6):
                if joint_pos_next[i]>math.pi*2:
                    multiple = math.floor(joint_pos_next[i]/(math.pi*2))
                    joint_pos_next[i] = joint_pos_next[i]-multiple*math.pi*2
                elif joint_pos_next[i]<math.pi*(-2):
                    multiple = math.floor(joint_pos_next[i]/(math.pi*(-2)))
                    joint_pos_next[i] = joint_pos_next[i]+multiple*math.pi*2

            if symbol == False:
                print("求解逆运动学失败")

            # 关节层限位
            # joint_pos_next = np.array([np.clip(joint_pos_next[i], -0.01, 0.01) for i in range(6)])

            print(joint_pos_next)

            action_client.position_control(joint_pos, joint_pos_next)

            joint_pos = joint_pos_next
            sleep(0.1)

    # 机械臂回零
    user_input = input("是否返回初始位置(yes or no):")
    if user_input == 'yes':
        action_client.go_init_position(joint_init)   
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()