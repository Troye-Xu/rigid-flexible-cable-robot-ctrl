from time import sleep
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.client import Client
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint
from baichuan_manipulation_interfaces.srv import GenerateManipulation
from baichuan_manipulation_interfaces.msg import *
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
import math

class TestClient(Node):

    def __init__(self):
        super().__init__('test_client')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/position_trajectory_controller/follow_joint_trajectory')
        self.joint_names = ["elfin_joint1", "elfin_joint2", "elfin_joint3", "elfin_joint4", "elfin_joint5", "elfin_joint6"]
        self.joint_state_sub = self.create_subscription(JointState,"/joint_states", self.joint_state_cb,10)
        
        self.traj = GenerateManipulation.Response()
        self.vel_factors = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0] 
        self.acc_factors = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        self.curr_positions = [0, 0, 0, 0, 0, 0]
        self.has_state = False

    def joint_state_cb(self, state: JointState):
        for i in range(len(self.joint_names)):
            for j in range(len(state.name)):
                if self.joint_names[i] == state.name[j]:
                    self.curr_positions[i] = state.position[j]
                    break
        self.has_state = True


    # 回零位函数
    def go_home(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.header = Header()
        goal.trajectory.joint_names = self.joint_names
        home = JointTrajectoryPoint()
        home.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        home.time_from_start.sec = 10
        goal.trajectory.points.append(home)
        print("Send goal")
        self.future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, self.future)

    # 参数辨识
    def canshu_bianshi(self):
        # 优化得到的傅里叶曲线参数（辨识用）
        x = [-0.066649,-0.182234,0.395935,-0.302076,-0.236603,-0.447454,0.477641,-0.078327,0.499308,-0.237926,-0.271084,-0.097935,-0.494438,-0.258045,0.448764,0.462144,-0.049582,-0.451156,0.495543,-0.088103,0.184381,-0.471845,-0.403021,-0.066616,-0.382071,-0.224352,0.303655,-0.395727,0.447769,-0.306604,0.106264,-0.257773,-0.183162,0.229105,0.199710,0.267375,-0.395571,-0.293837,-0.045116,-0.443098,0.289571,-0.330898]
        
        # 验证用
        # x= [-0.222056,-0.006059,-0.089869,0.263485,-0.189340,-0.087279,0.262281,-0.156426,-0.234083,-0.035434,-0.117689,0.262606,-0.265912,0.207782,0.285512,-0.075173,-0.156305,-0.007254,0.230377,0.243631,-0.269047,-0.252871,-0.233947,0.110876,-0.151933,-0.004493,0.221763,-0.223568,0.019015,0.072753,-0.255093,0.096363,0.190486,-0.080562,-0.030021,-0.226009,-0.120207,0.211191,-0.284508,-0.051374,-0.092913,0.217955]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.header = Header()
        goal.trajectory.joint_names = self.joint_names
        t = 15 # 第一步到达初始位置，所以多给了时间
        w = 0.1*math.pi
        w2 = w*w
        times = int(2*math.pi/w*5)
        # times = 4 # 测试用
        q_j = [0, 0, 0, 0, 0, 0]
        qd_j = [0, 0, 0, 0, 0, 0]
        qdd_j = [0, 0, 0, 0, 0, 0]
        for i in range(times):
            k = 0
            for j in range(6):
                q_j[j] = x[k]+x[k+1]*math.sin(w*(t-15))+x[k+2]*math.cos(w*(t-15))+x[k+3]*math.sin(2*w*(t-15))+x[k+4]*math.cos(2*w*(t-15))+x[k+5]*math.sin(3*w*(t-15))+x[k+6]*math.cos(3*w*(t-15))
                qd_j[j] = x[k+1]*w*math.cos(w*(t-15))-x[k+2]*w*math.sin(w*(t-15))+x[k+3]*2*w*math.cos(2*w*(t-15))-x[k+4]*2*w*math.sin(2*w*(t-15))+x[k+5]*3*w*math.cos(3*w*(t-15))-x[k+6]*3*w*math.sin(3*w*(t-15))
                qdd_j[j] = -x[k+1]*w2*math.sin(w*(t-15))-x[k+2]*w2*math.cos(w*(t-15))-x[k+3]*4*w2*math.sin(2*w*(t-15))-x[k+4]*4*w2*math.cos(2*w*(t-15))-x[k+5]*9*w2*math.sin(3*w*(t-15))-x[k+6]*9*w2*math.cos(3*w*(t-15))
                k = k+7
            waypoint = JointTrajectoryPoint()
            waypoint.positions = [q_j[0], q_j[1], q_j[2], q_j[3], q_j[4], q_j[5]]
            waypoint.velocities = [qd_j[0], qd_j[1], qd_j[2], qd_j[3], qd_j[4], qd_j[5]]
            waypoint.accelerations = [qdd_j[0], qdd_j[1], qdd_j[2], qdd_j[3], qdd_j[4], qdd_j[5]]
            waypoint.time_from_start.sec = t
            goal.trajectory.points.append(waypoint)
            t = t+1

        self._action_client.wait_for_server()
        self.future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self,self.future)
        print("success")

    # 是机械臂到达初始位置（预先给定）
    def go_chushi(self):
        chushi = FollowJointTrajectory.Goal()
        chushi.trajectory.header = Header()
        chushi.trajectory.joint_names = self.joint_names
        # x = [-0.276867,-0.295376,0.143849,0.298374,0.280011,-0.297769,0.298659,0.295401,-0.053200,0.299761,-0.284204,-0.216248,0.299639,0.299871,0.293911,0.281570,-0.257238,0.299739,0.298893,0.283484,0.296363,0.284515,-0.277831,0.299620,-0.296366,0.297807,-0.258588,-0.296312,0.288992,-0.264942,0.297539,0.295694,-0.107915,0.296857,0.089397,-0.057770,-0.293874,0.297317,0.182889,-0.284407,0.018427,-0.201622]
        chushi_point = JointTrajectoryPoint()
        chushi_point.positions = [0.610520497084958, 1.0021747181025304, 0.7863272784251787, 1.4263763861266683, 0.11477656625139224, -0.23087820257459704]  #不同轨迹不一样
        chushi_point.time_from_start.sec = 10
        chushi.trajectory.points.append(chushi_point)
        print("send goal")
        self.future = self._action_client.send_goal_async(chushi)
        rclpy.spin_until_future_complete(self,self.future)
        print("initial position")

class TestSubscriber(Node):

    def __init__(self):
        super().__init__('test_Subscriber')
        self.joint_names = ["elfin_joint1", "elfin_joint2", "elfin_joint3", "elfin_joint4", "elfin_joint5", "elfin_joint6"]
        self.joint_states_sub = self.create_subscription(JointState,"/joint_states", self.joint_states,10)
        # self.joint_states_sub = self.create_subscription(JointState,"/joint_states", 1)
        # time_period = 1  #seconds
        # self.timer = self.create_timer(time_period, self.joint_states)

    def joint_states(self, state: JointState):
        print("hello")
        fp = open(file="/home/fei/baichuan_ws/src/baichuan-moying-elfin-drivers/moying_elfin_hardware/scripts/joint_states.txt",mode="a+")
        pos = [0, 0, 0, 0, 0, 0]
        vel = [0, 0, 0, 0, 0, 0]
        eff = [0, 0, 0, 0, 0, 0]
        for i in range(len(self.joint_names)):
            for j in range(len(state.name)):
                if self.joint_names[i] == state.name[j]:
                    pos[i] = state.position[j]
                    vel[i] = state.velocity[j]
                    eff[i] = state.effort[j]
                    break
                    # for k in range(6):
        for m in range(6):
            fp.write(str(pos[m]))
            fp.write(' ')
        fp.write('\n')
        for n in range(6):
            fp.write(str(vel[n]))
            fp.write(' ')
        fp.write('\n')
        for p in range(6):
            fp.write(str(eff[p]))
            fp.write(' ')
        fp.write('\n')
        fp.close

def main(args=None):
    rclpy.init(args=args)

    action_client = TestClient()
    Subscriber = TestSubscriber()
    
    while not action_client.has_state:
        rclpy.spin_once(action_client)
        print("Wait for joint states")
        sleep(0.1)   
        
    # rclpy.spin_once(Subscriber)
    # sleep(0.1)

    action_client._action_client.wait_for_server()
    # action_client.go_home()
    # action_client.go_chushi()
    action_client.canshu_bianshi()
    # rclpy.spin(Subscriber)
    shuju_times = 0
    while shuju_times<130: # 运行10个周期（多于10个周期，取中间部分）
        shuju_times = shuju_times+1
        print(shuju_times)
        rclpy.spin_once(Subscriber)
        sleep(0.1)


if __name__ == '__main__':
    main()