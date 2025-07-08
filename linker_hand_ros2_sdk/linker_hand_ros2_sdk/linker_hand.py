#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
'''
编译: colcon build --symlink-install
启动命令:ros2 run linker_hand_ros2_sdk linker_hand_sdk
'''
import rclpy,math,sys                                     # ROS2 Python接口库
from rclpy.node import Node                      # ROS2 节点类
import rclpy.time
from std_msgs.msg import String, Header, Float32MultiArray
from sensor_msgs.msg import JointState
import time,threading, json

from linker_hand_ros2_sdk.LinkerHand.linker_hand_api import LinkerHandApi
from linker_hand_ros2_sdk.LinkerHand.utils.color_msg import ColorMsg
from linker_hand_ros2_sdk.LinkerHand.utils.open_can import OpenCan


class LinkerHand(Node):
    def __init__(self, name):
        super().__init__(name)
        # 声明参数（带默认值）
        # hand_type: 手型（左/右）
        # hand_joint: 关节类型（如L7/L10等）
        # is_touch: 是否带触觉功能
        # can: 使用的CAN通道
        self.declare_parameter('hand_type', 'right')
        self.declare_parameter('hand_joint', 'L7')
        self.declare_parameter('is_touch', True)
        self.declare_parameter('can', 'can0')
        
        # 获取参数值，并保存为成员变量
        self.hand_type = self.get_parameter('hand_type').value  # 手型（左/右）
        self.hand_joint = self.get_parameter('hand_joint').value  # 关节类型
        self.is_touch = self.get_parameter('is_touch').value  # 是否带触觉
        self.can = self.get_parameter('can').value  # CAN通道

        # 运动相关缓存变量
        self.last_left_hand_move_pose = []  # 左手上一次运动的位置
        self.last_right_hand_move_pose = [] # 右手上一次运动的位置
        self.vel = []  # 当前速度
        self.version = []  # 手的版本信息
        self.touch_type = -1  # 触觉类型，-1为无
        self.t_force = [-1] * 5  # 五指触觉力传感器初值

        # 保存上一次手的信息（如版本、关节类型、速度、电流、故障等）
        self.last_hand_info = {
            "version": [], # 灵巧手版本号
            "hand_joint": self.hand_joint, # 灵巧手关节类型
            "speed": [], # 当前速度阈值
            "current": [], # 当前电流
            "fault": [], # 当前故障
            "motor_temperature": [], # 当前电机温度
            "torque": [], # 当前力矩
            "is_touch":self.is_touch, # 是否带触觉
            "touch_type": self.touch_type, # 触觉类型
            "finger_order": [] # 手指电机顺序
        }
        # 保存上一次手的状态（如状态和速度）
        self.last_hand_state = {
            "state": [], # 当前状态
            "vel": []    # 当前速度
        }
        # 触觉信息缓存
        self.last_hand_matrix_touch = String()  # 矩阵触觉消息
        self.last_hand_touch = String()         # 单点触觉消息
        # CAN通信对象
        self.open_can = OpenCan()
        # 订阅设置命令的topic
        self.hand_setting_sub = self.create_subscription(String,'/hand_setting_cmd', self.hand_setting_cb, 10)
        # 控制频率相关变量
        self.last_process_time = 0  # 上一次处理的时间戳
        self.max_hz = 30            # 最大处理频率
        self.min_interval = 1.0 / self.max_hz # 最小处理间隔
        # 线程锁，保证多线程安全
        self.lock = threading.Lock()
        # 初始化手（根据参数自动初始化左手或右手）
        self.init_hand(hand_type=self.hand_type)

        
        

    def init_hand(self,hand_type):
        if hand_type == "left":
            self.api = LinkerHandApi(hand_type=hand_type, hand_joint=self.hand_joint,can=self.can)
            self.open_can.open_can(self.can)
            time.sleep(0.1)
            self.touch_type = self.api.get_touch_type()
            self.hand_cmd_sub = self.create_subscription(JointState, '/left_hand_control_cmd', self.left_hand_control_cb,10)
            self.hand_cmd_arc_sub = self.create_subscription(JointState, '/left_hand_control_cmd_arc', self.left_hand_control_arc_cb,10)
            self.hand_state_pub = self.create_publisher(JointState, '/left_hand_state',10)
            self.hand_state_arc_pub = self.create_publisher(JointState, '/left_hand_state_arc',10)
            self.hand_info_pub = self.create_publisher(String, '/left_hand_info', 10)
            if self.is_touch == True:
                if self.touch_type == 2:
                    self.matrix_touch_pub = self.create_publisher(String, '/left_hand_matrix_touch', 10)
                elif self.touch_type != -1:
                    self.touch_pub = self.create_publisher(Float32MultiArray, '/left_hand_force', 10)
        elif hand_type == "right":
            self.api = LinkerHandApi(hand_type=hand_type, hand_joint=self.hand_joint,can=self.can)
            self.open_can.open_can(self.can)
            time.sleep(0.1)
            self.touch_type = self.api.get_touch_type()
            self.hand_cmd_sub = self.create_subscription(JointState, '/right_hand_control_cmd', self.right_hand_control_cb,100)
            self.hand_cmd_arc_sub = self.create_subscription(JointState, '/right_hand_control_cmd_arc', self.right_hand_control_arc_cb,10)
            self.hand_state_pub = self.create_publisher(JointState, '/right_hand_state',10)
            self.hand_state_arc_pub = self.create_publisher(JointState, '/right_hand_state_arc',10)
            self.hand_info_pub = self.create_publisher(String, '/right_hand_info', 10)
            if self.is_touch == True:
                if self.touch_type == 2:
                    self.matrix_touch_pub = self.create_publisher(String, '/right_hand_matrix_touch', 10)
                elif self.touch_type != -1:
                    self.touch_pub = self.create_publisher(Float32MultiArray, '/right_hand_force', 10)
        self.version = self.api.get_version()
        pose = None
        torque = [200, 200, 200, 200, 200]
        speed = [200, 250, 250, 250, 250]
        if self.hand_joint == "L7":
            # The data length of L7 is 7, reinitialize here
            pose = [0, 120, 200, 200, 200, 200, 120]   
            torque = [120, 120, 120, 120, 120, 120, 120]
            speed = [120, 120, 120, 120, 120, 120, 120]
        elif self.hand_joint == "L10":
            pose = [255, 200, 255, 255, 255, 255, 180, 180, 180, 41]
            speed = [200, 250, 250, 250, 250, 250, 250, 250, 250, 250]
        elif self.hand_joint == "L20":
            pose = [255,255,255,255,255,255,10,100,180,240,245,255,255,255,255,255,255,255,255,255]
        elif self.hand_joint == "L21":
            pose = [75, 255, 255, 255, 255, 176, 97, 81, 114, 147, 202, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255]
        elif self.hand_joint == "L25":
            pose = [75, 255, 255, 255, 255, 176, 97, 81, 114, 147, 202, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255]
        if pose is not None:
            self.api.set_speed(speed=speed)
            self.api.set_torque(torque=torque)
            self.api.finger_move(pose=pose)


    def run(self):
        self.thread_get_state = threading.Thread(target=self._get_hand_state)
        self.thread_get_state.daemon = True
        self.thread_get_state.start()
        # self.thread_get_state.join()
        
        self.thread_get_info = threading.Thread(target=self.get_hand_info)
        self.thread_get_info.daemon = True
        self.thread_get_info.start()
        # self.thread_get_info.join()

        if self.touch_type == 2:
            self.thread_get_matrix_touch = threading.Thread(target=self.get_matrix_touch)
            self.thread_get_matrix_touch.daemon = True
            self.thread_get_matrix_touch.start()
            # self.thread_get_matrix_touch.join()
        elif self.touch_type != -1 and self.touch_type != 2:
            self.thread_get_touch = threading.Thread(target=self.get_hand_touch)
            self.thread_get_touch.daemon = True
            self.thread_get_touch.start()
            # self.thread_get_touch.join()




    def _get_hand_state(self):
        hand_state = {
            'state':[],
            'vel':[]
        }
        while True:
            if self.hand_state_pub.get_subscription_count() > 0:
                hand_state['state'] = self.api.get_state()
                hand_state['vel'] = self.api.get_joint_speed()
                self.pub_hand_state(hand_state=hand_state)
                time.sleep(0.02)

    def pub_hand_state(self,hand_state):
        state = hand_state['state']
        if self.hand_type == "left":
            state_arc = self.api.range_to_arc_left(state,self.hand_joint)
        if self.hand_type == "right":
            state_arc = self.api.range_to_arc_right(state,self.hand_joint)
        vel = hand_state['vel']
        if state == None:
            return
        if all(x == 0 for x in  vel):
            self.hand_state_pub.publish(self.joint_state_msg(state,[0]*len(state)))
            self.hand_state_arc_pub.publish(self.joint_state_msg(state_arc,[0]*len(state)))
        else:
            self.hand_state_pub.publish(self.joint_state_msg(state,vel))
            self.hand_state_arc_pub.publish(self.joint_state_msg(state_arc,vel))

    def joint_state_msg(self, pose,vel=[]):
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = []
        joint_state.position = [float(x) for x in pose]
        if len(vel) > 1:
            joint_state.velocity = [float(x) for x in vel]
        else:
            joint_state.velocity = [0.0] * len(pose)
        joint_state.effort = [0.0] * len(pose)
        return joint_state
    
    def get_hand_info(self):
        while True:
            if self.hand_info_pub.get_subscription_count() > 0:
                data = {
                    "version": self.version, # Dexterous hand version number
                    "hand_joint": self.hand_joint, # Dexterous hand joint type
                    "speed": self.api.get_speed(), # Current speed threshold of the dexterous hand
                    "current": self.api.get_current(), # Current of the dexterous hand
                    "fault": self.api.get_fault(), # Current fault of the dexterous hand
                    "motor_temperature": self.api.get_temperature(), # Current motor temperature of the dexterous hand
                    "torque": self.api.get_torque(), # Current torque of the dexterous hand
                    "is_touch":self.is_touch,
                    "touch_type": self.touch_type,
                    "finger_order": self.api.get_finger_order() # Finger motor order
                }
                #self.last_hand_info = data
                self.pub_hand_info(dic=data)
            time.sleep(0.1)

    def pub_hand_info(self,dic):
        msg = String()
        msg.data = json.dumps(dic)
        self.hand_info_pub.publish(msg)

    def get_hand_touch(self):
        while True:
            if self.touch_pub.get_subscription_count() > 0:
                if self.is_touch == True:
                    #self.touch_type = self.api.get_touch_type()
                    if self.touch_type == 2:
                        break
                        self.t_force = self.api.get_touch()
                    elif self.touch_type != -1:
                        force = self.api.get_force()
                        self.t_force = [item for sublist in force for item in sublist]
                else:
                    self.touch_type = -1
                if self.is_touch == True:
                    if self.touch_type != 2 and self.touch_type !=-1:
                        t_force = self.t_force
                        force_msg = Float32MultiArray()
                        force_msg.data = t_force
                        self.last_hand_touch = force_msg
                    self.touch_pub.publish(force_msg)
            time.sleep(0.04)

    def get_matrix_touch(self):
        while True:
            if self.matrix_touch_pub.get_subscription_count() > 0:
                if self.touch_type == 2:
                    thumb_matrix, index_matrix , middle_matrix , ring_matrix , little_matrix = self.api.get_matrix_touch()
                    matrix_dic = {
                        "thumb_matrix":thumb_matrix.tolist(),
                        "index_matrix":index_matrix.tolist(),
                        "middle_matrix":middle_matrix.tolist(),
                        "ring_matrix":ring_matrix.tolist(),
                        "little_matrix":little_matrix.tolist()
                    }
                    # print(matrix_dic,flush=True)
                    m_t = String()
                    m_t.data = json.dumps(matrix_dic)
                    self.matrix_touch_pub.publish(m_t)
            time.sleep(0.01)

    def left_hand_control_cb(self,msg):
        now = time.time()
        if now - self.last_process_time < self.min_interval:
            return  # 丢弃当前帧，限频处理
        self.last_process_time = now
        '''左手接收控制topic回调 for range'''
        pose = list(msg.position)
        print('left_hand_control_cb begin')
        self.api.finger_move(pose=list(msg.position))
        print('left_hand_control_cb end')
        vel = list(msg.velocity)
        self.vel = vel
        if all(x == 0 for x in vel):
            return
        else:
            if self.hand_joint == "L7" and len(vel) == 7:
                speed = vel
                self.api.set_joint_speed(speed=speed)
            elif self.hand_joint == "L10" and len(vel) == 10:
                speed = [vel[0],vel[2],vel[3],vel[4],vel[5]]
                self.api.set_joint_speed(speed=speed)
            elif self.hand_joint == "L20" and len(vel) == 20:
                speed = [vel[10],vel[1],vel[2],vel[3],vel[4]]
                self.api.set_joint_speed(speed=speed)
            elif self.hand_joint == "L21" and len(vel) == 25:
                speed = vel
                self.api.set_joint_speed(speed=speed)
            elif self.hand_joint == "L25" and len(vel) == 25:
                speed = vel
                self.api.set_joint_speed(speed=speed)

    def left_hand_control_arc_cb(self,msg):
        now = time.time()
        if now - self.last_process_time < self.min_interval:
            return  # 丢弃当前帧，限频处理
        self.last_process_time = now
        '''左手接收控制topic回调 for arc'''
        pose_range = self.api.arc_to_range_left(msg.position,self.hand_joint)
        self.api.finger_move(pose=list(pose_range))
        vel = list(msg.velocity)
        self.vel = vel
        if all(x == 0 for x in vel):
            return
        else:
            if self.hand_joint == "L7" and len(vel) == 7:
                speed = vel
                self.api.set_joint_speed(speed=speed)
            elif self.hand_joint == "L10" and len(vel) == 10:
                speed = [vel[0],vel[2],vel[3],vel[4],vel[5]]
                self.api.set_joint_speed(speed=speed)
            elif self.hand_joint == "L20" and len(vel) == 20:
                speed = [vel[10],vel[1],vel[2],vel[3],vel[4]]
                self.api.set_joint_speed(speed=speed)
            elif self.hand_joint == "L21" and len(vel) == 25:
                speed = vel
                self.api.set_joint_speed(speed=speed)
            elif self.hand_joint == "L25" and len(vel) == 25:
                speed = vel
                self.api.set_joint_speed(speed=speed)

    def right_hand_control_cb(self,msg):
        now = time.time()
        if now - self.last_process_time < self.min_interval:
            return  # 丢弃当前帧，限频处理
        self.last_process_time = now
        '''右手接收控制topic回调 for range'''
        pose = list(msg.position)
        self.api.finger_move(pose=list(msg.position))
        vel = list(msg.velocity)
        self.vel = vel
        if all(x == 0 for x in vel):
            return
        else:
            if self.hand_joint == "L7" and len(vel) == 7:
                speed = vel
                self.api.set_joint_speed(speed=speed)
            elif self.hand_joint == "L10" and len(vel) == 10:
                speed = [vel[0],vel[2],vel[3],vel[4],vel[5]]
                self.api.set_joint_speed(speed=speed)
            elif self.hand_joint == "L20" and len(vel) == 20:
                speed = [vel[10],vel[1],vel[2],vel[3],vel[4]]
                self.api.set_joint_speed(speed=speed)
            elif self.hand_joint == "L21" and len(vel) == 25:
                speed = vel
                self.api.set_joint_speed(speed=speed)
            elif self.hand_joint == "L25" and len(vel) == 25:
                speed = vel
                self.api.set_joint_speed(speed=speed)

    def right_hand_control_arc_cb(self,msg):
        now = time.time()
        if now - self.last_process_time < self.min_interval:
            return  # 丢弃当前帧，限频处理
        self.last_process_time = now
        '''右手接收控制topic回调 for arc'''
        pose_range = self.api.arc_to_range_right(msg.position,self.hand_joint)
        self.api.finger_move(pose=list(pose_range))
        vel = list(msg.velocity)
        self.vel = vel
        if all(x == 0 for x in vel):
            return
        else:
            if self.hand_joint == "L7" and len(vel) == 7:
                speed = vel
                self.api.set_joint_speed(speed=speed)
            elif self.hand_joint == "L10" and len(vel) == 10:
                speed = [vel[0],vel[2],vel[3],vel[4],vel[5]]
                self.api.set_joint_speed(speed=speed)
            elif self.hand_joint == "L20" and len(vel) == 20:
                speed = [vel[10],vel[1],vel[2],vel[3],vel[4]]
                self.api.set_joint_speed(speed=speed)
            elif self.hand_joint == "L21" and len(vel) == 25:
                speed = vel
                self.api.set_joint_speed(speed=speed)
            elif self.hand_joint == "L25" and len(vel) == 25:
                speed = vel
                self.api.set_joint_speed(speed=speed)

    def hand_setting_cb(self,msg):
        '''控制命令回调'''
        data = json.loads(msg.data)
        print(f"Received setting command: {data['setting_cmd']}",flush=True)
        try:
            if data["params"]["hand_type"] == "left":
                hand = self.api
                hand_left = True
            elif data["params"]["hand_type"] == "right":
                hand = self.api
                hand_right = True
            else:
                print("Please specify the hand part to be set",flush=True)
                return
            # Set maximum torque
            if data["setting_cmd"] == "set_max_torque_limits": # Set maximum torque
                torque = list(data["params"]["torque"])
                hand.set_torque(torque=torque)
                
            if data["setting_cmd"] == "set_speed": # Set speed
                if isinstance(data["params"]["speed"], list) == True:
                    speed = data["params"]["speed"]
                    hand.set_speed(speed=speed)
                else:
                    ColorMsg(msg=f"Speed parameter error, speed must be a list", color="red")
            if data["setting_cmd"] == "clear_faults": # Clear faults
                if hand_left == True and self.hand_joint == "L10" :
                    ColorMsg(msg=f"L10 left hand cannot clear faults")
                elif hand_right == True and self.hand_joint == "L10" :
                    ColorMsg(msg=f"L10 right hand cannot clear faults")
                else:
                    hand.clear_faults()
            if data["setting_cmd"] == "get_faults": # Get faults
                f = hand.get_fault()
                ColorMsg(msg=f"Get faults: {f}")
            if data["setting_cmd"] == "electric_current": # Get current
                ColorMsg(msg=f"Get current: {hand.get_current()}")
            if data["setting_cmd"] == "set_electric_current": # Set current
                if isinstance(data["params"]["current"], list) == True:
                    hand.set_current(data["params"]["current"])
        except:
            print("命令参数错误")

    def close_can(self):
        self.open_can.close_can0()
        sys.exit(0)
        
def main(args=None):
    rclpy.init(args=args)
    node = LinkerHand("linker_hand_sdk")
    
    try:
        node.run()               # 初始化线程或其他操作
        rclpy.spin(node)         # 主循环，监听 ROS 回调
    except KeyboardInterrupt:
        print("收到 Ctrl+C，准备退出...")
    finally:
        node.close_can()         # 关闭 CAN 或其他硬件资源
        node.destroy_node()      # 销毁 ROS 节点
        rclpy.shutdown()         # 关闭 ROS
        print("程序已退出。")
