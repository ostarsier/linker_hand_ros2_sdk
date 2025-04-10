#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
'''
Author: HJX
Date: 2025-04-01 17:50:14
LastEditors: Please set LastEditors
LastEditTime: 2025-04-10 13:54:12
FilePath: /linker_hand_ros2_sdk/src/gui_control/gui_control/gui_control.py
Description: 
编译: colcon build --symlink-install --packages-select gui_control
启动命令:ros2 run gui_control gui_control
'''
from PyQt5.QtWidgets import QMainWindow, QSplitter, QApplication,QMessageBox,QPushButton
from PyQt5.QtCore import Qt, QTimer
import yaml, os, sys,time,json,rclpy
from rclpy.node import Node
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from views.left_view import LeftView
from views.right_view import RightView
# 获取当前脚本的路径，并计算 src 目录的绝对路径
current_dir = os.path.dirname(os.path.abspath(__file__))
src_path = os.path.join(current_dir, "../../src/linker_hand_ros2_sdk/")
# 添加 src 目录到 Python 模块搜索路径
sys.path.append(src_path)
from linker_hand_ros2_sdk.LinkerHand.utils.init_linker_hand import InitLinkerHand
from linker_hand_ros2_sdk.LinkerHand.utils.color_msg import ColorMsg
from linker_hand_ros2_sdk.LinkerHand.utils.load_write_yaml import LoadWriteYaml
from std_msgs.msg import String, Header
from sensor_msgs.msg import JointState
import threading



class ROS2SliderPublisher(Node):
    def __init__(self):
        super().__init__('slider_publisher')
        self.init_pose = None
        self.hand_type = None
        self.hand_joint = None
        self.last_position = None
        self.running = True
        self.left_hand_publisher_ = self.create_publisher(JointState, '/cb_left_hand_control_cmd', 10)
        self.right_hand_publisher_ = self.create_publisher(JointState, '/cb_right_hand_control_cmd', 10)
        self.publish_thread = threading.Thread(target=self.publish_value)
        self.publish_thread.start()
    
    def set_hand_info(self,init_pose,hand_type,hand_joint):
        self.init_pose=init_pose
        self.last_position = init_pose
        self.hand_type=hand_type
        self.hand_joint=hand_joint

    def publish_value(self):
        rate = 1.0 / 60  # 60 FPS
        while self.running:
            if self.last_position == None:
                continue
            else:
                l_p = [float(p) if p is not None else 0.0 for p in self.last_position]
                msg = self.create_joint_state_msg(position=l_p, names=[])
                if self.hand_type == "left":
                    self.left_hand_publisher_.publish(msg)
                else:
                    self.right_hand_publisher_.publish(msg)
                time.sleep(rate)

    
    def create_joint_state_msg(self, position=[], names=[]):
        msg = JointState()
        # 设置 header（时间戳和坐标系）
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        #msg.header.frame_id = 'base_link'  # 可自定义
        # 设置关节数据
        msg.name = names  # 关节名称
        msg.position = position            # 关节位置（弧度）
        msg.velocity = [0.0] * len(position)             # 关节速度（可选）
        msg.effort = [0.0] * len(position)              # 关节力矩（可选）
        return msg

class SliderApp(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.yaml = LoadWriteYaml()

        
        self.ros_node = ros_node
        self._init_hand()
        self.ros_node.set_hand_info(self.init_pos,self.hand_type,self.hand_joint)
        self._init_ui()
    
    def _init_hand(self):
        self.yaml = LoadWriteYaml() # 初始化配置文件
        # 读取配置文件
        self.setting = self.yaml.load_setting_yaml()
        # 判断左手是否配置
        self.left_hand = False
        self.right_hand = False
        if self.setting['LINKER_HAND']['LEFT_HAND']['EXISTS'] == True:
            self.left_hand = True
        elif self.setting['LINKER_HAND']['RIGHT_HAND']['EXISTS'] == True:
            self.right_hand = True
        # gui控制只支持单手，这里进行左右手互斥
        if self.left_hand == True and self.right_hand == True:
            self.left_hand = True
            self.right_hand = False
        if self.left_hand == True:
            print("左手")
            self.hand_exists = True
            self.hand_joint = self.setting['LINKER_HAND']['LEFT_HAND']['JOINT']
            self.hand_type = "left"
        if self.right_hand == True:
            print("右手")
            self.hand_exists = True
            self.hand_joint = self.setting['LINKER_HAND']['RIGHT_HAND']['JOINT']
            self.hand_type = "right"
        
        if self.hand_joint == "L25":
            self.init_pos = [255] * 25
            # topic
            self.joint_name = ["大拇指根部","食指根部","中指根部","无名指根部","小拇指根部","大拇指侧摆","食指侧摆","中指侧摆","无名指侧摆","小拇指侧摆","大拇指横滚","预留","预留","预留","预留","大拇指中部","食指中部","中指中部","无名指中部","小拇指中部","大拇指指尖","食指指尖","中指指尖","无名指指尖","小拇指指尖"]

        elif self.hand_joint == "L20":
            self.init_pos = [255,255,255,255,255,255,10,100,180,240,245,255,255,255,255,255,255,255,255,255]
            # L20
            self.joint_name = ["拇指根部", "食指根部", "中指根部", "无名指根部","小指根部","拇指侧摆","食指侧摆","中指侧摆","无名指侧摆","小指侧摆","拇指横摆","预留","预留","预留","预留","拇指尖部","食指末端","中指末端","无名指末端","小指末端"]
        elif self.hand_joint == "L10":
            # L10
            self.init_pos = [255] * 10
            self.joint_name = ["拇指根部", "拇指侧摆","食指根部", "中指根部", "无名指根部","小指根部","食指侧摆","无名指侧摆","小指侧摆","拇指旋转"]
        elif self.hand_joint == "L7":
            # L7
            self.init_pos = [250] * 7
            self.joint_name = ["大拇指弯曲", "大拇指横摆","食指弯曲", "中指弯曲", "无名指弯曲","小拇指弯曲","拇指旋转"]

    def _init_ui(self):
        if self.hand_type == "left":
            self.setWindowTitle(f"Linker_Hand:左手- {self.hand_joint} Control - Qt5 with ROS")
        else:
            self.setWindowTitle(f"Linker_Hand:右手- {self.hand_joint} Control - Qt5 with ROS")
        self.setGeometry(100, 100, 600, 800)
        # 创建分割线
        splitter = QSplitter(Qt.Horizontal)
        splitter.setStyleSheet("""
            QSplitter::handle {
                width:1px;
                background-color: lightgray;
                margin: 15px 20px;
            }
        """)
        # 左侧滑动条界面
        self.left_view = LeftView(joint_name=self.joint_name, init_pos=self.init_pos,hand_type=self.hand_type)
        splitter.addWidget(self.left_view)
        self.left_view.slider_value_changed.connect(self.handle_slider_value_changed)
        # 右侧记录动作界面
        self.right_view = RightView(hand_joint=self.hand_joint, hand_type=self.hand_type, load_yaml=self.yaml)
        splitter.addWidget(self.right_view)
        # 接收到信号槽事件，这里用于记录动作序列更新滑动条数据
        self.right_view.handle_button_click.connect(self.handle_button_click)
        self.right_view.add_button_handle.connect(self.add_button_handle)
        splitter.setSizes([600, 450])
        self.setCentralWidget(splitter)

    # 通过信号机制实时获取滑动条的当前值
    def handle_slider_value_changed(self, slider_values):
        #print("实时获取滑动条的当前值:", slider_values)
        slider_values_list = []
        for key in slider_values:
            slider_values_list.append(slider_values[key])
        self.last_position = slider_values_list
        self.ros_node.set_hand_info(self.last_position,self.hand_type,self.hand_joint)


    # 点击按钮后将动作数值写入yaml文件
    def handle_button_click(self,text):
        all_action = self.yaml.load_action_yaml(hand_type=self.hand_type,hand_joint=self.hand_joint)
        for index,pos in enumerate(all_action):
            if pos['ACTION_NAME'] == text:
                position = pos['POSITION']
                print(type(position))
        ColorMsg(msg=f"动作名称:{text}, 动作数值:{position}", color="green")
        self.last_position = position
        self.left_view.set_slider_values(values=position)

    #点击添加按钮后将动作数值写入yaml文件
    def add_button_handle(self,text):
        self.add_button_position = self.left_view.get_slider_values()
        self.add_button_text = text
        self.yaml.write_to_yaml(action_name=text, action_pos=self.left_view.get_slider_values(),hand_joint=self.hand_joint,hand_type=self.hand_type)
        
    def on_slider_value_changed(self, value):
        self.label.setText(f'Value: {value}')
        self.ros_node.publish_value(value)


def main():
    rclpy.init()
    ros_node = ROS2SliderPublisher()
    app = QApplication(sys.argv)
    slider_app = SliderApp(ros_node)
    slider_app.show()
    
    sys.exit(app.exec_())

