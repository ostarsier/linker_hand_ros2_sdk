#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
import sys, os, time,threading
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from utils.mapping import *
from utils.color_msg import ColorMsg
from utils.load_write_yaml import LoadWriteYaml
from utils.open_can import OpenCan

class LinkerHandApi:
    def __init__(self, hand_type="left", hand_joint="L7", modbus = "None",can="can0"):  # Ubuntu:can0   win:PCAN_USBBUS1
        # 初始化最后一次位置为空列表
        self.last_position = []
        # 加载YAML配置工具
        self.yaml = LoadWriteYaml()
        # 读取配置文件内容
        self.config = self.yaml.load_setting_yaml()
        # 获取SDK版本号
        self.version = self.config["VERSION"]
        # 保存CAN通道参数
        self.can = can
        # 打印当前SDK版本信息（绿色字体）
        ColorMsg(msg=f"Current SDK version: {self.version}", color="green")
        # 保存手型参数（关节类型）
        self.hand_joint = hand_joint
        # 保存手的左右类型
        self.hand_type = hand_type

        # 根据手的类型分配CAN ID
        if self.hand_type == "left":
            self.hand_id = 0x28  # 左手ID
        if self.hand_type == "right":
            self.hand_id = 0x27  # 右手ID

        # 根据手的关节类型实例化对应的控制类
        if self.hand_joint == "L7":
            # L7手型CAN协议控制类
            from core.can.linker_hand_l7_can import LinkerHandL7Can
            self.hand = LinkerHandL7Can(can_id=self.hand_id,can_channel=self.can, yaml=self.yaml)
        if self.hand_joint == "L10":
            # L10手型，分为RML 485协议和默认CAN协议
            # if self.config['LINKER_HAND']['LEFT_HAND']['MODBUS'] == "RML": 
            if modbus == "RML": # RML API2 485协议
                # from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e
                from core.rml485.linker_hand_l10_485 import LinkerHandL10For485
                # 这里可扩展为通过485协议实例化L10手型
                self.hand = LinkerHandL10For485()
            else : # 默认CAN协议
                from core.can.linker_hand_l10_can import LinkerHandL10Can
                self.hand = LinkerHandL10Can(can_id=self.hand_id,can_channel=self.can, yaml=self.yaml)
        if self.hand_joint == "L20":
            # L20手型CAN协议控制类
            from core.can.linker_hand_l20_can import LinkerHandL20Can
            self.hand = LinkerHandL20Can(can_id=self.hand_id,can_channel=self.can, yaml=self.yaml)
        if self.hand_joint == "L21":
            # L21手型CAN协议控制类
            from core.can.linker_hand_l21_can import LinkerHandL21Can
            self.hand = LinkerHandL21Can(can_id=self.hand_id,can_channel=self.can, yaml=self.yaml)
        if self.hand_joint == "L25":
            # L25手型CAN协议控制类
            from core.can.linker_hand_l25_can import LinkerHandL25Can
            self.hand = LinkerHandL25Can(can_id=self.hand_id,can_channel=self.can, yaml=self.yaml)

        # Linux平台下自动打开CAN通道
        if sys.platform == "linux":
            self.open_can = OpenCan(load_yaml=self.yaml)
            self.open_can.open_can(self.can)
            self.is_can = self.open_can.is_can_up_sysfs(interface=self.can)
            # 检查CAN通道是否打开成功，失败则报错并退出
            if not self.is_can:
                ColorMsg(msg=f"{self.can} interface is not open", color="red")
                sys.exit(1)
        # 获取嵌入式固件版本信息
        version = self.get_version()
        ColorMsg(msg=f"Embedded:{version}", color="green")
    
    # Five-finger movement
    def finger_move(self, pose=[]):
        '''
        Five-finger movement
        @params: pose list L7 len(7) | L10 len(10) | L20 len(20) | L25 len(25) 0~255
        '''
        # if pose == self.last_position:
        #     return
        #ColorMsg(msg=f"Current LinkerHand is {self.hand_type} {self.hand_joint}, action sequence is {pose}", color="green")
        if self.hand_joint == "L7" and len(pose) == 7:
            self.hand.set_joint_positions(pose)
        elif self.hand_joint == "L10" and len(pose) == 10:
            self.hand.set_joint_positions(pose)
        elif self.hand_joint == "L20" and len(pose) == 20:
            self.hand.set_joint_positions(pose)
        elif self.hand_joint == "L21" and len(pose) == 25:
            self.hand.set_joint_positions(pose)
        elif self.hand_joint == "L25" and len(pose) == 25:
            self.hand.set_joint_positions(pose)
        else:
            ColorMsg(msg=f"Current LinkerHand is {self.hand_type}{self.hand_joint}, action sequence is {pose}, does not match", color="red")
        self.last_position = pose

    def _get_normal_force(self):
        '''# Get normal force'''
        self.hand.get_normal_force()
    
    def _get_tangential_force(self):
        '''# Get tangential force'''
        self.hand.get_tangential_force()
    
    def _get_tangential_force_dir(self):
        '''# Get tangential force direction'''
        self.hand.get_tangential_force_dir()
    
    def _get_approach_inc(self):
        '''# Get approach increment'''
        self.hand.get_approach_inc()
    
    def set_speed(self, speed=[100]*5):
        '''# Set speed'''
        ColorMsg(msg=f"{self.hand_type} {self.hand_joint} set speed to {speed}", color="green")
        self.hand.set_speed(speed=speed)
    
    def set_joint_speed(self, speed=[100]*5):
        '''Set speed by topic'''
        self.hand.set_speed(speed=speed)
    
    def set_torque(self, torque=[180] * 5):
        '''Set maximum torque'''
        ColorMsg(msg=f"{self.hand_type} {self.hand_joint} set maximum torque to {torque}", color="green")
        return self.hand.set_torque(torque=torque)
    
    
    def set_current(self, current=[]):
        '''Set current L7/L10/L25 not supported'''
        if self.hand_joint == "L20":
            return self.hand.set_current(current=current)
        else:
            pass

    def get_version(self):
        '''Get version'''
        
        return self.hand.get_version()
    
    def get_current(self):
        '''Get current'''
        return self.hand.get_current()
    
    def get_state(self):
        '''Get current joint state'''
        return self.hand.get_current_status()
    
    def get_speed(self):
        '''Get speed'''
        return self.hand.get_speed()
    
    def get_joint_speed(self):
        speed = []
        if self.hand_joint == "L7":
            return speed
        elif self.hand_joint == "L10":
            speed = self.hand.get_speed()
            return [speed[0], 255, speed[1], speed[2], speed[3], speed[4], 255, 255, 255, 255]
        elif self.hand_joint == "L20":
            speed = self.hand.get_speed()
            return [255, speed[1], speed[2], speed[3], speed[4], 255, 255, 255, 255, 255, speed[0], 255, 255, 255, 255, 255, 255, 255, 255, 255]
        elif self.hand_joint == "L21":
            return self.hand.get_speed()
        elif self.hand_joint == "L25":
            return speed

    def get_touch_type(self):
        '''Get touch type'''
        return self.hand.get_touch_type()
    
    def get_force(self):
        '''Get normal force, tangential force, tangential force direction, approach sensing data'''
        self._get_normal_force()
        self._get_tangential_force()
        self._get_tangential_force_dir()
        self._get_approach_inc()
        return self.hand.get_force()

    def get_touch(self):
        '''Get touch data'''
        return self.hand.get_touch()
    
    def get_matrix_touch(self):
        return self.hand.get_matrix_touch()

    def get_torque(self):
        '''Get current maximum torque'''
        return self.hand.get_torque()
    
    def get_temperature(self):
        '''Get current motor temperature'''
        return self.hand.get_temperature()
    
    def get_fault(self):
        '''Get motor fault code'''
        return self.hand.get_fault()
    
    def clear_faults(self):
        '''Clear motor fault codes Not supported yet, currently only supports L20'''
        if self.hand_joint == "L20":
            self.hand.clear_faults()
        else:
            return [0] * 5

    def set_enable(self):
        '''Set motor enable Only supports L25'''
        if self.hand_joint == "L25":
            self.hand.set_enable_mode()
        else:
            pass

    def set_disable(self):
        '''Set motor disable Only supports L25'''
        if self.hand_joint == "L25":
            self.hand.set_disability_mode()
        else:
            pass

    def get_finger_order(self):
        '''Get finger motor order'''
        if self.hand_joint == "L21" or self.hand_joint == "L25":
            return self.hand.get_finger_order()
        else:
            return []
        
    def range_to_arc_left(self, state, hand_joint):
        return range_to_arc_left(left_range=state, hand_joint=hand_joint)
    
    def range_to_arc_right(self, state, hand_joint):
        return range_to_arc_right(right_range=state, hand_joint=hand_joint)
    
    def arc_to_range_left(self,state,hand_joint):
        return arc_to_range_left(hand_arc_l=state,hand_joint=hand_joint)
    
    def arc_to_range_right(self,state,hand_joint):
        return arc_to_range_right(right_arc=state,hand_joint=hand_joint)
    

    def close_can(self):
        self.open_can.close_can0()                         

if __name__ == "__main__":
    hand = LinkerHandApi(hand_type="left", hand_joint="L7")
    hand.finger_move([255]*7)