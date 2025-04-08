'''
Author: HJX
Date: 2025-04-01 14:09:21
LastEditors: Please set LastEditors
LastEditTime: 2025-04-02 15:48:56
FilePath: /linker_hand_ros2_sdk/src/linker_hand_ros2_sdk/linker_hand_ros2_sdk/LinkerHand/linker_hand_api.py
Description: 
symbol_custom_string_obkorol_copyright: 
'''
#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
import sys,os,time


#current_dir = os.path.dirname(os.path.abspath(__file__))
#target_dir = os.path.abspath(os.path.join(current_dir, "../.."))
#sys.path.append(target_dir)
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from utils.color_msg import ColorMsg
#from linker_hand_ros2_sdk.LinkerHand.utils.color_msg import ColorMsg
from utils.load_write_yaml import LoadWriteYaml
from utils.open_can import OpenCan
class LinkerHandApi:
    def __init__(self,hand_type="left",hand_joint="L10"):
        self.yaml = LoadWriteYaml()
        self.config = self.yaml.load_setting_yaml()
        self.version = self.config["VERSION"]
        ColorMsg(msg=f"当前SDK version:{self.version}", color="green")
        self.hand_joint = hand_joint
        self.hand_type = hand_type
        if self.hand_type == "left":
            self.hand_id = 0x28 # 左手
        else:
            self.hand_id = 0x27 # 右手
        if self.hand_joint == "L7":
            from core.linker_hand_l7_can import LinkerHandL7Can
            self.hand = LinkerHandL7Can(can_id=self.hand_id)
        if self.hand_joint == "L10":
            from core.linker_hand_l10_can import LinkerHandL10Can
            self.hand = LinkerHandL10Can(can_id=self.hand_id)
        if self.hand_joint == "L25":
            from core.linker_hand_l25_can import LinkerHandL25Can
            self.hand = LinkerHandL25Can(config=self.config, can_id=self.hand_id)
        # 打开can0
        if sys.platform == "linux":
            self.open_can = OpenCan(load_yaml=self.yaml)
            self.open_can.open_can0()
            self.is_can = self.open_can.is_can_up_sysfs()
            if not self.is_can:
                ColorMsg(msg="CAN0接口未打开", color="red")
                sys.exit(1)
    
    # 五指运动
    def finger_move(self,pose=[]):
        '''
        五指移动
        @params: pose list L7 len(7) | L10 len(10) | L20 len(20) | L25 len(25) 0~255
        '''
        if self.hand_joint == "L7" and len(pose) == 7:
            self.hand.set_joint_positions(pose)
        elif self.hand_joint == "L10" and len(pose) == 10:
            self.hand.set_joint_positions(pose)
        elif self.hand_joint == "L25" and len(pose) == 25:
            self.hand.set_joint_positions(pose)
        else:
            ColorMsg(msg=f"当前LinkerHand为{self.hand_type}{self.hand_joint},动作序列为{pose},并不匹配", color="red")

    
    def _get_normal_force(self):
        '''# 获取法向压力'''
        self.hand.get_normal_force()
    
    def _get_tangential_force(self):
        '''# 获取切向压力'''
        self.hand.get_tangential_force()
    
    def _get_tangential_force_dir(self):
        '''# 获取切向压力方向'''
        self.hand.get_tangential_force_dir()
    
    def _get_approach_inc(self):
        '''# 获取接近度'''
        self.hand.get_approach_inc()

    def get_force(self):
        '''获取所有压感数据'''
        self._get_approach_inc()
        self._get_normal_force()
        self._get_tangential_force()
        self._get_tangential_force_dir()
        return self.hand.get_force()
    
    def set_speed(self,speed=[100]*5):
        '''# 设置速度'''
        if self.hand_joint == "L10":
            self.hand.set_joint_speed_l10(speed)
        else:
            self.hand.set_speed(speed=speed)
    
    def set_torque(self, torque=[]):
        '''设置最大扭矩'''
        return self.hand.set_torque(torque=torque)
    
    def get_state(self):
        '''# 获取当前关节状态'''
        return self.hand.get_current_status()
    def get_speed(self):
        '''# 获取速度'''
        return self.hand.get_speed()
    
    def get_torque(self):
        '''获取当前最大扭矩'''
        return self.hand.get_torque()
    
    def get_temperature(self):
        '''获取电机当前温度'''
        return self.hand.get_temperature()
    
    def get_fault(self):
        '''获取电机故障码'''
        return self.hand.get_fault()
    
    # 设置使能
    def set_enable(self):
        self.hand.set_enable_mode()
    # 设置失能
    def set_disable(self):
        self.hand.set_disability_mode()

    

        
    



if __name__ == "__main__":
    hand = LinkerHandApi()
