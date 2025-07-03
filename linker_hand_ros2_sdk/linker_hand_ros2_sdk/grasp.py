#!/usr/bin/env python3
import sys,os,time
current_dir = os.path.dirname(os.path.abspath(__file__))
target_dir = os.path.abspath(os.path.join(current_dir, "../../.."))
sys.path.append(target_dir)
from LinkerHand.linker_hand_api import LinkerHandApi
from LinkerHand.utils.load_write_yaml import LoadWriteYaml
from LinkerHand.utils.init_linker_hand import InitLinkerHand
from LinkerHand.utils.color_msg import ColorMsg
import numpy as np

#按大拇指，握紧
#按小指后松开，放开

    
def grasp():
    linkerhand = InitLinkerHand()
    left_hand, left_hand_joint, left_hand_type, left_hand_force, left_hand_pose, left_hand_torque, left_hand_speed, \
    right_hand, right_hand_joint, right_hand_type, right_hand_force, right_hand_pose, right_hand_torque, right_hand_speed, setting = linkerhand.current_hand()

    hand = None
    if left_hand_joint and left_hand_type:
        # 如果右手未连接，尝试使用左手
        hand = LinkerHandApi(hand_joint=left_hand_joint, hand_type=left_hand_type)
        ColorMsg(msg=f"使用左手: {left_hand_joint} {left_hand_type}", color="blue")
    elif right_hand_joint and right_hand_type:
        # 初始化API
        hand = LinkerHandApi(hand_joint=right_hand_joint, hand_type=right_hand_type)
        ColorMsg(msg=f"使用右手: {right_hand_joint} {right_hand_type}", color="blue")
    else:
        ColorMsg(msg="错误：未检测到连接的 LinkerHand 设备。", color="red")
        return
    # 设置速度 (Set speed)
    # 7个关节的速度: [拇指屈伸, 拇指侧摆, 食指, 中指, 无名指, 小指, 拇指旋转]
    speed = [120, 120, 120, 120, 120, 120, 120] 
    hand.set_speed(speed=speed)
    ColorMsg(msg=f"当前手型为 {hand.hand_joint} {hand.hand_type}, 设置速度为: {speed}", color="green")

    # 手指姿态数据 (Finger pose data)
    # [拇指屈伸, 拇指侧摆, 食指, 中指, 无名指, 小指, 拇指旋转]
    # 范围 0-255, 255为完全张开，0为完全握拳 (Range 0-255, 255 is fully open, 0 is fully clenched)
    initial_open_pose = [255, 255, 255, 255, 255, 255, 255]
    # 抓握话筒的目标姿态 (示例值, 可能需要根据实际话筒和期望的握持方式进行调整)
    # 例如: 手指闭合, 拇指对立姿态
    target_grasp_pose = [120, 120, 110, 110, 110, 110, 120]   
    
    current_pose = list(initial_open_pose) # 程序开始时，手爪完全张开

    ColorMsg(msg=f"设置初始张开姿态: {current_pose}", color="green")
    hand.finger_move(pose=current_pose)

    microphone_detected = False
    grasp_complete = False
    grasp_step = 10  # 每一步闭合的量 (0-255范围)。值越小，抓握越慢/越平滑。
    touch_threshold = 1  # 触发抓握所需的最少接触点数量
    # stop_grasp_threshold = 50  # 停止抓握所需的最少压力值
    prev_little_matrix_pressed = False # 追踪小指矩阵之前的状态

    while True:
        # 获取当前触摸传感器状态
        thumb_matrix, index_matrix, middle_matrix, ring_matrix, little_matrix = hand.get_matrix_touch()
        current_little_matrix_pressed = (little_matrix > 0).any() # 检查小指是否有任何部分被按压

        if prev_little_matrix_pressed and not current_little_matrix_pressed:
            ColorMsg(msg="小指矩阵松开，恢复初始张开姿态。", color="orange")
            current_pose = list(initial_open_pose)
            hand.finger_move(pose=current_pose)
            microphone_detected = False
            grasp_complete = False
            prev_little_matrix_pressed = False # 已松开，重置状态以备下次按压
            time.sleep(0.1) # 短暂延时以确保姿态已改变
            continue # 跳过本轮后续逻辑，重新开始循环

        # 更新 prev_little_matrix_pressed 以供下一轮迭代检查
        prev_little_matrix_pressed = current_little_matrix_pressed

        if grasp_complete:
            ColorMsg(msg=f"话筒已抓稳。当前姿态: {current_pose}", color="magenta")
            # 可选: 持续监控触摸传感器以调整握力或检测滑动
            # 此处暂时仅保持姿态并周期性打印信息
            # thumb_matrix, index_matrix, middle_matrix, ring_matrix, little_matrix = hand.get_matrix_touch()
            # print(f"握持时触摸传感器: T:{thumb_matrix}, I:{index_matrix}, M:{middle_matrix}, R:{ring_matrix}, L:{little_matrix}")
            time.sleep(1) # 每秒检查/打印一次状态

        elif not microphone_detected:
            ColorMsg(msg="等待放置话筒...", color="blue")
            # 此处不再需要单独获取 thumb_matrix, index_matrix 等，已在循环顶部获取
            activated_sensors = (thumb_matrix > 0).sum()

            if activated_sensors >= touch_threshold:
                ColorMsg(msg="检测到话筒! 开始抓握。", color="yellow")
                microphone_detected = True
            
            time.sleep(0.5) # 每0.5秒检测一次话筒

        else: # 检测到话筒，但抓握未完成
            ColorMsg(msg=f"正在抓握... 当前姿态: {current_pose}", color="cyan")
            # pressure_matrices = hand.get_matrix_touch()
            # high_pressure_count = 0
            # for matrix in pressure_matrices:
            #     if np.any(matrix > stop_grasp_threshold):
            #         high_pressure_count += 1
            
            # print(f"高压力计数: {high_pressure_count}")
            # if high_pressure_count >= 1:
            #     ColorMsg(msg="检测到至少两个手指压力过大，停止抓握", color="red")
            #     break


            made_change_this_step = False
            new_pose_calculated = list(current_pose) # 在副本上计算这一步的姿态

            for i in range(len(new_pose_calculated)):
                # 向目标姿态调整
                if new_pose_calculated[i] > target_grasp_pose[i]:
                    new_pose_calculated[i] = max(target_grasp_pose[i], new_pose_calculated[i] - grasp_step)
                    if new_pose_calculated[i] != current_pose[i]: # 检查姿态是否真的改变了
                            made_change_this_step = True
                elif new_pose_calculated[i] < target_grasp_pose[i]: # 某些关节需要向目标姿态增加
                    new_pose_calculated[i] = min(target_grasp_pose[i], new_pose_calculated[i] + grasp_step)
                    if new_pose_calculated[i] != current_pose[i]: # 检查姿态是否真的改变了
                            made_change_this_step = True

            if made_change_this_step:
                current_pose = new_pose_calculated # 更新当前姿态
                hand.finger_move(pose=current_pose) # 设置新的姿态
                ColorMsg(msg=f"调整姿态: {current_pose}", color="green")
            else:
                ColorMsg(msg="姿态已达到目标。", color="green")
                grasp_complete = True


if __name__ == "__main__":
    grasp()