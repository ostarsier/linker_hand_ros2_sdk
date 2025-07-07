import sys, os, time
import random # 新增导入

current_dir = os.path.dirname(os.path.abspath(__file__))
target_dir = os.path.abspath(os.path.join(current_dir, "../../.."))
sys.path.append(target_dir)

from LinkerHand.linker_hand_api import LinkerHandApi
from LinkerHand.utils.init_linker_hand import InitLinkerHand
from LinkerHand.utils.color_msg import ColorMsg

def main():
    linkerhand = InitLinkerHand()
    left_hand, left_hand_joint, left_hand_type, left_hand_force, left_hand_pose, left_hand_torque, left_hand_speed, \
    right_hand, right_hand_joint, right_hand_type, right_hand_force, right_hand_pose, right_hand_torque, right_hand_speed, setting = linkerhand.current_hand()

    hand = None
    if right_hand_joint and right_hand_type:
        # 初始化API
        hand = LinkerHandApi(hand_joint=right_hand_joint, hand_type=right_hand_type)
        ColorMsg(msg=f"使用右手: {right_hand_joint} {right_hand_type}", color="blue")
    elif left_hand_joint and left_hand_type:
        # 如果右手未连接，尝试使用左手
        hand = LinkerHandApi(hand_joint=left_hand_joint, hand_type=left_hand_type)
        ColorMsg(msg=f"使用左手: {left_hand_joint} {left_hand_type}", color="blue")
    else:
        ColorMsg(msg="错误：未检测到连接的 LinkerHand 设备。", color="red")
        return

    # 设置速度 - 您可以根据需要调整
    speed = [60, 60, 60, 60, 60, 60, 60]
    hand.set_speed(speed=speed)
    ColorMsg(msg=f"设置速度为: {speed}", color="green")
    # pose = [0, 120, 110, 110, 110, 110, 120]   
    # pose = [0, 0, 0, 0, 0, 0, 0] 
    pose = [180, 100, 200, 200, 200, 200, 100]
    hand.finger_move(pose=pose)
    time.sleep(1)
    

if __name__ == "__main__":
    main()
