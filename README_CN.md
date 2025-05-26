<!--
 * @Author: HJX
 * @Date: 2025-04-02 11:39:42
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2025-04-11 12:06:43
 * @FilePath: /linker_hand_ros2_sdk/src/README_CN.md
 * @Description: 
 * @symbol_custom_string_obkorol_copyright: 
-->
# LinkerHand灵巧手ROS2 SDK

## 概述
LinkerHand灵巧手ROS SDK 是灵心巧手(北京)科技有限公司开发，用于L7、O7、L10、O10等LinkerHand灵巧手的驱动软件和功能示例源码。可用于真机与仿真器使用。
LinkerHandROS2 SDK当前支持Ubuntu22.04 ROS humble Python3.10 及以上环境

## 安装
&ensp;&ensp;确保当前系统环境为Ubuntu20.04 ROS 2 Foxy Python3.8.20 及以上
- 下载

```bash
  $ mkdir -p linker_hand_ros2_sdk/src
  $ cd linker_hand_ros2_sdk/src
  $ git clone https://github.com/linkerbotai/linker_hand_ros2_sdk.git
```

- 编译

```bash
  $ cd linker_hand_ros2_sdk/src/
  $ pip install -r requirements.txt
```

## 使用
&ensp;&ensp; __使用前请先将 [setting.yaml](linker_hand_ros2_sdk/src/linker_hand_ros2_sdk/linker_hand_ros2_sdk/LinkerHand/config) 配置文件根据实际需求进行相应修改该.__
&ensp;&ensp; __使用前请先将 [load_write_yaml.py](linker_hand_ros2_sdk/src/linker_hand_ros2_sdk/linker_hand_ros2_sdk/LinkerHand/utils/)文件第28行的yaml_path ="" 改为config目录的绝对路径.__

- 启动SDK&ensp;&ensp;&ensp;&ensp;将linker_hand灵巧手的USB转CAN设备插入Ubuntu设备上
```bash
  # 开启CAN端口
  $ sudo /usr/sbin/ip link set can0 up type can bitrate 1000000 #USB转CAN设备蓝色灯常亮状态
  $ cd linker_hand_ros2_sdk/
  $ colcon build --symlink-install
  $ source ./install/setup.bash
  $ ros2 run linker_hand_ros2_sdk linker_hand_sdk
```
- 启动状态波形图(带有压力传感器的LinkerHand)
```bash
# 启动ROS2 SDK后新开终端
$ cd linker_hand_ros2_sdk/
$ source ./install/setup.bash
$ ros2 run graphic_display graphic_display
```

- position与手指关节对照表
```bash
$ ros2 topic echo /cb_left_hand_control_cmd
```
```bash
  header: 
    seq: 256
    stamp: 
      secs: 1744343699
      nsecs: 232647418
    frame_id: ''
  name: []
  position: [155.0, 162.0, 176.0, 125.0, 255.0, 255.0, 180.0, 179.0, 181.0, 68.0]
  velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```
  L7:  ["大拇指弯曲", "大拇指横摆","食指弯曲", "中指弯曲", "无名指弯曲","小拇指弯曲","拇指旋转"]

  L10: ["拇指根部", "拇指侧摆","食指根部", "中指根部", "无名指根部","小指根部","食指侧摆","无名指侧摆","小指侧摆","拇指旋转"]

  L20: ["拇指根部", "食指根部", "中指根部", "无名指根部","小指根部","拇指侧摆","食指侧摆","中指侧摆","无名指侧摆","小指侧摆","拇指横摆","预留","预留","预留","预留","拇指尖部","食指末端","中指末端","无名指末端","小指末端"]

  L25: ["大拇指根部", "食指根部", "中指根部","无名指根部","小拇指根部","大拇指侧摆","食指侧摆","中指侧摆","无名指侧摆","小拇指侧摆","大拇指横滚","预留","预留","预留","预留","大拇指中部","食指中部","中指中部","无名指中部","小拇指中部","大拇指指尖","食指指尖","中指指尖","无名指指尖","小拇指指尖"]

## 版本更新
- > ### release_1.0.3
  - 1、支持L20/L25版本灵巧手

- > ### release_1.0.2
  - 1、支持L10/O10版本灵巧手
  - 2、支持GUI控制L10/O10版本灵巧手
  - 3、增加支持压力传感器的LinkerHand波形图显示传感器状态
- > ### release_1.0.1
  - 1、支持L7/O7版本灵巧手
  - 2、支持GUI控制L7/O7版本灵巧手


## [示例](examples/)

&ensp;&ensp; __使用前请先将 [setting.yaml](linker_hand_ros2_sdk/src/linker_hand_ros2_sdk/linker_hand_ros2_sdk/LinkerHand/config) 配置文件根据实际需求进行相应修改该.__
&ensp;&ensp; __使用前请先将 [load_write_yaml.py](linker_hand_ros2_sdk/src/linker_hand_ros2_sdk/linker_hand_ros2_sdk/LinkerHand/utils/)文件第28行的yaml_path ="" 改为config目录的绝对路径.__

## 通用
- [0001-gui_control(图形界面控制)](图形界面控制)
开启ROS2 SDK后
```bash
# 新开终端
$ cd linker_hand_ros2_sdk/
$ source ./install/setup.bash
$ ros2 run gui_control gui_control
```
## L7
- [7001-action-group-show-ti(手指运动)](手指运动)

## L10
- [10001-action-group-show-normal(手指运动)](手指运动)


