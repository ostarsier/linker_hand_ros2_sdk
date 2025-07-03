# linker_hand.py 说明文档

## 简介

`linker_hand.py` 是面向 ROS2 的灵巧手（Linker Hand）SDK 的核心节点实现文件，主要用于通过 ROS2 框架对灵巧手进行初始化、参数配置、状态获取、运动控制、触觉反馈等操作。

本模块适用于搭载 Linker Hand 硬件的机器人系统，支持多种关节类型（如 L7、L10、L20、L21、L25）及触觉传感器。

---

## 主要功能

- 支持左手/右手多关节类型灵巧手的初始化与参数配置
- 通过 ROS2 topic 实现手部运动控制与速度设定
- 支持灵巧手状态/信息/触觉等多种消息的发布与订阅
- CAN 通信自动初始化与关闭
- 支持多线程安全和频率限制
- 支持灵巧手故障检测与清除、力矩/速度/电流等参数设置

---

## 主要类与方法

### LinkerHand(Node)
核心 ROS2 节点类，主要方法：
- `__init__`：参数声明、变量初始化、CAN 初始化、订阅/发布器创建
- `init_hand`：根据手型和关节类型初始化 API、CAN、topic
- `run`：主循环入口
- `left_hand_control_cb`/`right_hand_control_cb`：左右手运动控制回调
- `hand_setting_cb`：参数设置与指令回调
- 其它：状态/信息/触觉发布、故障处理、CAN 关闭等

---

## 主要依赖
- ROS2 (`rclpy`, `std_msgs`, `sensor_msgs`)
- threading, time, json, math, sys
- 项目内依赖：`linker_hand_api`、`color_msg`、`open_can`

---

## 使用说明

### 编译

```bash
colcon build --symlink-install
```

### 启动

```bash
ros2 run linker_hand_ros2_sdk linker_hand_sdk
```

### 主要参数
- `hand_type`：手型（left/right），默认 left
- `hand_joint`：关节类型（如 L7/L10），默认 L7
- `is_touch`：是否带触觉，默认 True
- `can`：CAN 通道，默认 can0

### 主要话题（已更新）
- 左手控制命令订阅：`/left_hand_control_cmd`、`/left_hand_control_cmd_arc`
- 左手状态/信息/触觉发布：`/left_hand_state`、`/left_hand_state_arc`、`/left_hand_info`、`/left_hand_matrix_touch`、`/left_hand_force`
- 右手控制命令订阅：`/right_hand_control_cmd`、`/right_hand_control_cmd_arc`
- 右手状态/信息/触觉发布：`/right_hand_state`、`/right_hand_state_arc`、`/right_hand_info`、`/right_hand_matrix_touch`、`/right_hand_force`
- 参数设置订阅：`/hand_setting_cmd`

---

## 使用方式示例

### 1. 启动节点
```bash
colcon build --symlink-install
ros2 run linker_hand_ros2_sdk linker_hand_sdk
```

### 2. 发布控制命令（以左手为例）
```bash
# 发布关节位置控制命令
ros2 topic pub /left_hand_control_cmd sensor_msgs/JointState '{position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'

# 发布弧线插值控制命令
ros2 topic pub /left_hand_control_cmd_arc sensor_msgs/JointState '{position: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7], velocity: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]}'
```

### 3. 订阅状态/信息/触觉
```bash
ros2 topic echo /left_hand_state
ros2 topic echo /left_hand_info
ros2 topic echo /left_hand_matrix_touch
ros2 topic echo /left_hand_force
```

### 4. 参数设置与故障处理
# 设置最大力矩
ros2 topic pub /hand_setting_cmd std_msgs/String '{"setting_cmd": "set_max_torque_limits", "params": {"hand_type": "left", "torque": [200, 200, 200, 200, 200]}}'

---
<br>

# LinkerHand HTTP Server (`linker_hand_http_server.py`)

`linker_hand_http_server.py` 是一个 ROS2 节点，它提供了一个 HTTP 接口来控制灵巧手。这使得通过简单的 HTTP 请求来控制机械手成为可能，便于与 Web 应用程序、脚本或其他非 ROS 系统集成。

## 主要功能

- 在 8000 端口启动一个 HTTP 服务器。
- 提供 API 端点以触发预定义动作和自定义姿态。
- 从 YAML 配置文件加载预定义动作。
- 与主要的灵巧手 ROS2 主题集成。

## 如何运行

1.  **编译工作空间:**
    ```bash
    colcon build --symlink-install
    ```

2.  **配置环境:**
    ```bash
    source install/setup.bash
    ```

3.  **运行节点:**
    ```bash
    ros2 run linker_hand_ros2_sdk linker_hand_http_server
    ```
    *(注意: 可执行文件名称可能因 `setup.py` 的配置而异。)*

## HTTP API 端点

服务器监听在 `http://<your-robot-ip>:8000`。

### `GET /`

-   **描述:** 显示一个简单的 HTML 页面，其中包含有关如何使用 API 的说明。

### `GET /left_hand/<action_name>`

-   **描述:** 为左手执行一个预定义的动作。`action_name` 必须与 `L7_positions.yaml` 文件中的条目相对应。
-   **示例:**
    ```bash
    curl http://localhost:8000/left_hand/close
    ```
-   **成功响应 (200 OK):**
    ```json
    {
      "status": "success",
      "action": "open",
      "pose": [ ... ]
    }
    ```
-   **错误响应 (404 Not Found):** 如果动作名称不存在。

### `POST /control`

-   **描述:** 为手部关节设置自定义的位置和速度。
-   **请求体 (JSON):**
    ```json
    {
      "pose": [p1, p2, p3, p4, p5, p6, p7],
      "velocity": [v1, v2, v3, v4, v5, v6, v7]
    }
    ```
    -   `pose`: 一个包含7个浮点数的列表，用于目标关节位置。(必需)
    -   `velocity`: 一个包含7个浮点数的列表，用于关节速度。(可选)
-   **示例:**
    ```bash
    curl -X POST -H "Content-Type: application/json" -d '{"pose": [0, 0, 0, 0, 0, 0, 0]}' http://localhost:8000/control
    ```
-   **成功响应 (200 OK):**
    ```json
    {
      "status": "success"
    }
    ```

### `POST /finger_dance`

-   **描述:** 触发一个预定义的“手指舞”序列。
-   **示例:**
    ```bash
    curl -X POST http://localhost:8000/finger_dance
    ```
-   **成功响应 (200 OK):**
    ```json
    {
      "status": "Finger dance started"
    }

## 配置文件

预定义的动作从 `linker_hand_ros2_sdk/linker_hand_ros2_sdk/LinkerHand/config/L7_positions.yaml` 加载。您可以在此文件中添加或修改动作。

每个动作都有一个 `ACTION_NAME` 和一个 `POSITION` 数组。

`L7_positions.yaml` 示例:
```yaml
LEFT_HAND:
  - ACTION_NAME: "close"
    POSITION: [255, 255, 255, 255, 255, 255, 0]
  - ACTION_NAME: "open"
    POSITION: [0, 0, 0, 0, 0, 0, 0]
```

## ROS2 接口

### 发布者

-   `/left_hand_control_cmd` (`sensor_msgs/msg/JointState`): 发布关节指令以控制手部。

### 订阅者

-   `/shake_hand` (`std_msgs/msg/Empty`): 监听执行握手姿势的命令。

# 设置速度
ros2 topic pub /hand_setting_cmd std_msgs/String '{"setting_cmd": "set_speed", "params": {"hand_type": "left", "speed": [1, 1, 1, 1, 1]}}'

# 清除故障
ros2 topic pub /hand_setting_cmd std_msgs/String '{"setting_cmd": "clear_faults", "params": {"hand_type": "left"}}'
```

---

## 代码结构

- linker_hand.py         —— 主节点实现
- linker_hand_api.py     —— 灵巧手 API 封装
- color_msg.py           —— 彩色日志输出
- open_can.py            —— CAN 通信接口

---

## 联系方式
如有问题或需求请联系 Linker Robotics 官方技术支持。
