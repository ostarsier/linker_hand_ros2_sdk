#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

import threading
import time
import json
from http.server import BaseHTTPRequestHandler, HTTPServer
import yaml
import os
from urllib.parse import unquote
import random

class SimpleHttpServer(Node):
    def __init__(self):
        super().__init__('linker_hand_http_server')
        self.publisher_ = self.create_publisher(JointState, '/right_hand_control_cmd', 10)
        self.subscription = self.create_subscription(
            Float32,
            '/right_hand1_mode',
            self.zhitian_callback,
            10)

        self.subscription = self.create_subscription(
            Float32,
            '/right_hand2_mode',
            self.close_callback,
            10)
        self.subscription = self.create_subscription(
            Float32,
            '/right_hand3_mode',
            self.close_callback,
            10)
        

        self.get_logger().info('LinkerHand HTTP Server has been started.')
        self.positions = {}
        self.load_positions()

        # 在单独的线程中运行HTTP服务器
        http_thread = threading.Thread(target=self.run_http_server)
        http_thread.daemon = True
        http_thread.start()

    def run_http_server(self):
        server_address = ('', 8000)
        # 将ROS节点实例传递给RequestHandler
        def handler(*args, **kwargs):
            SimpleHTTPRequestHandler(self, *args, **kwargs)

        httpd = HTTPServer(server_address, handler)
        self.get_logger().info('HTTP server running on port 8000')
        httpd.serve_forever()

    def load_positions(self):
        try:
            yaml_path = '/home/yons/linker_hand_ros2_sdk/src/linker_hand_ros2_sdk/linker_hand_ros2_sdk/LinkerHand/config/L7_positions.yaml'
            with open(yaml_path, 'r', encoding='utf-8') as file:
                data = yaml.safe_load(file)
                if 'LEFT_HAND' in data:
                    for action in data['LEFT_HAND']:
                        action_name = action.get('ACTION_NAME')
                        position = action.get('POSITION')
                        if action_name and position:
                            self.positions['left_' + action_name] = [float(p) for p in position]
                if 'RIGHT_HAND' in data:
                    for action in data['RIGHT_HAND']:
                        action_name = action.get('ACTION_NAME')
                        position = action.get('POSITION')
                        if action_name and position:
                            self.positions['right_' + action_name] = [float(p) for p in position]
                self.get_logger().info(f"Loaded positions: {list(self.positions.keys())}")
        except Exception as e:
            self.get_logger().error(f"Failed to load L7_positions.yaml: {e}")

    def close_callback(self, msg):
        self.get_logger().info('Received close command')
        pose = [67, 151, 0, 0, 0, 0, 37]
        velocity = [60, 60, 60, 60, 60, 60, 60]
        self.publish_pose(pose, velocity)
        self.get_logger().info('Executing close pose.')

    def zhitian_callback(self, msg):
        self.get_logger().info('Received zhitian command')
        pose = [67, 151, 255, 0, 0, 0, 37]
        velocity = [60, 60, 60, 60, 60, 60, 60]
        self.publish_pose(pose, velocity)
        self.get_logger().info('Executing zhitian pose.')


    def publish_pose(self, pose, velocity=[]):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = [float(p) for p in pose]
        msg.velocity = [float(v) for v in velocity]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing pose: {msg.position}')

class SimpleHTTPRequestHandler(BaseHTTPRequestHandler):
    def __init__(self, ros_node, *args, **kwargs):
        self.ros_node = ros_node
        super().__init__(*args, **kwargs)

    def do_POST(self):
        if self.path == '/control':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            try:
                data = json.loads(post_data)
                pose = data.get('pose')
                velocity = data.get('velocity', [])
                if pose is not None:
                    self.ros_node.publish_pose(pose, velocity)
                    self.send_response(200)
                    self.send_header('Content-type', 'application/json')
                    self.end_headers()
                    self.wfile.write(json.dumps({'status': 'success'}).encode())
                else:
                    self.send_error(400, 'Bad Request: Missing pose data')
            except json.JSONDecodeError:
                self.send_error(400, 'Bad Request: Invalid JSON')
        else:
            self.send_error(404, 'Not Found')

    def finger_dance_sequence(self):
        self.ros_node.get_logger().info('Starting a new SUPER complex finger dance...')

        def create_pose(thumb_rot=0, thumb_flex=0, index=0, middle=0, ring=0, pinky=0, wrist=0):
            return [float(thumb_rot), float(thumb_flex), float(index), float(middle), float(ring), float(pinky), float(wrist)]

        dance_moves = []
        slow = 1.5
        medium = 0.8
        fast = 0.4

        # --- 1. 全手掌大幅度开合 ---
        self.ros_node.get_logger().info('Dance: 1. 大幅度开合')
        dance_moves.append((create_pose(thumb_rot=0, thumb_flex=0, index=0, middle=0, ring=0, pinky=0), slow)) # 完全展开
        dance_moves.append((create_pose(thumb_rot=0, thumb_flex=255, index=255, middle=255, ring=255, pinky=255), slow)) # 紧握成拳
        dance_moves.append((create_pose(thumb_rot=0, thumb_flex=0, index=0, middle=0, ring=0, pinky=0), slow)) # 再次完全展开

        # --- 2. "钢琴" - 独立手指运动 ---
        self.ros_node.get_logger().info('Dance: 2. 钢琴演奏')
        # 保持其他手指伸展，依次弯曲再伸直每个手指
        for finger_name in ['index', 'middle', 'ring', 'pinky']:
            pose_args = {'index': 0, 'middle': 0, 'ring': 0, 'pinky': 0}
            pose_args[finger_name] = 255
            dance_moves.append((create_pose(**pose_args), fast))
        dance_moves.append((create_pose(), medium)) # 全部伸直

        # --- 3. 波浪动作 ---
        self.ros_node.get_logger().info('Dance: 3. 波浪')
        # 从小指开始的波浪
        dance_moves.append((create_pose(pinky=255), fast))
        dance_moves.append((create_pose(pinky=255, ring=255), fast))
        dance_moves.append((create_pose(pinky=255, ring=255, middle=255), fast))
        dance_moves.append((create_pose(pinky=255, ring=255, middle=255, index=255), fast))
        # 波浪回滚
        dance_moves.append((create_pose(ring=255, middle=255, index=255), fast))
        dance_moves.append((create_pose(middle=255, index=255), fast))
        dance_moves.append((create_pose(index=255), fast))
        dance_moves.append((create_pose(), medium))

        # --- 4. 拇指运动 (避免碰撞) ---
        self.ros_node.get_logger().info('Dance: 4. 拇指独舞')
        # 其他四指半握，为拇指提供运动空间
        dance_moves.append((create_pose(index=128, middle=128, ring=128, pinky=128), medium))
        # 拇指旋转
        dance_moves.append((create_pose(thumb_rot=255, index=128, middle=128, ring=128, pinky=128), medium))
        dance_moves.append((create_pose(thumb_rot=0, index=128, middle=128, ring=128, pinky=128), medium))
        # 拇指屈伸
        dance_moves.append((create_pose(thumb_flex=255, index=128, middle=128, ring=128, pinky=128), medium))
        dance_moves.append((create_pose(thumb_flex=0, index=128, middle=128, ring=128, pinky=128), medium))

        # --- 5. 结尾动作: OK手势 -> 展开 ---
        self.ros_node.get_logger().info('Dance: 5. OK手势收尾')
        # OK手势，注意拇指和食指的位置
        dance_moves.append((create_pose(thumb_rot=90, thumb_flex=180, index=200, middle=0, ring=0, pinky=0), slow))
        # 最终缓慢展开
        dance_moves.append((create_pose(), slow))

        for pose, delay in dance_moves:
            if not rclpy.ok():
                self.ros_node.get_logger().info('RCLPY shutdown detected, stopping dance.')
                break
            self.ros_node.publish_pose(pose)
            time.sleep(delay)
            
        self.ros_node.get_logger().info('The new SUPER complex finger dance finished.')

    def game_sequence(self):
        """Executes the rock-paper-scissors game sequence."""
        self.ros_node.get_logger().info('Starting rock-paper-scissors game.')

        pose_rock =     [0,   0,   0,   0,   0,   0,   0]  # 石头
        pose_paper =    [255, 255, 255, 255, 255, 255, 0]  # 布 (拇指旋转通常保持0或根据实际调整)
        pose_scissors = [0,   0,   255, 255, 0,   0,   0]  # 剪刀 (拇指内收，食指中指伸直)


        poses = [
            ('剪刀 (scissors)', pose_scissors),
            ('石头 (rock)', pose_rock),
            ('布 (paper)', pose_paper)
        ]

        # Randomly select a pose
        choice_name, chosen_pose = random.choice(poses)
        self.ros_node.get_logger().info(f'Playing: {choice_name}')

        # Execute the chosen pose
        self.ros_node.publish_pose(chosen_pose)
        time.sleep(2)

        # Return to default pose
        default_pose = [120, 120, 200, 200, 200, 200, 0]   
        self.ros_node.get_logger().info('Returning to default pose.')
        self.ros_node.publish_pose(default_pose)
        self.ros_node.get_logger().info('Game finished.')

    def do_GET(self):
        path_parts = self.path.split('/')
        # New path format: ['', 'predef', 'left_hand', 'close']
        if len(path_parts) == 4 and path_parts[1] == 'predef':
            hand_name = unquote(path_parts[2], encoding='utf-8')
            action_name = unquote(path_parts[3], encoding='utf-8')
            
            # Construct the key for the positions dictionary, e.g., 'left_close'
            position_key = f"{hand_name.replace('_hand', '')}_{action_name}"
            self.ros_node.get_logger().info(f'Received GET request for action: {action_name} on {hand_name}')
            
            pose = self.ros_node.positions.get(position_key)
            if pose:
                self.ros_node.publish_pose(pose)
                self.send_response(200)
                self.send_header('Content-type', 'application/json; charset=utf-8')
                self.end_headers()
                response = {'status': 'success', 'hand': hand_name, 'action': action_name, 'pose': pose}
                self.wfile.write(json.dumps(response, ensure_ascii=False).encode('utf-8'))
            else:
                self.send_error(404, f'Action "{action_name}" for hand "{hand_name}" not found')
        elif self.path == '/finger_dance':
            self.send_response(200)
            self.send_header('Content-type', 'application/json; charset=utf-8')
            self.end_headers()
            self.wfile.write(json.dumps({'status': '手指舞已开始'}).encode('utf-8'))
            # 在新线程中执行手指舞以避免阻塞HTTP服务器
            dance_thread = threading.Thread(target=self.finger_dance_sequence)
            dance_thread.start()
        elif self.path == '/game':
            self.send_response(200)
            self.send_header('Content-type', 'application/json; charset=utf-8')
            self.end_headers()
            self.wfile.write(json.dumps({'status': '剪刀石头布游戏已开始'}).encode('utf-8'))
            # 在新线程中执行游戏序列以避免阻塞HTTP服务器
            game_thread = threading.Thread(target=self.game_sequence)
            game_thread.start()
        else:
            self.send_response(200)
            self.send_header('Content-type', 'text/html; charset=utf-8')
            self.end_headers()
            message = """
            <html>
            <head><title>灵巧手 HTTP 控制</title></head>
            <body>
            <h1>灵巧手 HTTP 控制</h1>
            
            <h2>cURL 使用示例:</h2>
            
            <h3>执行预定义动作 (例如: open, close)</h3>
            <p><b>GET /predef/&lt;hand_name&gt;/&lt;action_name&gt;</b></p>
            <p><code>hand_name</code> 可以是 <code>left_hand</code> 或 <code>right_hand</code></p>
            <pre><code>curl http://localhost:8000/predef/left_hand/open</code></pre>
            <pre><code>curl http://localhost:8000/predef/right_hand/close</code></pre>
            
            <h3>设置自定义姿态</h3>
            <p><b>POST /control</b> 使用 JSON `{\"pose\": [p1, ..., p7], \"velocity\": [v1, ... , v7]}`</p>
            <pre><code>curl -X POST -H "Content-Type: application/json" -d '{\"pose\": [0, 0, 0, 0, 0, 0, 0]}' http://localhost:8000/control</code></pre>

            <h3>开始手指舞</h3>
            <p><b>GET /finger_dance</b></p>
            <pre><code>curl http://localhost:8000/finger_dance</code></pre>

            <h3>玩“剪刀、石头、布”</h3>
            <p><b>GET /game</b></p>
            <pre><code>curl http://localhost:8000/game</code></pre>
            
            </body>
            </html>
            """
            self.wfile.write(message.encode('utf-8'))

def main(args=None):
    rclpy.init(args=args)
    http_server_node = SimpleHttpServer()
    try:
        rclpy.spin(http_server_node)
    except KeyboardInterrupt:
        pass
    finally:
        http_server_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
