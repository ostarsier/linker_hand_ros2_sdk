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
            '/shake_hand_mode',
            self.shake_hand_callback,
            10)
        self.subscription = self.create_subscription(
            Float32,
            '/grasp_mode',
            self.grasp_hand_callback,
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

    def shake_hand_callback(self, msg):
        self.get_logger().info('Received shake hand command')
        # 握手姿态：所有手指半握
        pose = [180, 100, 200, 200, 200, 200, 100]
        velocity = [60, 60, 60, 60, 60, 60, 60]
        self.publish_pose(pose, velocity)
        self.get_logger().info('Executing handshake pose.')

    def grasp_hand_callback(self, msg):
        from .grasp import grasp
        self.get_logger().info('Received grasp hand command, starting in a new thread.')
        grasp_thread = threading.Thread(target=grasp)
        grasp_thread.daemon = True
        grasp_thread.start()

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
        fast = 0.05
        medium = 0.1
        slow = 0.2
        
        # --- Introduction: "The Calibrator" ---
        self.ros_node.get_logger().info('Dance Intro: The Calibrator')
        dance_moves.append((create_pose(), 0.5)) # Start open
        # Cycle through each finger
        for i in range(1, 6): # All fingers including thumb flex
            pose = create_pose()
            pose[i] = 255
            dance_moves.append((pose, slow))
            dance_moves.append((create_pose(), slow))
        # Thumb rotation
        dance_moves.append((create_pose(thumb_rot=255), slow))
        dance_moves.append((create_pose(), slow))

        # --- Part 1: "Ripples on a Pond" ---
        self.ros_node.get_logger().info('Dance Part 1: Ripples on a Pond')
        for _ in range(3):
            # Ripple out
            for i in range(2, 6): # index to pinky
                pose = create_pose()
                pose[i] = 255
                dance_moves.append((pose, fast))
            # Ripple in
            for i in range(5, 1, -1):
                pose = create_pose()
                pose[i] = 255
                dance_moves.append((pose, fast))
        dance_moves.append((create_pose(), medium)) # Clear

        # --- Part 2: "The Percussionist" ---
        self.ros_node.get_logger().info('Dance Part 2: The Percussionist')
        for _ in range(30):
            finger1 = random.randint(2, 5)
            finger2 = random.randint(2, 5)
            pose = create_pose()
            pose[finger1] = 255
            if finger1 != finger2:
                pose[finger2] = 255
            dance_moves.append((pose, fast))
            dance_moves.append((create_pose(), fast))

        # --- Part 3: "Thumb Acrobatics" ---
        self.ros_node.get_logger().info('Dance Part 3: Thumb Acrobatics')
        dance_moves.append((create_pose(thumb_rot=128, thumb_flex=128), medium)) # Thumb center
        # Touch each finger with thumb
        for i in range(2, 6):
            dance_moves.append((create_pose(thumb_rot=128, thumb_flex=255, **{['index', 'middle', 'ring', 'pinky'][i-2]: 255}), medium))
        dance_moves.append((create_pose(), medium)) # Reset

        # --- Part 4: "The Swarm" ---
        self.ros_node.get_logger().info('Dance Part 4: The Swarm')
        current_pose = [0.0] * 7
        for _ in range(50):
            # Each finger moves randomly up or down
            for i in range(1, 6):
                current_pose[i] += random.uniform(-80, 80)
                current_pose[i] = max(0, min(255, current_pose[i])) # Clamp
            dance_moves.append((list(current_pose), fast))
        
        # --- Crescendo: "The Power Grip" ---
        self.ros_node.get_logger().info('Dance Crescendo: The Power Grip')
        # Slowly close all fingers
        for i in range(0, 256, 16):
            val = float(i)
            dance_moves.append((create_pose(thumb_flex=val, index=val, middle=val, ring=val, pinky=val), fast))
        # Hold fist
        dance_moves.append((create_pose(thumb_flex=255, index=255, middle=255, ring=255, pinky=255), 1.0))
        # Slowly open
        for i in range(255, -1, -16):
            val = float(i)
            dance_moves.append((create_pose(thumb_flex=val, index=val, middle=val, ring=val, pinky=val), medium))

        # --- Finale: "Starlight" ---
        self.ros_node.get_logger().info('Dance Finale: Starlight')
        for _ in range(40):
            pose = create_pose()
            for i in range(1, 6):
                if random.random() > 0.5:
                    pose[i] = random.uniform(100, 200)
            dance_moves.append((pose, fast))

        # --- Conclusion: "The Bow" ---
        self.ros_node.get_logger().info('Dance Conclusion: The Bow')
        dance_moves.append((create_pose(thumb_flex=255, index=255, middle=255, ring=255, pinky=255), 1.0)) # Close
        dance_moves.append((create_pose(), 1.5)) # Open slowly

        for pose, delay in dance_moves:
            if not rclpy.ok():
                self.ros_node.get_logger().info('RCLPY shutdown detected, stopping dance.')
                break
            self.ros_node.publish_pose(pose)
            time.sleep(delay)
            
        self.ros_node.get_logger().info('The new SUPER complex finger dance finished.')

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
