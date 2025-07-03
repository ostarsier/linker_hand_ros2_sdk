#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty

import threading
import time
import json
from http.server import BaseHTTPRequestHandler, HTTPServer
import yaml
import os
from urllib.parse import unquote

class SimpleHttpServer(Node):
    def __init__(self):
        super().__init__('linker_hand_http_server')
        self.publisher_ = self.create_publisher(JointState, '/left_hand_control_cmd', 10)
        self.subscription = self.create_subscription(
            Empty,
            '/shake_hand',
            self.shake_hand_callback,
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
            script_dir = os.path.dirname(os.path.realpath(__file__))
            yaml_path = os.path.join(script_dir, 'LinkerHand', 'config', 'L7_positions.yaml')
            
            with open(yaml_path, 'r', encoding='utf-8') as file:
                data = yaml.safe_load(file)
                if 'LEFT_HAND' in data:
                    for action in data['LEFT_HAND']:
                        action_name = action.get('ACTION_NAME')
                        position = action.get('POSITION')
                        if action_name and position:
                            self.positions[action_name] = [float(p) for p in position]
                self.get_logger().info(f"Loaded positions: {list(self.positions.keys())}")
        except Exception as e:
            self.get_logger().error(f"Failed to load L7_positions.yaml: {e}")

    def shake_hand_callback(self, msg):
        self.get_logger().info('Received shake hand command')
        # 握手姿态：所有手指半握
        pose = [128, 128, 128, 128, 128, 128, 0] # 这是一个示例姿态，您可能需要根据实际情况调整
        self.publish_pose(pose)
        self.get_logger().info('Executing handshake pose.')

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
        self.ros_node.get_logger().info('Starting finger dance...')
        # 改编自finger_dance.py的动作序列
        dance_moves = [
            ([0, 0, 255, 0, 0, 0, 0], 0.15),
            ([0, 0, 0, 0, 0, 255, 0], 0.15),
        ] * 4
        dance_moves.append(([0] * 7, 0.2))

        for i in range(2, 7):
            pose = [0] * 7
            pose[i] = 255
            dance_moves.append((pose, 0.08))
        for i in range(5, 1, -1):
            pose = [0] * 7
            pose[i] = 255
            dance_moves.append((pose, 0.08))
        dance_moves.append(([0] * 7, 0.2))

        for pose, delay in dance_moves:
            self.ros_node.publish_pose(pose)
            time.sleep(delay)
        self.ros_node.get_logger().info('Finger dance finished.')

    def do_GET(self):
        path_parts = self.path.split('/')
        # Path will be ['', 'left_hand', 'close']
        if len(path_parts) == 3 and path_parts[1] == 'left_hand':
            action_name = unquote(path_parts[2], encoding='utf-8')
            self.ros_node.get_logger().info(f'Received GET request for action: {action_name}')
            
            pose = self.ros_node.positions.get(action_name)
            if pose:
                self.ros_node.publish_pose(pose)
                self.send_response(200)
                self.send_header('Content-type', 'application/json; charset=utf-8')
                self.end_headers()
                response = {'status': 'success', 'action': action_name, 'pose': pose}
                self.wfile.write(json.dumps(response, ensure_ascii=False).encode('utf-8'))
            else:
                self.send_error(404, f'Action "{action_name}" not found')
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
            <p><b>GET /left_hand/&lt;action_name&gt;</b></p>
            <pre><code>curl http://localhost:8000/left_hand/open</code></pre>
            <pre><code>curl http://localhost:8000/left_hand/close</code></pre>
            
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
