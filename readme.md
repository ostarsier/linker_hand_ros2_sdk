

https://document.linkeros.cn/developer/68

# 开启CAN端口
sudo ip link set can0 down # 先关闭接口
sudo ip link set can0 type can bitrate 1000000  #USB转CAN设备蓝色灯常亮状态
sudo ip link set can0 txqueuelen 1000 # 将发送队列长度设置为1000 
sudo ip link set can0 up # 重新启动接口

# 查看can0队列长度
ip -s link show can0 

# 查看can0状态
ip -details link show can0

# 查看can0数据
candump can0

# python example/L7/gesture/crasp.py


# 移动到固定姿势
python hand_api.py
curl http://localhost:5000/hand/handshake


---

修改配置文件路径
load_write_yaml.py

# 创建虚拟环境
python3.10 -m venv linker_hand_env
source linker_hand_env/bin/activate
pip install -r requirements.txt

# 1. 启动灵巧手ros2
- 安装的依赖包没有被ros2的python环境识别，需要手动添加
export PYTHONPATH="$PYTHONPATH:/home/yons/linker_hand_env/lib/python3.10/site-packages/"
export PYTHONPATH="$PYTHONPATH:/home/yons/linker_hand_ros2_sdk/src/linker_hand_ros2_sdk/linker_hand_ros2_sdk"

cd /home/yons/linker_hand_ros2_sdk
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 run linker_hand_ros2_sdk linker_hand_sdk

# 2. 启动灵巧手http服务器
ros2 run linker_hand_ros2_sdk linker_hand_http_server

# 3.调用http服务

curl -X POST -H "Content-Type: application/json" -d '{"pose": [255, 255, 255, 255, 255, 255, 255]}' http://localhost:8000/control