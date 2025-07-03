
修改配置文件路径
load_write_yaml.py

# 1. 启动灵巧手ros2

pip install -r requirements.txt
- 安装的依赖包没有被ros2的python环境识别，需要手动添加
export PYTHONPATH="/media/yons/843b68f6-dfaf-466a-871e-769728918988/miniconda3/envs/ros2/lib/python3.10/site-packages:$PYTHONPATH"
- 添加LinkerHand包
export PYTHONPATH=$PYTHONPATH:/home/yons/linker_hand_ros2_sdk/src/linker_hand_ros2_sdk/linker_hand_ros2_sdk

cd /home/yons/linker_hand_ros2_sdk
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 run linker_hand_ros2_sdk linker_hand_sdk

# 2. 启动灵巧手http服务器
ros2 run linker_hand_ros2_sdk linker_hand_http_server