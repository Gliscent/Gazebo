#!/bin/bash

# 1. 환경 설정 및 경로 변수
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

export GZ_MODEL_PATH=$GZ_MODEL_PATH:$HOME/PX4-Autopilot/Tools/simulation/gz/models
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$HOME/PX4-Autopilot/Tools/simulation/gz/models:$HOME/PX4-Autopilot/Tools/simulation/gz/worlds

# 2. 기존 프로세스 정리
echo "Cleaning up old processes..."
killall -9 ruby gz-sim-server px4 MicroXRCEAgent 2>/dev/null

# 3. Gazebo 실행
echo "Starting Gazebo World..."
gz sim test.sdf &
sleep 10

# 4. MicroXRCE DDS 실행
MicroXRCEAgent udp4 -p 8888 > /dev/null 2>&1 &
sleep 5

# 5. Spawn Vehicles (UDP 포트 0,1,2,3 까지는 충돌 가능성이 있어서 넉넉히 10부터 시작)

# Drone1
cd ~/PX4-Autopilot
echo "Spawning quadcopter 1..."
PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE="0,1,0,0,0,0" PX4_GZ_WORLD=test_world ./build/px4_sitl_default/bin/px4 -i 10 > /dev/null 2>&1 &
sleep 3

# Drone2
echo "Spawning quadcopter 2..."
PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE="0,2,0,0,0,0" PX4_GZ_WORLD=test_world ./build/px4_sitl_default/bin/px4 -i 11 > /dev/null 2>&1 &
sleep 3

# Rover1
cd ~/PX4-Autopilot
echo "Spawning rover 1..."
PX4_SYS_AUTOSTART=4009 PX4_GZ_MODEL=gz_r1_rover PX4_GZ_MODEL_POSE="0,5,0,0,0,0" PX4_GZ_WORLD=test_world ./build/px4_sitl_default/bin/px4 -i 12 > /dev/null 2>&1 &
sleep 3

sleep 15

# 사용 예시 (이후 삭제 가능성 고려)
# Drone1 Arm -> Takeoff
cd ~/PX4-Autopilot/build/px4_sitl_default/bin/
echo "Attempting Takeoff..."
./px4-commander --instance 10 arm
sleep 2
./px4-commander --instance 10 takeoff

# Drone2 Arm -> Takeoff
cd ~/PX4-Autopilot/build/px4_sitl_default/bin/
echo "Attempting Takeoff..."
./px4-commander --instance 11 arm
sleep 2
./px4-commander --instance 11 takeoff

# Rover1 Arm --> offboard
cd ~/PX4-Autopilot/build/px4_sitl_default/bin/
./px4-commander --instance 12 arm
./px4-commander --instance 12 mode offboard


