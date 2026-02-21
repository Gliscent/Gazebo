커맨드는 항상 바뀌기 때문에 명시적인 느낌의 메뉴얼

1. Ubuntu install

2. ROS2 install & colcon install(우분투 버전 고려)

3. Workspace generate & Msg defintion download
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git
```
```
cd ~/ros2_ws
colcon build
```
```
source ~/ros2_ws/install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```
4. PX4 install from github (https://github.com/PX4/PX4-Autopilot)
After install,
```
make px4_sitl gz_x500
```
```
make px4_sitl gz_r1_rover
```

6. Gazebo sim install

(4, 5)번 설치 이후,
```
echo 'export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds' >> ~/.bashrc && source ~/.bashrc
```

설치 이후 px4+가제보 사용 예시
```
make px4_sitl gz_x500
```
px4> 터미널 활성화 후,
```
commander arm
commander takeoff
```

5. QGC download

6. microxrce agent install from github (https://github.com/eProsima/Micro-XRCE-DDS-Agent.git/)

설치 이후 사용 예시
```
MicroXRCEAgent udp4 -p 8888
```

Non-blocking drone move
```
# drone move~!
ros2 topic pub -1 /px4_10/fmu/in/vehicle_command px4_msgs/msg/VehicleCommand "{
  timestamp: $(date +%s%N | cut -b 1-13),
  command: 192,
  param4: 0.0,
  param5: 47.79742,
  param6: 8.445594,
  param7: 10.0,
  target_system: 11,
  target_component: 1,
  source_system: 1,
  source_component: 1,
  from_external: true
}"
```

Non-blocking rover move
```
# rover move!!!!!
ros2 topic pub -1 /px4_12/fmu/in/vehicle_command px4_msgs/msg/VehicleCommand "{
  timestamp: $(date +%s%N | cut -b 1-13),
  command: 192,
  param5: 47.7742,
  param6: 8.545594,
  target_system: 13,
  target_component: 1,
  source_system: 1,
  source_component: 1,
  from_external: true
}"
```
