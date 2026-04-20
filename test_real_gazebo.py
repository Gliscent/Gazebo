import time
import math
import asyncio

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, VehicleGlobalPosition
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')

        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # publisher for move
        self.pubs = {
            0: self.create_publisher(VehicleCommand, '/vehicle1/fmu/in/vehicle_command', self.qos_profile),
            1: self.create_publisher(VehicleCommand, '/vehicle2/fmu/in/vehicle_command', self.qos_profile),
            2: self.create_publisher(VehicleCommand, '/vehicle3/fmu/in/vehicle_command', self.qos_profile),
            3: self.create_publisher(VehicleCommand, '/vehicle4/fmu/in/vehicle_command', self.qos_profile),
            4: self.create_publisher(VehicleCommand, '/vehicle5/fmu/in/vehicle_command', self.qos_profile),
            5: self.create_publisher(VehicleCommand, '/vehicle6/fmu/in/vehicle_command', self.qos_profile),
        }

        self.drone_positions = {0: None, 1: None, 2: None, 3: None, 4: None, 5: None}

        # subscriber for get position
        self.create_subscription(VehicleGlobalPosition, '/vehicle1/fmu/out/vehicle_global_position',
                                 lambda msg: self._pos_callback(0, msg), self.qos_profile)
        self.create_subscription(VehicleGlobalPosition, '/vehicle2/fmu/out/vehicle_global_position',
                                 lambda msg: self._pos_callback(1, msg), self.qos_profile)
        self.create_subscription(VehicleGlobalPosition, '/vehicle3/fmu/out/vehicle_global_position',
                                 lambda msg: self._pos_callback(2, msg), self.qos_profile)
        self.create_subscription(VehicleGlobalPosition, '/vehicle4/fmu/out/vehicle_global_position',
                                 lambda msg: self._pos_callback(3, msg), self.qos_profile)
        self.create_subscription(VehicleGlobalPosition, '/vehicle5/fmu/out/vehicle_global_position',
                                 lambda msg: self._pos_callback(4, msg), self.qos_profile)
        self.create_subscription(VehicleGlobalPosition, '/vehicle6/fmu/out/vehicle_global_position',
                                 lambda msg: self._pos_callback(5, msg), self.qos_profile)

        self.stop_mission_flag = {0: False, 1: False, 2: False, 3: False, 4: False, 5: False}

    # basic ros2 command
    def send_command(self, instance, command, **kwargs):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.target_system = instance + 1
        msg.target_component = 1
        msg.source_system = 255
        msg.source_component = 1
        msg.from_external = True
        msg.confirmation = 1
        for key, value in kwargs.items():
            setattr(msg, key, value)
        if instance in self.pubs:
            self.pubs[instance].publish(msg)

    def sleep(self, seconds):
        start_time = self.get_clock().now().nanoseconds
        while (self.get_clock().now().nanoseconds - start_time) < (seconds * 1e9):
            rclpy.spin_once(self, timeout_sec=0.05)

    async def arm(self, instance):
        # arm
        for i in range(20):  # 10번에서 20번으로 늘림
            self.send_command(instance, 400, param1=1.0, param2=21196.0)
            await asyncio.sleep(0.2)

        await asyncio.sleep(2.0)

    async def takeoff(self, instance, altitude):
        self.get_logger().info(f'[ID {instance}] takeoff')
        for _ in range(10):
            self.send_command(instance, 22, param7=altitude, param5=float('nan'), param6=float('nan'))
            await asyncio.sleep(0.1)

    async def land(self, instance):
        self.get_logger().info(f'[ID {instance}] land')

        for _ in range(10):
            self.send_command(instance, 21,
                              param5=float('nan'),
                              param6=float('nan'),
                              param7=0.0)  # 지면 고도
            await asyncio.sleep(0.1)

    def stop_rover(self, instance):
        curr_lat, curr_lon, _ = self.get_position(instance)
        if curr_lat is None: return

        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = 192  # MAV_CMD_DO_REPOSITION
        msg.param1 = -1.0  # 현재 속도 유지 혹은 기본값
        msg.param4 = float('nan')  # Yaw 유지
        msg.param5 = float(curr_lat)
        msg.param6 = float(curr_lon)
        msg.param7 = float('nan')  # 로버는 고도 무시

        msg.target_system = instance + 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.pubs[instance].publish(msg)
        self.get_logger().info(f"[Rover {instance}] stop")

    def _pos_callback(self, instance, msg):
        self.drone_positions[instance] = {'lat': msg.lat, 'lon': msg.lon, 'alt': msg.alt}

    def get_position(self, instance):
        pos = self.drone_positions.get(instance)
        if pos:
            return pos['lat'], pos['lon'], pos['alt']
        else:
            self.get_logger().warn(f"No data from Drone {instance}")
            return None, None, None

    def move_drone(self, instance, lat, lon, alt):
        if instance not in self.pubs:
            self.get_logger().error(f'ID {instance} not exist!')
            return

        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = 192  # MAV_CMD_DO_REPOSITION
        msg.param5 = float(lat)
        msg.param6 = float(lon)
        msg.param7 = float(alt)

        msg.target_system = instance + 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        # publish
        self.pubs[instance].publish(msg)

    def move_rover(self, instance, lat, lon):
        if instance not in self.pubs:
            self.get_logger().error(f'ID {instance} not exist!')
            return

        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = 192  # MAV_CMD_DO_REPOSITION
        msg.param5 = float(lat)
        msg.param6 = float(lon)
        msg.param7 = 0.0  # 로버는 고도 0.0

        msg.target_system = instance + 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        # publish
        self.pubs[instance].publish(msg)

    async def move_waypoints(self, instance, is_rover, waypoints):
        for i, wp in enumerate(waypoints):
            lat, lon, alt = wp

            if is_rover:
                self.move_rover(instance, lat, lon)
            else:
                self.move_drone(instance, lat, lon, alt)

            while rclpy.ok():
                if self.stop_mission_flag[instance]:
                    self.get_logger().info(f'[ID {instance}] mission failed flag!')
                    await self.land(instance)
                    return  # stop function

                else:
                    curr_lat, curr_lon, curr_alt = self.get_position(instance)

                    if curr_lat is not None:
                        dist = self.get_distance(curr_lat, curr_lon, lat, lon)

                        if dist < 0.00002:  # threshold
                            self.get_logger().info(f'go to next waypoint')
                            break

                    await asyncio.sleep(0.5)  # 2hz position check

    async def follow_target(self, drone_id, target_id, height):
        while rclpy.ok() and not self.stop_mission_flag[drone_id]:
            lat, lon, _ = self.get_position(target_id)

            if lat is not None:
                self.move_drone(drone_id, lat, lon, height)
            await asyncio.sleep(0.2)

    @staticmethod
    def get_distance(lat1, lon1, lat2, lon2):
        return math.sqrt((lat1 - lat2) ** 2 + (lon1 - lon2) ** 2)


async def run_mission(controller):
    UAV_L = [0, 1]
    UAV_S = [2, 3]
    UGV = [4, 5]
    all = UAV_L + UAV_S + UGV
    UAV = UAV_L + UAV_S

    # # 1. 모든 vehicles 시동
    controller.get_logger().info('drones 시동...')
    for i in all:
        asyncio.create_task(controller.arm(i))
        await asyncio.sleep(0.5)  # 0.5초 간격으로 arm 명령 시작 유도

    # 2. 드론 이륙
    controller.get_logger().info('모든 드론 이륙 시작...')
    await asyncio.gather(*[controller.takeoff(i, 145.0) for i in UAV])
    controller.get_logger().info('이동 전 대기...')
    await asyncio.sleep(30.0)

    # 3. 이동
    # controller.get_logger().info('이동...')
    # controller.move_drone(instance=0, lat=36.729104, lon=127.441943, alt=135.0)
    # controller.move_drone(instance=1, lat=-78.397842, lon=-17.545494, alt=135.0)
    # controller.move_drone(instance=2, lat=36.397842, lon=8.545594, alt=135.0)
    # controller.move_drone(instance=3, lat=36.397842, lon=8.545594, alt=135.0)
    # controller.move_rover(instance=4, lat=47.397842, lon=8.545694)
    # controller.move_rover(instance=5, lat=47.397842, lon=8.545694)
    # controller.sleep(1)

    lc62_waypoints1 = [
        [36.727563, 127.441557, 160.0],
        [36.727563, 127.441557, 160.0]
    ]

    lc62_waypoints2 = [
        [36.728360, 127.442670, 170.0],
        [36.728360, 127.442670, 170.0],
    ]

    lc62_waypoints3_breakdown = [
        [36.727917, 127.441806, 170.0],
        [36.727917, 127.441806, 170.0],
    ]


    rover_waypoints1 = [
        [36.728826, 127.441886, 0.0],
        [36.728731, 127.441803, 0.0],
        [36.728635, 127.441781, 0.0],
        [36.728503, 127.441835, 0.0],
        [36.728466, 127.441897, 0.0],
        [36.728458, 127.442093, 0.0],
        [36.728506, 127.442161, 0.0],
        [36.728697, 127.442262, 0.0],
        [36.728785, 127.442332, 0.0],
        [36.728828, 127.442402, 0.0],
        [36.728838, 127.442493, 0.0],
        [36.728835, 127.442607, 0.0],  # (out of white road)
    ]

    rover_waypoints2 = [
        [36.728585, 127.442481, 0.0],
        [36.728268, 127.442433, 0.0],
        [36.727950, 127.442413, 0.0],
        [36.727888, 127.442413, 0.0],

        [36.727858, 127.442327, 0.0],
        [36.727739, 127.441759, 0.0]
    ]

    rover_waypoints3 = [
        [36.728585, 127.442481, 0.0],
        [36.728268, 127.442433, 0.0],
        [36.727950, 127.442413, 0.0],
        [36.727888, 127.442413, 0.0],

        [36.727872, 127.442492, 0.0],
        [36.727909, 127.442804, 0.0]
    ]

    # Before Replan
    task1 = asyncio.create_task(
        controller.move_waypoints(instance=0, is_rover=False, waypoints=lc62_waypoints1))
    task2 = asyncio.create_task(
        controller.move_waypoints(instance=1, is_rover=False, waypoints=lc62_waypoints2))

    task3 = asyncio.create_task(controller.follow_target(drone_id=2, target_id=4, height=140.0))
    task4 = asyncio.create_task(controller.follow_target(drone_id=3, target_id=5, height=150.0))

    task5 = asyncio.create_task(
        controller.move_waypoints(instance=4, is_rover=True, waypoints=rover_waypoints1 + rover_waypoints2))
    task6 = asyncio.create_task(
        controller.move_waypoints(instance=5, is_rover=True, waypoints=rover_waypoints1 + rover_waypoints3))

    await asyncio.sleep(100.0)
    controller.stop_mission_flag[0] = True  # breakdown

    controller.get_logger().warn("lc62 (instance0), breakdown!!! ")
    await asyncio.sleep(0.5)
    task1_breakdown = asyncio.create_task(controller.land(0))

    # After Replan
    controller.get_logger().warn("replanning...")
    controller.stop_mission_flag[1] = True  # for replanning
    await asyncio.sleep(1.0)
    controller.stop_mission_flag[1] = False  # for replanning

    task2_replan = asyncio.create_task(
        controller.move_waypoints(instance=1, is_rover=False, waypoints=lc62_waypoints3_breakdown)
    )

    await asyncio.gather(task1_breakdown, task2_replan, task3, task4, task5, task6)

    # print(controller.get_position(instance=0))
    # print(controller.get_position(instance=1))
    # print(controller.get_position(instance=2))
    # print(controller.get_position(instance=3))
    # print(controller.get_position(instance=4))
    # print(controller.get_position(instance=5))

    # 4. 로버 정지 & 드론 착륙
    controller.stop_rover(instance=4)
    controller.stop_rover(instance=5)
    controller.get_logger().info('모든 드론 착륙 시작...')
    await asyncio.gather(*[controller.land(i) for i in UAV])
    await asyncio.sleep(10.0)
    controller.get_logger().info('미션 종료.')


def main(args=None):
    rclpy.init(args=args)
    controller = DroneController()
    loop = asyncio.get_event_loop()

    async def spin_loop():
        while rclpy.ok():
            rclpy.spin_once(controller, timeout_sec=0)
            await asyncio.sleep(0.01)

    try:
        loop.create_task(spin_loop())
        loop.run_until_complete(run_mission(controller))
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
