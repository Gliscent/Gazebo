import time
import asyncio
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, VehicleGlobalPosition, GotoSetpoint
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
            6: self.create_publisher(VehicleCommand, '/vehicle7/fmu/in/vehicle_command', self.qos_profile),
            7: self.create_publisher(VehicleCommand, '/vehicle8/fmu/in/vehicle_command', self.qos_profile),
        }
        self.goto_pubs = {
            1: self.create_publisher(GotoSetpoint, '/vehicle2/fmu/in/goto_setpoint', self.qos_profile),
        }

        self.drone_positions = {0: None, 1: None, 2: None, 3: None, 4: None, 5: None, 6: None, 7: None}

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
        self.create_subscription(VehicleGlobalPosition, '/vehicle7/fmu/out/vehicle_global_position',
                                 lambda msg: self._pos_callback(6, msg), self.qos_profile)
        self.create_subscription(VehicleGlobalPosition, '/vehicle8/fmu/out/vehicle_global_position',
                                 lambda msg: self._pos_callback(7, msg), self.qos_profile)

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

    def move_hexacopter(self, instance, lat, lon, alt):
        if instance not in self.pubs:
            self.get_logger().error(f'ID {instance} not exist!')
            return

        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = 192  # MAV_CMD_DO_REPOSITION
        msg.param5 = float(lat)
        msg.param6 = float(lon)
        msg.param7 = float(alt)

        # # 필수 파라미터들
        # msg.param1 = -1.0  # -1.0은 기체의 기본 속도를 사용한다는 뜻입니다.
        # msg.param2 = 0.0  # Reposition 타입 (0: 전역 좌표 이동)
        # msg.param4 = float('nan')  # 헤딩 유지 (필요하면 현재 yaw값 입력)

        msg.target_system = instance + 1
        msg.target_component = 1  # 반드시 1이어야 합니다.
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.pubs[instance].publish(msg)


async def run_mission(controller):
    drones = [0, 1, 2, 3]
    crash = [4]
    rovers = [5, 6, 7]

    # # 1. 모든 vehicles 시동
    controller.get_logger().info('drones 시동...')
    for i in [0, 1, 2, 3]:
        asyncio.create_task(controller.arm(i))
        await asyncio.sleep(0.5)  # 0.5초 간격으로 arm 명령 시작 유도

    # 2. 드론 이륙
    controller.get_logger().info('모든 드론 이륙 시작...')
    await asyncio.gather(*[controller.takeoff(i, 80.0) for i in [0, 1, 2, 3]])
    controller.get_logger().info('이동 전 대기...')
    await asyncio.sleep(30.0)

    # # 1. 모든 vehicles 시동
    controller.get_logger().info('rovers 시동...')
    for i in [5, 6, 7]:
        asyncio.create_task(controller.arm(i))
        await asyncio.sleep(0.5)  # 0.5초 간격으로 arm 명령 시작 유도
    await asyncio.sleep(10)

    # 3. 이동
    controller.get_logger().info('이동...')
    controller.move_drone(instance=0, lat=36.397842, lon=8.545494, alt=105.0)
    controller.move_drone(instance=1, lat=-78.397842, lon=-17.545494, alt=105.0)
    controller.move_drone(instance=2, lat=36.397842, lon=8.545594, alt=105.0)
    controller.move_drone(instance=3, lat=36.397842, lon=8.545594, alt=105.0)
    controller.move_rover(instance=5, lat=47.397842, lon=8.545694)
    controller.move_rover(instance=6, lat=47.397842, lon=8.545694)
    controller.move_rover(instance=7, lat=47.397842, lon=8.545694)
    controller.sleep(15)

    # 4. 로버 정지 & 드론 착륙
    # controller.stop_rover(instance=12)
    controller.get_logger().info('모든 드론 착륙 시작...')
    await asyncio.gather(*[controller.land(i) for i in [0, 1, 2, 3]])
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
