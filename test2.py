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

        # 퍼블리셔
        self.pubs = {
            10: self.create_publisher(VehicleCommand, '/px4_10/fmu/in/vehicle_command', self.qos_profile),
            11: self.create_publisher(VehicleCommand, '/px4_11/fmu/in/vehicle_command', self.qos_profile),
            12: self.create_publisher(VehicleCommand, '/px4_12/fmu/in/vehicle_command', self.qos_profile),
        }

        self.drone_positions = {10: None, 11: None, 12: None}

        # subscriber for get position
        self.create_subscription(VehicleGlobalPosition, '/px4_10/fmu/out/vehicle_global_position',
                                 lambda msg: self._pos_callback(10, msg), self.qos_profile)
        self.create_subscription(VehicleGlobalPosition, '/px4_11/fmu/out/vehicle_global_position',
                                 lambda msg: self._pos_callback(11, msg), self.qos_profile)
        self.create_subscription(VehicleGlobalPosition, '/px4_12/fmu/out/vehicle_global_position',
                                 lambda msg: self._pos_callback(12, msg), self.qos_profile)

    def send_command(self, instance, command, **kwargs):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.target_system = instance + 1
        msg.target_component = 1
        msg.source_system = 255 - (instance - 10)
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
        # 1. POSITION 모드 전환
        self.send_command(instance, 176, param1=1.0, param2=4.0)
        await asyncio.sleep(1.0)

        # 2. 시동(Arm)
        for _ in range(10):
            self.send_command(instance, 400, param1=1.0, param2=21196.0)
            await asyncio.sleep(0.1)

        await asyncio.sleep(3.0)

    async def arm_rover(self, instance):
        self.get_logger().info(f'[Rover {instance}] 시동 시퀀스 시작...')

        # 1. MANUAL 모드로 전환 (로버 시동의 필수 관문)
        # param1=1.0 (Custom Mode), param2=1.0 (PX4_ROVER_MODE_MANUAL)
        self.send_command(instance, 176, param1=1.0, param2=1.0)
        await asyncio.sleep(1.0)

        # 2. 강제 시동(Force Arming) 명령
        # 로버는 지면에 닿아 있어 진동이나 센서 보정 문제로 일반 Arm이 거절될 수 있으므로
        # param2=21196.0 (Force Arm)을 사용하여 안전 체크를 강제로 통과시킵니다.
        for _ in range(15):
            self.send_command(instance, 400, param1=1.0, param2=21196.0)
            await asyncio.sleep(0.1)

        # 시동 후 하드웨어가 응답할 시간 대기
        await asyncio.sleep(2.0)

        # 3. 이동을 위해 POSITION 모드로 전환
        # 시동이 걸린 상태에서 모드를 바꿔야 미션 수행(move_rover)이 가능합니다.
        self.get_logger().info(f'[Rover {instance}] 모드 전환: POSITION')
        self.send_command(instance, 176, param1=1.0, param2=4.0)
        await asyncio.sleep(1.0)

        # 4. 시동 후 Position 모드로 전환 (이동을 위해)
        self.send_command(instance, 176, param1=1.0, param2=4.0)

    async def takeoff(self, instance, altitude):
        self.get_logger().info(f'[ID {instance}] 이륙 중...')
        for _ in range(10):
            self.send_command(instance, 22, param7=altitude, param5=float('nan'), param6=float('nan'))
            await asyncio.sleep(0.1)


    async def land_sequence(self, instance):
        self.get_logger().info(f'[ID {instance}] 착륙 시작...')

        for _ in range(10):
            self.send_command(instance, 21,
                              param5=float('nan'),
                              param6=float('nan'),
                              param7=0.0)  # 지면 고도
            await asyncio.sleep(0.1)

        self.get_logger().info(f'[ID {instance}] 착륙 명령 완료')

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
        self.get_logger().info(f'[Drone {instance}] 이동 명령: Alt {alt}m')

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
        self.get_logger().info(f'[Rover {instance}] 이동: Lat {lat}, Lon {lon}')

async def run_mission(controller):
    all = [10, 11, 12]
    drones = [10, 11]
    rovers = [12]

    # # 1. 모든 드론 동시 이륙
    controller.get_logger().info('모든 드론 이륙 시작...')
    await asyncio.gather(*[controller.arm(i) for i in all])

    controller.get_logger().info('모든 드론 이륙 시작...')
    await asyncio.gather(*[controller.takeoff(i, -30.0) for i in drones])

    # 1.1. rover


    # 2. 10초간 공중 정지(Hovering) 대기
    controller.get_logger().info('10초간 고도 유지 중...')
    await asyncio.sleep(10.0)
    # move sequence (non-blocking)
    controller.move_drone(instance=10, lat=47.397842, lon=8.545494, alt=15.0)
    controller.move_drone(instance=11, lat=47.397842, lon=8.545594, alt=15.0)
    controller.move_rover(instance=12, lat=47.397842, lon=8.545694)
    controller.sleep(5)

    controller.stop_rover(instance=12)
    # 3. 모든 드론 동시 착륙
    controller.get_logger().info('모든 드론 착륙 시작...')
    await asyncio.gather(*[controller.land_sequence(i) for i in drones])

    # 4. 완전히 내려앉을 때까지 대기
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