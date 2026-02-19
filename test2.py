import time
import subprocess
import os
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, VehicleGlobalPosition
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class SwarmController(Node):
    def __init__(self):
        # 노드 이름을 swarm_controller로 변경
        super().__init__('swarm_controller')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # publisher for move
        self.pubs = {
            0: self.create_publisher(VehicleCommand, '/vehicle1/fmu/in/vehicle_command', 10),
            1: self.create_publisher(VehicleCommand, '/vehicle2/fmu/in/vehicle_command', 10),
            2: self.create_publisher(VehicleCommand, '/vehicle3/fmu/in/vehicle_command', 10),
        }

        self.drone_positions = {0: None, 1: None, 2: None}


        self.create_subscription(VehicleGlobalPosition, '/vehicle1/fmu/out/vehicle_global_position',
                                 lambda msg: self._pos_callback(0, msg), qos_profile)
        self.create_subscription(VehicleGlobalPosition, '/vehicle2/fmu/out/vehicle_global_position',
                                 lambda msg: self._pos_callback(1, msg), qos_profile)
        self.create_subscription(VehicleGlobalPosition, '/vehicle3/fmu/out/vehicle_global_position',
                                 lambda msg: self._pos_callback(2, msg), qos_profile)

        # Docker settings
        self.container_name = "realgazebo"
        self.inner_bin_path = "/home/user/realgazebo/RealGazebo-PX4/build/px4_sitl_default/bin/px4-commander"

    def _pos_callback(self, instance, msg):
        self.drone_positions[instance] = {'lat': msg.lat, 'lon': msg.lon, 'alt': msg.alt}

    def px4_command(self, instance, action):
        """modify with hard coding!!!!"""
        self.get_logger().info(f'[Vehicle {instance + 1}] Sending {action.upper()}...')

        cmd = f"docker exec -u user {self.container_name} {self.inner_bin_path} --instance {instance} {action}"

        try:
            subprocess.run(cmd, shell=True, check=True, capture_output=True, text=True)
            self.get_logger().info(f'[Vehicle {instance + 1}] {action.upper()} Success')
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'[Vehicle {instance + 1}] Failed: {e.stderr}')

    def move_drone(self, instance, lat, lon, alt):
        """
        드론을 특정 위도(lat), 경도(lon), 고도(alt)로 이동시킵니다.
        MAV_CMD_DO_REPOSITION (192)를 사용합니다.
        """
        if instance not in self.pubs:
            self.get_logger().error(f'ID {instance} not exist!')
            return

        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # MAV_CMD_DO_REPOSITION 명령 설정
        msg.command = 192

        # Param 1: Ground speed (-1은 기본값 유지)
        msg.param1 = -1.0
        # Param 2: Bitmask (보통 0)
        msg.param2 = 0.0
        # Param 4: Yaw heading (NaN은 현재 헤딩 유지)
        msg.param4 = float('nan')

        # 위치 설정 (위도, 경도, 고도)
        msg.param5 = float(lat)
        msg.param6 = float(lon)
        msg.param7 = float(alt)

        # 타겟 설정 (instance 0은 System ID 1에 대응하도록 설계됨)
        msg.target_system = instance + 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        # 퍼블리시
        self.pubs[instance].publish(msg)
        self.get_logger().info(f'[Drone {instance}] 이동 명령 전송: Lat {lat}, Lon {lon}, Alt {alt}m')

    def sleep(self, seconds):
        start_time = self.get_clock().now().nanoseconds
        while (self.get_clock().now().nanoseconds - start_time) < (seconds * 1e9):
            rclpy.spin_once(self, timeout_sec=0.05)

    def get_all_positions(self):
        print("\n" + "=" * 50)
        print(f"{'Vehicle':<12} | {'Latitude':<12} | {'Longitude':<12} | {'Altitude':<8}")
        print("-" * 50)

        for inst in range(3):  # 0, 1, 2 인스턴스 순차 확인
            pos = self.drone_positions.get(inst)
            if pos:
                # PX4 GlobalPosition 메시지는 보통 1e7이 곱해진 상태입니다.
                # 만약 이미 변환된 데이터라면 그대로 사용하고, 아니면 1e7로 나눠줍니다.
                lat = pos['lat'] if abs(pos['lat']) < 1000 else pos['lat'] / 1e7
                lon = pos['lon'] if abs(pos['lon']) < 1000 else pos['lon'] / 1e7
                alt = pos['alt']

                print(f"Drone {inst + 1:<7} | {lat:<12.7f} | {lon:<12.7f} | {alt:>7.2f}m")
            else:
                print(f"Drone {inst + 1:<7} | No Data Available")
        print("=" * 50)


def main(args=None):
    rclpy.init(args=args)
    controller = SwarmController()

    try:
        # --- ARM ---
        for i in range(3):
            controller.px4_command(instance=i, action="arm")
        controller.sleep(2.0)

        # --- TAKEOFF---
        for i in range(3):
            controller.px4_command(instance=i, action="takeoff")

        controller.sleep(10)

        # 10초간 모니터링
        start_t = time.time()
        while time.time() - start_t < 10.0:
            print("\n--- Current Status ---")
            controller.move_drone(0, 36.728, 127.44, 100)
            controller.get_all_positions()
            controller.sleep(1.0)

        # --- LAND ---
        for i in range(3):
            controller.px4_command(instance=i, action="land")
        controller.sleep(5.0)

    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()