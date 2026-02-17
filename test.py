import time
import math

import subprocess

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, VehicleGlobalPosition
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # publisher for move
        self.pubs = {
            10: self.create_publisher(VehicleCommand, '/px4_10/fmu/in/vehicle_command', 10),
            11: self.create_publisher(VehicleCommand, '/px4_11/fmu/in/vehicle_command', 10),
            12: self.create_publisher(VehicleCommand, '/px4_12/fmu/in/vehicle_command', 10),
        }

        self.drone_positions = {10: None, 11: None, 12: None}

        # subscriber for get position
        self.create_subscription(VehicleGlobalPosition, '/px4_10/fmu/out/vehicle_global_position',
                                 lambda msg: self._pos_callback(10, msg), qos_profile)
        self.create_subscription(VehicleGlobalPosition, '/px4_11/fmu/out/vehicle_global_position',
                                 lambda msg: self._pos_callback(11, msg), qos_profile)
        self.create_subscription(VehicleGlobalPosition, '/px4_12/fmu/out/vehicle_global_position',
                                 lambda msg: self._pos_callback(12, msg), qos_profile)

        # for arm, takeoff, land
        self.px4_bin_path = "~/PX4-Autopilot/build/px4_sitl_default/bin/px4-commander"

    def sleep(self, seconds):
        start_time = self.get_clock().now().nanoseconds
        while (self.get_clock().now().nanoseconds - start_time) < (seconds * 1e9):
            rclpy.spin_once(self, timeout_sec=0.05)

    def arm(self, instance):
        self.get_logger().info(f'[ID {instance}] ARM...')
        cmd = f"eval {self.px4_bin_path} --instance {instance} arm"
        subprocess.run(cmd, shell=True)

    def takeoff(self, instance):
        self.get_logger().info(f'[ID {instance}] TAKEOFF...')
        cmd = f"eval {self.px4_bin_path} --instance {instance} takeoff"
        subprocess.run(cmd, shell=True)

    def offboard(self, instance):
        self.get_logger().info(f'[ID {instance}] TAKEOFF...')
        cmd = f"eval {self.px4_bin_path} --instance {instance} mode offboard"
        subprocess.run(cmd, shell=True)

    def land(self, instance):
        self.get_logger().info(f'[ID {instance}] TAKEOFF...')
        cmd = f"eval {self.px4_bin_path} --instance {instance} land"
        subprocess.run(cmd, shell=True)

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

    def _pos_callback(self, instance, msg):
        self.drone_positions[instance] = {'lat': msg.lat, 'lon': msg.lon, 'alt': msg.alt}

    def get_position(self, instance):
        pos = self.drone_positions.get(instance)
        if pos:
            return pos['lat'], pos['lon'], pos['alt']
        else:
            self.get_logger().warn(f"No data from Drone {instance}")
            return None, None, None

    @staticmethod
    def calculate_distance(lat1, lon1, alt1, lat2, lon2, alt2):
        lat_dist = (lat1 - lat2) * 111139.0
        lon_dist = (lon1 - lon2) * 111139.0 * math.cos(math.radians(lat1))
        alt_dist = alt1 - alt2
        return math.sqrt(lat_dist ** 2 + lon_dist ** 2 + alt_dist ** 2)


def main(args=None):
    rclpy.init(args=args)
    controller = DroneController()

    # start-up
    controller.arm(instance=10)
    controller.arm(instance=11)
    controller.arm(instance=12)
    controller.sleep(2)

    # drone takeoff
    controller.takeoff(instance=10)
    controller.takeoff(instance=11)
    controller.sleep(10.123)

    while rclpy.ok():
        rclpy.spin_once(controller, timeout_sec=0.1)

        # move sequence (non-blocking)
        # print(controller.get_position(instance=10))
        # controller.move_drone(instance=10, lat=47.397842, lon=8.545494, alt=15.0)
        # controller.move_drone(instance=11, lat=47.397842, lon=8.545594, alt=15.0)
        # controller.move_rover(instance=12, lat=47.397842, lon=8.545694)
        # controller.sleep(5)

        # move sequence (blocking)
        target_lat = 36.724624
        target_lon = 127.440409
        target_alt = 15
        current_lat, current_lon, current_alt = controller.get_position(instance=10)
        if current_lat is None:
            continue
        dist1 = controller.calculate_distance(target_lat, target_lon, target_alt, current_lat, current_lon, current_alt)
        print(current_lat, current_lon, current_alt)
        if dist1 > 1:
            controller.move_drone(instance=10, lat=target_lat, lon=target_lon, alt=target_alt)
        else:
            break

        controller.move_drone(instance=11, lat=36.726194, lon=127.440377, alt=15.0)
        controller.move_rover(instance=12, lat=36.726182, lon=127.440733)

        time.sleep(0.1)

    # drone land
    controller.land(instance=10)
    controller.land(instance=11)
    controller.stop_rover(instance=12)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
