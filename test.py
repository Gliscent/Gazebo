import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand
import time
import subprocess

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        self.pubs = {
            10: self.create_publisher(VehicleCommand, '/px4_10/fmu/in/vehicle_command', 10),
            11: self.create_publisher(VehicleCommand, '/px4_11/fmu/in/vehicle_command', 10),
            12: self.create_publisher(VehicleCommand, '/px4_12/fmu/in/vehicle_command', 10),
        }

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

    # move sequence
    controller.move_drone(instance=10, lat=47.397842, lon=8.545494, alt=15.0)
    controller.move_drone(instance=11, lat=47.397842, lon=8.545594, alt=15.0)
    controller.move_rover(instance=12, lat=47.397842, lon=8.545694)
    controller.sleep(5)

    # drone land
    controller.land(instance=10)
    controller.land(instance=11)


    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
