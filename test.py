import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand


class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')

        # Vehicles 등록
        self.pubs = {
            10: self.create_publisher(VehicleCommand, '/px4_10/fmu/in/vehicle_command', 10),
            11: self.create_publisher(VehicleCommand, '/px4_11/fmu/in/vehicle_command', 10),
            12: self.create_publisher(VehicleCommand, '/px4_12/fmu/in/vehicle_command', 10),
        }

    def move_drone(self, drone_id, lat, lon, alt):
        if drone_id not in self.pubs:
            self.get_logger().error(f'ID {drone_id} not exist!')
            return

        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = 192  # MAV_CMD_DO_REPOSITION
        msg.param5 = float(lat)
        msg.param6 = float(lon)
        msg.param7 = float(alt)

        msg.target_system = drone_id + 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        # publish
        self.pubs[drone_id].publish(msg)
        self.get_logger().info(f'[Drone {drone_id}] 이동 명령: Alt {alt}m')

    def move_rover(self, rover_id, lat, lon):
        if rover_id not in self.pubs:
            self.get_logger().error(f'ID {rover_id} not exist!')
            return

        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = 192  # MAV_CMD_DO_REPOSITION
        msg.param5 = float(lat)
        msg.param6 = float(lon)
        msg.param7 = 0.0  # 로버는 고도 0.0

        msg.target_system = rover_id + 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        # publish
        self.pubs[rover_id].publish(msg)
        self.get_logger().info(f'[Rover {rover_id}] 이동: Lat {lat}, Lon {lon}')


def main(args=None):
    rclpy.init(args=args)
    controller = DroneController()

    # 10번 드론 이동
    controller.move_drone(10, 47.397742, 8.545594, 15.0)

    # 11번 드론 이동
    controller.move_drone(11, 47.397842, 8.545694, 20.0)

    # 로버 12번 이동
    controller.move_rover(12, 47.7742, 8.545594)

    # 메시지 전송 후 종료
    rclpy.spin_once(controller, timeout_sec=1.0)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
