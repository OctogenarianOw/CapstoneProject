import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

class WaypointNode(Node):
    def __init__(self):
        super().__init__('waypoint_node')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)

        self.count = 0
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Waypoint node started')

    def send_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)

    def timer_callback(self):
        # Publier offboard mode en continu
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

        # Publier waypoint en continu
        wp = TrajectorySetpoint()
        wp.position = [0.0, 0.0, -5.0]
        wp.yaw = 0.0
        wp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(wp)

        self.count += 1

        # A 2 secondes, passer en offboard
        if self.count == 20:
            self.get_logger().info('Switching to offboard mode...')
            self.send_command(176, 1.0, 6.0)

        # A 3 secondes, armer
        if self.count == 30:
            self.get_logger().info('Arming...')
            self.send_command(400, 1.0, 0.0)

def main():
    rclpy.init()
    node = WaypointNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
