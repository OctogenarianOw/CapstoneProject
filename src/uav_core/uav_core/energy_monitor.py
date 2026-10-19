"""
Energy monitor: the logical budget B_current of C1, measured in flight.

Role: integrate the rotor shaft power from the motor commands PX4 sends
to Gazebo, bridged as actuator_msgs/Actuators, and publish what is left
of the mission budget on /energy/state at 10 Hz.

Choice: a logical budget held here, capacity_j minus the integrated
shaft energy (S10, "budget logique"). The capacity is a parameter, to be
set so the budget bites: half to two thirds of what full coverage of the
world needs, or the baseline never fails and the experiment proves
nothing (S10, S13).

Rejected: PX4 battery_status, a ramp in armed time in SITL (17 Oct). And
integrating with the energy model instead of measuring: the model would
then check itself, and its error would never show up in I1.

The rotor topic arrives at about 250 Hz; integration is per message in
rotor_energy.EnergyIntegrator. last_sample_age_s tells the manager when
the signal has stopped, which must not pass for a full battery.
"""

from actuator_msgs.msg import Actuators
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Header
from uav_core.rotor_energy import EnergyIntegrator, shaft_power
from uav_interfaces.msg import EnergyState


class EnergyMonitor(Node):
    """Rotor commands in, remaining budget out."""

    def __init__(self):
        super().__init__('energy_monitor')
        self.declare_parameter('capacity_j', 200000.0)
        self.declare_parameter('motor_topic',
                               '/x500_slz_0/command/motor_speed')
        self.declare_parameter('publish_period_s', 0.1)
        self.capacity = float(self.get_parameter('capacity_j').value)
        self.integrator = EnergyIntegrator()
        self.power = 0.0
        self.create_subscription(
            Actuators, self.get_parameter('motor_topic').value,
            self.on_rotors, qos_profile_sensor_data)
        self.pub = self.create_publisher(EnergyState, '/energy/state', 10)
        self.create_timer(self.get_parameter('publish_period_s').value,
                          self.publish)
        self.get_logger().info(f'budget {self.capacity:.0f} J')

    def now_s(self):
        """Node clock in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def on_rotors(self, msg):
        """Integrate one rotor command message, stamped on reception."""
        self.power = shaft_power(msg.velocity)
        self.integrator.add(self.now_s(), self.power)

    def publish(self):
        """Publish the budget state."""
        i = self.integrator
        msg = EnergyState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.capacity_j = self.capacity
        msg.consumed_j = i.consumed_j
        msg.remaining_j = self.capacity - i.consumed_j
        msg.power_w = self.power
        msg.samples = i.samples
        msg.gaps = i.gaps
        msg.last_sample_age_s = (self.now_s() - i.last_t
                                 if i.last_t is not None else float('inf'))
        self.pub.publish(msg)


def main(args=None):
    """Run the node until interrupted."""
    rclpy.init(args=args)
    node = EnergyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
