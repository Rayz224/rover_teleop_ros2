import atexit
import sys

from abc import ABC

from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default


class Teleop(Node, ABC):
    def __init__(self):
        atexit.register(self._emergency_stop)
        Node.__init__(self, "rover_teleop")

        self.declare_parameter("linear_max", 1.0)
        self.declare_parameter("angular_max", 1.0)
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("cmd_topic", "cmd_vel")

        self.LINEAR_MAX = self.get_parameter("linear_max").value
        self.ANGULAR_MAX = self.get_parameter("angular_max").value
        self.CMD_TOPIC = self.get_parameter("cmd_topic").value

        self.publisher_ = self.create_publisher(
            Twist, self.CMD_TOPIC, qos_profile_system_default
        )
        rate = 1 / self.get_parameter("publish_rate").value
        self.create_timer(rate, self._publish)
        self.linear = 0.0
        self.angular = 0.0

    def write_twist(self, linear=None, angular=None):
        if linear is not None:
            if abs(linear) <= self.LINEAR_MAX:
                self.linear = linear
            else:
                self.get_logger().error(
                    f"Trying to set a linear speed {linear} outside of allowed range of [{-self.LINEAR_MAX}, {self.LINEAR_MAX}]"
                )
        if angular is not None:
            if abs(angular) <= self.ANGULAR_MAX:
                self.angular = angular
            else:
                self.get_logger().error(
                    f"Trying to set a angular speed {angular} outside of allowed range of [{-self.ANGULAR_MAX}, {self.ANGULAR_MAX}]"
                )
        self._update_screen()

    def _make_twist(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        return twist

    def _publish(self):
        twist = self._make_twist(self.linear, self.angular)
        self.publisher_.publish(twist)

    def _update_screen(self):
        sys.stdout.write(f"Linear: {self.linear:.2f}, Angular: {self.angular:.2f}\r")

    def _emergency_stop(self):
        self.publisher_.publish(self._make_twist(0.0, 0.0))
