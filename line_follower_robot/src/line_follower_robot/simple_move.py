#!/usr/bin/env python3
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class RobotMover(Node):
    def __init__(self):
        super().__init__('simple_move')

        # Parameters
        self.fixed_forward_vel = 0.5
        self.kp = 6.0
        self.ki = 0.2
        self.kd = 0.6
        self.delta = 0.1

        # Topic that controls differential drive
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.error_pub = self.create_publisher(Float32, '/error_value', 10)

        # Timer to publish to /cmd_vel
        self.timer = self.create_timer(self.delta, self.update)
        
        # Subsribe to left and right sensor values
        self.sub_left = self.create_subscription(Float32, '/left_camera/sensor_value', self.left_callback, 10)
        self.sub_middle = self.create_subscription(Float32, '/middle_camera/sensor_value', self.middle_callback, 10)
        self.sub_right = self.create_subscription(Float32, '/right_camera/sensor_value', self.right_callback, 10)

        # Sensor values
        self.left_value = None
        self.middle_value = None
        self.right_value = None

        # Terms that change across PID calculations
        self.cur_error = 0.0
        self.prev_error = 0.0
        self.accumulated_error = 0.0
        self.angular_vel = 0.0

    def left_callback(self, value):
        self.left_value = value.data

    def middle_callback(self, value):
        self.middle_value = value.data

    def right_callback(self, value):
        self.right_value = value.data

    def update(self):
        """ Processes sensor values and publishes computed linear/angular velocities. """

        if self.left_value is None or self.middle_value is None or self.right_value is None:
            return

        # The error is rightwards negative, leftwards positive
        self.prev_error = self.cur_error
        self.cur_error = self.left_value - self.right_value

        # Make the error more reliable when the robot is extremely off-line
        if self.cur_error < 0.01:
            self.cur_error += (1.0 if self.cur_error > 0.0 else -1.0) * 0.5 * (0.71 - self.middle_value)

        # (P)roportional
        error = self.cur_error

        # (I)ntegral
        self.accumulated_error += self.cur_error
        ierror = self.accumulated_error * self.delta

        # (D)erivative
        derror = (self.cur_error - self.prev_error) / self.delta

        # Calculate angular velocity using the PID formula
        self.angular_vel = self.kp * error + self.ki * ierror + self.kd * derror

        self.get_logger().info(f"kp*error={self.kp * error} ki*ierror={self.ki * ierror} kd*derror={self.kd * derror} angular_vel={self.angular_vel}")

        msg = Float32()
        msg.data = self.cur_error
        self.error_pub.publish(msg)

        msg = Twist()
        msg.linear.x = self.fixed_forward_vel
        msg.angular.z = self.angular_vel
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
