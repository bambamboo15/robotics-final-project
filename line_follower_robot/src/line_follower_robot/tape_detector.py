#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

class TapeDetector(Node):
    def __init__(self):
        super().__init__('tape_detector')

        # Bridge between ROS 2 and OpenCV images
        self.bridge = CvBridge()

        # Publishers for sensor data
        self.left_pub = self.create_publisher(Float32, '/left_camera/sensor_value', 10)
        self.middle_pub = self.create_publisher(Float32, '/middle_camera/sensor_value', 10)
        self.right_pub = self.create_publisher(Float32, '/right_camera/sensor_value', 10)

        # Holds OpenCV images for each sensor
        self.left_image = None
        self.middle_image = None
        self.right_image = None

        # Subscribe to the sensor topics
        self.sub_left = self.create_subscription(Image, '/left_camera/image_raw', self.left_callback, 10)
        self.sub_middle = self.create_subscription(Image, '/middle_camera/image_raw', self.middle_callback, 10)
        self.sub_right = self.create_subscription(Image, '/right_camera/image_raw', self.right_callback, 10)

    def left_callback(self, msg):
        self.left_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_images()

    def middle_callback(self, msg):
        self.middle_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_images()

    def right_callback(self, msg):
        self.right_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_images()

    def tape_value(self, cv_image):
        """ Obtains a [0, 1] value determining how much darkness (loosely tape) is in the sensor view. """
        
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        roi = gray[-50:, :]
        _, binary = cv2.threshold(roi, 50, 255, cv2.THRESH_BINARY_INV)
        return cv2.countNonZero(binary) / binary.size

    def process_images(self):
        if self.left_image is None or self.middle_image is None or self.right_image is None:
            return

        left_msg = Float32()
        left_msg.data = self.tape_value(self.left_image)
        self.left_pub.publish(left_msg)

        middle_msg = Float32()
        middle_msg.data = self.tape_value(self.middle_image)
        self.middle_pub.publish(middle_msg)
        
        right_msg = Float32()
        right_msg.data = self.tape_value(self.right_image)
        self.right_pub.publish(right_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TapeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
