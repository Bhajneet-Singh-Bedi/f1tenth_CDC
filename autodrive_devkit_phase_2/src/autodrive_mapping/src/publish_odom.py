#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Point
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import math


class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('odometry_publisher')

        # Publishers and subscribers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)  # Publishes to /odom
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to position data from /autodrive/f1tenth_1/ips
        self.create_subscription(Point, '/autodrive/f1tenth_1/ips', self.ips_callback, 10)

        # Subscribe to the left and right encoder data
        self.create_subscription(JointState, '/autodrive/f1tenth_1/left_encoder', self.left_encoder_callback, 10)
        self.create_subscription(JointState, '/autodrive/f1tenth_1/right_encoder', self.right_encoder_callback, 10)

        # Initialize variables for position, orientation, and encoder data
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Heading angle in radians

        self.left_wheel_position = 0.0
        self.right_wheel_position = 0.0
        self.wheel_base = 0.3240  # Distance between the front wheels

        # Timer to periodically publish odometry (every 50 ms)
        self.timer = self.create_timer(0.05, self.publish_odometry)

        # Variables to store the previous position for velocity calculations
        self.prev_x = None
        self.prev_y = None
        self.prev_time = self.get_clock().now()

    def ips_callback(self, msg: Point):
        """Callback to update position from the /autodrive/f1tenth_1/ips topic"""
        self.x = msg.x
        self.y = msg.y

    def left_encoder_callback(self, msg: JointState):
        """Callback for the left wheel encoder"""
        self.left_wheel_position = msg.position[0]  # Assuming the first element in 'position' is the wheel position

    def right_encoder_callback(self, msg: JointState):
        """Callback for the right wheel encoder"""
        self.right_wheel_position = msg.position[0]  # Assuming the first element in 'position' is the wheel position

    def publish_odometry(self):
        """Publishes odometry and TF transform"""
        current_time = self.get_clock().now()
        delta_time = (current_time - self.prev_time).nanoseconds / 1e9  # Convert nanoseconds to seconds

        if self.prev_x is not None and self.prev_y is not None:
            # Calculate the change in position
            delta_x = self.x - self.prev_x
            delta_y = self.y - self.prev_y

            # Calculate the heading (theta) based on the change in position
            self.theta = math.atan2(delta_y, delta_x) if delta_x != 0 or delta_y != 0 else self.theta

            # Calculate velocity based on the distance traveled
            linear_velocity = math.sqrt(delta_x**2 + delta_y**2) / delta_time if delta_time > 0 else 0.0

            # Calculate the angular velocity from the difference in wheel encoders
            left_wheel_velocity = self.left_wheel_position / delta_time
            right_wheel_velocity = self.right_wheel_position / delta_time
            angular_velocity = (right_wheel_velocity - left_wheel_velocity) / self.wheel_base

        else:
            linear_velocity = 0.0
            angular_velocity = 0.0

        # Store the current position and time for the next loop
        self.prev_x = self.x
        self.prev_y = self.y
        self.prev_time = current_time

        # Publish the odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'world'  # Use world as the global frame
        odom_msg.child_frame_id = 'f1tenth_1'  # Robot base link

        # Set the position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0  # Assuming a 2D plane
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        # Set the velocity
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.angular.z = angular_velocity

        # Publish odometry
        self.odom_pub.publish(odom_msg)

        # Publish the transform (world -> f1tenth_1)
        odom_tf = TransformStamped()
        odom_tf.header.stamp = current_time.to_msg()
        odom_tf.header.frame_id = 'world'  # world is the parent frame
        odom_tf.child_frame_id = 'f1tenth_1'  # f1tenth_1 is the child frame

        odom_tf.transform.translation.x = self.x
        odom_tf.transform.translation.y = self.y
        odom_tf.transform.translation.z = 0.0  # Assuming the robot is on a 2D plane
        odom_tf.transform.rotation.z = odom_msg.pose.pose.orientation.z
        odom_tf.transform.rotation.w = odom_msg.pose.pose.orientation.w

        # self.tf_broadcaster.sendTransform(odom_tf)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
