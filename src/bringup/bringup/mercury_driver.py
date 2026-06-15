import sys
sys.path = [p for p in sys.path if '.local' not in p]

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
import math
import tf_transformations

class MercuryDriver(Node):
    def __init__(self):
        super().__init__('mercury_driver')

        # Tunable kinematics parameters (must match physical robot specs)
        self.declare_parameter('wheel_radius', 0.155)
        self.declare_parameter('wheel_separation', 0.668)
        self.declare_parameter('cmd_timeout', 0.5) # seconds
        self.declare_parameter('initial_x', 25.5)
        self.declare_parameter('initial_y', -18.5)
        self.declare_parameter('initial_yaw', -1.57)
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        # Pose integration state (for open-loop odometry)
        self.x = self.get_parameter('initial_x').value
        self.y = self.get_parameter('initial_y').value
        self.yaw = self.get_parameter('initial_yaw').value
        self.last_time = self.get_clock().now()

        # Command velocities
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0
        self.last_cmd_time = self.get_clock().now()

        # Command subscriber (receives linear/angular command velocities)
        # Note: Subscribing to /cmd_vel (output of Nav2 controller)
        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_cb,
            10
        )

        # Joint velocity command publisher (for ros2_control velocity controller)
        self.pub_joints = self.create_publisher(
            Float64MultiArray,
            '/wheel_velocity_controller/commands',
            10
        )

        # Odometry publisher (for EKF / slam_toolbox)
        self.pub_odom = self.create_publisher(
            Odometry,
            '/diff_drive_controller/odom',
            10
        )

        # Timer for periodic odometry integration and publication (50 Hz)
        self.timer = self.create_timer(0.02, self.timer_cb)

        self.get_logger().info("Mercury Custom Driver initialized.")

    def cmd_vel_cb(self, msg: Twist):
        self.target_linear_x = msg.linear.x
        self.target_angular_z = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

        # Compute joint velocities and publish them immediately for low latency
        self.publish_wheel_velocities(self.target_linear_x, self.target_angular_z)

    def publish_wheel_velocities(self, linear_x, angular_z):
        # 1. Compute differential drive wheel velocities (m/s)
        left_vel = linear_x - angular_z * (self.wheel_separation / 2.0)
        right_vel = linear_x + angular_z * (self.wheel_separation / 2.0)

        # 2. Convert to joint velocities (rad/s)
        left_joint_vel = left_vel / self.wheel_radius
        right_joint_vel = right_vel / self.wheel_radius

        # 3. Both sides require positive velocity to roll forward (axis 0 1 0).
        # Note: Due to URDF swap (left joint names are physically on the right side,
        # and right joint names are physically on the left side), we swap the joint commands:
        front_left = right_joint_vel
        rear_left = right_joint_vel
        front_right = left_joint_vel
        rear_right = left_joint_vel

        # 4. Publish joint velocities
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [front_left, rear_left, front_right, rear_right]
        self.pub_joints.publish(cmd_msg)

    def timer_cb(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Check for command timeout
        cmd_age = (current_time - self.last_cmd_time).nanoseconds / 1e9
        if cmd_age > self.cmd_timeout:
            # Command timeout: stop the robot
            if self.target_linear_x != 0.0 or self.target_angular_z != 0.0:
                self.get_logger().warn("Command timeout! Stopping the robot.")
                self.target_linear_x = 0.0
                self.target_angular_z = 0.0
                self.publish_wheel_velocities(0.0, 0.0)

        # Integrate open-loop odometry
        # Using linear_x and angular_z
        d_x = 0.3976
        v_robot_x = self.target_linear_x
        v_robot_y = -self.target_angular_z * d_x

        delta_x = (v_robot_x * math.cos(self.yaw) - v_robot_y * math.sin(self.yaw)) * dt
        delta_y = (v_robot_x * math.sin(self.yaw) + v_robot_y * math.cos(self.yaw)) * dt
        delta_yaw = self.target_angular_z * dt

        self.x += delta_x
        self.y += delta_y
        self.yaw += delta_yaw

        # Publish Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        q = tf_transformations.quaternion_from_euler(0.0, 0.0, self.yaw)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Set velocity
        odom.twist.twist.linear.x = self.target_linear_x
        odom.twist.twist.linear.y = -self.target_angular_z * d_x
        odom.twist.twist.angular.z = self.target_angular_z

        self.pub_odom.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = MercuryDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
