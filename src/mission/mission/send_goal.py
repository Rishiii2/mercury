#!/usr/bin/env python3

import sys
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class SendGoal(Node):
    def __init__(self, x, y, yaw=0.0):
        super().__init__('send_goal_cli')
        self.pub_final = self.create_publisher(PoseStamped, '/final_goal', 10)
        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Short delay to ensure subscribers receive the message
        self.create_timer(0.5, lambda: self.publish_goal(x, y, yaw))

    def publish_goal(self, x, y, yaw):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.position.z = 0.0
        
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        goal.pose.orientation.w = cy
        goal.pose.orientation.z = sy
        
        self.pub_final.publish(goal)
        self.pub_goal.publish(goal)
        self.get_logger().info(f"Published goal (x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad) to /final_goal and /goal_pose")
        
        rclpy.shutdown()

def main():
    if len(sys.argv) < 3:
        print("Usage: ros2 run mission send_goal <x> <y> [yaw_in_rad]")
        print("Example: ros2 run mission send_goal 5.0 2.0")
        sys.exit(1)
        
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    yaw = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    
    rclpy.init()
    node = SendGoal(x, y, yaw)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass

if __name__ == '__main__':
    main()
