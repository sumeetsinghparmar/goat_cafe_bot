#!/usr/bin/python3
# -*- coding: utf-8 -*-



# import rclpy
# from geometry_msgs.msg import PoseStamped
# from nav2_simple_commander.robot_navigator import BasicNavigator

# rclpy.init()

# navigator = BasicNavigator()

# # Set initial pose
# initial_pose = PoseStamped()
# initial_pose.header.frame_id = 'map'
# initial_pose.header.stamp = navigator.get_clock().now().to_msg()
# initial_pose.pose.position.x = 3.25
# initial_pose.pose.position.y = 9.61
# initial_pose.pose.orientation.z = -0.7173561
# initial_pose.pose.orientation.w = 0.6967067
# navigator.setInitialPose(initial_pose)

# navigator.waitUntilNav2Active()

# print("✅ Initial pose set and Nav2 is active.")

# rclpy.shutdown()












import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import time

class AMCLPoseWatcher(Node):
    def __init__(self):
        super().__init__('amcl_pose_watcher')
        self.received_amcl_pose = False
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )

    def amcl_callback(self, msg):
        self.received_amcl_pose = True
        self.get_logger().info("✅ AMCL has received and published initial pose.")

def main():
    rclpy.init()

    navigator = BasicNavigator()
    amcl_checker = AMCLPoseWatcher()

    # Prepare initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.pose.position.x = 3.25
    initial_pose.pose.position.y = 9.61
    initial_pose.pose.orientation.z = -0.7173561
    initial_pose.pose.orientation.w = 0.6967067

    print("🧭 Waiting for Nav2 to become active...")
    navigator.waitUntilNav2Active()
    print("✅ Nav2 is active. Sending initial pose repeatedly until AMCL confirms...")

    # Keep trying until AMCL gives pose
    while not amcl_checker.received_amcl_pose:
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.setInitialPose(initial_pose)
        print("📤 Initial pose sent. Waiting for AMCL to accept...")
        rclpy.spin_once(amcl_checker, timeout_sec=1.0)
        time.sleep(1.0)

    print("🎯 Initial pose set and accepted by AMCL.")

    amcl_checker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
