#!/usr/bin/env python3

"""
    ────────────────────────────────────────────
            🤖 State Updater Service 🤖
    ────────────────────────────────────────────

    This service updates and publishes the robot's state.

    Usage:
        ros2 service call /state_updater_service ebot_nav2/srv/StateUpdater "{state: 'IDLE'}"

    The node continuously publishes the current state on the '/bot_state' topic so that other nodes 
    can subscribe and react accordingly.

    ────────────────────────────────────────────
"""

# Import necessary ROS2 packages and message types
import threading
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ebot_nav2.srv import StateUpdater  # Custom service message for updating state
from ebot_nav2.msg import State        # Custom message for publishing state
import os

os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = "[{name}]: {message}"     # Set a custom format for ROS logging output

# Define a class for your ROS2 node
class StatusUpdaterService(Node):

    def __init__(self):
        # Initialize the ROS2 node with a unique name
        super().__init__('status_updater_srv')

        # Create a service to update the robot's current state (self.state)
        self.state_updater_srv = self.create_service(StateUpdater, 'state_updater_service', self.state_updater_callback, callback_group=ReentrantCallbackGroup())

        # Create a publisher to broadcast the robot's current state on '/bot_state'.
        self.state_publisher = self.create_publisher(State, '/bot_state', 1, callback_group=ReentrantCallbackGroup())

        # Create a timer that periodically calls 'publish_state_func' every 0.7 seconds.
        self.create_timer(0.7, self.publish_state_func, callback_group=ReentrantCallbackGroup())

        # Initial state is set to 'NONE' (no valid state yet).
        self.state = 'NONE'


    def publish_state_func(self):
        """
        Publishes the current state of the robot on the '/bot_state' topic.
        """
        state_msg = State()
        state_msg.state = self.state
        self.state_publisher.publish(state_msg)



    # Callback function for the order_delivery service
    def state_updater_callback(self, request, response):
        """
        Callback for the 'state_updater_service'. This service updates the internal state 
        based on the incoming request and returns a confirmation response.
        
        Request fields:
            state (string): The desired new state for the robot.
        
        Response fields:
            success (bool): Indicates whether the state update was successful.
            message (string): A human-readable message indicating the result.
        """
        # Set the delivery parameters from the service request.
        self.state = request.state

        self.get_logger().info(f"🔄 State Updated to: {self.state}")
        # Set the service response indicating success
        response.success = True
        response.message = "State Updated Successfully 👍"
        return response





# Main function to initialize the ROS2 node and spin the executor
def main(args=None):
    rclpy.init(args=args)

    node = StatusUpdaterService()
    node.get_logger().info("\n\n────────────────────────────────────────────\n   🚀 STATUS_UPDATER SERVICE STARTED!! 🚀\n────────────────────────────────────────────\n")

    executor = MultiThreadedExecutor(1)
    executor.add_node(node)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)

    try:
        executor_thread.start()
        executor_thread.join()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()