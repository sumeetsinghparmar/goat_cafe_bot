#!/usr/bin/env python3

# Import required packages and modules
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import rclpy.executors
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from tf_transformations import quaternion_from_euler
from ebot_nav2.msg import Order, State
from ebot_nav2.srv import Confirmation, Delivery, StateUpdater

import yaml, os, time, threading

os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = "[{name}]: {message}"

'''
    => This node continuously checks for any orders placed on the /order topic.
    => If there is any order and the robot is in an IDLE state, it triggers the
       routine to navigate to the kitchen, confirm with the kitchen, deliver the order,
       and then re-check for any new orders before concluding.
'''

class OrderManagerNode(Node):
    def __init__(self):
        super().__init__('Order_Manager')

        self.home_pose = []
        self.kitchen_pose = []
        
        # YAML Stuff ------------------------
        package_name = 'ebot_nav2'
        config_path = os.path.join(
            os.path.dirname(os.path.realpath(__file__)),
            '..', 'config', 'positions.yaml'
        )
        
        # Loading data from YAML file
        with open(config_path, 'r') as file:
            data = yaml.safe_load(file)
        
        self.positions = data.get('position', {})

        for position in self.positions:
            for key in position:
                if key == 'home_pose':
                    self.home_pose = position[key]
                elif key == 'kitchen_pose':
                    self.kitchen_pose = position[key]
        
        print("- Home Pose: ", self.home_pose)
        print("- Kitchen Pose: ", self.kitchen_pose)
        # -----------------------------------------------

        self.navigator = BasicNavigator()

        # Callback Groups to ensure non-blocking calls
        self.order_cg = ReentrantCallbackGroup()
        self.confirmation_cg = ReentrantCallbackGroup()
        self.order_delivery_cg = ReentrantCallbackGroup()
        self.state_updater_cg = ReentrantCallbackGroup()
        self.state_cg = ReentrantCallbackGroup()

        # Create service clients
        self.confirmation_cli = self.create_client(Confirmation, '/confirmation_service', callback_group=self.confirmation_cg)
        self.order_delivery_cli = self.create_client(Delivery, '/delivery_service', callback_group=self.order_delivery_cg)
        self.state_updater_cli = self.create_client(StateUpdater, '/state_updater_service', callback_group=self.state_updater_cg)

        # Create subscriptions to order and state topics
        self.order_sub = self.create_subscription(Order, '/order', self.order_cb, 1, callback_group=self.order_cg)
        self.state_sub = self.create_subscription(State, '/bot_state', self.state_cb, 1, callback_group=self.state_cg)
        
        # Internal state variables
        self.current_order_list = []
        self.bot_state = 'IDLE'
        self.kitchen_response = None

        # Set initial state of the bot
        self.update_bot_state('IDLE')

    def req_kitchen_for_confirmation(self, confirmation_type, table_id=0, confirm_table_id=0, reset=False, start_delivery=False, order_ready_list=[]):
        print('\n-> Waiting for Confirmation from Kitchen !!!')
        while not self.confirmation_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /confirmation_service...')

        request = Confirmation.Request()
        request.confirmation_type = confirmation_type
        request.table_id = table_id
        request.confirm_table_id = confirm_table_id
        request.reset = reset
        request.start_delivery = start_delivery
        request.order_ready_list = order_ready_list

        future = self.confirmation_cli.call_async(request)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                try:
                    result = future.result()
                    response = {"success": result.success, "msg": result.message, "order_ready_list": result.order_ready_for}
                    return response
                except Exception as e:
                    self.get_logger().error('Service call to /confirmation_service failed.')
                break

    def call_delivery_service(self, start_delivery=False, deliver_to=[]):
        print(f'\n-> Calling Delivery Service!! To - {list(deliver_to)}')
        while not self.order_delivery_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /delivery_service...')

        request = Delivery.Request()
        request.start_delivery = start_delivery
        request.deliver_to = deliver_to

        future = self.order_delivery_cli.call_async(request)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                try:
                    result = future.result()
                    response = {"success": result.success, "msg": result.message}
                    return response
                except Exception as e:
                    self.get_logger().error('Service call to /delivery_service failed.')
                break

    def update_bot_state(self, state="NONE"):
        print(f'\n-> Updating bot state to = {state}')
        while not self.state_updater_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /state_updater_service...')
        request = StateUpdater.Request()
        request.state = state
        future = self.state_updater_cli.call_async(request)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                try:
                    result = future.result()
                    response = {"success": result.success, "msg": result.message}
                    return response
                except Exception as e:
                    self.get_logger().error('Service call to /state_updater_service failed.')
                break

    def order_cb(self, msg):
        order = list(msg.table_orders)
        self.get_logger().info('---LOOKING FOR ORDERS---')
        if order != self.current_order_list:
            self.current_order_list = order
            self.get_logger().info(f"[UPDATED ORDERS LIST] {self.current_order_list}")
        
    def state_cb(self, state_msg):
        self.bot_state = state_msg.state

    def goal_navigator(self, goal_pose):
        # Build goal pose message from the given goal_pose array
        _, _, z, w = quaternion_from_euler(0.0, 0.0, goal_pose[2])
        goal_pose_msg = PoseStamped()
        goal_pose_msg.header.frame_id = 'map'
        goal_pose_msg.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose_msg.pose.position.x = goal_pose[0]
        goal_pose_msg.pose.position.y = goal_pose[1]
        goal_pose_msg.pose.orientation.z = z
        goal_pose_msg.pose.orientation.w = w

        self.navigator.goToPose(goal_pose_msg)

    def wait_till_navigation_done(self):
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print("--------- Goal Reached ---------")
        elif result == TaskResult.CANCELED:
            print("--------- Goal Cancelled ---------")
        elif result == TaskResult.FAILED:
            print("--------- Goal Failed ---------")
        else:
            print("Goal has an invalid return status!")

    def start_working(self):
        # This function represents a single cycle of order processing.
        self.get_logger().info("...Starting processing of current orders...")
        
        self.navigator.waitUntilNav2Active()

        # Navigate to Kitchen
        self.update_bot_state('nav_2_kitchen')
        self.get_logger().info("Navigating to Kitchen...")
        self.goal_navigator(self.kitchen_pose)
        self.wait_till_navigation_done()

        # Wait at Kitchen for confirmation
        self.update_bot_state('wait_kitchen')
        response = self.req_kitchen_for_confirmation(confirmation_type='bot_from_kitchen')
        self.get_logger().info(f"[Confirmed] {response['msg']} Order_Ready_List = {list(response['order_ready_list'])}")
        
        # Process confirmation response
        if response["success"]:
            # Delivery branch: call the delivery service
            res = self.call_delivery_service(start_delivery=True, deliver_to=response["order_ready_list"])
            self.get_logger().info("Delivery service executed.")
        else:
            self.get_logger().info("No confirmation received from kitchen. Skipping delivery...")

        # Navigate back to Home Pose after processing
        self.get_logger().info("Navigating to Home Pose...")
        self.update_bot_state('nav_2_home')
        self.goal_navigator(self.home_pose)
        self.wait_till_navigation_done()
        self.update_bot_state('IDLE')
        time.sleep(5.0)
        
        # Clear current orders if processed (or customize this logic as needed)
        self.current_order_list = []
        
    def process_orders(self):
        # Continuous loop that checks for pending orders and the robot's idle state
        while rclpy.ok():
            if self.current_order_list and self.bot_state == 'IDLE':
                self.get_logger().info("Pending orders detected and bot is IDLE. Starting order processing cycle...")
                self.start_working()
            rclpy.spin_once(self, timeout_sec=0.1)

def main():
    rclpy.init()
    node = OrderManagerNode()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)

    # Thread for the continuous order processing loop
    process_thread = threading.Thread(target=node.process_orders, daemon=True)
    process_thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
