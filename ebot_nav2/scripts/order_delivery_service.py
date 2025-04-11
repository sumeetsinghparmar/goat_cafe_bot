#!/usr/bin/env python3

"""
    ────────────────────────────────────────────
            🚚 Delivery Service Node 🚚✨
    ────────────────────────────────────────────

    Usage:
    ros2 service call /delivery_service ebot_nav2/srv/Delivery "{start_delivery: true, deliver_to: [1, 3]}"

    This node is responsible for delivering orders to tables. It navigates to each
    table, waits for confirmation (via a confirmation service), updates the state,
    and finally returns home.
    ────────────────────────────────────────────
"""

# Import necessary ROS2 packages and message types
import threading
import rclpy
from rclpy.node import Node
from tf_transformations import quaternion_from_euler
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from ebot_nav2.srv import Delivery, Confirmation, OrderOrCancel, StateUpdater  # Import custom service message
import math, os, yaml, time
from ebot_nav2.msg import Order, State

os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = "[{name}]: {message}"

# Define a class for your ROS2 node
class OrderDeliveryService(Node):

    def __init__(self):
        # Initialize the ROS2 node with a unique name
        super().__init__('delivery_service')

        self.home_pose = []
        self.kitchen_pose = []
        self.table1_pose = []
        self.table2_pose = []
        self.table3_pose = []

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
                if (key == 'home_pose'):
                    self.home_pose = position[key]
                elif (key == 'kitchen_pose'):
                    self.kitchen_pose = position[key]
                elif (key == 'table_1 pose'):
                    self.table1_pose = position[key]
                elif (key == 'table_2 pose'):
                    self.table2_pose = position[key]
                elif (key == 'table_3 pose'):
                    self.table3_pose = position[key]
        
        self.get_logger().info(f"🏠 Home Pose: {self.home_pose}")
        self.get_logger().info(f"🍽️ Kitchen Pose: {self.kitchen_pose}")
        self.get_logger().info(f"🪑 Table 1: {self.table1_pose} | Table 2: {self.table2_pose} | Table 3: {self.table3_pose}")
        # -----------------------------------------------

        # Create a callback group for managing callbacks
        order_delivery_cg = ReentrantCallbackGroup()
        confirmation_cg = ReentrantCallbackGroup()
        order_or_cancel_cg = ReentrantCallbackGroup()
        state_updater_cg = ReentrantCallbackGroup()

        self.confirmation_cli = self.create_client(Confirmation, '/confirmation_service', callback_group=confirmation_cg)
        self.order_or_cancel_cli = self.create_client(OrderOrCancel, '/order_or_cancel_service', callback_group=order_or_cancel_cg)
        self.state_updater_cli = self.create_client(StateUpdater, '/state_updater_service', callback_group=state_updater_cg)

        # Create a ROS2 service for controlling docking behavior, can add another custom service message
        self.order_delivery_srv = self.create_service(Delivery, 'delivery_service', self.order_delivery_callback, callback_group=order_delivery_cg)

        # Initialize all flags and parameters here
        self.start_delivery = False
        self.deliver_to = None
        self.is_delivering = False
        self.delivery_done = False
        self.remaining_deliveries = None

        self.navigator = BasicNavigator()
        self.current_nav_to = None

        self.order_sub = self.create_subscription(Order, '/order', self.order_cb, 1, callback_group=ReentrantCallbackGroup())
        self.current_order_list = []

        # Creating a rate object to hold (sleep) the node at points like while waiting for services outputs
        self.rate = self.create_rate(10, self.get_clock())


    def order_cb(self, msg):
        order = list(msg.table_orders)
        if (order != self.current_order_list):
            self.current_order_list = order
            self.get_logger().info(f"📋 [UPDATED ORDERS LIST] {self.current_order_list}")
        else:
            return

    def remove_the_delivered_order(self, order=False, cancel=False, table_id=0):
        self.get_logger().info("🗑️ Removing delivered order data...")
        while not self.order_or_cancel_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("⏳ Waiting for /order_or_cancel_service...")

        request = OrderOrCancel.Request()
        request.order = order
        request.cancel = cancel
        request.table_id = table_id

        future = self.order_or_cancel_cli.call_async(request)
        while not future.done():
            self.rate.sleep()
        try:
            result = future.result()
            response = {"success": result.success, "msg": result.message}
            return response
        except Exception as e:
            self.get_logger().error("❌ Service call to /order_or_cancel_service failed.")


    

    def request_delivery_confirmation(self, confirmation_type, table_id=0, confirm_table_id=0, reset=False, start_delivery=False):
        self.get_logger().info("⏳ Waiting for user confirmation...")
        while not self.confirmation_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("⏳ Waiting for /confirmation_service...")

        request = Confirmation.Request()
        request.confirmation_type = confirmation_type
        request.table_id = table_id
        request.confirm_table_id = confirm_table_id
        request.reset = reset
        request.start_delivery = start_delivery

        future = self.confirmation_cli.call_async(request)
        while not future.done():
            self.rate.sleep()
        try:
            result = future.result()
            response = {"success": result.success, "msg": result.message, "order_ready_list": result.order_ready_for}
            return response
        except Exception as e:
            self.get_logger().error("❌ Service call to /confirmation_service failed.")


    def update_bot_state(self, state="NONE"):
        # self.get_logger().info(f"🔄 Updating bot state to: {state}")
        while not self.state_updater_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("⏳ Waiting for /state_updater_service...")
        request = StateUpdater.Request()
        request.state = state
        future = self.state_updater_cli.call_async(request)
        while not future.done():
            self.rate.sleep()
        try:
            result = future.result()
            response = {"success": result.success, "msg": result.message}
            return response
        except Exception as e:
            self.get_logger().error('Service call to /state_update_service failed.')



    def goal_navigator(self, goal_pose):
        if(self.current_nav_to == 'Home' or self.current_nav_to == 'Kitchen'):
            self.get_logger().info(f"🚀 Navigating to {self.current_nav_to}...")
        else:
            self.get_logger().info(f"🚀 Navigating to Table {self.current_nav_to}...")
        # Build goal pose
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
            self.rate.sleep()
            # Periodically check if the order is still valid.
            if int(self.current_nav_to) not in self.current_order_list:
                self.get_logger().info(f"❌ Order - {self.current_nav_to} cancelled; aborting navigation!")
                self.update_bot_state('HALT')
                self.navigator.cancelTask()

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("--------- 🎯 Goal Reached ---------")
            self.update_bot_state('wait_table'+self.current_nav_to)
            self.remove_the_delivered_order(order=False, cancel=True, table_id=int(self.current_nav_to))
            response = self.request_delivery_confirmation(confirmation_type='bot_from_table', table_id=int(self.current_nav_to))
            if(response["success"] == True):
                self.get_logger().info(f"✅ {response['msg']}")
                self.remaining_deliveries -= 1
            else:
                self.get_logger().warn(f"⚠️ {response['msg']}")
        elif result == TaskResult.CANCELED:
            self.get_logger().info("--------- ⛔ Goal Cancelled ---------")
        elif result == TaskResult.FAILED:
            self.get_logger().error('--------- ❌ Goal Failed ---------')
        else:
            self.get_logger().error('❌ Invalid goal status!')

    
    


    
    def start_delivery_func(self):
        """
        Executes the delivery sequence:
         1. For each table ID in deliver_to list, navigate to that table,
            wait upto confirmation or timeout (food delivery wait), then continue.
         2. Finally, navigate back home.
        """
        if self.start_delivery:
                # Navigate to each table in the specified order.
                for table_id in self.current_order_list:
                    self.get_logger().info(f"🚚 Navigating to Table {table_id}...current order_list = {self.current_order_list}")
                    if int(table_id) == 1:
                        pose = self.table1_pose
                        self.current_nav_to = "1"
                    elif int(table_id) == 2:
                        pose = self.table2_pose
                        self.current_nav_to = "2"
                    elif int(table_id) == 3:
                        pose = self.table3_pose
                        self.current_nav_to = "3"
                    else:
                        self.get_logger().warn(f"⚠️ Unknown table id: {table_id}. Skipping.")
                        continue
                    
                    self.update_bot_state('nav_2_table'+self.current_nav_to)
                    self.goal_navigator(pose)
                    self.wait_till_navigation_done()
                    # self.get_logger().info(f'deliver_to_prev : {self.deliver_to}')
                    # self.deliver_to.pop(0)
                    # self.get_logger().info(f'deliver_to_after : {self.deliver_to}')


                if len(self.current_order_list) > 0:
                    self.start_delivery_func()
                else:
                    if self.remaining_deliveries > 0:
                        self.get_logger().warn("⚠️ Some orders couldn't be delivered! Returning to Kitchen...")
                        self.current_nav_to = "Kitchen"
                        self.update_bot_state('nav_2_kitchen_2_return_food')
                        self.goal_navigator(self.kitchen_pose)
                        while not self.navigator.isTaskComplete():
                            pass
                        result = self.navigator.getResult()
                        if result == TaskResult.SUCCEEDED:
                            self.get_logger().info("--------- 🎯 Kitchen Pose Reached ---------")
                            self.update_bot_state('waiting_at_kitchen_2_return_food')
                            time.sleep(5.0)
                        else:
                            self.get_logger().error("❌ Error navigating to Kitchen!")
                    # After completing table visits, return home.
                    self.current_nav_to = "Home"
                    self.update_bot_state('nav_2_home')
                    self.goal_navigator(self.home_pose)
                    while not self.navigator.isTaskComplete():
                        pass
                    result = self.navigator.getResult()
                    if result == TaskResult.SUCCEEDED:
                        self.get_logger().info("--------- 🎯 Home Pose Reached ---------")
                        self.update_bot_state('IDLE')
                    else:
                        self.get_logger().error("❌ Error navigating Home!")
                    self.get_logger().info("Delivery Completed.")
                    self.delivery_done = True
                    self.get_logger().info('self.delivery_done = '+str(self.delivery_done))
            





    # Callback function for the order_delivery service
    def order_delivery_callback(self, request, response):
        # Set the delivery parameters from the service request.
        self.start_delivery = request.start_delivery
        self.deliver_to = list(request.deliver_to)

        # Log a message indicating that docking has started
        self.get_logger().info("🚚 Delivery process started!")
        self.get_logger().info(f"📦 Delivery order: {self.deliver_to}")

        if(len(self.deliver_to) > 0):
            self.remaining_deliveries = len(self.deliver_to)
        else:
            self.remaining_deliveries = 0

        self.start_delivery_func()

        # Create a rate object to control the loop frequency
        # rate = self.create_rate(2, self.get_clock())

        # Wait until the robot is aligned for docking
        # while not self.delivery_done:
        #     self.get_logger().info("Waiting for Disalignment...")
        #     rate.sleep()

        self.delivery_done = False
        self.is_delivering = False
        self.start_delivery = False
        self.deliver_to = None
        self.current_nav_to = None
        self.remaining_deliveries = None

        self.get_logger().info("✅ Delivery Successful!")
        # Set the service response indicating success
        response.success = True
        response.message = "Delivery initiated"
        return response





# Main function to initialize the ROS2 node and spin the executor
def main(args=None):
    rclpy.init(args=args)

    node = OrderDeliveryService()
    node.get_logger().info("\n\n────────────────────────────────────────────\n   🚀 DELIVERY SERVICE STARTED!! 🚀\n────────────────────────────────────────────\n")

    executor = MultiThreadedExecutor(3)
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