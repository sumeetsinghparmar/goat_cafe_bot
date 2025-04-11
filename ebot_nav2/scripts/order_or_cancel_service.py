#!/usr/bin/env python3

'''
    This service handles the orders and cancellations done by customer ---

    ros2 service call /order_or_cancel_service ebot_nav2/srv/OrderOrCancel "{order: false, cancel: true, table_id: 1}"

'''
# Import necessary ROS2 packages and message types
import threading
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ebot_nav2.msg import Order
from ebot_nav2.srv import OrderOrCancel
import math, os, yaml, time

os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = "[{name}]: {message}"

# Define a class for your ROS2 node
class OrderOrCancelService(Node):

    def __init__(self):
        # Initialize the ROS2 node with a unique name
        super().__init__('order_cancel_srv')

        # Create a callback group for managing callbacks
        o_or_c_cg = ReentrantCallbackGroup()
        order_cg = ReentrantCallbackGroup()
        timer_cg = ReentrantCallbackGroup()

        self.order_list = []

        # Create a ROS2 service for controlling docking behavior, can add another custom service message
        self.o_or_c_srv = self.create_service(OrderOrCancel, 'order_or_cancel_service', self.o_or_c_callback, callback_group=o_or_c_cg)


        self.order_pub = self.create_publisher(Order, '/order', 10, callback_group=order_cg)
        
        self.create_timer(0.7, self.order_or_cancel_manager, callback_group=timer_cg)

        self.__reset_param()

    
    def __reset_param(self):
        self.order = None
        self.cancel = None
        self.table_id = None


    def order_or_cancel_manager(self):
        order_msg = Order()
        order_msg.table_orders = self.order_list
        self.order_pub.publish(order_msg)





    # Callback function for the o_or_c_callback service
    def o_or_c_callback(self, request, response):
        # Set the delivery parameters from the service request.
        self.order = request.order
        self.cancel = request.cancel
        self.table_id = int(request.table_id)

        if self.order and not self.cancel:
            print(f'New Order Arrived ! Table - {self.table_id}')
            if self.table_id not in self.order_list:
                self.order_list.append(self.table_id)
                response.success = True
                response.message = "Order Placed!!"
            else:
                response.success = False
                response.message = "Order Can't Be Placed (Reason - Already Placed)!!"
        elif self.cancel and not self.order:
            print(f'Cancellation Recieved ! Table - {self.table_id}')
            if self.table_id in self.order_list:
                self.order_list.remove(self.table_id)
                response.success = True
                response.message = "Order Cancelled!!"
            else:
                response.success = False
                response.message = "Order Not Available!!"
        else:
            print("Both order and cancel can't have same value")
            response.success = False
            response.message = "Both order and cancel can't have same value"


        self.__reset_param()
        return response





# Main function to initialize the ROS2 node and spin the executor
def main(args=None):
    rclpy.init(args=args)

    node = OrderOrCancelService()
    node.get_logger().info("\n\n────────────────────────────────────────────\n   🚀 ORDER OR CANCEL SERVICE STARTED!! 🚀\n────────────────────────────────────────────\n")


    executor = MultiThreadedExecutor(3)
    executor.add_node(node)

    executor.spin()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()