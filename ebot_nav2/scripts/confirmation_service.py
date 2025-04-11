#!/usr/bin/env python3

'''

    ros2 service call /confirmation_service ebot_nav2/srv/Confirmation "{confirmation_type: 'bot_from_table', table_id: 2, confirm_table_id: 0, reset: false}"
    ros2 service call /confirmation_service ebot_nav2/srv/Confirmation "{confirmation_type: 'bot_from_kitchen', table_id: 3, confirm_table_id: 0, reset: false}"

    ros2 service call /confirmation_service ebot_nav2/srv/Confirmation "{confirmation_type: 'kitchen_to_bot', table_id: 3, confirm_table_id: 0, reset: false, start_delivery: true, order_ready_list: [1, 2]}"
    ros2 service call /confirmation_service ebot_nav2/srv/Confirmation "{confirmation_type: 'table_to_bot', table_id: 1, confirm_table_id: 1, reset: false}"


'''

# Import necessary ROS2 packages and message types
import threading
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ebot_nav2.srv import Confirmation  # Import custom service message
import math, os, yaml, time

os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = "[{name}]: {message}"

# Define a class for your ROS2 node
class ConfirmationService(Node):

    def __init__(self):
        # Initialize the ROS2 node with a unique name
        super().__init__('confirmation_srv')

        # Create a callback group for managing callbacks
        confirmation_cg = ReentrantCallbackGroup()

        # Create a ROS2 service for controlling docking behavior, can add another custom service message
        self.confirmation_srv = self.create_service(Confirmation, 'confirmation_service', self.confirmation_callback, callback_group=confirmation_cg)

        self._reset_param()

        self.confirmation_rate = self.create_rate(1, clock=self.get_clock())
    
    def _reset_param(self):
        # Initialize all flags and parameters here
        self.confirmation_type = None
        self.table_id_req_confirmation = None
        self.confirm_table_id = None
        self.confirmed_table_ids = []
        self.reset = False
        self.confirmation_done = False
        self.is_delivered = False

        self.timeout_sec = 10

        self.start_delivery = False
        self.order_ready_list = None

    def ask_for_confirmation(self, table_id, ask_from='table'):
        try:
            if ask_from == 'table':
                print("Waiting for Confirmation (Table/Bot) ...")
                start_time = time.time()
                while not self.confirmation_done:
                    if time.time() - start_time >= self.timeout_sec:
                        print('TIMEOUT!!!')
                        self.is_delivered = False
                        self.confirmation_done = True
                        break
                    if int(table_id) == int(self.confirm_table_id):
                        self.is_delivered = True
                        self.confirmation_done = True
                        break
                    time.sleep(1)
            else:
                print("Waiting for Confirmation (Kitchen/Bot) ...")
                start_time = time.time()
                while not self.confirmation_done:
                    if time.time() - start_time >= self.timeout_sec:
                        print('TIMEOUT!!!')
                        self.start_delivery = False
                        self.confirmation_done = True
                        break
                    if self.start_delivery:
                        self.confirmation_done = True
                        break
                    time.sleep(1)
        except Exception as e:
            self.get_logger().error(f"Exception in ask_for_confirmation thread: {e}")
            self.confirmation_done = True

            





    # Callback function for the order_delivery service
    def confirmation_callback(self, request, response):
        # Set the delivery parameters from the service request.
        try:
            self.confirmation_type = request.confirmation_type
            self.table_id = request.table_id
            self.confirm_table_id = request.confirm_table_id
            self.reset = request.reset

            response.order_ready_for = []

            if self.reset == True:

                print("\n--- FIRING RESET ---")
                self._reset_param()
                print(f"-> RESET DONE!")
                # Set the service response indicating success
                response.success = True
                response.message = "RESET DONE!!"
                
            elif self.confirmation_type == 'kitchen_to_bot':

                self.start_delivery = request.start_delivery
                self.order_ready_list = list(request.order_ready_list)
                print("\nOrder Ready List : ", self.order_ready_list)

                response.success = True
                response.message = "Thanks For the Confirmation!! Starting to Deliver..."
                response.order_ready_for = [int(x) for x in self.order_ready_list]

            elif self.confirmation_type == 'bot_from_kitchen':

                print('\nWaiting For Confirmation From Kitchen To Deliver !!')
                self.confirmation_thread = threading.Thread(target=self.ask_for_confirmation, args=(self.table_id_req_confirmation,'kitchen',), daemon=True)
                self.confirmation_thread.start()

                # Wait until confirmation is complete.
                while not self.confirmation_done:
                    self.confirmation_rate.sleep()

                if self.start_delivery:
                    print(f"\n-> Got Permission To Start Delivery!! => {self.order_ready_list}!!")
                    # Set the service response indicating success
                    response.success = True
                    response.message = "Delivery Started!"
                    response.order_ready_for = [int(x) for x in self.order_ready_list]
                else:
                    print(f"\n-> No Response From Kitchen, Moving back to Home Pose !!")
                    # Set the service response indicating success
                    response.success = False
                    response.message = "No response from Kitchen, Moving back to Home Pose !!"

                self._reset_param()

            elif self.confirmation_type == 'bot_from_table':

                self.table_id_req_confirmation = self.table_id
                print("\nDelivery Confirmation Required By Table Id : ", self.table_id_req_confirmation)

                self.confirmation_thread = threading.Thread(target=self.ask_for_confirmation, args=(self.table_id_req_confirmation,'table',), daemon=True)
                self.confirmation_thread.start()

                # Wait until confirmation is complete.
                while not self.confirmation_done:
                    self.confirmation_rate.sleep()

                if self.is_delivered:
                    print(f"\n-> Delivery Confirmed For Table {self.table_id_req_confirmation} !!")
                    # Set the service response indicating success
                    response.success = True
                    response.message = "Delivery Confirmed!"
                else:
                    print(f"\n-> Delivery Failed For Table {self.table_id_req_confirmation} !!")
                    # Set the service response indicating success
                    response.success = False
                    response.message = "Delivery Failed!"

                self._reset_param()

            elif self.confirmation_type == 'table_to_bot':

                print("\nDelivery Confirmed By Table Id : ", self.confirm_table_id)
                self.confirmed_table_ids.append(int(self.confirm_table_id))

                response.success = True
                response.message = f"Thanks for the Confirmation! (Table Id = {self.confirm_table_id})"
            else:
                response.success = False
                response.message = "Wrong confirmation_type! Please Type correct value..."
            
            return response
        except Exception as e:
            self.get_logger().error(f"Exception in confirmation_callback: {e}")
            response.success = False
            response.message = f"Service handler crashed: {e}"
            return response


        





# Main function to initialize the ROS2 node and spin the executor
def main(args=None):
    rclpy.init(args=args)

    node = ConfirmationService()
    node.get_logger().info("\n\n────────────────────────────────────────────\n   🚀 CONFIRMATION SERVICE STARTED!! 🚀\n────────────────────────────────────────────\n")


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