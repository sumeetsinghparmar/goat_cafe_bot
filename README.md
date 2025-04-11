```markdown
# Goat Cafe Bot Documentation

────────────────────────────────────────────────────────────  
         🤖 Goat Cafe Bot 🤖  (for ebot_nav2 pkg)  
────────────────────────────────────────────────────────────  

_Last updated: April 2025_

This documentation explains the design, implementation, and usage of the **Goat Cafe Bot**—an autonomous delivery system built using ROS 2. This system integrates several nodes, topics, and services to manage food orders in a restaurant.

---

## Table of Contents

- [Introduction](#introduction)
- [System Architecture](#system-architecture)
  - [Overview Diagram](#overview-diagram)
- [Nodes & Components](#nodes--components)
  - [Order Delivery Service](#order-delivery-service)
  - [Order/Cancel Service](#ordercancel-service)
  - [Confirmation Service](#confirmation-service)
  - [Status Updater Service](#status-updater-service)
  - [Order Manager Node](#order-manager-node)
- [Services](#services)
  - [Delivery Service](#delivery-service)
  - [OrderOrCancel Service](#orderorcancel-service)
  - [Confirmation Service Usage](#confirmation-service-usage)
  - [StateUpdater Service](#stateupdater-service)
- [Topics](#topics)
  - [/order Topic](#order-topic)
  - [/bot_state Topic](#bot_state-topic)
- [Setup & Launch](#setup--launch)
  - [Gazebo](#gazebo)
  - [RViz](#rviz)
  - [Launch All Services](#launch-all-services)
  - [Start the Main Manager Node](#start-the-main-manager-node)
- [Usage](#usage)
  - [Building and Running](#building-and-running)
  - [Service Call Examples](#service-call-examples)
- [Demo Videos](#demo-videos)
- [Conclusion](#conclusion)
- [Appendix: Code Structure](#appendix-code-structure)

---

## Introduction

Welcome to the **Goat Cafe Bot** documentation.

The Goat Cafe Bot autonomously handles food orders by:
- Receiving and updating orders/cancellations.
- Navigating between the kitchen, delivery tables, and home.
- Processing confirmations from both the kitchen (order preparation) and customers (order delivered).
- Maintaining and publishing the robot's state.

*Core Features:*
- 🔄 Real-time order updates via the `/order` topic.
- 🚚 Autonomous navigation using the BasicNavigator.
- ✅ Service-based order placement, cancellation, and confirmations.
- 📡 Continuous state updates on the `/bot_state` topic.

> **Demo Videos:** Refer to the [Demo Videos](#demo-videos) section for live demonstrations.

---

## System Architecture

### Overview Diagram

```
               ┌─────────────────────────┐
               │   Order/Cancel Service  │
               └──────────┬──────────────┘
                          │
        ┌─────────────────┴─────────────────┐
        │     Order Delivery Service        │
        └─────────────────┬─────────────────┘
                          │
    ┌────────────┐      ┌────────────┐      ┌────────────┐
    │ Confirmation│◄────►│ StateUpdater│◄────►│  Navigator │
    │  Service   │      │  Service    │      │ (BasicNav) │
    └────────────┘      └────────────┘      └────────────┘
                          │
                    ┌─────┴─────┐
                    │ /bot_state│
                    └───────────┘

            ┌─────────────┐
            │  /order     │
            └─────────────┘
```

- **Order/Cancel Service:** Updates orders from customers.
- **Delivery Service:** Manages the delivery sequence (navigation + confirmations).
- **Confirmation Service:** Processes order confirmations from the kitchen and customers.
- **StateUpdater Service:** Updates and publishes the robot's state.
- **BasicNavigator:** Executes navigation goals.
- **Topics:** `/order` (order updates) and `/bot_state` (robot state).

---

## Nodes & Components

### Order Delivery Service

**File:** `scripts/delivery_service.py`

- **Purpose:**  
  Coordinates the autonomous delivery process:
  - Sends navigation goals.
  - Waits for confirmations.
  - Updates the robot's state accordingly.
- **Key Functions:**
  - `start_delivery_func()`: Runs the complete delivery sequence.
  - `goal_navigator()`: Publishes navigation goals.
  - `wait_till_navigation_done()`: Waits for navigation to complete.
  - `update_bot_state()`: Invokes the StateUpdater service.
  - `request_delivery_confirmation()`: Calls the Confirmation service.

---

### Order/Cancel Service

**File:** `scripts/order_or_cancel_service.py`

- **Purpose:**  
  Enables customers to place or cancel orders by specifying the table id.
- **Example Usage:**  
  ```bash
  ros2 service call /order_or_cancel_service ebot_nav2/srv/OrderOrCancel "{order: true, cancel: false, table_id: 2}"
  ```

---

### Confirmation Service

**File:** `scripts/confirmation_service.py`

- **Purpose:**  
  Processes confirmations from the kitchen and the customer.
- **Usage Examples:**
  - **Customer Confirmation:**  
    ```bash
    ros2 service call /confirmation_service ebot_nav2/srv/Confirmation "{confirmation_type: 'table_to_bot', table_id: 1, confirm_table_id: 2, reset: false}"
    ```
  - **Kitchen Confirmation:**  
    ```bash
    ros2 service call /confirmation_service ebot_nav2/srv/Confirmation "{confirmation_type: 'kitchen_to_bot', table_id: 3, confirm_table_id: 0, reset: false, start_delivery: true, order_ready_list: [2]}"
    ```

---

### Status Updater Service

**File:** `scripts/status_updater_service.py`

- **Purpose:**  
  Updates and continuously publishes the robot's current state on `/bot_state`.
- **Usage Example:**  
  ```bash
  ros2 service call /state_updater_service ebot_nav2/srv/StateUpdater "{state: 'IDLE'}"
  ```

---

### Order Manager Node

**File:** `scripts/order_manager.py`

- **Purpose:**  
  Acts as the main manager node that monitors orders, coordinates with the delivery services, and controls the overall workflow.
- **Role:**  
  - Subscribes to the `/order` topic.
  - Monitors the robot state via `/bot_state`.
  - Calls the Delivery service when new orders arrive.
  - Handles cancellation and state changes accordingly.

---

## Services

### Delivery Service

- **Name:** `/delivery_service`
- **Type:** `ebot_nav2/srv/Delivery`
- **Function:**  
  Initiates the delivery sequence based on the provided table order list.
- **Example Call:**
  ```bash
  ros2 service call /delivery_service ebot_nav2/srv/Delivery "{start_delivery: true, deliver_to: [1, 3]}"
  ```

---

### OrderOrCancel Service

- **Name:** `/order_or_cancel_service`
- **Type:** `ebot_nav2/srv/OrderOrCancel`
- **Function:**  
  Enables order placement or cancellation.

---

### Confirmation Service Usage

- **Name:** `/confirmation_service`
- **Type:** `ebot_nav2/srv/Confirmation`
- **Function:**  
  - **`table_to_bot`**: Customer confirms order delivery.
  - **`kitchen_to_bot`**: Kitchen indicates order readiness.
- **Usage Examples:**
  - **Customer Confirmation:**
    ```bash
    ros2 service call /confirmation_service ebot_nav2/srv/Confirmation "{confirmation_type: 'table_to_bot', table_id: 1, confirm_table_id: 2, reset: false}"
    ```
  - **Kitchen Confirmation:**
    ```bash
    ros2 service call /confirmation_service ebot_nav2/srv/Confirmation "{confirmation_type: 'kitchen_to_bot', table_id: 3, confirm_table_id: 0, reset: false, start_delivery: true, order_ready_list: [2]}"
    ```

---

### StateUpdater Service

- **Name:** `/state_updater_service`
- **Type:** `ebot_nav2/srv/StateUpdater`
- **Function:**  
  Updates the robot’s current state.
- **Typical States:**  
  - `IDLE`
  - `nav_2_kitchen`
  - `wait_kitchen`
  - `nav_2_tableX`
  - `wait_tableX`
  - `nav_2_home`
  - `HALT`

---

## Topics

### /order Topic

- **Type:** `ebot_nav2/msg/Order`
- **Function:**  
  Publishes the current orders received from customers.

---

### /bot_state Topic

- **Type:** `ebot_nav2/msg/State`
- **Function:**  
  Continuously broadcasts the current state of the robot.

---

## Setup & Launch

### Gazebo

To start the Gazebo simulation:
```bash
ros2 launch ebot_description goat_gazebo_launch.py
```

### RViz

To start RViz and view navigation:
```bash
ros2 launch ebot_nav2 ebot_bringup_launch.py
```

### Launch All Services

Use the provided launch file to start all the services:
```bash
ros2 launch ebot_nav2 ebot_ALL_scripts_launch.py
```
*This launch file starts the following services:*
- Order Delivery Service
- Confirmation Service
- Order/Cancel Service
- Status Updater Service

### Start the Main Manager Node

Run the main order manager node that controls the whole process:
```bash
ros2 run ebot_nav2 order_manager.py
```
*The Order Manager Node monitors orders from the `/order` topic, tracks robot state from `/bot_state`, and coordinates delivery by calling the corresponding services.*

---

## Usage

### Building and Running the System

1. **Build the Package:**
   ```bash
   colcon build --packages-select ebot_nav2
   source install/setup.bash
   ```
2. **Launch the System:**
   Start all nodes and services:
   ```bash
   ros2 launch ebot_nav2 ebot_ALL_scripts_launch.py
   ```

### Service Call Examples

- **Place an Order:**
  ```bash
  ros2 service call /order_or_cancel_service ebot_nav2/srv/OrderOrCancel "{order: true, cancel: false, table_id: 2}"
  ```
- **Customer Confirmation:**
  ```bash
  ros2 service call /confirmation_service ebot_nav2/srv/Confirmation "{confirmation_type: 'table_to_bot', table_id: 1, confirm_table_id: 2, reset: false}"
  ```
- **Kitchen Confirmation:**
  ```bash
  ros2 service call /confirmation_service ebot_nav2/srv/Confirmation "{confirmation_type: 'kitchen_to_bot', table_id: 3, confirm_table_id: 0, reset: false, start_delivery: true, order_ready_list: [2]}"
  ```
- **Start Delivery:**
  ```bash
  ros2 service call /delivery_service ebot_nav2/srv/Delivery "{start_delivery: true, deliver_to: [1, 3]}"
  ```
- **Update Bot State to IDLE:**
  ```bash
  ros2 service call /state_updater_service ebot_nav2/srv/StateUpdater "{state: 'IDLE'}"
  ```

---

## Demo Videos

Here are some demo videos showcasing the system in action:

- **Order Pickup & Delivery:**  
  [![Demo Video 1](https://img.youtube.com/vi/avaCW78hfr0/0.jpg)](https://www.youtube.com/watch?v=avaCW78hfr0)

---

## Conclusion

The Goat Cafe Bot system integrates multiple ROS 2 nodes, services, and topics to autonomously manage food orders in a restaurant setting. With robust service-based communication, real-time state updates, and intuitive navigation, the system adapts to changing orders and provides a reliable, automated delivery process.

This documentation should provide a comprehensive guide for understanding, setting up, and operating the Goat Cafe Bot. For further customization or troubleshooting, please refer to the individual node and service documentation.

---

## Appendix: Code Structure

```
ebot_nav2/
├── config/
│   └── positions.yaml         # Pose configurations for home, kitchen, tables
├── launch/
│   └── ebot_ALL_scripts_launch.py
├── msg/
│   ├── Order.msg              # Message definition for order updates
│   └── State.msg              # Message definition for robot state
├── srv/
│   ├── Delivery.srv           # Service for initiating delivery
│   ├── OrderOrCancel.srv      # Service for order placement/cancellation
│   ├── Confirmation.srv       # Service for confirmation messages
│   └── StateUpdater.srv       # Service for updating robot state
├── scripts/
│   ├── delivery_service.py    # Delivery Service node
│   ├── order_or_cancel_service.py  # Order/Cancel Service node
│   ├── confirmation_service.py     # Confirmation Service node
│   ├── status_updater_service.py   # Status Updater Service node
│   └── order_manager.py       # Main Order Manager Node
└── README.md
```
