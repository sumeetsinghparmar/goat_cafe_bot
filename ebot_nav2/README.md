```markdown
# FoodBot Delivery System Documentation

────────────────────────────────────────────  
         🤖 FoodBot Delivery System 🤖  
────────────────────────────────────────────  

_Last updated: April 2025_

This documentation explains the design, implementation, and usage of the FoodBot Delivery System. The system is built using ROS 2 and comprises several nodes, services, and topics that work together to autonomously deliver food orders in a restaurant environment.

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
- [Services](#services)
  - [Delivery Service](#delivery-service)
  - [OrderOrCancel Service](#orderorcancel-service)
  - [Confirmation Service Usage](#confirmation-service-usage)
  - [StateUpdater Service](#stateupdater-service)
- [Topics](#topics)
  - [/order Topic](#order-topic)
  - [/bot_state Topic](#bot_state-topic)
- [Usage](#usage)
  - [Building and Running the System](#building-and-running-the-system)
  - [Service Call Examples](#service-call-examples)
- [Demo Videos](#demo-videos)
- [Troubleshooting](#troubleshooting)
- [Conclusion](#conclusion)
- [Appendix: Code Structure](#appendix-code-structure)

---

## Introduction

Welcome to the **FoodBot Delivery System** documentation.

This system autonomously handles food orders in a restaurant by:
- Receiving orders and cancellations.
- Navigating autonomously between the kitchen, tables, and home.
- Processing confirmations from both the kitchen (order readiness) and the customers (order delivered).
- Managing and broadcasting the robot’s current state.

*Core Features:*
- 🔄 Real-time order updates via the `/order` topic.
- 🚚 Autonomous navigation using the BasicNavigator.
- ✅ Service-based order placement, cancellation, and confirmation.
- 📡 Continuous state updates on the `/bot_state` topic.

> **Demo Videos Available:** See [Demo Videos](#demo-videos) for live demonstrations.

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
- **Delivery Service:** Handles the delivery sequence (navigation and confirmation).
- **Confirmation Service:** Manages confirmations from the kitchen and customer.
- **StateUpdater Service:** Updates and publishes the robot state.
- **BasicNavigator:** Executes navigation goals.
- **Topics:** `/order` for order updates and `/bot_state` for robot state.

---

## Nodes & Components

### Order Delivery Service

**File:** `src/delivery_service.py`

- **Purpose:**  
  Coordinates the autonomous delivery process. It sends navigation goals, waits for confirmations, and updates the robot's state.
- **Key Functions:**
  - `start_delivery_func()`: Executes the delivery sequence.
  - `goal_navigator()`: Publishes a navigation goal.
  - `wait_till_navigation_done()`: Waits for navigation to complete and processes results.
  - `update_bot_state()`: Calls the StateUpdater service to update the robot’s state.
  - `request_delivery_confirmation()`: Calls the Confirmation service for delivery verification.

### Order/Cancel Service

**File:** `src/order_or_cancel_service.py`

- **Purpose:**  
  Allows customers to place or cancel orders by specifying table IDs.
- **Usage Example:**  
  ```bash
  ros2 service call /order_or_cancel_service ebot_nav2/srv/OrderOrCancel "{order: true, cancel: false, table_id: 2}"
  ```

### Confirmation Service

**File:** `src/confirmation_service.py`

- **Purpose:**  
  Processes confirmations from the kitchen and customers.
- **Usage Examples:**
  - **Customer Confirmation:**  
    ```bash
    ros2 service call /confirmation_service ebot_nav2/srv/Confirmation "{confirmation_type: 'table_to_bot', table_id: 1, confirm_table_id: 2, reset: false}"
    ```
  - **Kitchen Confirmation:**  
    ```bash
    ros2 service call /confirmation_service ebot_nav2/srv/Confirmation "{confirmation_type: 'kitchen_to_bot', table_id: 3, confirm_table_id: 0, reset: false, start_delivery: true, order_ready_list: [2]}"
    ```

### Status Updater Service

**File:** `src/status_updater_service.py`

- **Purpose:**  
  Updates and publishes the robot's current state on the `/bot_state` topic.
- **Usage Example:**  
  ```bash
  ros2 service call /state_updater_service ebot_nav2/srv/StateUpdater "{state: 'IDLE'}"
  ```

---

## Services

### Delivery Service

- **Name:** `/delivery_service`
- **Type:** `ebot_nav2/srv/Delivery`
- **Function:**  
  Starts the delivery process based on the table order list.
- **Sample Call:**
  ```bash
  ros2 service call /delivery_service ebot_nav2/srv/Delivery "{start_delivery: true, deliver_to: [1, 3]}"
  ```

### OrderOrCancel Service

- **Name:** `/order_or_cancel_service`
- **Type:** `ebot_nav2/srv/OrderOrCancel`
- **Function:**  
  Places or cancels orders.
  
### Confirmation Service Usage

- **Name:** `/confirmation_service`
- **Type:** `ebot_nav2/srv/Confirmation`
- **Function:**  
  - `table_to_bot`: Customer confirms the order is delivered.
  - `kitchen_to_bot`: Kitchen indicates that an order is ready.
- **Usage Examples:**
  - **Customer Confirmation:**
    ```bash
    ros2 service call /confirmation_service ebot_nav2/srv/Confirmation "{confirmation_type: 'table_to_bot', table_id: 1, confirm_table_id: 2, reset: false}"
    ```
  - **Kitchen Confirmation:**
    ```bash
    ros2 service call /confirmation_service ebot_nav2/srv/Confirmation "{confirmation_type: 'kitchen_to_bot', table_id: 3, confirm_table_id: 0, reset: false, start_delivery: true, order_ready_list: [2]}"
    ```

### StateUpdater Service

- **Name:** `/state_updater_service`
- **Type:** `ebot_nav2/srv/StateUpdater`
- **Function:**  
  Updates the robot's current state. Typical states include:
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
  Publishes the current orders that the customer places.

### /bot_state Topic

- **Type:** `ebot_nav2/msg/State`
- **Function:**  
  Continuously broadcasts the robot's current state.

---

## Usage

### Building and Running the System

1. **Build the Package:**
   ```bash
   colcon build --packages-select ebot_nav2
   source install/setup.bash
   ```
2. **Launch the System:**
   Use the provided launch file:
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
- **Update Bot State:**
  ```bash
  ros2 service call /state_updater_service ebot_nav2/srv/StateUpdater "{state: 'IDLE'}"
  ```

---

## Demo Videos

Here are some demo videos illustrating the FoodBot Delivery System in action:

- **Order Placement & Delivery:**  
  [![Demo Video 1](https://img.youtube.com/vi/YOUR_VIDEO_ID/0.jpg)](https://www.youtube.com/watch?v=YOUR_VIDEO_ID)

---


## Conclusion

The FoodBot Delivery System integrates multiple ROS 2 nodes, services, and topics to achieve a fully autonomous food delivery process. By using a centralized state management system, robust service-based communication, and real-time order updates, the system is highly adaptable and reliable. This documentation provides a detailed overview for understanding and operating the system.

---

## Appendix: Code Structure

```
ebot_nav2/
├── config/
│   └── positions.yaml         # Pose configurations for home, kitchen, tables
├── launch/
│   └── ebot_ALL_scripts_launch.py
├── msg/
│   ├── Order.msg              # Order message definition
│   └── State.msg              # State message definition
├── srv/
│   ├── Delivery.srv           # Service for initiating delivery
│   ├── OrderOrCancel.srv      # Service for order placement/cancellation
│   ├── Confirmation.srv       # Service for confirmation messages
│   └── StateUpdater.srv       # Service for updating robot state
├── src/
│   ├── delivery_service.py    # Delivery Service node
│   ├── order_or_cancel_service.py  # Order/Cancel Service node
│   ├── confirmation_service.py     # Confirmation Service node
│   └── status_updater_service.py   # Status Updater Service node
└── README.md
```

---
