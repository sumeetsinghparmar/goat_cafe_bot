Below is the complete `README.md` content in Markdown format. You can copy and paste this into your project's `README.md` file:

```markdown
# 🦾 Robot Delivery System Web Interface

This project provides a web-based interface for a Robot Delivery System built using **ROS2**, **Node.js**, and **Socket.IO**. It offers two portals:

- **Customer Portal:** Where users can place and cancel orders.
- **Kitchen Portal:** Where kitchen staff can view pending orders and confirm deliveries.

---

## 🔧 Technologies Used

- **ROS2** (Humble)
- **rclnodejs** — ROS2 interface for Node.js
- **Express.js** — Web server framework
- **Socket.IO** — Real-time communication between server and clients
- **HTML/CSS/JavaScript** — Frontend user interfaces

---

## 📂 Project Structure

```
goat_cafe_web/
│
├── public/
│   ├── customer.html      # Customer ordering portal
│   └── kitchen.html       # Kitchen order management portal
│
├── server.js              # Main Node.js server connecting ROS2 with browser clients
└── README.md              # Project documentation
```

---

## 🚀 Getting Started

### Prerequisites

- **ROS2** installed and sourced
- **Node.js** (v16+ recommended)
- **rclnodejs** installed:
  ```bash
  npm install rclnodejs
  ```
- Other npm dependencies:
  ```bash
  npm install express socket.io body-parser
  ```

### Running the Server

1. Make sure your ROS2 environment is correctly sourced.
2. Start the server with:
   ```bash
   node server.js
   ```
3. The server will run on [http://localhost:3000](http://localhost:3000).

---

## 🌐 Accessing the Portals

- **Customer Portal:** [http://localhost:3000/customer](http://localhost:3000/customer)
- **Kitchen Portal:** [http://localhost:3000/kitchen](http://localhost:3000/kitchen)

---

## 📡 How It Works

### ROS2 Interfaces

- **Services**
  - `/order_or_cancel_service` (using `ebot_nav2/srv/OrderOrCancel`)
  - `/confirmation_service` (using `ebot_nav2/srv/Confirmation`)
  
- **Topics**
  - `/bot_state` (using `ebot_nav2/msg/State`) — Publishes the robot's current state.
  - `/order` (using `ebot_nav2/msg/Order`) — Publishes the current pending orders.

### WebSocket Events

#### From Client to Server

| Event             | Example Data              | Description                                   |
|-------------------|---------------------------|-----------------------------------------------|
| `make_order`      | `{ table_id: 1 }`         | Customer places an order.                     |
| `cancel_order`    | `{ table_id: 1 }`         | Customer cancels an order.                    |
| `confirm_delivery`| `{ table_id: 1 }`         | Customer confirms the order delivery.         |
| `get_pending_orders`| —                      | Kitchen requests the list of pending orders.  |
| `deliver_order`   | `{ table_ids: [1, 2] }`    | Kitchen initiates delivery for given orders.  |

#### From Server to Client

| Event            | Example Data             | Description                                  |
|------------------|--------------------------|----------------------------------------------|
| `order_status`   | `"Processing Order"`     | Order/cancellation status response.          |
| `bot_state`      | `"wait_table1"`, etc.    | Robot’s current state.                       |
| `pending_orders` | `[ { table_id: 1 }, ... ]`| Array of pending orders for the kitchen UI.  |
| `order_response` | Service response objects | Responses from service calls (for logging).  |
| `confirmation_response` | Service response objects | Responses from confirmation calls.         |

---

## ✅ Features

- **Real-time Order Management:**  
  Customers can place and cancel orders while receiving real-time feedback.
  
- **Kitchen Portal:**  
  Displays pending orders as checkboxes and shows controls to initiate order delivery. The UI updates based on the robot's `bot_state`.
  
- **Robot Status Display:**  
  Shows the current state of the robot (e.g., navigating, waiting, returning) using emojis.
  
- **ROS2 Integration:**  
  Uses `rclnodejs` to interact with ROS2 topics and services, ensuring seamless communication between the robot and the web interface.

- **Responsive and Modern UI:**  
  Both portals feature a playful, cartoonish design, thanks to Google Fonts and Font Awesome, offering a delightful user experience.

---

## 💡 How to Customize

- **Modifying Food Items:**  
  In the customer portal (`customer.html`), you can update the default food items or add interactive selection mechanisms.

- **Extending Robot States:**  
  In the code, additional states (e.g., `nav_2_home`) can be added to the robot status logic in the client-side scripts.

- **Adding More Features:**  
  You can extend this project by adding authentication, multi-robot support, mobile responsiveness, or advanced analytics dashboards.

---

## 🙋‍♂️ Contributing

Feel free to fork the repository and submit pull requests! When contributing, please follow these guidelines:

1. Write clean, modular code.
2. Test your changes locally with both ROS2 and the web interface.
3. Update documentation as needed.

---

## 📞 Support

For any questions or issues, please reach out to **Sumeet** or open an issue in the repository.

---

## 📦 License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---

Happy Coding and Robot Deliveries! 🚚🤖
```

---

