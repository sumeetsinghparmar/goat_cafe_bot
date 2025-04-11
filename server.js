const express = require('express');
const bodyParser = require('body-parser');
const http = require('http');
const socketio = require('socket.io');
const rclnodejs = require('rclnodejs');

// Declare these globally so they are accessible in service functions
let OrderOrCancel;
let Confirmation;

// Create Express App
const app = express();
const server = http.createServer(app);
const io = socketio(server);

app.use(bodyParser.json());
app.use(express.static('public'));

rclnodejs.init().then(() => {
  // Load service definitions
  OrderOrCancel = rclnodejs.require('ebot_nav2/srv/OrderOrCancel');
  Confirmation = rclnodejs.require('ebot_nav2/srv/Confirmation');

  const rosNode = new rclnodejs.Node('web_interface_node');

  let botState = 'IDLE';

  const Int32MultiArray = rclnodejs.require('std_msgs/msg/Int32MultiArray');

  rosNode.createSubscription(
    'ebot_nav2/msg/State',
    '/bot_state',
    (msg) => {
      botState = msg.state;
      io.emit('bot_state', botState);
    },
    { qos: 10 }
  );

  rosNode.createSubscription(
    'ebot_nav2/msg/Order',
    '/order',
    (msg) => {
      const tableOrders = msg.table_orders; // Correct field
  
      if (tableOrders && typeof tableOrders === 'object') {
        const tableIds = Object.values(tableOrders);
        const formattedOrders = tableIds.map(id => ({ table_id: id }));
        io.emit('pending_orders', formattedOrders);
      } else {
        console.warn('Invalid order format:', msg);
      }
    },
    { qos: 10 }
  );

  const orderCancelClient = rosNode.createClient(
    'ebot_nav2/srv/OrderOrCancel',
    '/order_or_cancel_service'
  );

  const confirmationClient = rosNode.createClient(
    'ebot_nav2/srv/Confirmation',
    '/confirmation_service'
  );

  function callOrderOrCancelService(orderOptions) {
    if (!orderCancelClient.waitForService(1000)) {
      console.log('Service /order_or_cancel_service not available');
      return;
    }
    const request = new OrderOrCancel.Request(orderOptions);
    orderCancelClient.sendRequest(request, (response) => {
      console.log('OrderOrCancel Service Response:', response);
      io.emit('order_response', response);
    });
  }

  function callConfirmationService(options) {
    if (!confirmationClient.waitForService(1000)) {
      console.log('Service /confirmation_service not available');
      return;
    }
    const request = new Confirmation.Request(options);
    confirmationClient.sendRequest(request, (response) => {
      console.log('Confirmation Service Response:', response);
      io.emit('confirmation_response', response);
    });
  }

  io.on('connection', (socket) => {
    console.log('New client connected');

    socket.on('make_order', (data) => {
      const serviceRequest = { order: true, cancel: false, table_id: data.table_id };
      callOrderOrCancelService(serviceRequest);
      socket.emit('order_status', 'Processing Order');
    });

    socket.on('cancel_order', (data) => {
      const serviceRequest = { order: false, cancel: true, table_id: data.table_id };
      callOrderOrCancelService(serviceRequest);
      socket.emit('order_status', 'Order Canceled');
    });

    socket.on('confirm_delivery', (data) => {
      const serviceRequest = {
        confirmation_type: 'table_to_bot',
        table_id: data.table_id,
        confirm_table_id: data.table_id,
        reset: false,
      };
      callConfirmationService(serviceRequest);
    });

    socket.on('deliver_order', (data) => {
      const serviceRequest = {
        confirmation_type: 'kitchen_to_bot',
        table_id: data.table_id || 0,
        confirm_table_id: 0,
        reset: false,
        start_delivery: true,
        order_ready_list: data.table_ids,
      };
      callConfirmationService(serviceRequest);
    });
  });

  setInterval(() => {
    rclnodejs.spinOnce(rosNode);
  }, 100);
});

app.get('/customer', (req, res) => {
  res.sendFile(__dirname + '/public/customer.html');
});

app.get('/kitchen', (req, res) => {
  res.sendFile(__dirname + '/public/kitchen.html');
});

const PORT = 3000;
server.listen(PORT, () => {
  console.log(`Server listening on http://localhost:${PORT}`);
});
