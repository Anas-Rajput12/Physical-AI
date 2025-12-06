# Creating a ROS 2 Service Client

This chapter guides you through implementing a service client in both C++ and Python to interact with the service server created in the previous chapter.

## 1. Implementing the Service Client (C++)

The C++ service client sends a request to the `add_two_ints` service and waits for a response. Create `static/code-examples/module-2-services/cpp_service_client.cpp`:

```cpp
#include "rclcpp/rclcpp.hpp"
#include "module_2_services/srv/add_two_ints.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 3) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");
  rclcpp::Client<module_2_services::srv::AddTwoInts>::SharedPtr client =
    node->create_client<module_2_services::srv::AddTwoInts>("add_two_ints");

  auto request = std::make_shared<module_2_services::srv::AddTwoInts::Request>();
  request->a = atoll(argv[1]);
  request->b = atoll(argv[2]);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }

  rclcpp::shutdown();
  return 0;
}
```

## 2. Implementing the Service Client (Python)

The Python service client performs the same function. Create `static/code-examples/module-2-services/py_service_client.py`:

```python
import sys

import rclpy
from rclpy.node import Node

from module_2_services.srv import AddTwoInts


def main(args=None):
    rclpy.init(args=args)

    node = Node('add_two_ints_client')
    cli = node.create_client(AddTwoInts, 'add_two_ints')

    req = AddTwoInts.Request()
    req.a = int(sys.argv[1])
    req.b = int(sys.argv[2])

    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(
            'Result of add_two_ints: for %d + %d = %d' %
            (req.a, req.b, future.result().sum))
    else:
        node.get_logger().error('Service call failed %r' % (future.exception(),))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 3. Running the Service Client

First, ensure your ROS 2 workspace is built and sourced, and that a service server (either C++ or Python) is running. For example, if you built the `module_2_services` package and are running the C++ server:

```bash
source install/setup.bash
ros2 run module_2_services cpp_service_server
```

In a **new terminal**, run the client, providing two integers as arguments:

To run the C++ client:

```bash
ros2 run module_2_services cpp_service_client 5 3
```

To run the Python client:

```bash
ros2 run module_2_services py_service_client 10 2
```

You should see output from both the client and server, indicating a successful request and response.