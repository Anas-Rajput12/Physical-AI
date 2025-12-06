# Creating a ROS 2 Service

This chapter guides you through defining a custom ROS 2 service and implementing a service server in both C++ and Python.

## 1. Defining the Service Interface (`.srv` file)

A service interface defines the structure of the request and response messages. We'll use a simple `AddTwoInts.srv` service that takes two integers (`a`, `b`) and returns their sum.

Create the file `static/code-examples/module-2-services/srv/AddTwoInts.srv` with the following content:

```srv
int64 a
int64 b
---
int64 sum
```

- The `---` separates the request fields from the response fields.

## 2. Implementing the Service Server (C++)

The C++ service server waits for requests, performs the `add` operation, and sends back the sum. Create `static/code-examples/module-2-services/cpp_service_server.cpp`:

```cpp
#include "rclcpp/rclcpp.hpp"
#include "module_2_services/srv/add_two_ints.hpp"

#include <memory>

void add(const std::shared_ptr<module_2_services::srv::AddTwoInts::Request> request,
         std::shared_ptr<module_2_services::srv::AddTwoInts::Response>      response)
{
  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

  rclcpp::Service<module_2_services::srv::AddTwoInts>::SharedPtr service =
    node->create_service<module_2_services::srv::AddTwoInts>("add_two_ints", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
```

## 3. Implementing the Service Server (Python)

The Python service server performs the same function. Create `static/code-examples/module-2-services/py_service_server.py`:

```python
import rclpy
from rclpy.node import Node

from module_2_services.srv import AddTwoInts


class AddTwoIntsService(Node):

    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Ready to add two ints.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        self.get_logger().info('Sending back response: [%d]' % response.sum)
        return response


def main(args=None):
    rclpy.init(args=args)

    add_two_ints_service = AddTwoIntsService()

    rclpy.spin(add_two_ints_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 4. Building Your ROS 2 Package

To make your service available, you need to create a ROS 2 package (e.g., `module_2_services`) and configure its `CMakeLists.txt` and `package.xml` to generate the service interface and build the executables. Assuming you have a basic ROS 2 package structure, here are the key additions:

**`package.xml` additions:**

```xml
<depend>rclcpp</depend>
<depend>rclpy</depend>
<depend>std_msgs</depend>
<buildtool_depend>ament_cmake</buildtool_depend>
<buildtool_depend>ament_cmake_python</buildtool_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<exec_depend>python3-rclpy</exec_depend>
<exec_depend>python3-std-msgs</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

**`CMakeLists.txt` additions:**

```cmake
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddTwoInts.srv"
)

ament_python_install_module(${PROJECT_NAME})

# For C++ executable
add_executable(cpp_service_server static/code-examples/module-2-services/cpp_service_server.cpp)
ament_target_dependencies(cpp_service_server rclcpp module_2_services)
install(TARGETS cpp_service_server
  DESTINATION lib/${PROJECT_NAME})

# For Python executable
install(PROGRAMS
  static/code-examples/module-2-services/py_service_server.py
  DESTINATION lib/${PROJECT_NAME})

```

After setting up your `package.xml` and `CMakeLists.txt`, navigate to your workspace root and run:

```bash
colcon build --packages-select module_2_services
source install/setup.bash
```

## 5. Running the Service Server

To start the C++ server:

```bash
ros2 run module_2_services cpp_service_server
```

To start the Python server:

```bash
ros2 run module_2_services py_service_server
```

You should see output indicating that the server is ready.