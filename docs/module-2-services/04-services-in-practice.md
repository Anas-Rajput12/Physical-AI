# ROS 2 Services in Practice: Command-Line Tools

ROS 2 provides a set of command-line tools to interact with services without writing any code. These tools are invaluable for debugging, introspection, and quick testing of services.

First, ensure you have a service server running. For example, launch the C++ server from the previous chapter:

```bash
source install/setup.bash
ros2 run module_2_services cpp_service_server
```

## 1. Listing Services: `ros2 service list`

This command shows all currently active services in your ROS 2 graph.

```bash
ros2 service list
```

**Expected Output** (you might see other services depending on your system):

```
/add_two_ints
/rosout
/parameter_events
```

You should see `/add_two_ints` listed, confirming our service server is advertising its service.

## 2. Getting Service Type: `ros2 service type`

To find out the type of a specific service, use `ros2 service type` followed by the service name.

```bash
ros2 service type /add_two_ints
```

**Expected Output**:

```
module_2_services/srv/AddTwoInts
```

This tells us that the service `/add_two_ints` expects requests and provides responses defined by the `AddTwoInts` interface within the `module_2_services` package.

## 3. Calling a Service: `ros2 service call`

The `ros2 service call` command allows you to send a request to a service directly from the command line.

Its syntax is: `ros2 service call <service_name> <service_type> <arguments>`

The arguments need to be provided in YAML format for structured types.

```bash
ros2 service call /add_two_ints module_2_services/srv/AddTwoInts "{a: 5, b: 3}"
```

**Expected Output** (from the client side, after the server processes):

```
requester: making request: {a: 5, b: 3}
response:
  sum: 8
```

On the server side, you would see the incoming request and the sum being sent back, similar to the output when using a client node.

These command-line tools are powerful for quickly inspecting and interacting with services, making development and debugging much easier.