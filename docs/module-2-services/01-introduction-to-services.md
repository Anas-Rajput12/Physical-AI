# Introduction to ROS 2 Services

ROS 2 services provide a **request-response communication pattern** between nodes. Unlike topics, which are designed for continuous, one-to-many data streaming, services are used for discrete, transactional interactions. A client node sends a request to a service server, and the server processes the request and sends back a single response.

## Services vs. Topics

Here's a breakdown of the key differences between ROS 2 services and topics:

| Feature           | ROS 2 Topics                                  | ROS 2 Services                                   |
|-------------------|-----------------------------------------------|--------------------------------------------------|
| **Communication Pattern** | Publish-Subscribe (one-to-many)               | Request-Response (one-to-one)                    |
| **Data Flow**     | Continuous stream of messages (e.g., sensor data) | Discrete, single transaction                     |
| **Blocking**      | Non-blocking (publishers don't wait for subscribers) | Typically blocking (client waits for response)   |
| **Use Cases**     | Telemetry, sensor readings, robot state       | Triggering actions, querying data, performing computations |

## When to use Services

Use ROS 2 services when you need:

-   **To request a specific action**: For example, telling a robot to perform a task (e.g., "move arm to position X").
-   **To query data once**: Such as asking a sensor for its current reading or a map server for a specific map region.
-   **Transactional interactions**: Where a client needs a direct response from a server before proceeding.
-   **To implement functional calls**: Similar to calling a function or method in traditional programming, but distributed across your ROS 2 system.