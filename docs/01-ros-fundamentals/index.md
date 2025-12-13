---
id: module-1
title: "Module 1: The Robotic Nervous System (ROS 2)"
sidebar_label: ROS 2 Fundamentals
---

# Module 1: The Robotic Nervous System (ROS 2)

## Overview
This module introduces the Robotic Operating System (ROS 2) as the foundational middleware for developing complex robotic applications. Students will learn how ROS 2 facilitates communication between different software components, enabling a modular and distributed approach to robot control and sensing. We will cover the core concepts necessary to build a robust and scalable robotic system.

## Key Concepts / Topics
- **ROS 2 Architecture**: Understanding the fundamental building blocks of a ROS 2 system, including nodes, topics, services, and actions.
- **Client Libraries**: Working with `rclpy` (Python) and `rclcpp` (C++) for developing ROS 2 applications.
- **Message Types**: Defining and using standard and custom message types for data exchange.
- **ROS 2 Packages**: Structuring and managing ROS 2 code within a workspace.
- **Launch System**: Orchestrating multiple ROS 2 nodes and configurations using launch files.
- **Parameter System**: Configuring node behavior dynamically using parameters.

## Practical Examples / Exercises
In this section, we will develop basic ROS 2 Python nodes to demonstrate inter-process communication. These examples will illustrate how to create publishers, subscribers, service servers, and service clients, forming the backbone of any ROS 2 application.

### Example: Simple ROS 2 Publisher Node (Python)
This node continuously publishes a string message to a topic.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2! Count: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example: Simple ROS 2 Subscriber Node (Python)
This node subscribes to the 'chatter' topic and prints received messages.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## ROS 2 Implementation Notes
When developing ROS 2 applications, adherence to best practices is crucial for maintainability and performance. This includes proper package structuring, using the `ament` build system, and correctly managing dependencies. Python nodes typically reside within a package's `package_name/package_name/` directory structure, with executables defined in `setup.py`.

## Building and Running
To build these examples, ensure your workspace is sourced and run `colcon build --packages-select robot_control`. After building, source your `install/setup.bash` (or equivalent) and you can run the nodes using `ros2 run robot_control simple_publisher` and `ros2 run robot_control simple_subscriber`.