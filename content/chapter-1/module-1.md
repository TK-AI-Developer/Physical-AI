---
title: The Robotic Nervous System (ROS 2)
slug: module-1-ros2
example: |
  ```python
  import rclpy
  from rclpy.node import Node
  from std_msgs.msg import String

  class MinimalPublisher(Node):
      def __init__(self):
          super().__init__('minimal_publisher')
          self.publisher_ = self.create_publisher(String, 'topic', 10)
          timer_period = 0.5  # seconds
          self.timer = self.create_timer(timer_period, self.timer_callback)
          self.i = 0

      def timer_callback(self):
          msg = String()
          msg.data = f'Hello ROS 2: {self.i}'
          self.publisher_.publish(msg)
          self.get_logger().info(f'Publishing: "{msg.data}"')
          self.i += 1

  def main(args=None):
      rclpy.init(args=args)
      minimal_publisher = MinimalPublisher()
      rclpy.spin(minimal_publisher)
      minimal_publisher.destroy_node()
      rclpy.shutdown()

  if __name__ == '__main__':
      main()
  ```
exercise: |
  1. Install ROS 2 Foxy Fitzroy or later on your system.
  2. Create a new ROS 2 workspace.
  3. Create a new Python package named `my_ros2_pkg`.
  4. Implement a ROS 2 publisher node that publishes a "Hello World" message every second on a topic named "greeting".
  5. Implement a ROS 2 subscriber node that subscribes to the "greeting" topic and prints the received message.
  6. Run both nodes simultaneously and verify the communication.
---

# The Robotic Nervous System (ROS 2)

## Introduction to ROS 2

The Robot Operating System (ROS) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behaviors. ROS 2 is the latest generation of this framework, built to address modern requirements such as multi-robot systems, real-time control, and embedded platforms.

Think of ROS 2 as the central nervous system for a robot. Just as our nervous system connects different parts of our body, allowing them to communicate and coordinate actions, ROS 2 provides the infrastructure for various robot components (sensors, actuators, processing units) to exchange information and work together seamlessly.

## Key Concepts in ROS 2

### Nodes

Nodes are executable processes in ROS 2. They perform computation, such as reading sensor data, controlling motors, or processing algorithms. A robot system typically consists of many nodes, each responsible for a specific task. For example, one node might control the robot's wheels, another might process camera images, and yet another might handle navigation logic.

### Topics

Topics are named buses over which nodes exchange messages. A node can publish messages to a topic, and other nodes can subscribe to that topic to receive the messages. This publish/subscribe mechanism allows for decoupled communication: nodes don't need to know about each other directly; they only need to agree on the topic name and message type.

### Messages

Messages are data structures used to send information over topics. They are strongly typed, ensuring that publishers and subscribers agree on the format of the data being exchanged. ROS 2 provides a wide range of standard message types for common data (e.g., integers, strings, arrays), and you can also define custom message types for more specialized data.

### Services

Services provide a request/reply mechanism for communication between nodes. When a node needs to perform an action or query data from another node, it can call a service. The service server node processes the request and sends back a response. This is useful for tasks that require an immediate response or are triggered infrequently.

### Actions

Actions are used for long-running tasks that provide periodic feedback and can be preempted. They extend the service concept by allowing clients to send goals, receive continuous feedback on the goal's progress, and cancel the goal if needed. This is ideal for navigation, manipulation, and other complex robot behaviors.

## Why ROS 2 for Physical AI?

ROS 2 is particularly well-suited for Physical AI and humanoid robotics due to several advantages:

-   **Distributed System Support**: ROS 2 is designed for distributed environments, enabling complex robots to be broken down into modular, interconnected components running on various hardware.
-   **Real-time Capabilities**: With features like Quality of Service (QoS) policies, ROS 2 can meet the timing requirements for critical robot control loops.
-   **Security**: It includes security features like authentication and encryption, crucial for autonomous systems operating in real-world environments.
-   **Language Agnostic**: While C++ and Python are primary, ROS 2 supports other languages, allowing developers to choose the best tool for the job.
-   **Vibrant Community and Ecosystem**: A large and active community means extensive documentation, tutorials, and a wealth of existing packages for common robotics tasks.

In the context of Physical AI, ROS 2 provides the necessary middleware to integrate perception (sensors), cognition (AI algorithms), and action (actuators) into a coherent, functional robot system. It allows researchers and developers to focus on the AI algorithms and hardware design, abstracting away the complexities of inter-process communication and system integration.