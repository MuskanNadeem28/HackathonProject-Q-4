# ROS 2 Architecture & Concepts

## Overview

ROS 2 (Robot Operating System 2) is the nervous system of modern robots, providing a flexible framework for writing robot software. Think of it as the communication network that allows different parts of a robot to work together seamlessly.

## Why ROS 2?

Imagine a humanoid robot that needs to:
- See with cameras (perception)
- Move its limbs (actuation)
- Process information (computation)
- Navigate spaces (navigation)

ROS 2 allows these different functions to communicate with each other without being directly connected, just like how your nervous system allows different parts of your body to coordinate.

## Key Concepts

### Nodes
Nodes are the basic building blocks of ROS 2. Each node performs a specific task:
- A camera node processes visual information
- A motor control node moves the robot's joints
- A navigation node plans paths

Think of nodes like specialized organs in a body, each with its own function.

### Topics and Messages
Topics are communication channels where nodes share information. Messages are the data sent through these channels.
- A camera node publishes images to the `/camera/image_raw` topic
- A perception node subscribes to this topic to process the images

This is like a news feed where some nodes post updates and others read them.

### Services
Services provide request-response communication. When a node needs a specific task done, it sends a request and receives a response.
- Request: "Please calculate the distance to the obstacle"
- Response: "The obstacle is 2 meters away"

This is like asking a specific question and getting a direct answer.

## ROS 2 Architecture

ROS 2 uses DDS (Data Distribution Service) as its middleware, which provides:
- Automatic discovery of nodes
- Reliable message delivery
- Configurable quality of service settings

## Practical Example: Simple Publisher Node

Let's create a simple node that publishes messages:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher = self.create_publisher(String, 'robot_status', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Robot is operational: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()

    try:
        rclpy.spin(simple_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        simple_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

To run this code:
1. Save it as `simple_publisher.py`
2. Make sure you're in a ROS 2 environment
3. Run: `python3 simple_publisher.py`

## Exercises

1. Create a simple ROS 2 package and run the publisher example
2. Modify the publisher to send different types of messages
3. Observe the messages using the command line tool: `ros2 topic echo /robot_status`

## Summary

ROS 2 provides the communication infrastructure that allows different parts of a robot to work together. Understanding nodes, topics, and services is fundamental to working with humanoid robots, as these concepts form the basis for all robot communication.