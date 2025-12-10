# rclpy: Connecting Python Agents to ROS Controllers

## Introduction to rclpy

rclpy is the Python client library for ROS 2. It allows you to write ROS 2 nodes in Python, which is particularly useful for humanoid robot control since Python is beginner-friendly and has extensive libraries for AI and robotics.

## Setting Up Your First rclpy Node

Let's create a simple node that controls a humanoid robot's joint. This example simulates controlling an arm joint:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')

        # Create a publisher to send joint commands
        self.joint_pub = self.create_publisher(Float64, '/arm_joint/command', 10)

        # Create a subscriber to receive joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Timer to send commands periodically
        self.timer = self.create_timer(0.5, self.send_command)
        self.joint_position = 0.0
        self.direction = 1  # 1 for forward, -1 for backward

        self.get_logger().info('Joint Controller node initialized')

    def joint_state_callback(self, msg):
        # Process joint state messages
        for name, position in zip(msg.name, msg.position):
            if 'arm_joint' in name:
                self.joint_position = position
                self.get_logger().info(f'Current {name} position: {position}')

    def send_command(self):
        # Send a command to move the joint
        msg = Float64()

        # Simple oscillating movement
        if self.direction == 1:
            msg.data = min(self.joint_position + 0.1, 1.57)  # Limit to 90 degrees
        else:
            msg.data = max(self.joint_position - 0.1, -1.57)  # Limit to -90 degrees

        # Change direction when reaching limits
        if msg.data >= 1.57:
            self.direction = -1
        elif msg.data <= -1.57:
            self.direction = 1

        self.joint_pub.publish(msg)
        self.get_logger().info(f'Sending joint command: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    joint_controller = JointController()

    try:
        rclpy.spin(joint_controller)
    except KeyboardInterrupt:
        pass
    finally:
        joint_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Understanding the Code

1. **Node Initialization**: We create a `JointController` class that inherits from `Node`
2. **Publisher**: Creates a publisher to send commands to `/arm_joint/command`
3. **Subscriber**: Subscribes to `/joint_states` to receive feedback
4. **Timer**: Runs the `send_command` function every 0.5 seconds
5. **Callback**: Processes incoming joint state messages

## Creating a ROS 2 Package

To organize your code properly, create a ROS 2 package:

```bash
# Create a new workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create a package for your humanoid robot controller
ros2 pkg create --build-type ament_python humanoid_robot_controller
cd humanoid_robot_controller

# Create the nodes directory
mkdir humanoid_robot_controller
```

Then save your Python file as `humanoid_robot_controller/joint_controller.py` in the package.

## More Complex Example: Multi-Joint Controller

Here's a more sophisticated example that controls multiple joints of a humanoid robot:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import math

class MultiJointController(Node):
    def __init__(self):
        super().__init__('multi_joint_controller')

        # Publisher for multiple joint commands
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_group_position_controller/commands',
            10
        )

        # Subscriber for joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Timer for control loop
        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz
        self.joint_states = JointState()

        # Define joint names for a simple humanoid arm
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_joint'
        ]

        self.get_logger().info('Multi-Joint Controller initialized')

    def joint_state_callback(self, msg):
        self.joint_states = msg

    def control_loop(self):
        # Create a sinusoidal wave pattern for the joints
        msg = Float64MultiArray()

        # Calculate wave pattern for each joint
        current_time = self.get_clock().now().nanoseconds / 1e9
        joint_commands = []

        for i, joint_name in enumerate(self.joint_names):
            # Each joint has a slightly different phase
            phase_offset = i * 0.5
            command = 0.5 * math.sin(current_time + phase_offset)
            joint_commands.append(command)

        msg.data = joint_commands
        self.joint_cmd_pub.publish(msg)

        self.get_logger().info(f'Sending joint commands: {joint_commands}')

def main(args=None):
    rclpy.init(args=args)
    controller = MultiJointController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Running Your Nodes

To run your nodes:

1. Source your ROS 2 installation:
```bash
source /opt/ros/humble/setup.bash  # or your ROS 2 distribution
```

2. Navigate to your workspace and build:
```bash
cd ~/ros2_ws
colcon build --packages-select humanoid_robot_controller
source install/setup.bash
```

3. Run your node:
```bash
ros2 run humanoid_robot_controller joint_controller
```

## Key rclpy Concepts

- **Node**: The basic execution unit in ROS 2
- **Publisher**: Sends messages to topics
- **Subscriber**: Receives messages from topics
- **Timer**: Executes callbacks at regular intervals
- **Services**: For request-response communication
- **Actions**: For long-running tasks with feedback

## Exercises

1. Create a simple rclpy node that publishes the current time
2. Modify the joint controller to accept parameters for movement speed and range
3. Create a subscriber that logs all joint positions to a file
4. Implement a service that allows external nodes to set a target joint position

## Summary

rclpy provides the bridge between Python applications and ROS 2 systems, making it easier to develop humanoid robot controllers. Understanding how to create publishers, subscribers, and timers is essential for controlling humanoid robots effectively.