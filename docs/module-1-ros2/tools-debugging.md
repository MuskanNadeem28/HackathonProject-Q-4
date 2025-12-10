# ROS 2 Tools & Debugging

## Introduction

Working with humanoid robots requires effective debugging and monitoring tools. ROS 2 provides several powerful tools to help you understand what's happening in your robot system, identify issues, and optimize performance.

## Essential ROS 2 Command Line Tools

### ros2 topic

The `ros2 topic` command allows you to inspect and interact with topics in your ROS 2 system.

**View all topics:**
```bash
ros2 topic list
```

**Get information about a specific topic:**
```bash
ros2 topic info /joint_states
```

**Echo messages from a topic:**
```bash
ros2 topic echo /joint_states
```

**Publish a message to a topic:**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.2}}'
```

### ros2 service

Use `ros2 service` to interact with services.

**List all services:**
```bash
ros2 service list
```

**Call a service:**
```bash
ros2 service call /calibrate_joints example_interfaces/srv/SetBool '{data: true}'
```

### ros2 action

For working with actions:

**List all actions:**
```bash
ros2 action list
```

**Send a goal to an action:**
```bash
ros2 action send_goal /walk_to_goal example_interfaces/action/Fibonacci '{order: 5}'
```

### ros2 node

To work with nodes:

**List all nodes:**
```bash
ros2 node list
```

**Get information about a specific node:**
```bash
ros2 node info /joint_state_publisher
```

### ros2 param

To work with parameters:

**List parameters for a node:**
```bash
ros2 param list /robot_state_publisher
```

**Get a specific parameter:**
```bash
ros2 param get /robot_state_publisher use_sim_time
```

## Debugging Techniques for Humanoid Robots

### 1. Topic Monitoring

Monitor sensor topics to verify data is flowing correctly:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class TopicMonitor(Node):
    def __init__(self):
        super().__init__('topic_monitor')

        # Monitor joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        # Monitor IMU data
        self.imu_sub = self.create_subscription(
            sensor_msgs.msg.Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.get_logger().info('Topic monitor started')

    def joint_callback(self, msg):
        # Log joint information
        self.get_logger().debug(f'Joint names: {msg.name}')
        self.get_logger().debug(f'Joint positions: {msg.position}')

        # Check for unexpected values
        for i, pos in enumerate(msg.position):
            if abs(pos) > 3.14:  # Unusually large joint angle
                self.get_logger().warn(f'Large joint angle detected: {msg.name[i]} = {pos}')

    def imu_callback(self, msg):
        # Check IMU data for plausibility
        linear_accel = (msg.linear_acceleration.x**2 +
                       msg.linear_acceleration.y**2 +
                       msg.linear_acceleration.z**2)**0.5

        if linear_accel > 20.0:  # Unusually high acceleration
            self.get_logger().warn(f'High acceleration detected: {linear_accel}')

def main(args=None):
    rclpy.init(args=args)
    monitor = TopicMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Logging Best Practices

Proper logging is crucial for debugging humanoid robots:

```python
import rclpy
from rclpy.node import Node

class WellLoggedNode(Node):
    def __init__(self):
        super().__init__('well_logged_node')

        # Use different log levels appropriately
        self.get_logger().info('Node initialized')
        self.get_logger().debug('Detailed debug information')
        self.get_logger().warn('Warning message')

        # Log important state changes
        self.state = 'idle'
        self.change_state('active')

    def change_state(self, new_state):
        self.get_logger().info(f'State changed from {self.state} to {new_state}')
        self.state = new_state

        # Log important parameters
        self.declare_parameter('control_frequency', 100)
        freq = self.get_parameter('control_frequency').value
        self.get_logger().debug(f'Control frequency set to {freq}Hz')

def main(args=None):
    rclpy.init(args=args)
    node = WellLoggedNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. Using rqt Tools

rqt is a Qt-based framework that provides GUI tools for monitoring and debugging:

```bash
# Launch rqt with various plugins
rqt

# Launch specific plugins
rqt_graph                    # Shows node connections
rqt_plot                    # Plots topic data over time
rqt_console                 # Shows log messages
rqt_bag                     # Records and plays back data
rqt_publisher              # Publishes messages manually
```

### 4. TF (Transform) Debugging

For humanoid robots, transforms between different body parts are critical:

```bash
# View the TF tree
ros2 run tf2_tools view_frames

# Echo a specific transform
ros2 run tf2_ros tf2_echo map base_link

# View TF in RViz
ros2 run rviz2 rviz2
```

## Common Debugging Scenarios in Humanoid Robots

### 1. Joint Control Issues

When joints aren't responding as expected:

```bash
# Check if joint state messages are being published
ros2 topic echo /joint_states

# Verify the joint controller is running
ros2 node list | grep controller

# Check controller parameters
ros2 param list /position_controllers
```

### 2. Sensor Data Problems

When sensor data seems incorrect:

```bash
# Monitor sensor topics
ros2 topic echo /imu/data
ros2 topic echo /camera/image_raw

# Check for dropped messages
ros2 topic hz /camera/image_raw
```

### 3. Communication Delays

When there are delays in the system:

```bash
# Monitor topic frequency
ros2 topic hz /joint_states

# Test service response time
time ros2 service call /some_service example_interfaces/srv/SetBool '{data: true}'
```

## Advanced Debugging with Python

### Creating a Debug Node

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time

class DebugNode(Node):
    def __init__(self):
        super().__init__('debug_node')

        # Subscribe to joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        # Subscribe to commands
        self.cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/joint_commands',
            self.command_callback,
            10
        )

        # Track timing
        self.last_joint_time = time.time()
        self.last_cmd_time = time.time()

        # Timer for periodic checks
        self.timer = self.create_timer(1.0, self.periodic_check)

    def joint_callback(self, msg):
        current_time = time.time()
        dt = current_time - self.last_joint_time
        self.last_joint_time = current_time

        # Log timing information
        if dt > 0.2:  # Expected frequency is 10Hz, so 0.1s interval
            self.get_logger().warn(f'Joint state delay: {dt:.3f}s')

        # Log joint positions
        self.get_logger().debug(f'Joint positions: {msg.position}')

    def command_callback(self, msg):
        current_time = time.time()
        dt = current_time - self.last_cmd_time
        self.last_cmd_time = current_time

        if dt > 0.05:  # Expected frequency is 20Hz, so 0.05s interval
            self.get_logger().warn(f'Command delay: {dt:.3f}s')

        self.get_logger().debug(f'Command values: {msg.data}')

    def periodic_check(self):
        current_time = time.time()

        # Check if we're receiving messages
        joint_age = current_time - self.last_joint_time
        cmd_age = current_time - self.last_cmd_time

        if joint_age > 1.0:
            self.get_logger().error('No joint states received in 1 second')
        if cmd_age > 1.0:
            self.get_logger().error('No commands received in 1 second')

def main(args=None):
    rclpy.init(args=args)
    debug_node = DebugNode()

    try:
        rclpy.spin(debug_node)
    except KeyboardInterrupt:
        pass
    finally:
        debug_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Performance Optimization Tips

### 1. Reduce Message Frequency

For debugging, you might want to reduce the frequency of high-bandwidth topics:

```python
from functools import partial

class ThrottledNode(Node):
    def __init__(self):
        super().__init__('throttled_node')

        # Subscribe to high-frequency data
        self.sub = self.create_subscription(
            sensor_msgs.msg.Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Throttle processing to 5Hz
        self.process_timer = self.create_timer(0.2, self.process_throttled)
        self.pending_image = None

    def image_callback(self, msg):
        # Store the latest image
        self.pending_image = msg

    def process_throttled(self):
        if self.pending_image is not None:
            # Process the image at reduced frequency
            self.get_logger().info('Processing image')
            # Your processing code here
            self.pending_image = None
```

### 2. Use Proper QoS Settings

Match QoS settings to your application needs:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# For critical commands
critical_qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST
)

# For sensor data
sensor_qos = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST
)
```

## Exercises

1. Create a node that monitors and logs all joint positions with timestamps
2. Implement a service that returns the current status of all robot subsystems
3. Use rqt_plot to visualize joint positions over time
4. Create a debugging node that checks for common issues in a humanoid robot system

## Summary

Effective debugging is crucial for developing reliable humanoid robots. ROS 2 provides a rich set of tools for monitoring, debugging, and optimizing your robot system. Understanding how to use these tools and implementing good logging practices will save you significant time when developing and maintaining complex humanoid robot systems.