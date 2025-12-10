# ROS 2 Communication Patterns

## Overview

Communication is the backbone of any robotic system. In ROS 2, there are several communication patterns that allow different nodes to exchange information. Understanding these patterns is crucial for building humanoid robots where multiple systems need to coordinate effectively.

## Communication Patterns in ROS 2

### 1. Topics (Publish/Subscribe)

Topics enable asynchronous, many-to-many communication. Publishers send messages to a topic, and any number of subscribers can receive those messages.

**Use Cases for Humanoid Robots:**
- Sensor data distribution (camera images, IMU readings)
- Robot state broadcasting (joint positions, battery status)
- Command distribution (walking patterns, gesture commands)

**Example: Publishing Joint States**

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10Hz
        self.time = 0.0

    def publish_joint_states(self):
        msg = JointState()
        msg.name = ['hip_joint', 'knee_joint', 'ankle_joint']

        # Simulate oscillating joint positions
        self.time += 0.1
        msg.position = [
            0.5 * math.sin(self.time),           # hip
            0.3 * math.sin(self.time + 0.5),     # knee
            0.2 * math.sin(self.time + 1.0)      # ankle
        ]

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        self.publisher.publish(msg)
        self.get_logger().info(f'Published joint states: {msg.position}')

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Example: Subscribing to Joint States**

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def joint_state_callback(self, msg):
        self.get_logger().info(f'Received joint positions: {msg.position}')

        # Process joint data for control decisions
        if len(msg.position) > 0:
            hip_pos = msg.position[0]
            # Implement control logic based on joint positions
            if abs(hip_pos) > 0.8:
                self.get_logger().warn('Hip joint approaching limit!')

def main(args=None):
    rclpy.init(args=args)
    node = JointStateSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Services (Request/Response)

Services provide synchronous, one-to-one communication. A client sends a request and waits for a response from the server.

**Use Cases for Humanoid Robots:**
- Calibration procedures
- Mode switching
- Complex computation requests
- Emergency stop activation

**Example: Service Server for Joint Calibration**

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class JointCalibrationService(Node):
    def __init__(self):
        super().__init__('joint_calibration_service')
        self.srv = self.create_service(
            SetBool,
            'calibrate_joints',
            self.calibrate_joints_callback
        )
        self.get_logger().info('Joint calibration service ready')

    def calibrate_joints_callback(self, request, response):
        self.get_logger().info(f'Calibration requested: {request.data}')

        # Simulate calibration process
        if request.data:  # If calibration is requested
            self.get_logger().info('Starting joint calibration...')
            # In a real robot, this would move joints to calibration positions
            # and set encoders to zero
            response.success = True
            response.message = 'Joints calibrated successfully'
        else:
            response.success = False
            response.message = 'Calibration cancelled'

        return response

def main(args=None):
    rclpy.init(args=args)
    node = JointCalibrationService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Example: Service Client for Joint Calibration**

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class CalibrationClient(Node):
    def __init__(self):
        super().__init__('calibration_client')
        self.cli = self.create_client(SetBool, 'calibrate_joints')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = SetBool.Request()

    def send_calibration_request(self, calibrate):
        self.req.data = calibrate
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        try:
            response = self.future.result()
            self.get_logger().info(f'Result: {response.success}, {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    client = CalibrationClient()

    # Send calibration request
    client.send_calibration_request(True)

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. Actions (Goal/Feedback/Result)

Actions are for long-running tasks that require feedback and the ability to cancel. They're perfect for humanoid robot behaviors that take time to complete.

**Use Cases for Humanoid Robots:**
- Walking to a location
- Grasping an object
- Executing a complex movement sequence
- Navigation tasks

**Example: Action Server for Walking**

```python
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from example_interfaces.action import Fibonacci

class WalkActionServer(Node):
    def __init__(self):
        super().__init__('walk_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,  # Using Fibonacci as an example; in practice, you'd define your own action
            'walk_to_goal',
            self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_service_qos_profile=rclpy.qos.qos_profile_services_default,
            result_service_qos_profile=rclpy.qos.qos_profile_services_default,
            cancel_service_qos_profile=rclpy.qos.qos_profile_services_default,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        self.get_logger().info('Walk action server ready')

    def goal_callback(self, goal_request):
        # Accept all goals
        self.get_logger().info('Received goal request')
        return rclpy.action.server.GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        # Accept all cancel requests
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # Simulate walking progress
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            # Check if there's a cancel request
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            # Simulate walking step
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1]
            )

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')

            # Sleep to simulate walking time
            time.sleep(0.5)

        # Complete the goal
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info(f'Result: {result.sequence}')

        return result

def main(args=None):
    rclpy.init(args=args)
    node = WalkActionServer()

    try:
        executor = MultiThreadedExecutor()
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    import time
    main()
```

## Quality of Service (QoS) Settings

QoS settings allow you to configure how messages are delivered:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# For sensor data (best effort, volatile)
sensor_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE
)

# For critical commands (reliable, transient local)
command_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)

# Example usage
self.sensor_sub = self.create_subscription(
    LaserScan,
    '/scan',
    self.scan_callback,
    qos_profile=sensor_qos
)
```

## Communication Design Patterns for Humanoid Robots

### 1. Publish Robot State, Subscribe to Commands
- Multiple nodes can monitor robot state
- Centralized command processing
- Decoupled architecture

### 2. Hierarchical Control
- High-level planners send goals to low-level controllers
- Feedback flows upward
- Clear separation of concerns

### 3. Event-Driven Architecture
- Nodes react to specific events
- Reduces unnecessary communication
- Improves responsiveness

## Exercises

1. Create a publisher that simulates IMU data from a humanoid robot
2. Implement a service that allows external nodes to query robot battery status
3. Design a simple action for moving the robot's arm to a specific position
4. Implement a node that subscribes to multiple sensor topics and fuses the data

## Summary

Understanding ROS 2 communication patterns is essential for creating coordinated humanoid robot behaviors. Topics provide asynchronous data distribution, services offer synchronous request/response communication, and actions handle long-running tasks with feedback. By choosing the right pattern for each interaction, you can build robust and responsive humanoid robot systems.