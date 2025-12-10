# URDF Fundamentals for Humanoid Robots

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML-based format used to describe robot models in ROS. Think of it as a blueprint that defines the physical structure of your humanoid robot, including its links (body parts), joints (connections), and properties (mass, visual appearance).

## Why URDF is Important for Humanoid Robots

Humanoid robots have complex structures with multiple limbs, each with several joints. URDF allows you to:
- Define the physical structure of the robot
- Specify how different parts are connected
- Set visual and collision properties
- Simulate the robot in environments like Gazebo

## Basic URDF Structure

A URDF file contains:
- **Links**: Rigid parts of the robot (like arms, legs, torso)
- **Joints**: Connections between links (like elbows, knees)
- **Materials**: Visual properties (colors, textures)
- **Gazebo plugins**: Simulation-specific configurations

## Simple URDF Example: Single Joint

Let's start with a simple example of a robot with one joint:

```xml
<?xml version="1.0"?>
<robot name="simple_arm">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Arm link -->
  <link name="arm_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.5"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint connecting base and arm -->
  <joint name="arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

## Humanoid Robot URDF Structure

A humanoid robot has a more complex structure. Here's a simplified version:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.5"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="1 0.8 0.6 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Neck joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="2"/>
  </joint>

  <!-- Left arm (simplified) -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="arm_color">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="arm_color">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Shoulder joint -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 1.57"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="2"/>
  </joint>

  <!-- Elbow joint -->
  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.35" effort="15" velocity="2"/>
  </joint>

  <!-- Left hand -->
  <link name="left_hand">
    <visual>
      <geometry>
        <box size="0.1 0.08 0.15"/>
      </geometry>
      <material name="skin"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.08 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_wrist_joint" type="revolute">
    <parent link="left_lower_arm"/>
    <child link="left_hand"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="2"/>
  </joint>
</robot>
```

## Key URDF Elements

### Links
- Define rigid parts of the robot
- Include visual, collision, and inertial properties
- Visual: How the link looks in simulation
- Collision: How the link interacts physically
- Inertial: Physical properties for physics simulation

### Joints
- Connect links together
- Types: revolute (rotational), prismatic (linear), fixed (no movement), etc.
- Define movement limits and physical properties
- Origin: Position and orientation relative to parent

### Materials
- Define colors and textures
- Reusable across multiple links
- RGBA values: Red, Green, Blue, Alpha (transparency)

## URDF Best Practices

1. **Use meaningful names**: Name links and joints descriptively
2. **Proper inertial properties**: Accurate mass and inertia for stable simulation
3. **Consistent units**: Use meters for length, kilograms for mass
4. **Origin placement**: Place origins at logical connection points
5. **Joint limits**: Set realistic limits to prevent damage in simulation

## Loading URDF in ROS 2

To use your URDF in ROS 2, you typically load it into the parameter server:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from urdf_parser_py.urdf import URDF
import os

class URDFLoader(Node):
    def __init__(self):
        super().__init__('urdf_loader')

        # Load URDF from file
        urdf_file = os.path.join(
            os.path.dirname(__file__),
            'urdf',
            'simple_humanoid.urdf'
        )

        with open(urdf_file, 'r') as file:
            urdf_string = file.read()

        # Publish URDF to robot_state_publisher
        self.declare_parameter('robot_description', urdf_string)

        self.get_logger().info('URDF loaded successfully')

def main(args=None):
    rclpy.init(args=args)
    loader = URDFLoader()

    try:
        rclpy.spin(loader)
    except KeyboardInterrupt:
        pass
    finally:
        loader.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Visualizing URDF

To visualize your URDF in RViz:

1. Launch the robot state publisher:
```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:='$(cat urdf/simple_humanoid.urdf)'
```

2. Launch RViz:
```bash
ros2 run rviz2 rviz2
```

3. In RViz, add a RobotModel display and set the topic to `/robot_description`

## Exercises

1. Create a simple URDF file for a robot with a torso, head, and two arms
2. Add visual properties to make your robot look more realistic
3. Modify the joint limits to create different movement ranges
4. Create a launch file to automatically load and visualize your URDF

## Summary

URDF is essential for defining the structure of humanoid robots in ROS 2. By properly defining links and joints, you can create accurate robot models for simulation and control. Understanding URDF is crucial for working with humanoid robots as it forms the basis for all robot modeling and simulation tasks.