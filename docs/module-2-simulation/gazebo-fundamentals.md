# G/spazebo Fundamentals

## Introduction to Gazebo

Gazebo is a 3D dynamic simulator designed for robotics applications. It provides high-fidelity physics simulation, realistic rendering, and various sensors that make it an ideal platform for testing robotic algorithms before deploying them on real robots.

## Key Features

### Physics Engine
Gazebo uses advanced physics engines like ODE, Bullet, Simbody, and DART to accurately simulate rigid body dynamics, collisions, and contact forces.

### Sensor Simulation
Gazebo provides realistic simulation of various sensors:
- Camera sensors (monocular, stereo, RGB-D)
- LiDAR and 3D laser scanners
- IMU and accelerometer sensors
- Force/torque sensors
- GPS and magnetometer sensors

### Rendering
The simulator features high-quality rendering capabilities using OGRE, enabling realistic visual simulation for computer vision applications.

## Installation and Setup

### Installing Gazebo
```bash
# On Ubuntu
sudo apt-get install gazebo libgazebo-dev

# Or install the latest version
sudo apt-get install gazebo11 libgazebo11-dev
```

### Basic Commands
```bash
# Launch Gazebo GUI
gazebo

# Launch Gazebo without GUI
gzserver

# Launch Gazebo with a specific world
gazebo empty.world
```

## Basic Concepts

### Worlds
World files define the environment in which robots operate. They contain:
- Physical properties (gravity, atmosphere)
- Models (robots, objects, obstacles)
- Light sources
- Plugins

### Models
Models represent physical objects in the simulation. They include:
- Visual properties (meshes, colors, textures)
- Collision properties
- Inertial properties
- Joints and actuators

### Plugins
Plugins extend Gazebo's functionality:
- Model plugins: Attach to specific models
- World plugins: Affect the entire world
- Sensor plugins: Process sensor data

## Creating Your First Simulation

### Basic World File
```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <!-- A global light source -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- A ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Your robot would go here -->
  </world>
</sdf>
```

### Robot Model (URDF/SDF)
```xml
<?xml version="1.0" ?>
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>
</robot>
```

## Integration with ROS 2

Gazebo integrates seamlessly with ROS 2 through Gazebo ROS packages:

```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

This enables:
- Publishing sensor data to ROS topics
- Controlling robots via ROS services/actions
- Spawning/destroying models through ROS services

## Practical Example: TurtleBot3 Simulation

```bash
# Install TurtleBot3 packages
sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-gazebo

# Set environment variables
export TURTLEBOT3_MODEL=burger

# Launch simulation
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

## Exercises

1. Install Gazebo and run the basic simulation
2. Create a simple world file with obstacles
3. Import a robot model into the simulation
4. Connect the simulated robot to ROS 2 nodes
5. Implement a basic navigation algorithm in simulation