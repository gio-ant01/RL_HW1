# Armando Robot Manipulator

ROS 2 Humble package for controlling the Armando robot manipulator in Gazebo simulation and RViz visualization.

## 📋 Prerequisites

Ensure you have ROS 2 Humble installed on your system.

## 🔨 Build

### Install Required Packages
```bash
sudo apt install ros-humble-urdf-launch
sudo apt install ros-humble-controller-manager
sudo apt install ros-humble-ign-ros2-control
sudo apt install ros-humble-xacro
sudo apt install ros-humble-rqt-image-view
```

### Build the Workspace

Navigate to your ROS 2 workspace and build the packages:
```bash
cd ~/ros2_ws
colcon build --packages-select armando_description armando_gazebo armando_controller
```

### Source the Setup Files
```bash
source install/setup.bash
```

## ✅ Usage

### Visualization in RViz

Launch the robot description to visualize in RViz:
```bash
ros2 launch armando_description armando_display.launch.py
```

### Simulation and Control in Gazebo
At first:
```bash
cd ~/ros2_ws/src
```

#### Option 1: Position Controller

Launch Gazebo with position controller:
```bash
ros2 launch armando_gazebo armando_world.launch.py controller_type:=position
```

In another terminal, run the controller node:
```bash
ros2 run armando_controller arm_controller_node --ros-args -p controller_type:=position
```

#### Option 2: Trajectory Controller

Launch Gazebo with trajectory controller:
```bash
ros2 launch armando_gazebo armando_world.launch.py controller_type:=trajectory
```

In another terminal, run the controller node:
```bash
ros2 run armando_controller arm_controller_node --ros-args -p controller_type:=trajectory
```

### Camera Visualization

To view the camera feed:
```bash
ros2 run rqt_image_view rqt_image_view /armando/camera
```

## 🎮 Manual Control

### Position Controller

Publish position commands manually, after launching position controller:
```bash
ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.5, 0.5, 0.5]}"
```

### Trajectory Controller

Publish trajectory commands manually, after launching trajectory controller:
```bash
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['j0', 'j1', 'j2', 'j3'],
  points: [
    {
      positions: [0.0, 0.5, 0.5, 0.5],
      velocities: [0.0, 0.0, 0.0, 0.0],
      time_from_start: {sec: 2, nanosec: 0}
    }
  ]
}"
```

## 📦 Package Structure
```
├── armando_description/     # URDF/XACRO robot description, meshes, RViz config
├── armando_gazebo/          # Gazebo launch files and world configuration
└── armando_controller/      # C++ controller node for position and trajectory control
```

## 🔧 Troubleshooting

### Check Active Controllers
```bash
ros2 control list_controllers
```

### View Node Graph
```bash
rqt_graph
```

### List Available Topics
```bash
ros2 topic list
```

## 📝 License

Apache-2.0
