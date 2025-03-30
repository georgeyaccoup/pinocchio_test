# ROS 2 Pinocchio Node

## Overview

This ROS 2 package provides a node that subscribes to a robot description (URDF) and utilizes the [Pinocchio](https://github.com/stack-of-tasks/pinocchio) Rigid Body Dynamics Library for essential robotic computations. These include forward kinematics, Jacobian calculations, and collision detection, which are crucial for motion planning and control.

## Features

- Parses a URDF model and loads it into Pinocchio.
- Computes forward kinematics to determine the transformation of frames.
- Computes the Jacobian matrix for a given frame to analyze motion effects.
- Performs collision detection between defined robot links.

## Usage

Launch the ROS 2 node:

```bash
ros2 run pino pin
```

Ensure the correct URDF ([https://github.com/georgeyaccoup/Color-Tracking-Robot\_URDF.git](https://github.com/georgeyaccoup/Color-Tracking-Robot_URDF.git)) file path is set before executing the node.

## Code Overview and Explanation

### 1. URDF Model Loading

The node initializes by loading the robot's URDF model using `RobotWrapper.BuildFromURDF`, enabling Pinocchio to construct the robot's kinematic and dynamic model.

```python
urdf_path = "/home/george/ros2_ws/src/urdf_4/urdf/Assem1.urdf"
mesh_dir = ["/home/george/ros2_ws/src/urdf_4/meshes/visual"]
self.robot = RobotWrapper.BuildFromURDF(urdf_path, mesh_dir)
```

### 2. Forward Kinematics

Forward kinematics computes the position and orientation of each frame relative to the base. The transformation matrix is extracted for a specified frame.

```python
q = pin.randomConfiguration(self.robot.model)
pin.forwardKinematics(self.robot.model, self.robot.data, q)
frame_name = "Link_2"
frame_id = self.robot.model.getFrameId(frame_name)
transform = self.robot.data.oMf[frame_id].homogeneous
```

### 3. Jacobian Calculation

The Jacobian matrix describes how small changes in joint angles influence the position and orientation of a given frame. This is useful for velocity control and inverse kinematics.

```python
jacobian = pin.computeFrameJacobian(
    self.robot.model, self.robot.data, q, frame_id, pin.ReferenceFrame.LOCAL
)
```

### 4. Collision Detection

Pinocchio’s collision detection system checks for intersections between robot links, identifying any potential collisions that could affect motion.

```python
collision_model = self.robot.collision_model
collision_data = self.robot.collision_data
pin.computeCollisions(self.robot.model, self.robot.data, collision_model, collision_data, q)
```

## Output Example

Example terminal output when executing the node:

```bash
george@george:~$ /bin/python3.10 /home/george/new_ws/src/pino/pino/test.py
[INFO] [1743341516.318226234] [pino_robot_node]: PinoRobot Node Started
[INFO] [1743341516.426705918] [pino_robot_node]: URDF Model Loaded Successfully
[INFO] [1743341516.427348376] [pino_robot_node]: Transform of Link_2:
[[1. 0. 0. 0.]
 [0. 1. 0. 0.]
 [0. 0. 1. 0.]
 [0. 0. 0. 1.]]
[INFO] [1743341516.427701872] [pino_robot_node]: Jacobian of Link_2:
[[ 0.      0.    ]
 [ 0.      0.    ]
 [ 0.      0.    ]
 [ 0.4758  0.    ]
 [ 0.8795  0.    ]
 [ 0.0073 -1.    ]]
[INFO] [1743341516.428041597] [pino_robot_node]: No collisions detected!
```

## Contributors

- **George Read**

## Contact

- **LinkedIn:** [George Yaccoup](https://www.linkedin.com/in/george-yaccoup/)
- **Email:** [georgeyaccoup124@gmail.com](mailto\:georgeyaccoup124@gmail.com)
- **CV:** [View CV](https://drive.google.com/file/d/1Kr5_MOKHBG08xqPxs7G4jBSK3icI1eJk/view?usp=sharing)

