#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
import numpy as np

class PinoRobot(Node):
    def __init__(self):
        super().__init__("pino_robot_node")
        self.get_logger().info("PinoRobot Node Started")

        # load URDF Model
        urdf_path = "/home/george/ros2_ws/src/urdf_4/urdf/Assem1.urdf"
        mesh_dir = ["/home/george/ros2_ws/src/urdf_4/meshes/visual"]

        self.robot = RobotWrapper.BuildFromURDF(urdf_path, mesh_dir)
        self.get_logger().info("URDF model loaded successfully")
        self.run_pinocchio_tests()

    def run_pinocchio_tests(self):        
        q = pin.randomConfiguration(self.robot.model)

        # forward kinematics
        pin.forwardKinematics(self.robot.model, self.robot.data, q)

        # transformation materix of a frame
        frame_name = "Link_2"
        frame_id = self.robot.model.getFrameId(frame_name)

        if frame_id < len(self.robot.data.oMf):
            transform = self.robot.data.oMf[frame_id].homogeneous
            formatted_transform = np.array_str(transform, precision=4, suppress_small=True)
            self.get_logger().info(f"Transform of {frame_name}:\n{formatted_transform}")
        else:
            self.get_logger().error("Frame ID out of range")

        # Jacobian
        jacobian = pin.computeFrameJacobian(
            self.robot.model, self.robot.data, q, frame_id, pin.ReferenceFrame.LOCAL
        )
        self.get_logger().info(f"Jacobian of {frame_name}:\n{np.array_str(jacobian, precision=4, suppress_small=True)}")

        #collisions
        collision_model = self.robot.collision_model
        collision_data = self.robot.collision_data

        pin.computeCollisions(self.robot.model, self.robot.data, collision_model, collision_data, q)

        collision_detected = False
        for i, is_in_collision in enumerate(collision_data.activeCollisionPairs):
            if is_in_collision:
                pair = collision_model.collisionPairs[i]
                self.get_logger().warn(f"Collision detected: {pair.first} <-> {pair.second}")
                collision_detected = True

        if not collision_detected:
            self.get_logger().info("No collisions detected!")

def main(args=None):
    rclpy.init(args=args)
    node = PinoRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
