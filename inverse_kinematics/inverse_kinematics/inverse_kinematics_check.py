import sys
import numpy as np
from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node


class InverseKinematics(Node):
    def __init__(self):
        """Constructor for the InverseKinematics Node"""
        super().__init__("inverse_kinematics")

        # Define DH parameters for the manipulator
        self.dh_params = [
            {"a": 0, "alpha": -np.pi / 2, "d": 0.76, "theta_offset": 0},
            {"a": 2.35, "alpha": 0, "d": 0, "theta_offset": 0},
            {"a": 2.30, "alpha": 0, "d": 0, "theta_offset": 0},
            {"a": 0, "alpha": -np.pi / 2, "d": 0.85, "theta_offset": 0},
            {"a": 0, "alpha": np.pi / 2, "d": 0.76, "theta_offset": 0},
            {"a": 0, "alpha": 0, "d": 0.43, "theta_offset": 0},
        ]

    def workspaceSanityCheck(self, x: float, y: float, z: float) -> bool:
        """
        Checks if the given point (x, y, z) is within the manipulator's reachable workspace.

        Args:
            x (float): X-coordinate of the point.
            y (float): Y-coordinate of the point.
            z (float): Z-coordinate of the point.

        Returns:
            bool: True if the point is reachable, False otherwise.
        """
        self.get_logger().info(f"Checking point: x={x}, y={y}, z={z}")

        # Calculate distance to the point
        point_position = np.array([x, y, z])
        max_reach = sum([abs(param["a"])/10 + abs(param["d"])/10 for param in self.dh_params])
        distance_to_point = np.linalg.norm(point_position)

        self.get_logger().info(f"Distance to point: {distance_to_point}, Max reach: {max_reach}")

        # Check if the distance is within the manipulator's reach
        if distance_to_point > max_reach:
            self.get_logger().warn("Point is outside the reachable workspace.")
            return False

        self.get_logger().info("Point is within the reachable workspace.")
        return True


def main():
    """Main function to test the InverseKinematics Node"""
    rclpy.init()
    node = InverseKinematics()

    # Check for correct number of command-line arguments
    if len(sys.argv) != 4:
        print("Usage: python3 inverse_kinematics_check.py <x> <y> <z>")
        print("Example: python3 inverse_kinematics_check.py 1.0 2.0 3.0")
        node.destroy_node()
        rclpy.shutdown()
        return

    # Parse the command-line arguments
    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
    except ValueError:
        print("Error: All inputs must be numbers.")
        node.destroy_node()
        rclpy.shutdown()
        return

    # Check if the point is within the reachable workspace
    if node.workspaceSanityCheck(x, y, z):
        print(f"Point ({x}, {y}, {z}) is reachable.")
    else:
        print(f"Point ({x}, {y}, {z}) is NOT reachable.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


