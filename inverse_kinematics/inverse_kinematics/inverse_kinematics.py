import time
import numpy as np
from geometry_msgs.msg import Pose
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from std_msgs.msg import Int32
from inverse_kinematics_interfaces.action import Posegoal  # Ensure this package is built and sourced


class InverseKinematics(Node):

    def __init__(self):
        """Constructor for the InverseKinematics Node"""
        super().__init__("inverse_kinematics")

        self.setup()

    def setup(self):
        """Sets up subscribers, publishers, and the action server"""
        # Subscriber to handle incoming messages
        self.subscriber = self.create_subscription(
            Int32,
            "/inverse_kinematics/input",
            self.topicCallback,
            qos_profile=10
        )
        self.get_logger().info(f"Subscribed to '{self.subscriber.topic_name}'")

        # Publisher to send outgoing messages
        self.publisher = self.create_publisher(
            Int32,
            "/inverse_kinematics/output",
            qos_profile=10
        )
        self.get_logger().info(f"Publishing to '{self.publisher.topic_name}'")

        # Action server to handle inverse kinematics goals
        self.action_server = ActionServer(
            self,
            Posegoal,
            "/inverse_kinematics/goal",
            execute_callback=self.actionExecute,
            goal_callback=self.actionHandleGoal,
            cancel_callback=self.actionHandleCancel
        )

    def topicCallback(self, msg: Int32):
        """Callback function to process incoming messages"""
        self.get_logger().info(f"Message received: {msg.data}")

    def workspaceSanityCheck(self, goalPose: Pose) -> bool:
        """
        Checks if the given pose is within the manipulator's reachable workspace.

        Args:
            goalPose (Pose): The target pose to check.

        Returns:
            bool: True if the pose is reachable, False otherwise.
        """
        # Define the manipulator's DH parameters
        dh_params = [
            {'a': 0, 'alpha': -np.pi / 2, 'd': 0.76, 'theta_offset': 0},
            {'a': 2.35, 'alpha': 0, 'd': 0, 'theta_offset': 0},
            {'a': 2.30, 'alpha': 0, 'd': 0, 'theta_offset': 0},
            {'a': 0, 'alpha': -np.pi / 2, 'd': 0.85, 'theta_offset': 0},
            {'a': 0, 'alpha': np.pi / 2, 'd': 0.76, 'theta_offset': 0},
            {'a': 0, 'alpha': 0, 'd': 0.43, 'theta_offset': 0}
        ]

        # Extract goal position from Pose
        goal_position = np.array([goalPose.position.x, goalPose.position.y, goalPose.position.z])

        # Maximum reach calculation
        max_reach = sum([abs(param['a']) + abs(param['d']) for param in dh_params])

        # Distance to the goal position
        distance_to_goal = np.linalg.norm(goal_position)

        # Check if the position is within reach
        if distance_to_goal > max_reach:
            self.get_logger().warn("Position is outside the reachable workspace")
            return False

        # Additional checks for orientation can be added if needed
        self.get_logger().info("Pose is within the reachable workspace")
        return True

    def actionHandleGoal(self, goal: Posegoal.Goal) -> GoalResponse:
        """Processes action goal requests"""
        self.get_logger().info(f"Received goal pose: {goal.pose}")
        if not self.workspaceSanityCheck(goal.pose):
            self.get_logger().warn("Goal pose is invalid or unreachable")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def actionHandleCancel(self, goal: Posegoal.Goal) -> CancelResponse:
        """Processes action cancel requests"""
        self.get_logger().info("Cancel request received for action goal")
        return CancelResponse.ACCEPT

    async def actionExecute(self, goal: Posegoal.Goal) -> Posegoal.Result:
        """
        Executes the action goal.

        Args:
            goal (Posegoal.Goal): The target pose.

        Returns:
            Posegoal.Result: Result of the action execution.
        """
        self.get_logger().info(f"Executing action for goal pose: {goal.pose}")

        # Simulate IK computation
        feedback = Posegoal.Feedback()
        result = Posegoal.Result()

        # Feedback simulation
        for progress in range(0, 101, 20):  # Simulate progress updates
            feedback.progress = float(progress)
            goal.publish_feedback(feedback)
            self.get_logger().info(f"Progress: {feedback.progress}%")
            await rclpy.sleep(0.5)  # Simulate computation delay

        # Simulate a successful IK solution
        result.success = True
        self.get_logger().info("Action goal successfully completed")
        goal.succeed()
        return result


def main():
    """Main function to run the ROS 2 node"""
    rclpy.init()
    node = InverseKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()