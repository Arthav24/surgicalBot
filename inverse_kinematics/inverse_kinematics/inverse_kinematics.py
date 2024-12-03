import time
from typing import Any, Optional, Union

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from std_msgs.msg import Int32
from inverse_kinematics_interfaces.action import Fibonacci


class InverseKinematics(Node):

    def __init__(self):
        """Constructor"""

        super().__init__("inverse_kinematics")

        self.subscriber = None
        self.publisher = None

        self.setup()

    def setup(self):
        """Sets up subscribers, publishers, etc. to configure the node"""

        # subscriber for handling incoming messages
        self.subscriber = self.create_subscription(Int32,
                                                   "~/input",
                                                   self.topicCallback,
                                                   qos_profile=10)
        self.get_logger().info(f"Subscribed to '{self.subscriber.topic_name}'")

        # publisher for publishing outgoing messages
        self.publisher = self.create_publisher(Int32,
                                               "~/output",
                                               qos_profile=10)
        self.get_logger().info(f"Publishing to '{self.publisher.topic_name}'")

        # action server for handling action goal requests
        self.action_server = ActionServer(
            self,
            Fibonacci,
            "~/action",
            execute_callback=self.actionExecute,
            goal_callback=self.actionHandleGoal,
            cancel_callback=self.actionHandleCancel,
            handle_accepted_callback=self.actionHandleAccepted
        )

    def topicCallback(self, msg: Int32):
        """Processes messages received by a subscriber

        Args:
            msg (Int32): message
        """

        self.get_logger().info(f"Message received: '{msg.data}'")

    def actionHandleGoal(self, goal: Fibonacci.Goal) -> GoalResponse:
        """Processes action goal requests

        Args:
            goal (Fibonacci.Goal): action goal

        Returns:
            GoalResponse: goal response
        """

        self.get_logger().info("Received action goal request")

        return GoalResponse.ACCEPT

    def actionHandleCancel(self, goal: Fibonacci.Goal) -> CancelResponse:
        """Processes action cancel requests

        Args:
            goal (Fibonacci.Goal): action goal

        Returns:
            CancelResponse: cancel response
        """

        self.get_logger().info("Received request to cancel action goal")

        return CancelResponse.ACCEPT

    def actionHandleAccepted(self, goal: Fibonacci.Goal):
        """Processes accepted action goal requests

        Args:
            goal (Fibonacci.Goal): action goal
        """

        # execute action in a separate thread to avoid blocking
        goal.execute()

    async def actionExecute(self, goal: Fibonacci.Goal) -> Fibonacci.Result:
        """Executes an action

        Args:
            goal (Fibonacci.Goal): action goal

        Returns:
            Fibonacci.Result: action goal result
        """

        self.get_logger().info("Executing action goal")

        # define a sleeping rate between computing individual Fibonacci numbers
        loop_rate = 1

        # create the action feedback and result
        feedback = Fibonacci.Feedback()
        result = Fibonacci.Result()

        # initialize the Fibonacci sequence
        feedback.partial_sequence = [0, 1]

        # compute the Fibonacci sequence up to the requested order n
        for i in range(1, goal.request.order):

            # cancel, if requested
            if goal.is_cancel_requested:
                result.sequence = feedback.partial_sequence
                goal.canceled()
                self.get_logger().info("Action goal canceled")
                return result

            # compute the next Fibonacci number
            feedback.partial_sequence.append(feedback.partial_sequence[i] + feedback.partial_sequence[i-1])

            # publish the current sequence as action feedback
            goal.publish_feedback(feedback)
            self.get_logger().info("Publishing action feedback")

            # sleep before computing the next Fibonacci number
            time.sleep(loop_rate)

        # finish by publishing the action result
        if rclpy.ok():
            result.sequence = feedback.partial_sequence
            goal.succeed()
            self.get_logger().info("Goal succeeded")

        return result


def main():

    rclpy.init()
    node = InverseKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
