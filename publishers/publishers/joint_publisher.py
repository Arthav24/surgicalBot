import time
from typing import Any, Optional, Union

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float64


class JointPublisher(Node):

    def __init__(self):
        """Constructor"""
        super().__init__("joint_publisher")

        self.subscriber = None
        self.publisher = None

        self.setup()
        time.sleep(5)


        self.scissors_pick_up()
        time.sleep(5)


        self.gripper_close()
        time.sleep(5)

        self.scissors_lift_up()
        time.sleep(5)

        self.scissors_place()
        time.sleep(5)
        
        self.gripper_open()
        time.sleep(5)

        self.syringe_pick_up()
        time.sleep(5)
        
        self.gripper_close()
        time.sleep(5)
        
        self.syringe_lift_up()
        time.sleep(5)
        
        self.syringe_place()
        time.sleep(5)
        
        self.gripper_open()
        time.sleep(5)
        
        self.bottle_pick_up()
        time.sleep(5)
        
        self.gripper_close()
        time.sleep(5)
        
        self.bottle_lift_up()
        time.sleep(5)
        
        self.bottle_place()
        time.sleep(5)
        
        self.gripper_open()
        time.sleep(5)
        
        self.home()
        time.sleep(5)

    def setup(self):
        """Sets up subscribers, publishers, etc. to configure the node"""

        # publisher for publishing outgoing messages
        self.publisher1 = self.create_publisher(Float64,
                                               "/joint_angles/joint1",
                                               qos_profile=10)
        self.publisher2 = self.create_publisher(Float64,
                                               "/joint_angles/joint2",
                                               qos_profile=10)
        self.publisher3 = self.create_publisher(Float64,
                                               "/joint_angles/joint3",
                                               qos_profile=10)
        self.publisher4 = self.create_publisher(Float64,
                                               "/joint_angles/joint4",
                                               qos_profile=10)
        self.publisher5 = self.create_publisher(Float64,
                                               "/joint_angles/joint5",
                                               qos_profile=10)
        self.publisher6 = self.create_publisher(Float64,
                                               "/joint_angles/joint6",
                                               qos_profile=10)
        self.publisher7 = self.create_publisher(Float64,
                                               "/joint_angles/gripper_left_joint",
                                               qos_profile=10)
        self.publisher8 = self.create_publisher(Float64,
                                               "/joint_angles/gripper_right_joint",
                                               qos_profile=10)



    def home(self):
        j1 = Float64()
        j2 = Float64()
        j3 = Float64()
        j4 = Float64()
        j5 = Float64()
        j6 = Float64()
        j7 = Float64()
        j8 = Float64()
        
        j1.data = 0.0
        j2.data = 0.0
        j3.data = 0.0
        j4.data = 0.0
        j5.data = 0.0
        j6.data = 0.0
        j7.data = 0.0
        j8.data = 0.0
        
        self.publisher1.publish(j1)
        self.publisher2.publish(j2)
        self.publisher3.publish(j3)
        self.publisher4.publish(j4)
        self.publisher5.publish(j5)
        self.publisher6.publish(j6)
        self.publisher6.publish(j7)
        self.publisher6.publish(j8)


    def scissors_pick_up(self):
        print("Moving Scissors to Pick Position")
        j1 = Float64()
        j2 = Float64()
        j3 = Float64()
        j4 = Float64()
        j5 = Float64()
        j6 = Float64()
        j7 = Float64()
        j8 = Float64()
        
        x =  [2.72, 0.49, 1.28, 0.05, 1.56, 0.96, 0.00, 0.00]
        
        j1.data = x[0]
        j2.data = x[1]
        j3.data = x[2]
        j4.data = x[3]
        j5.data = x[4]
        j6.data = x[5]
        j7.data = x[6]
        j8.data = x[7]
        self.publisher1.publish(j1)
        self.publisher2.publish(j2)
        self.publisher3.publish(j3)
        self.publisher4.publish(j4)
        self.publisher5.publish(j5)
        self.publisher6.publish(j6)


    def scissors_lift_up(self):
        print("Moving Scissors Lift Up")
        j1 = Float64()
        j2 = Float64()
        j3 = Float64()
        j4 = Float64()
        j5 = Float64()
        j6 = Float64()
        j7 = Float64()
        j8 = Float64()
        
        x =  [2.72, 0.25, 1.28, 0.05, 1.56, 0.96, 0.00, 0.00]
        
        j1.data = x[0]
        j2.data = x[1]
        j3.data = x[2]
        j4.data = x[3]
        j5.data = x[4]
        j6.data = x[5]
        j7.data = x[6]
        j8.data = x[7]
        self.publisher1.publish(j1)
        self.publisher2.publish(j2)
        self.publisher3.publish(j3)
        self.publisher4.publish(j4)
        self.publisher5.publish(j5)
        self.publisher6.publish(j6)

    def scissors_place(self):
        print("Moving Scissors to Place Position")
        j1 = Float64()
        j2 = Float64()
        j3 = Float64()
        j4 = Float64()
        j5 = Float64()
        j6 = Float64()
                
        j1.data = 0.09
        j2.data = 0.49
        j3.data = 1.22
        j4.data = 0.19
        j5.data = 1.56
        j6.data = 1.26
        
        self.publisher1.publish(j1)
        self.publisher2.publish(j2)
        self.publisher3.publish(j3)
        self.publisher4.publish(j4)
        self.publisher5.publish(j5)
        self.publisher6.publish(j6)

    def syringe_pick_up(self):
        print("Moving Syringe to Pick Position")
        j1 = Float64()
        j2 = Float64()
        j3 = Float64()
        j4 = Float64()
        j5 = Float64()
        j6 = Float64()
        
        j1.data = 0.94
        j2.data = 0.49
        j3.data = 1.22
        j4.data = 0.19
        j5.data = 1.56
        j6.data = 1.84
        
        self.publisher1.publish(j1)
        self.publisher2.publish(j2)
        self.publisher3.publish(j3)
        self.publisher4.publish(j4)
        self.publisher5.publish(j5)
        self.publisher6.publish(j6)

    def syringe_lift_up(self):
        print("Moving Syringe to Lift up Position")
        j1 = Float64()
        j2 = Float64()
        j3 = Float64()
        j4 = Float64()
        j5 = Float64()
        j6 = Float64()
        
        j1.data = 0.94
        j2.data = 0.39
        j3.data = 1.22
        j4.data = 0.19
        j5.data = 1.56
        j6.data = 1.84
        
        self.publisher1.publish(j1)
        self.publisher2.publish(j2)
        self.publisher3.publish(j3)
        self.publisher4.publish(j4)
        self.publisher5.publish(j5)
        self.publisher6.publish(j6)

    def syringe_place(self):
        print("Moving Syringe to Place Position")
        j1 = Float64()
        j2 = Float64()
        j3 = Float64()
        j4 = Float64()
        j5 = Float64()
        j6 = Float64()
        
        j1.data = -2.16
        j2.data = 0.62
        j3.data = 1.13
        j4.data = 0.19
        j5.data = 1.62
        j6.data = 2.34
        
        self.publisher1.publish(j1)
        self.publisher2.publish(j2)
        self.publisher3.publish(j3)
        self.publisher4.publish(j4)
        self.publisher5.publish(j5)
        self.publisher6.publish(j6)

    def bottle_pick_up(self):
        print( "Moving Bottle to Pick Position")
        j1 = Float64()
        j2 = Float64()
        j3 = Float64()
        j4 = Float64()
        j5 = Float64()
        j6 = Float64()
        
        j1.data = 1.38
        j2.data = 0.43
        j3.data = 1.38
        j4.data = 0.20
        j5.data = 1.56
        j6.data = 1.52
        
        self.publisher1.publish(j1)
        self.publisher2.publish(j2)
        self.publisher3.publish(j3)
        self.publisher4.publish(j4)
        self.publisher5.publish(j5)
        self.publisher6.publish(j6)


    def bottle_lift_up(self):
        print( "Moving Bottle to lift up Position")
        j1 = Float64()
        j2 = Float64()
        j3 = Float64()
        j4 = Float64()
        j5 = Float64()
        j6 = Float64()
        
        j1.data = 1.38
        j2.data = 0.36
        j3.data = 1.38
        j4.data = 0.20
        j5.data = 1.76
        j6.data = 1.52
        
        self.publisher1.publish(j1)
        self.publisher2.publish(j2)
        self.publisher3.publish(j3)
        self.publisher4.publish(j4)
        self.publisher5.publish(j5)
        self.publisher6.publish(j6)
        
    def bottle_place(self):
        print("Moving Bottle to Place Position")
        j1 = Float64()
        j2 = Float64()
        j3 = Float64()
        j4 = Float64()
        j5 = Float64()
        j6 = Float64()
        
        j1.data = -1.53
        j2.data = 0.46
        j3.data = 1.38
        j4.data = 0.20
        j5.data = 1.76
        j6.data = 1.52
        
        self.publisher1.publish(j1)
        self.publisher2.publish(j2)
        self.publisher3.publish(j3)
        self.publisher4.publish(j4)
        self.publisher5.publish(j5)
        self.publisher6.publish(j6)
        

    def gripper_open(self):
        print("Gripper Open")
        j7 = Float64()
        j8 = Float64()
        
        j7.data = 0.0
        j8.data = 0.0
        
        self.publisher7.publish(j7)
        self.publisher8.publish(j8)
        
    def gripper_close(self):
        print("Gripper Close")
        j7 = Float64()
        j8 = Float64()
        
        j7.data = -0.038
        j8.data = 0.038
        
        self.publisher7.publish(j7)
        self.publisher8.publish(j8)


def main():

    rclpy.init()
    node = JointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
