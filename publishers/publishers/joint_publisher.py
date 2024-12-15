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


        self.move1()
        time.sleep(5)


        print("M10")
        self.movegc()
        time.sleep(5)


        print("M2")

        self.move2()
        time.sleep(5)
        print("M3")

        self.move3()
        time.sleep(5)
        print("Mgo")
        self.movego()
        time.sleep(5)



        print("M4")
        self.move4()
        time.sleep(5)
        print("Mgc")
        self.movegc()
        time.sleep(5)
        print("M5")
        self.move5()
        time.sleep(5)
        print("M6")
        self.move6()
        time.sleep(5)
        print("Mgo")
        self.movego()
        time.sleep(5)
        print("M7")
        self.move7()
        time.sleep(5)
        print("Mgc")
        self.movegc()
        time.sleep(5)
        print("M8")
        self.move8()

        time.sleep(5)
        print("M9")
        self.move9()
        time.sleep(5)
        print("Mgo")
        self.movego()
        time.sleep(5)
        print("home")
        self.home()
        time.sleep(5)
        print("done")

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
        x =  [ 0.00, 0.00]
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


    def move1(self):
        print("Moving Scissors to Pick Position")
        j1 = Float64()
        j2 = Float64()
        j3 = Float64()
        j4 = Float64()
        j5 = Float64()
        j6 = Float64()
        j7 = Float64()
        j8 = Float64()
        x =  [0.20, 0.49, 1.28, 0.05, 1.56, 0.96, 0.00, 0.00]
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
        # self.publisher7.publish(j7)
        # self.publisher8.publish(j8)


    def move2(self):
        print("Moving Scissors Lift Up")
        j1 = Float64()
        j2 = Float64()
        j3 = Float64()
        j4 = Float64()
        j5 = Float64()
        j6 = Float64()
        j7 = Float64()
        j8 = Float64()
        x =  [0.20, 0.25, 1.28, 0.05, 1.56, 0.96, 0.00, 0.00]
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
        # self.publisher7.publish(j7)
        # self.publisher8.publish(j8)



    def move3(self):
        print("Moving Scissors to Place Position")
        j1 = Float64()
        j2 = Float64()
        j3 = Float64()
        j4 = Float64()
        j5 = Float64()
        j6 = Float64()
        j7 = Float64()
        j8 = Float64()
        x =  [2.53, 0.49, 1.28, 0.05, 1.60 ,0.79, 0.00, 0.00]
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
        # self.publisher7.publish(j7)
        # self.publisher8.publish(j8)

    def move4(self):
        print("Moving Syringe to Pick Position")
        j1 = Float64()
        j2 = Float64()
        j3 = Float64()
        j4 = Float64()
        j5 = Float64()
        j6 = Float64()
        j7 = Float64()
        j8 = Float64()
        x =  [1.78, 0.49, 1.30, 0.06, 16.62, 1.87, 0.00, 0.00]
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
        # self.publisher7.publish(j7)
        # self.publisher8.publish(j8)

    def move5(self):
        print("Moving Syringe to Lift up Position")
        j1 = Float64()
        j2 = Float64()
        j3 = Float64()
        j4 = Float64()
        j5 = Float64()
        j6 = Float64()
        j7 = Float64()
        j8 = Float64()
        x =  [1.78, 0.25, 1.30, 0.06, 16.62, 1.87, 0.00, 0.00]
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
        # self.publisher7.publish(j7)
        # self.publisher8.publish(j8)



    def move6(self):
        print("Moving Syringe to Place Position")
        j1 = Float64()
        j2 = Float64()
        j3 = Float64()
        j4 = Float64()
        j5 = Float64()
        j6 = Float64()
        j7 = Float64()
        j8 = Float64()
        x =  [1.39, 0.49, 1.28, 0.06, 1.60, 1.87, 0.00, 0.00]
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
        # self.publisher7.publish(j7)
        # self.publisher8.publish(j8)


    def move7(self):
        print( "Moving Bottle to Pick Position")
        j1 = Float64()
        j2 = Float64()
        j3 = Float64()
        j4 = Float64()
        j5 = Float64()
        j6 = Float64()
        j7 = Float64()
        j8 = Float64()
        x =  [1.65 ,0.49 ,1.28, 0.05, 1.61, 1.58, 0.00, 0.00]
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
        # self.publisher7.publish(j7)
        # self.publisher8.publish(j8)


    def move8(self):
        print( "Moving Bottle to lift up Position")
        j1 = Float64()
        j2 = Float64()
        j3 = Float64()
        j4 = Float64()
        j5 = Float64()
        j6 = Float64()
        j7 = Float64()
        j8 = Float64()
        x =  [1.65 ,0.25, 1.28, 0.05, 1.61, 1.58, 0.00, 0.00]
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
        # self.publisher7.publish(j7)
        # self.publisher8.publish(j8)
    def move9(self):
        print("Moving Bottle to Place Position")
        j1 = Float64()
        j2 = Float64()
        j3 = Float64()
        j4 = Float64()
        j5 = Float64()
        j6 = Float64()
        j7 = Float64()
        j8 = Float64()
        x = [1.73, 0.49, 1.28, 0.05, 1.60, 1.46, 0.00 ,0.00]
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
        # self.publisher7.publish(j7)
        # self.publisher8.publish(j8)

    def movego(self):
        j1 = Float64()
        j2 = Float64()
        j3 = Float64()
        j4 = Float64()
        j5 = Float64()
        j6 = Float64()
        j7 = Float64()
        j8 = Float64()
        x =  [1.73, 0.49, 1.28, 0.05, 1.60, 1.46, 0.00, 0.00]
        j1.data = x[0]
        j2.data = x[1]
        j3.data = x[2]
        j4.data = x[3]
        j5.data = x[4]
        j6.data = x[5]
        j7.data = x[6]
        j8.data = x[7]
        # self.publisher1.publish(j1)
        # self.publisher2.publish(j2)
        # self.publisher3.publish(j3)
        # self.publisher4.publish(j4)
        # self.publisher5.publish(j5)
        # self.publisher6.publish(j6)
        self.publisher7.publish(j7)
        self.publisher8.publish(j8)
    def movegc(self):
        j1 = Float64()
        j2 = Float64()
        j3 = Float64()
        j4 = Float64()
        j5 = Float64()
        j6 = Float64()
        j7 = Float64()
        j8 = Float64()
        x =  [0.00, 0.00, 0.00, 0.00, 0.00, 0.00, -0.038, 0.038]
        j1.data = x[0]
        j2.data = x[1]
        j3.data = x[2]
        j4.data = x[3]
        j5.data = x[4]
        j6.data = x[5]
        j7.data = x[6]
        j8.data = x[7]
        # self.publisher1.publish(j1)
        # self.publisher2.publish(j2)
        # self.publisher3.publish(j3)
        # self.publisher4.publish(j4)
        # self.publisher5.publish(j5)
        # self.publisher6.publish(j6)
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
