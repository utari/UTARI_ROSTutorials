#!usr/bin/env python3

#------------------------------------------------------------
# Importing Packages
#------------------------------------------------------------
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from turtlesim.msg import Pose
import math

#------------------------------------------------------------
# In ROS2, we follow an Object-Oriented Programming (OOP) approach. 
# Therefore, we define a class that inherits from the "Node" class in the rclpy library.
#------------------------------------------------------------
class PathNav(Node):
    #------------------------------------------------------------
    # Defining class attributes. These attributes can later be be accessed and overwritten by instance (object-level) attributes.
    #------------------------------------------------------------
    STOP=True
    curPose=Pose()
    desPose=Pose2D()
    cmd_vel=Twist()

    # OOP deals with constructors which is special functions starting with double underscores (dunder methods). 
    # We initialize most of our variables in the constructor since it runs automatically when the program starts.
    def __init__(self):
        super().__init__("Controller_Node")  # (self.) refers to the current instance of the class
        self.get_logger().info("Starting turtle simulator...")
        self.vel_pub= self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.create_subscription(Pose2D, "/turtle1/PositionCommand", self.comPoseCallback, 10)
        self.create_subscription(Pose, "/turtle1/pose", self.curPoseCallback, 10)
        self.create_timer(0.5, self.calculate)
        self.get_logger().info("Connection with Turtlesim for control has been successfully activated")

    # Define what should happen when the given instance or object is destroyed or shut down.
    def __del__(self):
        self.get_logger().info("Connection for control has been disconnceted.....")

    #------------------------------------------------------------
    # All functions defined within the class are called methods.
    #------------------------------------------------------------
    
    # ROS2 Callback function for the commanded (desired) Position from Subscriber or user
    def comPoseCallback(self, comPose_msg:Pose2D):
        self.STOP=False
        self.desPose.x= comPose_msg.x
        self.desPose.y= comPose_msg.y
        self.get_logger().info("Rerouting to new destination.....") #displays new location with coordinates has been given
    
    # ROS2 Callback function for the current Position Subscriber of the topic
    def curPoseCallback(self, curPose_msg: Pose):
        self.curPose.x= curPose_msg.x
        self.curPose.y= curPose_msg.y
        self.curPose.theta=curPose_msg.theta

    #------------------------------------------------------------ 
    # In this control system, we use a Proportional Controller to move the turtle to the desired location. 
    # It calculates the error between the desired and current positions. 
    # The linear error controls the forward velocity, while the angular error controls the rotational/angular velocity.
    #------------------------------------------------------------

    # Function to calculate angular error between the desired and current heading angles
    def getErrorAng(self, curPose, desPose)-> float:
        eX=desPose.x - curPose.x
        eY=desPose.y - curPose.y
        desTheta=math.atan2(eY,eX)
        eTheta=desTheta-curPose.theta
     #   self.get_logger().info(f'destheta: {desTheta}, eTheta: {eTheta}')
        return eTheta
    
    # Function to calculate linear error projected along X axis
    def getErrorLin(self, curPose, desPose)-> float:
        eX=desPose.x - curPose.x
        eY=desPose.y - curPose.y
        eTheta=self.getErrorAng(curPose,desPose)
        eTX=math.hypot(eX,eY)*math.cos(eTheta)
       # self.get_logger().info(f'costheta: {math.cos(eTheta)}')
        return eTX
    
    
#-------------------------------------------------
#   Main Method where we calculate the error and change the velocity and angular direction from Proportional Controller
#-------------------------------------------------
    def calculate(self):
        linPgain=0.5
        angPgain=0.8
        if self.STOP==False:
            errorLin=self.getErrorLin(self.curPose,self.desPose)
            errorAng=self.getErrorAng(self.curPose, self.desPose)
            self.get_logger().info(f'Error linear: {errorLin}, Error Angular: {errorAng}')

            if(errorLin>0):
                self.cmd_vel.linear.x= linPgain * errorLin
            else:
                self.cmd_vel.linear.x=0.0
            self.cmd_vel.angular.z=  angPgain * errorAng
            self.vel_pub.publish(self.cmd_vel)
            if (errorLin==0 and errorAng==0):
                self.get_logger().info("Destination Reached..........")
                self.get_logger().info("Give me new co-ordinates......")

        else:
            self.get_logger().info("Waiting for Co-ordinates...")


#-------------------------------------------------
#   Main Function: This is where we create our instances or object of the class
#-------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    way_path=PathNav()
    rclpy.spin(way_path)
    rclpy.shutdown()

# This ensures the program only program perform certain task only when executed directly, and not when imported as a module.
if __name__=="__main__":
    main()