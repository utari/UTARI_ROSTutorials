#!/usr/bin/env python

#-------------------------------------------------
#   Package Imports
#-------------------------------------------------
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from turtlesim.msg import Pose
import math

#-------------------------------------------------
#   Global Variable Declaration
#-------------------------------------------------
STOP = True
cmdvel = Twist()
desPose = Pose2D()
curPose = Pose()

#-------------------------------------------------
#   Function Definitions
#-------------------------------------------------
# Function to calculate angular error between the desired and current heading angles
def getErrorAng(curPose, desPose) -> float:
    eX = desPose.x - curPose.x                      # Error in x
    eY = desPose.y - curPose.y                      # Error in y
    desTheta = math.atan2(eY, eX)                   # Desired heading angle
    eTheta = desTheta - curPose.theta               # Error in theta
    return eTheta

# Function to calculate linear error projected along X axis
def getErrorLin(curPose, desPose) -> float:         
    eX = desPose.x - curPose.x                      # Error in x
    eY = desPose.y - curPose.y                      # Error in y
    eTheta = getErrorAng(curPose, desPose)          # Error in theta
    eTX = math.hypot(eX, eY) * math.cos(eTheta)     # Linear error projected along X axis
    return eTX

# ROS Callback function for the commanded (desired) Position Subscriber
def comPoseCallback(comPose_msg):
    #rospy.loginfo("Received command position msg \n")
    global STOP                                     # global variable call (reqd. in Python)
    STOP = False
    desPose.x = comPose_msg.x
    desPose.y = comPose_msg.y
    #cmdvel.angular.z += 1                          # To spin the turtle with increasing angular velocity

# ROS Callback function for the current Position Subscriber
def curPoseCallback(curPose_msg):
    #rospy.loginfo("Received current position msg \n")
    curPose.x = curPose_msg.x
    curPose.y = curPose_msg.y
    curPose.theta = curPose_msg.theta

#-------------------------------------------------
#   Main Function
#-------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('TurtlesimPositionController_pubsub', anonymous = True)         # Initializing ROS node
    twistPub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)          # Register a Publisher for Twist message
    rospy.Subscriber('/turtle1/PositionCommand', Pose2D, comPoseCallback)           # Register a Subscriber to Pose2D message
    rospy.Subscriber('/turtle1/pose', Pose, curPoseCallback)                        # Register a Subscriber to Pose message
    
    loopRate = rospy.Rate(1)                                                        # Loop rate frequency (1 Hz)
    linPGain = 0.2                                                                  # P controller gain for linear error
    angPGain = 0.5                                                                  # P controller gain for angular error
    
    while not rospy.is_shutdown():
        if STOP == False:
            #rospy.loginfo("... Processing ... \n")
            errorLin = getErrorLin(curPose, desPose)                                # Get linear error
            errorAng = getErrorAng(curPose, desPose)                                # Get angular error
            rospy.loginfo("Error linear: %f, Error angular: %f", errorLin, errorAng)
            # This if-else loop ensures that the turtle does not move backwards before starting motion towards the goal point
            if (errorLin > 0):                                                      
                cmdvel.linear.x = linPGain * errorLin                               # P control for linear velocity
            else:                                                                   
                cmdvel.linear.x = 0                                               
            
            cmdvel.angular.z = angPGain * errorAng                                  # P control for angular velocity (yaw)
            twistPub.publish(cmdvel)
            
        else:
            rospy.loginfo("... Waiting ... \n")                                     # Wait for command position to be published
        loopRate.sleep()                                                            # Sleep for reqd. amount of time to maintain the Loop rate frequency
