#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/Pose2D.h"										// to get desired position command
#include "turtlesim/Pose.h"												// to read current position
#include "turtlesim/Velocity.h"											// to send velocity command

void ComPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
void CurPoseCallback(const turtlesim::Pose::ConstPtr& msg);
float GetErrorLin(turtlesim::Pose curpose, geometry_msgs::Pose2D despose);
float GetErrorAng(turtlesim::Pose curpose, geometry_msgs::Pose2D despose);


geometry_msgs::Pose2D DesPose;											// variable to hold desired pose
bool STOP = true;														// to hold stop flag, wait till first command given
turtlesim::Velocity CmdVel;
turtlesim::Pose CurPose;												// to hold current pose

int main(int argc, char **argv)
{
	ros::init(argc, argv, "LookatAroundModes_sub");						// connect to roscore
	
	ros::NodeHandle n;													// node object
	// register sub to get desired position/pose commands
	ros::Subscriber ComPose_sub = n.subscribe("/turtlesim/PositionCommand", 5, ComPoseCallback);
	// register sub to get current position/pose
	ros::Subscriber CurPose_sub = n.subscribe("/turtle1/pose", 5, CurPoseCallback);
	// register pub to send twist velocity (cmd_vel)
	ros::Publisher Twist_pub = n.advertise<turtlesim::Velocity>("/turtle1/command_velocity", 100);
	
	ros::Rate loop_rate(10);											// freq to run loops in (10 Hz)
	float ErrorLin = 0;
	float ErrorAng = 0;
	
	
	ROS_INFO("Ready to send position commands");						// let user know we are ready and good
	while (ros::ok() && n.ok() )										// while ros and the node are ok
	{
		ros::spinOnce();
		if (STOP == false)												// and no stop command
		{
			ErrorLin = GetErrorLin(CurPose, DesPose);
			ErrorAng = GetErrorAng(CurPose, DesPose);
			printf("Error linear: %f, Error angular: %f\n", ErrorLin, ErrorAng);
			
			if (ErrorLin > 0)
			{
				CmdVel.linear = 0.2 * ErrorLin;
			}
			else
			{
				CmdVel.linear = 0;
			}
			CmdVel.angular = 0.5 * ErrorAng;
			Twist_pub.publish(CmdVel);
		}
		else
		{
			printf("Waiting...\n");
		}
		
		loop_rate.sleep();												// sleep to maintain loop rate
	}
}

// call back to send new desired Pose msgs
void ComPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)			
{
	STOP = false;														// start loop
	DesPose.x = msg->x;													// copy msg to varible to use elsewhere
	DesPose.y = msg->y;
	//~ DesPose.theta = msg->theta; // ignore theta, not doing
	return;
}

// call back to send new current Pose msgs
void CurPoseCallback(const turtlesim::Pose::ConstPtr& msg)			
{
	CurPose.x = msg->x;
	CurPose.y = msg->y;
	CurPose.theta = msg->theta;											// copy msg to varible to use elsewhere
	return;
}

float GetErrorLin(turtlesim::Pose curpose, geometry_msgs::Pose2D despose)
{
	// create error vector
	float Ex = despose.x - curpose.x;
	float Ey = despose.y - curpose.y;
	float Et = GetErrorAng(curpose, despose);
	
	// project error onto turtle x axis
	float Etx =  pow( pow(Ex,2.0) + pow(Ey,2.0), 0.5 )*cos(Et);
	return Etx;
}

float GetErrorAng(turtlesim::Pose curpose, geometry_msgs::Pose2D despose)
{
	// create error vector
	float Ex = despose.x - curpose.x;
	float Ey = despose.y - curpose.y;
	
	// get desire angle
	float dest = atan2(Ey, Ex);
	
	// get angle error
	float Et = dest - curpose.theta;
	
	//~ ROS_INFO("Ex: %f, Ey: %f, Et: %f", Ex, Ey, Et);
	return Et;
}
