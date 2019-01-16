/*
 * Standard libs include
 */
#include <iostream>
#include <float.h>
#include <time.h>
#include <cmath>
#include <string>

// /*
//  * Ros includes
//  */
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"

nav_msgs::Path path;

void pathCallback(const nav_msgs::Path::ConstPtr& msg)//, nav_msgs::Path& p)
{
	// std::cout << msg->poses.size() << std::endl;
	path = nav_msgs::Path(*msg);
	// std::cout << "Callback" << std::endl;
	// std::cout << p.poses.size() << std::endl;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "path_follower_node");
	ros::NodeHandle n;
	ros::Rate loop_rate(100); //100 Hz
	// nav_msgs::Path path;

	ros::Subscriber subPath = n.subscribe<nav_msgs::Path>("/rrt/path", 10, pathCallback);//boost::bind(pathCallback, _1, path));
	ros::Publisher pubCmd = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

	tf::TransformListener t;
	double robot_pos[3];
	double Kp = 1;
	double forward_speed = 0;
	while(ros::ok())
	{
		ros::spinOnce();

		// Get frame change between slam_karto map frame and the frame of the odom of the robot
		tf::StampedTransform transform_slam;
		try
		{
			t.lookupTransform("map", "base_footprint", ros::Time(0), transform_slam);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}
		robot_pos[0] = transform_slam.getOrigin().x();
		robot_pos[1] = transform_slam.getOrigin().y();
		tf::Quaternion q(transform_slam.getRotation().x(), transform_slam.getRotation().y(), transform_slam.getRotation().z(), transform_slam.getRotation().w());
		robot_pos[2] = q.getAngle();
		// std::cout << path.poses.size() << std::endl;
		
		if(path.poses.size() != 0) // Path is not empty
		{
			// std::cout << "points: " << atan2(path.poses[1].pose.position.y - path.poses[0].pose.position.y, path.poses[1].pose.position.x - path.poses[0].pose.position.x) << std::endl;
			// std::cout << "robot: " << robot_pos[2] << std::endl;
			geometry_msgs::Twist cmd_vel;

			cmd_vel.linear.x = forward_speed;
			cmd_vel.angular.z = Kp * ((atan2(path.poses[1].pose.position.y - path.poses[0].pose.position.y, path.poses[1].pose.position.x - path.poses[0].pose.position.x)) - robot_pos[2]);
			std::cout << cmd_vel << std::endl;
			pubCmd.publish(cmd_vel);
		}
	}
	return 0;
}