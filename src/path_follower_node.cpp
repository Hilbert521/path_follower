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
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/LaserScan.h"

/* Globals */
nav_msgs::Path path;
sensor_msgs::LaserScan lasers;

int nb_points = 0;
bool new_path = false;


void lasersCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	lasers = sensor_msgs::LaserScan(*msg);
}

void pathCallback(const nav_msgs::Path::ConstPtr& msg)//, nav_msgs::Path& p)
{
	nb_points = msg->poses.size();
	int old_nb_points = path.poses.size();
	if(old_nb_points == 0 || (path.poses[old_nb_points - 1].pose.position.x != msg->poses[nb_points - 1].pose.position.x && path.poses[old_nb_points - 1].pose.position.y != msg->poses[nb_points - 1].pose.position.y))
	{
		path = nav_msgs::Path(*msg);
		new_path = true;
	}
	// std::cout << "Callback" << std::endl;
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

	double min_speed = 0.1;
	double forward_speed = 0.5 - min_speed;
	/* PI corrector */
	double Kp = 3;
	double Ki = 0.00000001;

	double max_cmd = 1;
	double K_rot = forward_speed/max_cmd;

	int current = 1;
	double delta = 0.30; // 30 cm tolerance
	// int nb_points = 0;
	double sum_err = 0;
	double err = 0;
	while(ros::ok())
	{
		geometry_msgs::Twist cmd_vel;

		// int old_nb_points = nb_points;
		ros::spinOnce();

		// nb_points = path.poses.size(); 

		// if(old_nb_points != 0)
			// if(old_nb_points != nb_points || ) // New Goal detected
				// current = 1;
		if(new_path == true)
		{
			current = 1;
			new_path = false;
		}
		// Get frame change between slam_karto map frame and the frame of the odom of the robot
		tf::StampedTransform transform_slam;
		try
		{
			t.lookupTransform("map", "base_link", ros::Time(0), transform_slam);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}
		robot_pos[0] = transform_slam.getOrigin().x();
		robot_pos[1] = transform_slam.getOrigin().y();
		tf::Quaternion q(transform_slam.getRotation());
		robot_pos[2] = tf::getYaw(q);
		
		if(nb_points != 0 && current < nb_points) // Path is not empty or not finished
		{
			double err_x = fabs(robot_pos[0] - path.poses[current].pose.position.x);
			double err_y = fabs(robot_pos[1] - path.poses[current].pose.position.y);
			if( err_x < delta &&  err_y < delta)
				current++;
			// std::cout << "ERR x : " << err_x << " y : " << err_y << std::endl;
			// std::cout << nb_points << " " << current << std::endl;
			// double angle_des = atan2(path.poses[current].pose.position.y - path.poses[current - 1].pose.position.y, path.poses[current].pose.position.x - path.poses[current - 1].pose.position.x);
			double angle_des = atan2(path.poses[current].pose.position.y - robot_pos[1], path.poses[current].pose.position.x - robot_pos[0]);
			
			// std::cout << transform_slam.getRotation().x() << " " << transform_slam.getRotation().y() << " " << transform_slam.getRotation().z() << " " << transform_slam.getRotation().w() << std::endl;
			// std::cout << "Angle forme par les points: " << angle_des << std::endl;
			// std::cout << "Orientation du robot: " << robot_pos[2] << std::endl;
			
			// Send the command to the robot

			if(angle_des > robot_pos[2] + M_PI && robot_pos[2] < 0)
				err = - 2 * M_PI + (angle_des - robot_pos[2]);
			else if(angle_des < robot_pos[2] -  M_PI && robot_pos[2] > 0)
				err = 2 * M_PI + (angle_des - robot_pos[2]);
			else
				err = angle_des - robot_pos[2];

			double dist = sqrt( (path.poses[current].pose.position.x - robot_pos[0])*(path.poses[current].pose.position.x - robot_pos[0]) +  (path.poses[current].pose.position.y - robot_pos[1])*(path.poses[current].pose.position.y - robot_pos[1]) );
			sum_err += err;
			
			/* PI corrector */
			double cmd_angular;
			int signe = 1;
			cmd_angular= Kp * err ;//+ Ki * sum_err;
			if(fabs(cmd_angular) > max_cmd)
			{
				if(cmd_angular > 0)
					cmd_angular = max_cmd;
				else
				{
					signe = -1;
					cmd_angular = -max_cmd;
				}
			}
			cmd_vel.angular.z = cmd_angular;
			cmd_vel.linear.x = min_speed + forward_speed - K_rot * signe * fabs(cmd_vel.angular.z);

			// std::cout << "linear : " << cmd_vel.linear.x << " angular : " << cmd_vel.angular.z << std::endl;

			pubCmd.publish(cmd_vel);
		}
		cmd_vel.angular.z = 0;
		cmd_vel.linear.x = 0;
		pubCmd.publish(cmd_vel);

	}
	return 0;
}