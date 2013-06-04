#include "ros/ros.h"
#include <stdio.h>

using namespace std;

/*
**returns the current robot location
*/

bool getObjectsLoc(nao_msgs::RobotLocation::Request  &req,
         nao_msgs::RobotLocation::Response &res){
	return true;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_robot_loc");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("RobotLocation", getLoc);
  ROS_INFO("Retrieving current robot pose");
  ros::spin();

  return 0;
}

