#include "ros/ros.h"
#include <stdio.h>

using namespace std;

/*method to get from the preception the locations of the 
**balls and boxes currently observed by the robot and 
**returns them in visualization_msgs::MarkerArray objects
*/

bool getObjectsLoc(nao_msgs::ObjectLocations::Request  &req,
         nao_msgs::ObjectLocations::Response &res){
	return true;

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_object_locs");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("ObjectLocations", getObjectsLoc);
  ROS_INFO("Retrieving information from vision");
  ros::spin();

  return 0;
}


