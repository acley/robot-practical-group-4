#include "ros/ros.h"
#include <stdio.h>
#include "nao_world_msgs/RobotLocation.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>

using namespace std;

/*
**returns the current robot location
*/

bool getLoc(nao_world_msgs::RobotLocation::Request  &req,
         nao_world_msgs::RobotLocation::Response &res){
         
  // get cell size
  ros::NodeHandle nhPriv("~");
  double cell_size = 0.23;
  //nhPriv.getParam("cell_size", cell_size);
  
  // get robot position
  tf::TransformListener listener;
  ros::Rate rate(10.0);
  ros::NodeHandle node;
  tf::StampedTransform map_to_robot;
  while (node.ok()){
    try{
      listener.lookupTransform("/map", "/base_link",  
                               ros::Time(0), map_to_robot);
      break;
    }
    catch (tf::TransformException ex){
      //ROS_ERROR("%s",ex.what());
    }

    rate.sleep();
  }
  
  res.robotLocation.header.stamp= ros::Time::now();
  res.robotLocation.header.frame_id= "/map";
  res.robotLocation.pose.position.x= map_to_robot.getOrigin().getX();
  res.robotLocation.pose.position.y= map_to_robot.getOrigin().getY();
  geometry_msgs::Quaternion rotation;
  tf::quaternionTFToMsg(map_to_robot.getRotation(), rotation);  
  res.robotLocation.pose.orientation= rotation;
  
  double map_origin = 1.84;
  double delta_x = map_to_robot.getOrigin().getX();
  double delta_y = map_to_robot.getOrigin().getY();
  nao_world_msgs::GridCoordinate cell;
  int x_coord = -floor(delta_x / cell_size);
  int y_coord = floor((delta_y + map_origin) / cell_size) + 1;
  cell.x = x_coord;
  cell.y = y_coord;
  res.robot_grid_cell = cell;
  
  std::cout << "robot cell: [" << x_coord << ", " << y_coord << "]\n";
         
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

