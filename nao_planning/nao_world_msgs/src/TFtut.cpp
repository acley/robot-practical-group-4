#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  /*ros::service::waitForService("spawn");
  ros::ServiceClient add_turtle = 
       node.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn srv;
  add_turtle.call(srv);

  ros::Publisher turtle_vel = 
       node.advertise<turtlesim::Velocity>("turtle2/command_velocity", 10);*/

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/map", "/base_link",  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }

    std::cout << transform.getOrigin().getX() << " " << transform.getOrigin().getY() << std::endl;

    rate.sleep();
  }
  return 0;
};
