#include <nao_world_msgs/PickupAction.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<nao_world_msgs::PickupAction> Server;

void execute(const nao_world_msgs::PickupGoalConstPtr& goal, Server* as)
{
  // Do lots of awesome groundbreaking robot stuff here
  as->setSucceeded();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pickup_server");
  ros::NodeHandle n;
  Server server(n, "/nao_world_msgs/pickup_action_server", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}
