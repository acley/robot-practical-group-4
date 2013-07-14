#include "ros/ros.h"
#include <cstdlib>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>

#include "nao_world_msgs/RobotLocation.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "loc_test");
  
  ros::NodeHandle node;
	ros::ServiceClient objClient= node.serviceClient<nao_world_msgs::RobotLocation>("RobotLocation");
	nao_world_msgs::RobotLocation getRobotLocsSrv;
	
	ros::Publisher _markerPub = node.advertise<visualization_msgs::MarkerArray>("robot", 1000);
	
	visualization_msgs::MarkerArray ma;
	if(objClient.call(getRobotLocsSrv)){
	  visualization_msgs::Marker robotLoc;
	  robotLoc.header.frame_id= getRobotLocsSrv.response.robotLocation.header.frame_id;
	  robotLoc.header.stamp= getRobotLocsSrv.response.robotLocation.header.stamp;
	  robotLoc.ns= "Robot";
	  robotLoc.id= 0;
	  robotLoc.pose= getRobotLocsSrv.response.robotLocation.pose;
	  robotLoc.type= visualization_msgs::Marker::ARROW;
	  robotLoc.action= visualization_msgs::Marker::ADD;
	  robotLoc.color.b= 1.0;
	  robotLoc.color.a= 1.0;
	  
	  ma.markers.push_back(robotLoc);
	  _markerPub.publish(ma);
	
	
	}
	
	
	/*if(objClient.call(getObjLocsSrv)){
		visualization_msgs::MarkerArray currBoxLocs= getObjLocsSrv.response.boxLocs;
		visualization_msgs::MarkerArray currBallLocs= getObjLocsSrv.response.ballLocs;
		for(int i=0; i<currBoxLocs.markers.size(); i++){
			ma.markers.push_back(currBoxLocs.markers[i]);
		}
			
		for(int i=0; i<currBallLocs.markers.size(); i++){
		  ma.markers.push_back(currBallLocs.markers[i]);
		}
	}
	else{
		ROS_ERROR("Cannot extract object locations");
		return 0;
	}

	_markerPub.publish(ma);*/
	
	ros::spin();
	
  return 0;
}
