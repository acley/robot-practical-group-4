#include "ros/ros.h"
#include <cstdlib>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>

#include "nao_world_msgs/ObjectLocations.h"

nao_world_msgs::ObjectLocations getObjLocsSrv;

void color_detection(const sensor_msgs::PointCloud2ConstPtr& cloud){
		getObjLocsSrv.request.cloud= *cloud;
	
		ros::NodeHandle node;
		ros::ServiceClient objClient= node.serviceClient<nao_world_msgs::ObjectLocations>("ObjectLocations");
		ros::Publisher _markerPub = node.advertise<visualization_msgs::MarkerArray>("objects", 1000);
	
	  visualization_msgs::MarkerArray ma;
	
	  if(objClient.call(getObjLocsSrv)){
		  visualization_msgs::MarkerArray currBoxLocs= getObjLocsSrv.response.boxLocs;
		  visualization_msgs::MarkerArray currBallLocs= getObjLocsSrv.response.ballLocs;
		  for(int i=0; i<currBoxLocs.markers.size(); i++){
			  ma.markers.push_back(currBoxLocs.markers[i]);
		  }
			
		  for(int i=0; i<currBallLocs.markers.size(); i++){
		    ma.markers.push_back(currBallLocs.markers[i]);
		  }
	  }
	  
	  _markerPub.publish(ma);	  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "loc_test");
  
  ros::NodeHandle node;
	ros::ServiceClient objClient= node.serviceClient<nao_world_msgs::ObjectLocations>("ObjectLocations");
	//subscribe to the camera images
	ros::Subscriber cameraImg= node.subscribe("/xtion/depth_registered/points", 1, color_detection);
	
	ros::Publisher _markerPub = node.advertise<visualization_msgs::MarkerArray>("objects", 1000);
	
	visualization_msgs::MarkerArray ma;
	
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
