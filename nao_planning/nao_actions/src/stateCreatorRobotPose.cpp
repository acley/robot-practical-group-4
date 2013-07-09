#include "nao_actions/stateCreatorRobotPose.h"
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/MarkerArray.h>
#include <angles/angles.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <sstream>
#include <XmlRpcValue.h>
#include <XmlRpcException.h>
#include <nao_world_msgs/ObjectLocations.h>
#include <nao_world_msgs/RobotLocation.h>


PLUGINLIB_DECLARE_CLASS(nao_actions, state_creator_robot_pose,
        nao_actions::StateCreatorRobotPose, continual_planning_executive::StateCreator)

namespace nao_actions
{

    StateCreatorRobotPose::StateCreatorRobotPose()
    {
        ros::NodeHandle nhPriv("~");
        ros::NodeHandle nh;
	nhPriv.getParam("cell_size", cell_size);
	nhPriv.getParam("grid_size", grid_size);
	nhPriv.getParam("robotLoc", robotLoc);
	nhPriv.getParam("robotDir", robotDir);
	nhPriv.getParam("goalLoc", goalLoc);
	nhPriv.getParam("boxLocs", xmlboxLocs);
	nhPriv.getParam("ballLocs", xmlballLocs);
	nhPriv.getParam("boxes", xmlboxes);
	nhPriv.getParam("balls", xmlballs);

	ROS_ASSERT(xmlboxLocs.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(xmlballLocs.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(xmlboxes.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(xmlballs.getType() == XmlRpc::XmlRpcValue::TypeArray);

	//copying xmlrpc to vector
	for(int i=0; i<xmlboxLocs.size(); i++){
		boxLocs.push_back(xmlboxLocs[i]);
		boxes.push_back(xmlboxes[i]);
	}
	for(int i=0; i<xmlballLocs.size(); i++){
		ballLocs.push_back(xmlballLocs[i]);
		balls.push_back(xmlballs[i]);
	}

        //nhPriv.param("nav_target_tolerance_xy", _goalToleranceXY, 0.5);
        //nhPriv.param("nav_target_tolerance_yaw", _goalToleranceYaw, 0.26);  //15deg

        //bool relative;
      /*  nhPriv.param("nav_target_tolerance_relative_to_move_base", relative, false);
        if(relative) {
            // relative mode: 1. get the namespace for base_local_planner
            std::string base_local_planner_ns;
            if(!nhPriv.getParam("nav_base_local_planner_ns", base_local_planner_ns)) {
                ROS_WARN("nav_target_tolerance_relative_to_move_base set, but nav_base_local_planner_ns not set - trying to estimate");
                std::string local_planner;
                if(!nh.getParam("move_base_node/base_local_planner", local_planner)
                        && !nh.getParam("move_base/base_local_planner", local_planner)) {
                    ROS_ERROR("move_base(_node)/base_local_planner not set - falling back to absolute mode.");
                } else {
                    // dwa_local_planner/DWAPlannerROS -> DWAPlannerROS
                    std::string::size_type x = local_planner.find_last_of("/");
                    if(x == std::string::npos)
                        base_local_planner_ns = local_planner;
                    else
                        base_local_planner_ns = local_planner.substr(x + 1);
                    ROS_INFO("Estimated base_local_planner_ns to %s.", base_local_planner_ns.c_str());
                }
            }
            
            if(!base_local_planner_ns.empty()) { // success: 2. get the xy_goal_tolerance
                double move_base_tol_xy;
                if(!nh.getParam(base_local_planner_ns + "/xy_goal_tolerance", move_base_tol_xy)) {
                    ROS_ERROR_STREAM("nav_target_tolerance_relative_to_move_base was true, but "
                            << (base_local_planner_ns + "/xy_goal_tolerance") << " was not set"
                            << " - falling back to absolute mode");
                } else { // 2. add move_base's tolerance to our relative tolerance
                    _goalToleranceXY += move_base_tol_xy;
                }

                double move_base_tol_yaw;
                if(!nh.getParam(base_local_planner_ns + "/yaw_goal_tolerance", move_base_tol_yaw)) {
                    ROS_ERROR_STREAM("nav_target_tolerance_relative_to_move_base was true, but "
                            << (base_local_planner_ns + "/yaw_goal_tolerance") << " was not set"
                            << " - falling back to absolute mode");
                } else { // 2. add move_base's tolerance to our relative tolerance
                    _goalToleranceYaw += move_base_tol_yaw;
                }
            }
        }

            ROS_INFO("Tolerance for accepting nav goals set to %f m, %f deg.",
                    _goalToleranceXY, angles::to_degrees(_goalToleranceYaw));
	*/
            if(s_PublishLocationsAsMarkers) {
                _markerPub = nh.advertise<visualization_msgs::MarkerArray>("robot_pose_markers", 5, true);
                ROS_INFO("marker topic: %s", _markerPub.getTopic().c_str());
            }
        }

        StateCreatorRobotPose::~StateCreatorRobotPose()
        {
        }

        void StateCreatorRobotPose::initialize()
        {
            

        }

	//sets the current location for the robot and the observed objects
        bool StateCreatorRobotPose::fillState(SymbolicState & state)
        {
		/*geometry_msgs::PoseStamped robotPose;
		
		//call to service to get the current robot location
		ros::NodeHandle node;
		ros::ServiceClient client = node.serviceClient<nao_world_msgs::RobotLocation>("RobotLocation");
		nao_world_msgs::RobotLocation srv;
		int robotLocX, robotLocY;
		if(client.call(srv)){
			robotPose= srv.response.robotLocation;
			robotLocX= robotPose.pose.position.x/cell_size+1;
			robotLocY= robotPose.pose.position.y/cell_size+1;
			std::stringstream ss;
			ss << "pos-" << robotLocX << "-" << robotLocY;
			robotLoc= ss.str();
			std::vector<std::string> atPredicate;
			atPredicate.push_back("robot");
			atPredicate.push_back(robotLoc);
			//setting the current robot location
			state.setBooleanPredicate("at", atPredicate, true);
			state.setBooleanPredicate("clear", robotLoc, false);
		}
		else{
			ROS_ERROR("Cannot extract current robot location");
			return false;
		}

		//call to service to get the current positions of the balls and boxes
		ros::ServiceClient objClient= node.serviceClient<nao_world_msgs::ObjectLocations>("ObjectLocations");
		nao_world_msgs::ObjectLocations srv1;
		if(objClient.call(srv1)){
			visualization_msgs::MarkerArray currBoxLocs= srv1.response.boxLocs;
			visualization_msgs::MarkerArray currBallLocs= srv1.response.ballLocs;
			for(int i=0; i<currBoxLocs.markers.size(); i++){
				std::stringstream ss;
				ss << "pos-" << currBoxLocs.markers[i].pose.position.x/cell_size+1 << "-" << currBoxLocs.markers[i].pose.position.y/cell_size+1;
				std::vector<std::string> atPredicate;
				atPredicate.push_back(currBoxLocs.markers[i].ns);
				atPredicate.push_back(ss.str());
				state.setBooleanPredicate("at", atPredicate, true);
				state.setBooleanPredicate("clear", ss.str(), false);
			}
			for(int i=0; i<currBallLocs.markers.size(); i++){
				std::stringstream ss;
				ss << "pos-" << currBallLocs.markers[i].pose.position.x/cell_size+1 << "-" << currBallLocs.markers[i].pose.position.y/cell_size+1;
				std::vector<std::string> atPredicate;
				atPredicate.push_back(currBallLocs.markers[i].ns);
				atPredicate.push_back(ss.str());
				state.setBooleanPredicate("at", atPredicate, true);
				state.setBooleanPredicate("clear", ss.str(), false);
			}

		}
		else{
			ROS_ERROR("Cannot extract object locations");
			return false;
		}
		*/
		if(s_PublishLocationsAsMarkers)
               		publishLocationsAsMarkers(state);
		return true;


        }

	
       
    /**
     * Publishes locations for boxes, balls and the robot
     */
    void StateCreatorRobotPose::publishLocationsAsMarkers(const SymbolicState & state)
    {
	ros::NodeHandle nhPriv("~");
	nhPriv.getParam("robotLoc", robotLoc);
	robotLoc= robotLoc.substr(4);
	int index= robotLoc.find("-");
	double robotLocX= atof((robotLoc.substr(0, index).c_str()))*cell_size + 0.5 * cell_size;
	double robotLocY= atof((robotLoc.substr(index+1).c_str()))*cell_size + 0.5 * cell_size;
	ros::NodeHandle node;
	visualization_msgs::MarkerArray ma;
	visualization_msgs::Marker robLoc;
	robLoc.ns= "robot";
	robLoc.id= 0;
	robLoc.pose.position.x= robotLocX;
	robLoc.pose.position.y= robotLocY;
	robLoc.header.frame_id= "/base_link";
	robLoc.header.stamp= ros::Time::now();
	robLoc.type= visualization_msgs::Marker::ARROW;
	robLoc.action= visualization_msgs::Marker::ADD;
    	robLoc.scale.x= 100;
    	robLoc.scale.y= 100;
    	robLoc.scale.z= 10;
    	robLoc.color.r= 1.0;
    	robLoc.color.a= 1.0;
	ma.markers.push_back(robLoc);
    
   

	nhPriv.getParam("boxes", xmlboxes);
	nhPriv.getParam("boxLocs", xmlboxLocs);
	ROS_ASSERT(xmlboxLocs.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(xmlboxes.getType() == XmlRpc::XmlRpcValue::TypeArray);
	visualization_msgs::Marker box;
	double boxX, boxY;
	std::string currBoxLoc;
	for(int i=0; i<xmlboxes.size(); i++){
		box.ns= static_cast<std::string>(xmlboxes[i]);
		box.id= i;
		currBoxLoc= static_cast<std::string>(xmlboxLocs[i]);
		currBoxLoc= currBoxLoc.substr(4);
		index= currBoxLoc.find("-");		
		boxX= atof((currBoxLoc.substr(0,index).c_str()))*cell_size + 0.5 * cell_size;
		boxY= atof((currBoxLoc.substr(index+1).c_str()))*cell_size + 0.5 * cell_size;
        
		double box_size = 0; //25;
		// Check if the robot is holding this box.
		Predicate bp;
		bp.name= "Holding";
		std::vector<string> parameters;
		parameters.push_back("robot");
		parameters.push_back(box.ns);
		bp.parameters= parameters;
		bool value = true;
		if (state.hasBooleanPredicate(bp, &value)) {
		    box.pose.position.x= robotLocX;
		    box.pose.position.y= robotLocY;
		    box.pose.position.z= box_size;
		    box.scale.x= 15;
		    box.scale.y= 15;
		    box.scale.z= 15;
		} else {
		    box.pose.position.x= boxX;
		    box.pose.position.y= boxY;
		    box.pose.position.z= box_size;
		    box.scale.x= 25;
		    box.scale.y= 25;
		    box.scale.z= 25;
		}
		
		box.header.frame_id= "/base_link";
		box.header.stamp= ros::Time::now();
		box.type= visualization_msgs::Marker::CUBE;
		box.action= visualization_msgs::Marker::ADD;
		box.color.g= 1.0;
		box.color.a= 1.0;
		ma.markers.push_back(box);
	}	

	nhPriv.getParam("balls", xmlballs);
	nhPriv.getParam("ballLocs", xmlballLocs);
	ROS_ASSERT(xmlballLocs.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(xmlballs.getType() == XmlRpc::XmlRpcValue::TypeArray);
	visualization_msgs::Marker ball;
	double ballX, ballY;
	std::string currballLoc;
	for(int i=0; i<xmlballs.size(); i++){
		ball.ns= static_cast<std::string>(xmlballs[i]);
		ball.id= 0;
		currballLoc= static_cast<std::string>(xmlballLocs[i]);
		currballLoc= currballLoc.substr(4);
		index= currballLoc.find("-");		
		ballX= atof((currballLoc.substr(0,index).c_str()))*cell_size + 0.5 * cell_size;
		ballY= atof((currballLoc.substr(index+1).c_str()))*cell_size + 0.5 * cell_size;
        
		ball.pose.position.x= ballX;
		ball.pose.position.y= ballY;
		ball.header.frame_id= "/base_link";
		ball.header.stamp= ros::Time::now();
		ball.type= visualization_msgs::Marker::SPHERE;
		ball.action= visualization_msgs::Marker::ADD;
		ball.scale.x= 25;
		ball.scale.y= 25;
		ball.scale.z= 25;
		ball.color.b= 1.0;
		ball.color.a= 1.0;
		ma.markers.push_back(ball);
	}

/*

	ros::NodeHandle node;
	ros::ServiceClient client = node.serviceClient<nao_world_msgs::RobotLocation>("RobotLocation");
	nao_world_msgs::RobotLocation srv;
	visualization_msgs::MarkerArray ma;
	if(client.call(srv)){
		visualization_msgs::Marker robLoc;
		robLoc.ns= "robot";
		robLoc.id= 0;
		robLoc.pose= srv.response.robotLocation.pose;
		robLoc.header= srv.response.robotLocation.header;
		robLoc.type= visualization_msgs::Marker::ARROW;
		robLoc.action= visualization_msgs::Marker::ADD;
		ma.markers.push_back(robLoc);
	}
	else{
		ROS_ERROR("Cannot extract current robot location");
		return;
	}

	//call to service to get the current positions of the balls and boxes
	ros::ServiceClient objClient= node.serviceClient<nao_world_msgs::ObjectLocations>("ObjectLocations");
	nao_world_msgs::ObjectLocations srv1;
	if(objClient.call(srv1)){
		visualization_msgs::MarkerArray currBoxLocs= srv1.response.boxLocs;
		visualization_msgs::MarkerArray currBallLocs= srv1.response.ballLocs;
		for(int i=0; i<currBoxLocs.markers.size(); i++){
			ma.markers.push_back(currBoxLocs.markers[i]);
		}
		for(int i=0; i<currBallLocs.markers.size(); i++){
			ma.markers.push_back(currBallLocs.markers[i]);
		}

	}
	else{
		ROS_ERROR("Cannot extract object locations");
		return;
	}*/

	_markerPub.publish(ma);
    }

};

