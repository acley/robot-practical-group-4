#include "nao_actions/stateCreatorRobotPose.h"
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/MarkerArray.h>
#include <angles/angles.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <sstream>
#include <XmlRpcValue.h>
#include <XmlRpcException.h>
#include <nao_msgs/ObjectLocations.h>
#include <nao_msgs/RobotLocation.h>


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
                _markerPub = nhPriv.advertise<visualization_msgs::MarkerArray>("robot_pose_markers", 5, true);
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
		geometry_msgs::PoseStamped robotPose;
		//call to service to get the current robot location
		//PS fix the include statements!!!
		ros::NodeHandle node;
		ros::ServiceClient client = node.serviceClient<nao_msgs::RobotLocation>("RobotLocation");
		nao_msgs::RobotLocation srv;
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
		ros::ServiceClient objClient= node.serviceClient<nao_msgs::ObjectLocations>("ObjectLocations");
		nao_msgs::ObjectLocations srv1;
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

		if(s_PublishLocationsAsMarkers)
               		publishLocationsAsMarkers(state);
		return true;

/*

            tf::StampedTransform transform;
            try{
                _tf.lookupTransform("/map", "/base_link", ros::Time(0), transform);
            }
            catch (tf::TransformException& ex){
                ROS_ERROR("%s",ex.what());
                return false;
            }

            // 1. Real robot location
            if(!_robotPoseObject.empty()) {
                ROS_ASSERT(!_robotPoseType.empty());
                state.addObject(_robotPoseObject, _robotPoseType);
                state.setNumericalFluent("x", _robotPoseObject, transform.getOrigin().x());
                state.setNumericalFluent("y", _robotPoseObject, transform.getOrigin().y());
                state.setNumericalFluent("z", _robotPoseObject, transform.getOrigin().z());
                state.setNumericalFluent("qx", _robotPoseObject, transform.getRotation().x());
                state.setNumericalFluent("qy", _robotPoseObject, transform.getRotation().y());
                state.setNumericalFluent("qz", _robotPoseObject, transform.getRotation().z());
                state.setNumericalFluent("qw", _robotPoseObject, transform.getRotation().w());
                state.setNumericalFluent("timestamp", _robotPoseObject, ros::Time::now().toSec());
                state.addObject("/map", "frameid");
                state.setObjectFluent("frame-id", _robotPoseObject, "/map");
            }

            // 2.b check if we are at any _locations
            pair<SymbolicState::TypedObjectConstIterator, SymbolicState::TypedObjectConstIterator> targets =
                state.getTypedObjects().equal_range(_locationType);

            double minDist = HUGE_VAL;
            string nearestTarget = "";

            int atLocations = 0;
            for(SymbolicState::TypedObjectConstIterator it = targets.first; it != targets.second; it++) {
                string target = it->second;
                if(target == _robotPoseObject)  // skip current robot location
                    continue;

                geometry_msgs::PoseStamped targetPose;
                if(!extractPoseStamped(state, target, targetPose)) {
                    ROS_ERROR("%s: could not extract pose for target object: %s.", __func__, target.c_str());
                    continue; 
                }
                if(targetPose.header.frame_id != "/map") {
                    ROS_ERROR("Target pose %s had frame-id: %s - should be /map.",
                            target.c_str(), targetPose.header.frame_id.c_str());
                    continue;
                }

                // compute dXY, dYaw between current pose and target
                tf::Transform targetTransform;//(btQuaternion(qx, qy, qz, qw), btVector3(posX, posY, 0.0));
                tf::poseMsgToTF(targetPose.pose, targetTransform);
                tf::Transform deltaTransform = targetTransform.inverseTimes(transform);

                double dDist = hypot(deltaTransform.getOrigin().x(), deltaTransform.getOrigin().y());
                double dAng = tf::getYaw(deltaTransform.getRotation());
                ROS_INFO("Target %s dist: %f m ang: %f deg", target.c_str(), dDist, angles::to_degrees(dAng));

                if(!_atPredicate.empty()) {
                    // Found a target - update state!
                    if(dDist < _goalToleranceXY && fabs(dAng) < _goalToleranceYaw) {
                        ROS_INFO("(at) target %s !", target.c_str());
                        state.setBooleanPredicate(_atPredicate, target, true);
                        atLocations++;
                    } else {
                        state.setBooleanPredicate(_atPredicate, target, false);
                    }
                    if(dDist < minDist) {
                        minDist = dDist;
                        nearestTarget = target;
                    }
                }
            }

            ROS_INFO("Nearest target is %s (%f m).", nearestTarget.c_str(), minDist);

            // 2.a Set the robot pose, if we are not already at another pose
            if(!_atPredicate.empty() && !_robotPoseObject.empty()) {
                if(atLocations == 0) {
                    state.setBooleanPredicate(_atPredicate, _robotPoseObject, true);
                } else {
                    state.setBooleanPredicate(_atPredicate, _robotPoseObject, false);
                    if(atLocations > 1) {
                        ROS_WARN("We are at %d locations at the same time!.", atLocations);
                    }
                }
            }

            if(s_PublishLocationsAsMarkers)
                publishLocationsAsMarkers(state);

            return true;*/
        }

	
       
    /**
     * Publishes locations for boxes, balls and the robot
     */
    void StateCreatorRobotPose::publishLocationsAsMarkers(const SymbolicState & state)
    {
	ros::NodeHandle node;
	ros::ServiceClient client = node.serviceClient<nao_msgs::RobotLocation>("RobotLocation");
	nao_msgs::RobotLocation srv;
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
	ros::ServiceClient objClient= node.serviceClient<nao_msgs::ObjectLocations>("ObjectLocations");
	nao_msgs::ObjectLocations srv1;
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
	}

	_markerPub.publish(ma);
        /*if(!_markerPub) {
            ROS_WARN("%s: _markerPub invalid.", __func__);
            return;
        }

        visualization_msgs::MarkerArray ma;

        if(!_locationType.empty()) {
            // Check if we are at any grasp_locations
            pair<SymbolicState::TypedObjectConstIterator, SymbolicState::TypedObjectConstIterator> targets =
                state.getTypedObjects().equal_range(_locationType);

            unsigned int count = 0;
            for(SymbolicState::TypedObjectConstIterator it = targets.first; it != targets.second; it++) {
                string target = it->second;
                if(target == _robotPoseObject)  // skip current robot location
                    continue;

                visualization_msgs::MarkerArray marks = getLocationMarkers(state, target,
                        "target_locations", count, false);
                forEach(visualization_msgs::Marker & mark, marks.markers) {
                    if(mark.header.frame_id.empty())    // invalid mark
                        continue;
                    ma.markers.push_back(mark);
                }
                count += 2;
                
                if(s_PublishMeshMarkers) {
                    visualization_msgs::MarkerArray marks = getLocationMarkers(state, target,
                            "target_locations", count, true);
                    forEach(visualization_msgs::Marker & mark, marks.markers) {
                        if(mark.header.frame_id.empty())    // invalid mark
                            continue;
                        ma.markers.push_back(mark);
                    }
                }
            }

            // all should be overwritten as #targets is const, but to be safe
            for(unsigned int i = count; i < 100; i++) {
                visualization_msgs::Marker mark;
                mark.header.frame_id = "/map";
                mark.ns = "target_locations";
                mark.id = i;
                mark.action = visualization_msgs::Marker::DELETE;
                ma.markers.push_back(mark);
            }
        }

        // finally robot location marker
        if(!_robotPoseObject.empty()) {
            visualization_msgs::MarkerArray marks = getLocationMarkers(state, _robotPoseObject,
                    "robot_location", 0, false);
            forEach(visualization_msgs::Marker & mark, marks.markers) {
                if(mark.header.frame_id.empty())    // invalid mark
                    mark.action = visualization_msgs::Marker::DELETE;
                ma.markers.push_back(mark);
            }

            if(s_PublishMeshMarkers) {
                visualization_msgs::MarkerArray marks = getLocationMarkers(state, _robotPoseObject,
                        "robot_location", 0, true);
                forEach(visualization_msgs::Marker & mark, marks.markers) {
                    if(mark.header.frame_id.empty())    // invalid mark
                        continue;
                    ma.markers.push_back(mark);
                }
            }
        }

        _markerPub.publish(ma);*/
    }

};

