#include "nao_actions/goalCreatorTidyupObjects.h"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <set>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <stringstream>
#include <iostream>


PLUGINLIB_DECLARE_CLASS(nao_actions, goal_creator_tidyup_objects,
        nao_actions::GoalCreatorTidyupObjects, continual_planning_executive::GoalCreator)

namespace nao_actions
{
	bool GoalCreatorTidyupObjects::worldResetPerformed = false;

    GoalCreatorTidyupObjects::GoalCreatorTidyupObjects()
    {
    }

    GoalCreatorTidyupObjects::~GoalCreatorTidyupObjects()
    {
    }

    bool GoalCreatorTidyupObjects::fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal)
    {
        ros::NodeHandle nhPriv("~");
        /*if (! worldResetPerformed)
        {
        	worldResetPerformed = true;
        	std_srvs::Empty msg;
        	ros::service::call("/tidyup/reset_world_interface", msg);
        }
	*/
        // first add the type hierarchy
	currentState.addSuperType("direction", "object");
	currentState.addSuperType("thing", "object");
	currentState.addSuperType("location", "object");
	currentState.addSuperType("robot", "thing");
	currentState.addSuperType("box", "thing");
	currentState.addSuperType("ball", "thing");
	currentState.addObject("robot", "robot");

	goal.addSuperType("direction", "object");
	goal.addSuperType("thing", "object");
	goal.addSuperType("location", "object");
	goal.addSuperType("robot", "thing");
	goal.addSuperType("box", "thing");
	goal.addSuperType("ball", "thing");
	goal.addObject("robot", "robot");
	

	
        /*currentState.addSuperType("pose", "pose");
        currentState.addSuperType("frameid", "frameid");
        currentState.addSuperType("location", "pose");
        currentState.addSuperType("manipulation_location", "location");
        currentState.addSuperType("door_location", "location");
        currentState.addSuperType("door_in_location", "door_location");
        currentState.addSuperType("door_out_location", "door_location");
        currentState.addSuperType("room", "room");
        currentState.addSuperType("static_object", "static_object");
        currentState.addSuperType("door", "door");
        currentState.addSuperType("movable_object", "pose");
        currentState.addSuperType("arm", "arm");
        currentState.addSuperType("arm_state", "arm_state");
        goal.addSuperType("pose", "pose");
        goal.addSuperType("frameid", "frameid");
        goal.addSuperType("location", "pose");
        goal.addSuperType("manipulation_location", "location");
        goal.addSuperType("door_location", "location");
        goal.addSuperType("door_in_location", "door_location");
        goal.addSuperType("door_out_location", "door_location");
        goal.addSuperType("room", "room");
        goal.addSuperType("static_object", "static_object");
        goal.addSuperType("door", "door");
        goal.addSuperType("movable_object", "pose");
        goal.addSuperType("arm", "arm");
        goal.addSuperType("arm_state", "arm_state");
	*/
        currentState.printSuperTypes();

        // load grid size, cell size, initial robot location and goal location
	// assume that the cells are 8 way connected
	// cell size is in cms
	int cell_size, grid_size;
	std::string robotLoc, goalLoc;
	std::vector<std::string> boxLocs, ballLocs, boxes, balls, connections, directions;

	nhPriv.getParam("cell_size", cell_size);
	nhPriv.getParam("grid_size", grid_size);
	nhPriv.getParam("robotLoc", robotLoc);
	nhPriv.getParam("goalLoc", goalLoc);
	nhPriv.getParam("boxLocs", boxLocs);
	nhPriv.getParam("ballLocs", ballLocs);
	nhPriv.getParam("boxes", boxes);
	nhPriv.getParam("balls", balls);
	nhPriv.getParam("connections", connections);
	nhPriv.getParam("directions", directions);

	//adding locations
	for(int i=1; i<=grid_size; i++)
		for(int j=1; j<=grid_size; j++){
			stringstream pos;
			pos << "pos-" << i << "-" << j;
			currentState.addObject(pos.str(), "location");
			goal.addObject(pos.str(), "location");
		}

	//adding the directions
	for(std::vector<std::string>::iterator it= directions.begin(); it!=directions.end(); ++it){
		currentState.addObject(*it, "direction");
		goal.addObject(*it, "direction");
	}

	//adding the boxes and balls
	for(std::vector<std::string>::iterator it= boxes.begin(); it!=boxes.end(); ++it){
		currentState.addObject(*it, "box");
		goal.addObject(*it, "box");
	}

	for(std::vector<std::string>::iterator it= balls.begin(); it!=balls.end(); ++it){
		currentState.addObject(*it, "ball");
		goal.addObject(*it, "ball");
	}

	//check how the goal should be defined
	//goal.setForEachGoalStatement("robot", "at", true);
        
	//setting the clear locations
	for(int i= 1; i<=grid_size; i++)
		for(int j=1; j<=grid_size; j++){
			stringstream currLoc;
			currLoc << "pos-" << i << "-" << j;
			if(strcmp(robotLoc,currLoc.str()))
				continue;
			std::vector<std::string>::iterator it;
			it= find(boxLocs.begin(), boxLocs.end(), currLoc.str());
			if(it!=boxLocs.end())
				continue;
			it= find(ballLocs.begin(), ballLocs.end(), currLoc.str());
			if(it!=ballLocs.end())
				continue;
			currentState.setBooleanPredicate("clear", currLoc.str(), true);
		}
	
	//setting the connections
	for(std::vector<std::string>::iterator it= connections.begin(); it!= connections.end(); ++it){
		currentState.setBooleanPredicate("MOVE-DIR", *it, true);
	}

	currentState.setBooleanPredicate("HandEmpty", "robot", true);
	
	//setting the robot location initially	
	std::vector<std::string> atPredicate;
	atPredicate.push_back("robot");
	atPredicate.push_back(robotLoc);
	currentState.setBooleanPredicate("at", atPredicate, true);
	
	//setting the box locations initially
	for(std::vector<std::string>::iterator it= boxes.begin(); it!= boxes.end(); ++it){
		std::vector<std::string>::iterator ij= boxLocs.begin();
		atPredicate.erase();
		atPredicate.push_back(*it);
		atPredicate.push_back(*ij);
		++ij;
		currentState.setBooleanPredicate("at", atPredicate, true);	
	
	}

	//setting the ball locations initially
	for(std::vector<std::string>::iterator it=balls.begin(); it!=balls.end(); ++it){
		std::vector<std::string>::iterator ij= ballLocs.begin();
		atPredicate.erase();
		atPredicate.push_back(*it);
		atPredicate.push_back(*ij);
		++ij;
		currentState.setBooleanPredicate("at", atPredicate, true);		
	
	}

		


        /*std::set<string> rooms;
        std::set<string> doors;
        std::set<string> static_objects;

        goal.setForEachGoalStatement("manipulation_location", "searched", true);
        goal.setForEachGoalStatement("movable_object", "tidy", true);
        goal.setForEachGoalStatement("arm", "hand-free", true);
        goal.setForEachGoalStatement("wipe_point", "wiped", true);

        currentState.setBooleanPredicate("can-grasp", "right_arm", true);
        currentState.setBooleanPredicate("can-grasp", "left_arm", true);

        currentState.setObjectFluent("arm-state", "right_arm", "arm_unknown");
        currentState.setObjectFluent("arm-state", "left_arm", "arm_unknown");
	*/
        return true;
    }

};

