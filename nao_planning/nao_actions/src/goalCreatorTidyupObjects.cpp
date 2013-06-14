#include "nao_actions/goalCreatorTidyupObjects.h"
#include <pluginlib/class_list_macros.h>
#include <tf/transform_datatypes.h>
#include <set>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <XmlRpcValue.h>
#include <XmlRpcException.h>
#include <sstream>
#include <iostream>
#include <algorithm>


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
	

	
        
        currentState.printSuperTypes();

        

	nhPriv.getParam("cell_size", cell_size);
	nhPriv.getParam("grid_size", grid_size);
	nhPriv.getParam("robotLoc", robotLoc);
	nhPriv.getParam("robotDir", robotDir);
	nhPriv.getParam("goalLoc", goalLoc);
	nhPriv.getParam("boxLocs", xmlboxLocs);
	nhPriv.getParam("ballLocs", xmlballLocs);
	nhPriv.getParam("boxes", xmlboxes);
	nhPriv.getParam("balls", xmlballs);
	nhPriv.getParam("connections", xmlconnections);
	nhPriv.getParam("directions", xmldirections);
	nhPriv.getParam("leftOf", xmlLeftOf);

	ROS_ASSERT(xmlboxLocs.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(xmlballLocs.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(xmlboxes.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(xmlballs.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(xmlconnections.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(xmldirections.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(xmlLeftOf.getType() == XmlRpc::XmlRpcValue::TypeArray);

	//copying xmlrpc data to vectors
	for(int i=0; i<xmlboxLocs.size(); i++){
		boxLocs.push_back(xmlboxLocs[i]);
		boxes.push_back(xmlboxes[i]);
	}
	for(int i=0; i<xmlballLocs.size(); i++){
		ballLocs.push_back(xmlballLocs[i]);
		balls.push_back(xmlballs[i]);
	}
	for(int i=0; i<xmlconnections.size(); i++){
		connections.push_back(xmlconnections[i]);
	}
	for(int i=0; i<xmldirections.size(); i++){
		directions.push_back(xmldirections[i]);
	}
	for(int i=0; i<xmlLeftOf.size(); i++){
		leftOf.push_back(xmlLeftOf[i]);
	}

	//adding locations
	for(int i=1; i<=grid_size; i++)
		for(int j=1; j<=grid_size; j++){
			std::stringstream pos;
			pos << "pos-" << i << "-" << j;
			currentState.addObject(pos.str(), "location");
			goal.addObject(pos.str(), "location");
			currentState.setBooleanPredicate("clear", pos.str(), true);
		}

	//adding the directions
	for(int it= 0; it<directions.size(); ++it){
		currentState.addObject(directions[it], "direction");
		goal.addObject(directions[it], "direction");
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

	//defining the goal condition
	std::vector<std::string> goalPred;
	goalPred.push_back("robot");
	goalPred.push_back(goalLoc);
	goal.setBooleanPredicate("at", goalPred, true);
        
	//setting the clear locations
	/*for(int i= 1; i<=grid_size; i++)
		for(int j=1; j<=grid_size; j++){
			std::stringstream currLoc;
			currLoc << "pos-" << i << "-" << j;
			if(robotLoc.compare(currLoc.str())==0){
				currentState.setBooleanPredicate("clear", currLoc.str(), false);			
				continue;
			}
			std::vector<std::string>::iterator it;
			it= find(boxLocs.begin(), boxLocs.end(), currLoc.str());
			if(it!=boxLocs.end()){
				currentState.setBooleanPredicate("clear", currLoc.str(), false);			
				continue;
			}
			it= find(ballLocs.begin(), boxLocs.end(), currLoc.str());
			if(it!=ballLocs.end()){
				currentState.setBooleanPredicate("clear", currLoc.str(), false);			
				continue;
			}
			currentState.setBooleanPredicate("clear", currLoc.str(), true);
		}*/
	
	//setting the connections
	for(std::vector<std::string>::iterator it= connections.begin(); it!= connections.end(); ++it){
		currentState.setBooleanPredicate("MOVE-DIR", *it, true);
	}

	//setting leftOf
	for(std::vector<std::string>::iterator it= leftOf.begin(); it!= leftOf.end(); ++it){
		currentState.setBooleanPredicate("LeftOf", *it, true);
	}

	currentState.setBooleanPredicate("HandEmpty", "robot", true);
	
	//setting the robot location initially	
	std::vector<std::string> atPredicate;
	atPredicate.push_back("robot");
	atPredicate.push_back(robotLoc);
	currentState.setBooleanPredicate("at", atPredicate, true);
	currentState.setBooleanPredicate("clear", robotLoc, false);
	
	//setting the robot orientation initially
	std::vector<std::string> orient;
	orient.push_back("robot");
	orient.push_back(robotDir);
	currentState.setBooleanPredicate("Orientation", orient, true);

	//setting the box locations initially
	std::vector<std::string>::iterator ij= boxLocs.begin();
	for(std::vector<std::string>::iterator it= boxes.begin(); it!= boxes.end(); ++it){
		atPredicate.erase(atPredicate.begin(), atPredicate.end());
		atPredicate.push_back(*it);
		atPredicate.push_back(*ij);
		currentState.setBooleanPredicate("clear", *ij, false);		
		++ij;
		currentState.setBooleanPredicate("at", atPredicate, true);	
		
	}

	//setting the ball locations initially
	ij= ballLocs.begin();
	for(std::vector<std::string>::iterator it=balls.begin(); it!=balls.end(); ++it){
		atPredicate.erase(atPredicate.begin(), atPredicate.end());
		atPredicate.push_back(*it);
		atPredicate.push_back(*ij);		
		currentState.setBooleanPredicate("clear", *ij, false);
		++ij;
		currentState.setBooleanPredicate("at", atPredicate, true);
	
	}

		


       
        return true;
    }

};

