#include "nao_actions/goalCreatorTidyupObjects.h"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <set>
#include <std_srvs/Empty.h>

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
        if (! worldResetPerformed)
        {
        	worldResetPerformed = true;
        	std_srvs::Empty msg;
        	ros::service::call("/tidyup/reset_world_interface", msg);
        }

        // first add the type hierarchy
        currentState.addSuperType("pose", "pose");
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

        currentState.printSuperTypes();

        // load object_locations
        std::string locationsFile;
        // load grasp_locations
        ROS_INFO("%s: file_name: %s", __PRETTY_FUNCTION__, locationsFile.c_str());

        std::set<string> rooms;
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

        return true;
    }

};

