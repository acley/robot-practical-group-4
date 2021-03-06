#include "nao_actions/actionExecutorWalkholding.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(nao_actions, action_executor_walkholding,
        nao_actions::ActionExecutorWalkHolding,
        continual_planning_executive::ActionExecutorInterface)

namespace nao_actions
{

    bool ActionExecutorWalkHolding::fillGoal(nao_world_msgs::WalkHoldingGoal & goal,
            const DurativeAction & a, const SymbolicState & current)
    {
        ROS_ASSERT(a.parameters.size() == 4);
        string robot = a.parameters[0];
        string loc_from = a.parameters[1];
        string loc_to = a.parameters[2];
        string direction = a.parameters[3];
        
        ros::NodeHandle nhPriv("~");
        double cell_size;
		nhPriv.getParam("cell_size", cell_size);
		goal.distance = cell_size;
        
        // set corresponding WalkGoal parameters
        // i.e.: goal.<fieldName> = <value>;
        // i.e.: goal.<fieldName> = a.parameters[...];

        return true;
    }

    void ActionExecutorWalkHolding::updateState(const actionlib::SimpleClientGoalState & actionReturnState,
            const nao_world_msgs::WalkHoldingResult & result,
            const DurativeAction & a, SymbolicState & current)
    {
        ROS_INFO("WalkHolding returned result");
        ROS_ASSERT(a.parameters.size() == 4);
        string robot = a.parameters[0];
        string loc_from = a.parameters[1];
        string loc_to = a.parameters[2];
        string direction = a.parameters[3];
       
        if (actionReturnState == actionlib::SimpleClientGoalState::SUCCEEDED) {
		ROS_INFO("WalkHolding succeeded.");
		current.setBooleanPredicate("at", robot + " " + loc_from, false);
		current.setBooleanPredicate("clear", loc_to, false);
		current.setBooleanPredicate("at", robot + " " + loc_to, true);
		current.setBooleanPredicate("clear", loc_from, true);

		/*ros::NodeHandle nhPriv("~");
		nhPriv.setParam("robotLoc", loc_to);*/
		
       }
    }

};

