#include "nao_actions/actionExecutorTurn.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(nao_actions, action_executor_turn,
        nao_actions::ActionExecutorTurn,
        continual_planning_executive::ActionExecutorInterface)

namespace nao_actions
{

    bool ActionExecutorTurn::fillGoal(nao_world_msgs::TurnGoal & goal,
            const DurativeAction & a, const SymbolicState & current)
    {
        ROS_ASSERT(a.parameters.size() == 3);
        string robot = a.parameters[0];
        string dir_from = a.parameters[1];
        string dir_to = a.parameters[2];
        
        // set corresponding WalkGoal parameters
        // i.e.: goal.<fieldName> = <value>;
        // i.e.: goal.<fieldName> = a.parameters[...];

        return true;
    }


    void ActionExecutorTurn::updateState(const actionlib::SimpleClientGoalState & actionReturnState,
            const nao_world_msgs::TurnResult & result,
            const DurativeAction & a, SymbolicState & current)
    {
        ROS_INFO("Turn returned result");
        ROS_ASSERT(a.parameters.size() == 3);
        string robot = a.parameters[0];
        string dir_from = a.parameters[1];
        string dir_to = a.parameters[2];
       
        if (actionReturnState == actionlib::SimpleClientGoalState::SUCCEEDED) {
		ROS_INFO("Turn succeeded.");
		current.setBooleanPredicate("Orientation", robot + " " + dir_from, false);
		current.setBooleanPredicate("Orientation", robot + " " + dir_to, true);		

		/*ros::NodeHandle nhPriv("~");
		nhPriv.setParam("robotDir", dir_to);*/
		
       }
    }

};

