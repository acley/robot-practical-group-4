#include "nao_actions/actionExecutorWalk.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(nao_actions, action_executor_Walk,
        nao_actions::ActionExecutorWalk,
        continual_planning_executive::ActionExecutorInterface)

namespace nao_actions
{

    bool ActionExecutorWalk::fillGoal(test_msgs::WalkGoal & goal,
            const DurativeAction & a, const SymbolicState & current)
    {
        ROS_ASSERT(a.parameters.size() == 4);
        string robot = a.parameters[0];
        string loc_from = a.parameters[1];
        string loc_to = a.parameters[2];
        string direction = a.parameters[3]
        
        // set corresponding WalkGoal parameters
        // i.e.: goal.<fieldName> = <value>;
        // i.e.: goal.<fieldName> = a.parameters[...];

        return true;
    }

    void ActionExecutorWalk::updateState(const actionlib::SimpleClientGoalState & actionReturnState,
            const test_msgs::WalkResult & result,
            const DurativeAction & a, SymbolicState & current)
    {
        ROS_INFO("Walk returned result");
        ROS_ASSERT(a.parameters.size() == 4);
        string robot = a.parameters[0];
        string loc_from = a.parameters[1];
        string loc_to = a.parameters[2];
        string direction = a.parameters[3]]
        
        		//~ (:durative-action move
	   //~ :parameters (?r - robot ?from ?to - location ?dir - direction)
	   //~ :duration  (= ?duration 1)
	   //~ :condition (and (at start (at ?r ?from))
		           //~ (at start (clear ?to))
		           //~ (over all (MOVE-DIR ?from ?to ?dir))
		           //~ )
	   //~ :effect    (and (at start (not (at ?p ?from)))
		           //~ (at start (not (clear ?to)))
		           //~ (at end (at ?r ?to))
		           //~ (at end (clear ?from))
		           //~ )
	   //~ )
       
       if (actionReturnState == actionlib::SimpleClientGoalState::SUCCEEDED) {
           ROS_INFO("Walk succeeded.");
           current.setBooleanPredicate("at", robot + " " + loc_from, false);
           current.setBooleanPredicate("clear", loc_to, false);
           current.setBooleanPredicate("at", robot + " " + loc_to, true);
           current.setBooleanPredicate("clear", loc_from, true);
       }
    }

};

