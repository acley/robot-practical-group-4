#include "nao_actions/actionExecutorKick.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(nao_actions, action_executor_Kick,
        nao_actions::ActionExecutorKick,
        continual_planning_executive::ActionExecutorInterface)

namespace nao_actions
{

    bool ActionExecutorKick::fillGoal(test_msgs::KickGoal & goal,
            const DurativeAction & a, const SymbolicState & current)
    {
        ROS_ASSERT(a.parameters.size() == 6);
        string robot = a.parameters[0];
        string ball = a.parameters[1];
        string loc_1 = a.parameters[2];
        string loc_2 = a.parameters[3];
        string loc_3 = a.parameters[4];
        string direction = a.parameters[5];
        
        // set corresponding KickGoal parameters
        // i.e.: goal.<fieldName> = <value>;
        // i.e.: goal.<fieldName> = a.parameters[...];

        return true;
    }

    void ActionExecutorKick::updateState(const actionlib::SimpleClientGoalState & actionReturnState,
            const test_msgs::KickResult & result,
            const DurativeAction & a, SymbolicState & current)
    {
        ROS_INFO("Kick returned result");
        ROS_ASSERT(a.parameters.size() == 6);
        string robot = a.parameters[0];
        string ball = a.parameters[1];
        string loc_1 = a.parameters[2];
        string loc_2 = a.parameters[3];
        string loc_3 = a.parameters[4];
        string direction = a.parameters[5]
        
        	//~ (:durative-action kick
		//~ :parameters (?r - robot ?b - ball ?l1 ?l2 ?l3 - location ?d1 - direction)
		//~ :duration (= ?duration 1)
		//~ :condition (and (at start (at ?b ?l2))
				//~ (at start (at ?r ?l1)) 
				//~ (at start (clear ?l3))
				//~ (at start (MOVE-DIR ?l1 ?l2 ?d1)) 
				//~ (at start (MOVE-DIR ?l2 ?l3 ?d1)) 
				//~ )
		//~ :effect	(and (at end (not (at ?b ?l2)))
			     //~ (at end (at ?b ?l3)) 
			     //~ (at end (not (clear ?l3))) 
			     //~ (at end(clear ?l2)) 
			     //~ ) 
		//~ )))
        
        if (actionReturnState == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Kick succeeded.");
            current.setBooleanPredicate("at", box + " " + loc_2, false);
            current.setBooleanPredicate("at", box + " " + loc_3, true);
            current.setBooleanPredicate("clear", loc_3, false);
            current.setBooleanPredicate("clear", loc_2, true);
        }
    }

};

