#include "nao_actions/actionExecutorPickup.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(nao_actions, action_executor_pickup,
        nao_actions::ActionExecutorPickup,
        continual_planning_executive::ActionExecutorInterface)

namespace nao_actions
{

    bool ActionExecutorPickup::fillGoal(nao_msgs::PickupGoal & goal,
            const DurativeAction & a, const SymbolicState & current)
    {
        ROS_ASSERT(a.parameters.size() == 5);
        string robot = a.parameters[0];
        string box = a.parameters[1];
        string loc_1 = a.parameters[2];
        string loc_2 = a.parameters[3];
        string direction = a.parameters[4];
        
        // set corresponding PickupGoal parameters
        // i.e.: goal.<fieldName> = <value>;
        // i.e.: goal.<fieldName> = a.parameters[...];

        return true;
    }

    void ActionExecutorPickup::updateState(const actionlib::SimpleClientGoalState & actionReturnState,
            const nao_msgs::PickupResult & result,
            const DurativeAction & a, SymbolicState & current)
    {
        ROS_INFO("PickupObject returned result");
        ROS_ASSERT(a.parameters.size() == 5);
        string robot = a.parameters[0];
        string box = a.parameters[1];
        string loc_1 = a.parameters[2];
        string loc_2 = a.parameters[3];
        string direction = a.parameters[4];
        
        	//~ (:durative-action pick-up
		//~ :parameters (?r - robot ?b - box ?l1 ?l2 - location ?d - direction)
		//~ :duration (= ?duration 3)
		//~ :condition (and (at start (HandEmpty ?r)) 
				//~ (at start (at ?r ?l1)) 
				//~ (at start (at ?b ?l2)) 
				//~ (at start (MOVE-DIR ?l1 ?l2 ?d)) 
				//~ )
		//~ :effect (and (at end (not (HandEmpty ?r)))
			     //~ (at end (not (at ?b ?l2)))
			     //~ (at end (clear ?l2)) 
                             //~ (at end (Holding ?r ?b)) 
			     //~ )
		//~ )
        
        if (actionReturnState == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Pickup succeeded.");
            current.setBooleanPredicate("HandEmpty", robot, false);
            current.setBooleanPredicate("at", box + " " + loc_2, false);
            current.setBooleanPredicate("clear", loc_2, true);
            current.setBooleanPredicate("Holding", robot + " " + box, true);
        }
    }

};

