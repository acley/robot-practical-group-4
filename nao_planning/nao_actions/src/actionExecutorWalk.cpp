#include "nao_actions/actionExecutorWalk.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(nao_actions, action_executor_walk,
        nao_actions::ActionExecutorWalk,
        continual_planning_executive::ActionExecutorInterface)

namespace nao_actions
{

    bool ActionExecutorWalk::fillGoal(nao_world_msgs::WalkGoal & goal,
            const DurativeAction & a, const SymbolicState & current)
    {
        ROS_ASSERT(a.parameters.size() == 4);
        string robot = a.parameters[0];
        string loc_from = a.parameters[1];
        string loc_to = a.parameters[2];
        string direction = a.parameters[3];
        
        // extract coordinates from loc_to
        /*string x_pos= loc_to.substr(loc_to.find("-")+1);
        goal.x_loc= str2num(x_pos.substr(0,x_pos.find("-")));
        goal.y_loc= str2num(x_pos.substr(x_pos.find("-")+1));*/
        
        ros::NodeHandle nhPriv("~");
        double cell_size;
        nhPriv.getParam("cell_size", cell_size);
        goal.distance = cell_size;
        
        // set corresponding WalkGoal parameters
        // i.e.: goal.<fieldName> = <value>;
        // i.e.: goal.<fieldName> = a.parameters[...];

        return true;
    }

    void ActionExecutorWalk::updateState(const actionlib::SimpleClientGoalState & actionReturnState,
            const nao_world_msgs::WalkResult & result,
            const DurativeAction & a, SymbolicState & current)
    {
        ROS_INFO("Walk returned result");
        ROS_ASSERT(a.parameters.size() == 4);
        string robot = a.parameters[0];
        string loc_from = a.parameters[1];
        string loc_to = a.parameters[2];
        string direction = a.parameters[3];
      
        if (actionReturnState == actionlib::SimpleClientGoalState::SUCCEEDED) {
		    ROS_INFO("Walk succeeded.");
		    current.setBooleanPredicate("at", robot + " " + loc_from, false);
		    //current.setBooleanPredicate("clear", loc_to, false);
		    //current.setBooleanPredicate("at", robot + " " + loc_to, true);
		    current.setBooleanPredicate("clear", loc_from, true);

		    //SnhPriv.setParam("robotLoc", loc_to);
		
       }
    }

};

