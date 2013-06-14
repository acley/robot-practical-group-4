#include "nao_actions/actionExecutorKick.h"
#include <pluginlib/class_list_macros.h>
#include <XmlRpcValue.h>
#include <XmlRpcException.h>

PLUGINLIB_DECLARE_CLASS(nao_actions, action_executor_kick,
        nao_actions::ActionExecutorKick,
        continual_planning_executive::ActionExecutorInterface)

namespace nao_actions
{

    bool ActionExecutorKick::fillGoal(nao_msgs::KickGoal & goal,
            const DurativeAction & a, const SymbolicState & current)
    {
        ROS_ASSERT(a.parameters.size() == 8);
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
            const nao_msgs::KickResult & result,
            const DurativeAction & a, SymbolicState & current)
    {
        ROS_INFO("Kick returned result");
        ROS_ASSERT(a.parameters.size() == 8);
        string robot = a.parameters[0];
	string robotDir= a.parameters[1];
        string ball = a.parameters[2];
        string loc_1 = a.parameters[3];
        string loc_2 = a.parameters[4];
        string loc_3 = a.parameters[5];
        string dir1 = a.parameters[6];
	string dir2 = a.parameters[7];

        
	if (actionReturnState == actionlib::SimpleClientGoalState::SUCCEEDED) {
		ROS_INFO("Kick succeeded.");
		current.setBooleanPredicate("at", ball + " " + loc_2, false);
		current.setBooleanPredicate("at", ball + " " + loc_3, true);
		current.setBooleanPredicate("clear", loc_3, false);
		current.setBooleanPredicate("clear", loc_2, true);

		ros::NodeHandle nhPriv("~");
		XmlRpc::XmlRpcValue ballLocs, balls;
		nhPriv.getParam("ballLocs", ballLocs);
		nhPriv.getParam("balls", balls);

		ROS_ASSERT(ballLocs.getType() == XmlRpc::XmlRpcValue::TypeArray);
		ROS_ASSERT(balls.getType() == XmlRpc::XmlRpcValue::TypeArray);	
		for(int i=0; i<ballLocs.size(); i++){
			if(static_cast<std::string>(balls[i]).compare(ball)==0){
				ballLocs[i]= loc_3;
				break;
			}
		}
		nhPriv.setParam("ballLocs", ballLocs);

        }
    }

};

