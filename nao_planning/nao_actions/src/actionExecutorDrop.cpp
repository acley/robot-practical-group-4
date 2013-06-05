#include "nao_actions/actionExecutorDrop.h"
#include <pluginlib/class_list_macros.h>
#include <XmlRpcValue.h>
#include <XmlRpcException.h>

PLUGINLIB_DECLARE_CLASS(nao_actions, action_executor_drop,
        nao_actions::ActionExecutorDrop,
        continual_planning_executive::ActionExecutorInterface)

namespace nao_actions
{

    bool ActionExecutorDrop::fillGoal(nao_msgs::DropGoal & goal,
            const DurativeAction & a, const SymbolicState & current)
    {
        ROS_ASSERT(a.parameters.size() == 5);
        string robot = a.parameters[0];
        string box = a.parameters[1];
        string loc_1 = a.parameters[2];
        string loc_2 = a.parameters[3];
        string direction = a.parameters[4];
        
        // set corresponding DropGoal parameters
        // i.e.: goal.<fieldName> = <value>;
        // i.e.: goal.<fieldName> = a.parameters[...];

        return true;
    }

    void ActionExecutorDrop::updateState(const actionlib::SimpleClientGoalState & actionReturnState,
            const nao_msgs::DropResult & result,
            const DurativeAction & a, SymbolicState & current)
    {
        ROS_INFO("Drop returned result");
        ROS_ASSERT(a.parameters.size() == 5);
        string robot = a.parameters[0];
        string box = a.parameters[1];
        string loc_1 = a.parameters[2];
        string loc_2 = a.parameters[3];
        string direction = a.parameters[4];
        
	if (actionReturnState == actionlib::SimpleClientGoalState::SUCCEEDED) {
		ROS_INFO("Drop succeeded");
		current.setBooleanPredicate("HandEmpty", robot, true);
		current.setBooleanPredicate("at", box + " " + loc_2, true);
		current.setBooleanPredicate("Holding", robot + " " + box, false);
		current.setBooleanPredicate("clear", loc_2, false);	

		ros::NodeHandle nhPriv("~");
		XmlRpc::XmlRpcValue xmlBoxes, xmlBoxLocs;
		nhPriv.getParam("boxLocs", xmlBoxLocs);
		nhPriv.getParam("boxes", xmlBoxes);
		ROS_ASSERT(xmlBoxLocs.getType() == XmlRpc::XmlRpcValue::TypeArray);
		ROS_ASSERT(xmlBoxes.getType() == XmlRpc::XmlRpcValue::TypeArray);
		for(int i=0; i<xmlBoxLocs.size(); i++){
			if(static_cast<std::string>(xmlBoxes[i]).compare(box)==0){
				xmlBoxLocs[i]= loc_2;
				break;
			}
		}
		nhPriv.setParam("boxLocs", xmlBoxLocs);
        }
    }

};

