//#ifndef ACTION_EXECUTOR_GRASP_OBJECT_H
//#define ACTION_EXECUTOR_GRASP_OBJECT_H

#include "continual_planning_executive/actionExecutorActionlib.hpp"
#include "continual_planning_executive/symbolicState.h"
#include <nao_msgs/PickupAction.h>
#include "nao_msgs/PickupActionServer.cpp"

namespace nao_actions
{

    class ActionExecutorPickup : public ActionExecutorActionlib<nao_msgs::PickupAction,
                                                    nao_msgs::PickupGoal, nao_msgs::PickupResult>
    {
        public:
	    nao_msgs::PickupActionServer as_;
            virtual bool fillGoal(nao_msgs::PickupGoal & goal,
                    const DurativeAction & a, const SymbolicState & current);

            virtual void updateState(const actionlib::SimpleClientGoalState & actionReturnState,
                    const nao_msgs::PickupResult & result,
                    const DurativeAction & a, SymbolicState & current);
    };

};

//#endif
