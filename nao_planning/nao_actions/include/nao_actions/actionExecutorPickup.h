//#ifndef ACTION_EXECUTOR_GRASP_OBJECT_H
//#define ACTION_EXECUTOR_GRASP_OBJECT_H

#include "continual_planning_executive/actionExecutorActionlib.hpp"
#include "continual_planning_executive/symbolicState.h"
#include <nao_world_msgs/PickupAction.h>

namespace nao_actions
{

    class ActionExecutorPickup : public ActionExecutorActionlib<nao_world_msgs::PickupAction,
                                                    nao_world_msgs::PickupGoal, nao_world_msgs::PickupResult>
    {
        public:
            virtual bool fillGoal(nao_world_msgs::PickupGoal & goal,
                    const DurativeAction & a, const SymbolicState & current);

            virtual void updateState(const actionlib::SimpleClientGoalState & actionReturnState,
                    const nao_world_msgs::PickupResult & result,
                    const DurativeAction & a, SymbolicState & current);
    };

};

//#endif
