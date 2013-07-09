//#ifndef ACTION_EXECUTOR_GRASP_OBJECT_H
//#define ACTION_EXECUTOR_GRASP_OBJECT_H

#include "continual_planning_executive/actionExecutorActionlib.hpp"
#include "continual_planning_executive/symbolicState.h"
#include <nao_world_msgs/KickAction.h>

namespace nao_actions
{

    class ActionExecutorKick : public ActionExecutorActionlib<nao_world_msgs::KickAction,
                                                    nao_world_msgs::KickGoal, nao_world_msgs::KickResult>
    {
        public:
            virtual bool fillGoal(nao_world_msgs::KickGoal & goal,
                    const DurativeAction & a, const SymbolicState & current);

            virtual void updateState(const actionlib::SimpleClientGoalState & actionReturnState,
                    const nao_world_msgs::KickResult & result,
                    const DurativeAction & a, SymbolicState & current);
    };

};

//#endif
