//#ifndef ACTION_EXECUTOR_GRASP_OBJECT_H
//#define ACTION_EXECUTOR_GRASP_OBJECT_H

#include "continual_planning_executive/actionExecutorActionlib.hpp"
#include "continual_planning_executive/symbolicState.h"
#include <nao_msgs/WalkAction.h>

namespace nao_actions
{

    class ActionExecutorWalk : public ActionExecutorActionlib<nao_msgs::WalkAction,
                                                    nao_msgs::WalkGoal, nao_msgs::WalkResult>
    {
        public:
            virtual bool fillGoal(nao_msgs::WalkGoal & goal,
                    const DurativeAction & a, const SymbolicState & current);

            virtual void updateState(const actionlib::SimpleClientGoalState & actionReturnState,
                    const nao_msgs::WalkResult & result,
                    const DurativeAction & a, SymbolicState & current);
    };

};

//#endif
