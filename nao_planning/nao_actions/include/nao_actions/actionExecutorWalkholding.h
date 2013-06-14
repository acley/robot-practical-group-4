//#ifndef ACTION_EXECUTOR_GRASP_OBJECT_H
//#define ACTION_EXECUTOR_GRASP_OBJECT_H

#include "continual_planning_executive/actionExecutorActionlib.hpp"
#include "continual_planning_executive/symbolicState.h"
#include <nao_msgs/WalkHoldingAction.h>

namespace nao_actions
{

    class ActionExecutorWalkHolding : public ActionExecutorActionlib<nao_msgs::WalkHoldingAction,
                                                    nao_msgs::WalkHoldingGoal, nao_msgs::WalkHoldingResult>
    {
        public:
            virtual bool fillGoal(nao_msgs::WalkHoldingGoal & goal,
                    const DurativeAction & a, const SymbolicState & current);

            virtual void updateState(const actionlib::SimpleClientGoalState & actionReturnState,
                    const nao_msgs::WalkHoldingResult & result,
                    const DurativeAction & a, SymbolicState & current);
    };

};

//#endif
