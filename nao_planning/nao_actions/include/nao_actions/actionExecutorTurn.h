//#ifndef ACTION_EXECUTOR_GRASP_OBJECT_H
//#define ACTION_EXECUTOR_GRASP_OBJECT_H

#include "continual_planning_executive/actionExecutorActionlib.hpp"
#include "continual_planning_executive/symbolicState.h"
#include <nao_world_msgs/TurnAction.h>

namespace nao_actions
{

    class ActionExecutorTurn : public ActionExecutorActionlib<nao_world_msgs::TurnAction,
                                                    nao_world_msgs::TurnGoal, nao_world_msgs::TurnResult>
    {
        public:
            virtual bool fillGoal(nao_world_msgs::TurnGoal & goal,
                    const DurativeAction & a, const SymbolicState & current);

            virtual void updateState(const actionlib::SimpleClientGoalState & actionReturnState,
                    const nao_world_msgs::TurnResult & result,
                    const DurativeAction & a, SymbolicState & current);
    };

};

//#endif
