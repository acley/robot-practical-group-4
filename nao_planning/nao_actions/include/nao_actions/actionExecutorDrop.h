//#ifndef ACTION_EXECUTOR_GRASP_OBJECT_H
//#define ACTION_EXECUTOR_GRASP_OBJECT_H

#include "continual_planning_executive/actionExecutorActionlib.hpp"
#include "continual_planning_executive/symbolicState.h"
#include <nao_world_msgs/DropAction.h>

namespace nao_actions
{

    class ActionExecutorDrop : public ActionExecutorActionlib<nao_world_msgs::DropAction,
                                                    nao_world_msgs::DropGoal, nao_world_msgs::DropResult>
    {
        public:
            virtual bool fillGoal(nao_world_msgs::DropGoal & goal,
                    const DurativeAction & a, const SymbolicState & current);

            virtual void updateState(const actionlib::SimpleClientGoalState & actionReturnState,
                    const nao_world_msgs::DropResult & result,
                    const DurativeAction & a, SymbolicState & current);
    };

};

//#endif
