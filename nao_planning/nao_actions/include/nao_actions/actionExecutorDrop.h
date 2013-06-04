//#ifndef ACTION_EXECUTOR_GRASP_OBJECT_H
//#define ACTION_EXECUTOR_GRASP_OBJECT_H

#include "continual_planning_executive/actionExecutorActionlib.hpp"
#include "continual_planning_executive/symbolicState.h"
#include <nao_msgs/DropAction.h>

namespace nao_actions
{

    class ActionExecutorDrop : public ActionExecutorActionlib<nao_msgs::DropAction,
                                                    nao_msgs::DropGoal, nao_msgs::DropResult>
    {
        public:
            virtual bool fillGoal(nao_msgs::DropGoal & goal,
                    const DurativeAction & a, const SymbolicState & current);

            virtual void updateState(const actionlib::SimpleClientGoalState & actionReturnState,
                    const nao_msgs::DropResult & result,
                    const DurativeAction & a, SymbolicState & current);
    };

};

//#endif
