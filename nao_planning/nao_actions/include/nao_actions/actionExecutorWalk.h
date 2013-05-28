//#ifndef ACTION_EXECUTOR_GRASP_OBJECT_H
//#define ACTION_EXECUTOR_GRASP_OBJECT_H

#include "continual_planning_executive/actionExecutorActionlib.hpp"
#include "continual_planning_executive/symbolicState.h"
#include <test_msgs/WalkAction.h>

namespace nao_actions
{

    class ActionExecutorDrop : public ActionExecutorActionlib<test_msgs::WalkAction,
                                                    test_msgs::WalkGoal, test_msgs::WalkResult>
    {
        public:
            virtual bool fillGoal(test_msgs::WalkGoal & goal,
                    const DurativeAction & a, const SymbolicState & current);

            virtual void updateState(const actionlib::SimpleClientGoalState & actionReturnState,
                    const test_msgs::WalkResult & result,
                    const DurativeAction & a, SymbolicState & current);
    };

};

//#endif
