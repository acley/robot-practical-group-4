//#ifndef ACTION_EXECUTOR_GRASP_OBJECT_H
//#define ACTION_EXECUTOR_GRASP_OBJECT_H

#include "continual_planning_executive/actionExecutorActionlib.hpp"
#include "continual_planning_executive/symbolicState.h"
#include <test_msgs/KickAction.h>

namespace nao_actions
{

    class ActionExecutorDrop : public ActionExecutorActionlib<test_msgs::KickAction,
                                                    test_msgs::KickGoal, test_msgs::KickResult>
    {
        public:
            virtual bool fillGoal(test_msgs::KickGoal & goal,
                    const DurativeAction & a, const SymbolicState & current);

            virtual void updateState(const actionlib::SimpleClientGoalState & actionReturnState,
                    const test_msgs::KickResult & result,
                    const DurativeAction & a, SymbolicState & current);
    };

};

//#endif
