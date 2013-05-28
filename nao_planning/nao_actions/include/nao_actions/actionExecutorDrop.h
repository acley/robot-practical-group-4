//#ifndef ACTION_EXECUTOR_GRASP_OBJECT_H
//#define ACTION_EXECUTOR_GRASP_OBJECT_H

#include "continual_planning_executive/actionExecutorActionlib.hpp"
#include "continual_planning_executive/symbolicState.h"
#include <test_msgs/DropAction.h>

namespace nao_actions
{

    class ActionExecutorDrop : public ActionExecutorActionlib<test_msgs::DropAction,
                                                    test_msgs::DropGoal, test_msgs::DropResult>
    {
        public:
            virtual bool fillGoal(test_msgs::PickupGoal & goal,
                    const DurativeAction & a, const SymbolicState & current);

            virtual void updateState(const actionlib::SimpleClientGoalState & actionReturnState,
                    const test_msgs::PickupResult & result,
                    const DurativeAction & a, SymbolicState & current);
    };

};

//#endif
