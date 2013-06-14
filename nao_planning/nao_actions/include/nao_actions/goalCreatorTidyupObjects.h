#ifndef GOAL_CREATOR_TIDYUP_OBJECTS_H
#define GOAL_CREATOR_TIDYUP_OBJECTS_H

#include "continual_planning_executive/goalCreator.h"
#include <ros/ros.h>

namespace nao_actions
{

    class GoalCreatorTidyupObjects : public continual_planning_executive::GoalCreator
    {
    private:
    	static bool worldResetPerformed;
        public:
            GoalCreatorTidyupObjects();
            ~GoalCreatorTidyupObjects();

            virtual bool fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal);
	protected:
		// load grid size, cell size, initial robot location and goal location
	// assume that the cells are 8 way connected
	// cell size is in cms
	int cell_size, grid_size;
	std::string robotLoc, goalLoc, robotDir;
	//check XmlRpc format
	XmlRpc::XmlRpcValue xmlboxLocs, xmlballLocs, xmlboxes, xmlballs, xmlconnections, xmldirections, xmlLeftOf;
	std::vector<std::string> boxLocs, ballLocs, boxes, balls, connections, directions, leftOf;
    };

};

#endif

