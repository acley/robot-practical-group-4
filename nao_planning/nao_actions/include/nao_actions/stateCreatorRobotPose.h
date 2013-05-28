#ifndef STATE_CREATOR_ROBOT_POSE_H
#define STATE_CREATOR_ROBOT_POSE_H

#include "continual_planning_executive/stateCreator.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

namespace nao_actions 
{


    /// This state creator adds the current robot pose to the state.
    /**
     * The current pose is estimated via tf as the transform from /map to /base_link.
     * A stamped pose in the state is represented by the fluents: x y z qx qy qz qw as well as timestamp and frame-id
     *
     * The state creator serves two purposes:
     * 1. The fluents for the robot_pose are set to the real values
     * 2. An at-style predicate of the form (at location) is set.
     *
     * 1. If _robotPoseObject is not empty the x,y,z,etc. fluents will be filled accordingly.
     * 2.a If the _atPredicate is not empty, the at predicate will be set to the current robot pose.
     * 2.b If a _locationType is given it is checked if the current robot pose is one of the
     *      poses in the objects of _locationType and thus _atPredicate is possibly set to
     *      a location instead of the current pose.
     *      If a robot is "at" a location is determined by the _goalToleranceXY and _goalToleranceYaw.
     */
    class StateCreatorRobotPose : public continual_planning_executive::StateCreator
    {
        public:
            StateCreatorRobotPose();
            ~StateCreatorRobotPose();

            
            virtual void initialize();

            virtual bool fillState(SymbolicState & state);

        protected:
            void publishLocationsAsMarkers(const SymbolicState & state);

        protected:
	
	    int cell_size, grid_size;
	    std::string robotLoc, goalLoc;
	    std::vector<std::string> boxLocs, ballLocs, boxes, balls;	    

            tf::TransformListener _tf;

            double _goalToleranceXY;
            double _goalToleranceYaw;

            static const bool s_PublishLocationsAsMarkers = true;
            static const bool s_PublishMeshMarkers = true;

            ros::Publisher _markerPub;

            std::string _robotPoseObject;   ///< the name of the robot pose's object (e.g. robot_pose, or l0)
            std::string _robotPoseType;     ///< the type of the _robotPoseObject - required if _robotPoseObject.
            std::string _atPredicate;       ///< the name of the "at" predicate (e.g. at-base)
            std::string _locationType;      ///< the type of location objects that a robot might be "at"

    };

};

#endif

