#pragma once

// 
#include <vector>
#include <string>

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/GetPlan.h>

#include <pluginlib/class_loader.h>
#include <std_srvs/Empty.h>

#include "move_base/MoveBaseConfig.h"

namespace move_base_jw{
    // typedefs to help us out with the action server so that we don't have to type so much
    typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

    enum MoveBaseState{
        PLANNING,
        CONTROLLING,
        CLEARING
    };

    enum RecoveryTrigger{
        PLANNING_R,
        CONTROLLING_R,
        OSCILLATION_R
    };

    /**
     * @class MoveBaseJW
     * @brief A class that uses the actionlib::ActionServer interface that moves the robot base to a goal location
    */
   class MoveBase{
    public:

   }


}