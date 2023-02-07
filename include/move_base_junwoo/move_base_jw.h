#pragma once

// includes c++ libraries
#include <vector>
#include <string>

#include <ros/ros.h>

// includes libraries that concerns action
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

// 
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
        /**
         * @brief Constructor for the actions
         * @param tf A reference to a TransformListener
        */
        MoveBase(tf2_ros::Buffer& tf);

        /**
         * @brief Destructor - Cleans up
        */
       virtual ~MoveBase();

        /**
         * @brief Performs a control cycle.
         * @param goal A reference to the goal to pursue
         * @return True if processing of the goal is done, False otherwise.
        */
       bool executeCycle(geometry_msgs::PoseStamped& goal);

    private:
        /**
         * @brief A service call that clears the costmaps of obstacles. Needed for recovery
         * @param req The service request. datatype: Empty
         * @param resp The service response. datatype: Empty
         * @return True if the service call succeeds, False otherwise
        */
        bool clearCostmapService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);

        /**
         * @brief A service call that can be made when the action is inactive that will return a plan
         * @param req The service request. datatype: geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal, float32 tolerance
         * @param resp The service response. datatype: nav_msgs::Plan plan
         * @return True if the service call succeeds, False otherwise
        */
        bool planService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);

        /**
         * @brief Make a new global plan
         * @param goal The goal of the plan
         * @param plan Will be filled in with the plan made by the planner
         * @return True if planning succeeds, False otherwise
        */
        bool makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

        /**
         * @brief Load the recovery behaviors for the navigation stack from the parameter server
         * @param node The ros::NodeHandle to be used for loading parameters
         * @return True if the recovery behaviors were loaded successfully, False otherwise
        */
        bool loadRecoveryBehaviors(ros::NodeHandle node);

        /**
         * @brief Loads the default recovery behaviors for the navigation stack
        */
        void loadDefaultRecoveryBehaviors();

        /**
         * @brief Clears obstacles within a window around the robot
         * @param size_x The x size of the window
         * @param size_y The y size of the window
        */
        void clearCostmapWindows(double size_x, double size_y);

        /**
         * @brief Pulbishes zero velocity command
        */
        void publishZeroVelocity();
        
        /**
         * @brief Reset the state of the move_base action and send a zero velocity command to the base
        */
        void resetState();

        /**
         * @brief Goal Callback function for action
         * @param goal Contains information about the goal
        */
        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);

        void planThread();

        /** @brief Execute Callback function for action
         * @param move_base_goal Contains information about goal
        */
        void executeCB(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal);

        bool isQuaternionValid(const geometry_msgs::Quaternion& q);

        bool getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap);

        double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);

        /**
         * @brief Converts goal in robot frame to global frame
         * @param goal_pose_msg Goal in robot's frame
        */
        geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);

        /**
         * @brief This is used to wake the planner at periodic intervals
        */
        void wakePlanner(const ros::TimerEvent& event);


        //parameters
        tf2_ros::Buffer& tf_;

        MoveBaseActionServer* as_;

        boost::shared_ptr<nav_core::BaseLocalPlanner> tc_;
        costmap_2d::Costmap2DROS *planner_costmap_ros_, *controller_costmap_ros_;
 
   };


}