#ifndef ADAPTIVE_LOCAL_PLANNER
#define ADAPTIVE_LOCAL_PLANNER

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_local_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <geometry_msgs/PoseArray.h>
#include "3rd/mapbuilder.h"
#include "simple_ccl.h"
namespace local_planner
{

class AdaptiveLocalPlanner : public nav_core::BaseLocalPlanner
{
  public:
    /**
        * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
        * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
        * @return True if a valid velocity command was found, false otherwise
        */
    bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

    /**
        * @brief  Check if the goal pose has been achieved by the local planner
        * @return True if achieved, false otherwise
        */
    bool isGoalReached();

    /**
        * @brief  Set the plan that the local planner is following
        * @param orig_global_plan The plan to pass to the local planner
        * @return True if the plan was updated successfully, false otherwise
        */
    bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan);

    /**
        * @brief  Constructs the local planner
        * @param name The name to give this instance of the local planner
        * @param tf A pointer to a transform listener
        * @param costmap The cost map to use for assigning costs to local plans
        */
    void initialize(std::string name, tf::TransformListener *tf, costmap_2d::Costmap2DROS *costmap_ros);

    /**
        * @brief  Virtual destructor for the interface
        */
    ~AdaptiveLocalPlanner() {}
    AdaptiveLocalPlanner()
    {
        initialized_ = false;
        reached = false;
    }

  private:
    bool select_goal(geometry_msgs::PoseStamped *);
    bool my_pose(geometry_msgs::PoseStamped *);
    double dist(geometry_msgs::Point to);

    map<int,geometry_msgs::Point> cc_min_dist_to_robot();

    double robot_radius;
    double map_resolution;
    int fw, fh, min_obstacle_size_px;
    std::string goal_frame_id;
    std::string cmd_frame_id;
    std::string scan_topic;
    double attractive_gain, repulsive_gain, safe_goal_dist, safe_obs_dist,max_local_goal_dist;
    std::vector<geometry_msgs::PoseStamped> global_plan;
    bool initialized_;
    bool reached;
    tf::TransformListener *tf;
    ros::NodeHandle private_nh;
    ros::Publisher local_pub, local_goal_pub, obstacles_pub;
    ros::Subscriber laser_sub;
    nav_msgs::OccupancyGrid local_map;
    local_map::MapBuilder *map_builder;
};
};
#endif
