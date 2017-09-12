#ifndef ADAPTIVE_LOCAL_PLANNER_H
#define ADAPTIVE_LOCAL_PLANNER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_local_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#include "vmap/rosvmap.h"

using namespace std;
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

    std::vector<geometry_msgs::PoseStamped> global_plan;
    double robot_radius,perception_distance, max_local_goal_dist,dmax_l_speed,max_a_speed;
    std::string goal_frame_id;
    std::string cmd_frame_id;
    std::string scan_topic,vmap_topic;
    bool initialized_, verbose;
    bool reached;
    tf::TransformListener *tf;
    ros::NodeHandle private_nh;
    ros::Publisher local_goal_pub,vmap_publisher;
    ros::Subscriber laser_sub;
    RosVmap * _vmap;
    sensor_msgs::LaserScan scan;
};
};

#endif