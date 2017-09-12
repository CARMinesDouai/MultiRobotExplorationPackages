/**
Copyright (c) 2017 Guillaume Lozenguez 
Adapted as Movebase plugin by Xuan Sang LE <xsang.le@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
**/

#include "adaptive_local_planner.h"
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(local_planner::AdaptiveLocalPlanner, nav_core::BaseLocalPlanner)

namespace local_planner
{

/*AdaptiveLocalPlanner::AdaptiveLocalPlanner()
    {
    }*/

bool AdaptiveLocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
{
    if (!initialized_)
    {
        ROS_ERROR("AdaptiveLocalPlanner hasn't initialized correctly! Quit.");
        return false;
    }

    geometry_msgs::PoseStamped goal;
    if (!this->select_goal(&goal))
    {
        ROS_ERROR("No local goal is selected");
        return false;
    }
    tf::Vector3 _goal;
    _goal.setX(goal.pose.position.x);
    _goal.setY(goal.pose.position.y);
    _vmap->scan_subscriber(scan, true);
    _vmap->extraFrontierNodes( robot_radius*2.f );

    // Get Transforms (goal -> scan frames and scan -> commande frames):

    tf::StampedTransform goalToScan;
    try
    {
        tf->lookupTransform(scan.header.frame_id, goal_frame_id,ros::Time(0), goalToScan);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("TF: %s - %s: %s", scan.header.frame_id.c_str(), goal_frame_id.c_str() ,ex.what());
        return false;
    }

    tf::StampedTransform scanToCmd;
    try
    {
        tf->lookupTransform(cmd_frame_id, scan.header.frame_id,ros::Time(0), scanToCmd);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("TF: %s - %s: %s", cmd_frame_id.c_str(), scan.header.frame_id.c_str(), ex.what());
        return false;
    }

    tf::Vector3 localGoal(0.f, 0.f, 0.f);

    // If transforms exist move goal in scan frame:

    localGoal = goalToScan * _goal;

    mia::Float2 goalF2(localGoal.x(), localGoal.y());
    _vmap->add_vertex(goalF2, mia::Node2::type_free);

    mia::Float2 obs;

    // If goal not in direct area get closest frontier node:
    if (!_vmap->free_segment(mia::Float2(0.f, 0.f), goalF2) || _vmap->is_vertex_on_segment(mia::Float2(0.f, 0.f), goalF2, mia::Node2::type_obstacle, obs))
    {
        goalF2 = _vmap->get_closest_vertex(goalF2, mia::Node2::type_frontier);
        cout << "\tget frontier vertex: " << goalF2
                << "(" << _vmap->add_vertex(goalF2, mia::Node2::type_free)
                << ")" << endl;
    }

    if (_vmap->is_vertex_on_segment(mia::Float2(0.f, 0.f), goalF2,
                                    mia::Node2::type_obstacle, obs))
    {
        float d(obs.normalize());
        goalF2 = obs * (d - _vmap->visimap.getEpsilon());

        cout << "\tget safe obstacle position :" << goalF2
                << "(" << _vmap->add_vertex(goalF2, mia::Node2::type_free)
                << ")" << endl;
    }

    localGoal.setX(goalF2.x);
    localGoal.setY(goalF2.y);

    cout << "\tlocal goal ("
            << localGoal.x() << ", " << localGoal.y()
            << ") in " << scan.header.frame_id << endl;

    localGoal = scanToCmd * localGoal;

    cout << "\tlocal goal ("
            << localGoal.x() << ", " << localGoal.y()
            << ") in " << cmd_frame_id << endl;

    // generate appropriate commande message :

    cout << "\tmove to (" << _goal.x() << ", " << _goal.y()
            << ") in " << goal_frame_id << " -> ("
            << localGoal.x() << ", " << localGoal.y()
            << ") in " << cmd_frame_id << endl;

    // generate appropriate commande message :
    mia::Float2 norm_goal(localGoal.x(), localGoal.y());
    float d = norm_goal.normalize();

    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;

    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;

    cout << "-> generate the command: distance "
         << d << " vs " << _vmap->a_min_scan_distance * 0.2 << endl;

    if (d > _vmap->a_min_scan_distance * 0.2) // !stop condition :
    {
        if (localGoal.x() > _vmap->a_min_scan_distance * 0.2)
        {
            cmd_vel.linear.x = dmax_l_speed + min(0.5 * localGoal.x() * dmax_l_speed, dmax_l_speed);

            if (0.005f < norm_goal.y)
                cmd_vel.angular.z = min(norm_goal.y * 2.0 * max_a_speed, max_a_speed);

            if (-0.005f > norm_goal.y)
                cmd_vel.angular.z = -(min(norm_goal.y * -2.0 * max_a_speed, max_a_speed));
        }
        else
        {
            cmd_vel.angular.z = max_a_speed;
        }
    }
    else 
    {
        reached = true;
    }
    if (!_vmap->safe_move(mia::Float2(0.f, 0.f)))
    {
        cout << "\tlinear movement desabled" << endl;
        cmd_vel.linear.x = 0.f;
    }

    cout << "\tcommande linear: " << cmd_vel.linear.x << ", angular: " << cmd_vel.angular.z << endl;
    _vmap->publish_vmap(vmap_publisher, scan.header);
    return true;
}
/*
    get my pose on the base_link frame
*/
bool AdaptiveLocalPlanner::my_pose(geometry_msgs::PoseStamped *pose)
{
    tf::StampedTransform transform;
    try
    {
        this->tf->lookupTransform(this->goal_frame_id, this->cmd_frame_id, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("Cannot get transform %s-%s: %s", this->goal_frame_id.c_str(), this->cmd_frame_id.c_str(), ex.what());
        return false;
    }

    pose->header.stamp = ros::Time::now();
    pose->header.frame_id = this->goal_frame_id;

    pose->pose.position.x = transform.getOrigin().getX();
    pose->pose.position.y = transform.getOrigin().getY();
    pose->pose.position.z = transform.getOrigin().getZ();

    pose->pose.orientation.x = transform.getRotation().getX();
    pose->pose.orientation.y = transform.getRotation().getY();
    pose->pose.orientation.z = transform.getRotation().getZ();
    pose->pose.orientation.w = transform.getRotation().getW();
    return true;
}
bool AdaptiveLocalPlanner::select_goal(geometry_msgs::PoseStamped *_goal)
{
    geometry_msgs::PoseStamped pose;
    if (!this->my_pose(&pose))
    {
        ROS_ERROR("Cannot get pose on the goal frame: %s", this->goal_frame_id.c_str());
        return false;
    }
    // just get last pose in global goal for now
    if (this->global_plan.size() == 0)
        return false;
    geometry_msgs::PoseStamped tmpgoal; //mypose

    std::vector<geometry_msgs::PoseStamped>::iterator it;
    double d, dx, dy;

    tf::Vector3 candidate;
    for (it = this->global_plan.begin(); it != this->global_plan.end(); it++)
    {
        candidate.setX(it->pose.position.x);
        candidate.setY(it->pose.position.y);
        d = sqrt(pow(candidate.x() - pose.pose.position.x, 2) + pow(candidate.y() -  pose.pose.position.y, 2));
        if (d > max_local_goal_dist)
            break;
    }

    _goal->pose.position.x = candidate.x();
    _goal->pose.position.y = candidate.y();
    _goal->header.frame_id = this->goal_frame_id;

    local_goal_pub.publish(*_goal);
    return true;
}

bool AdaptiveLocalPlanner::isGoalReached()
{
    return this->reached;
}

bool AdaptiveLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
{
    if (!initialized_)
    {
        ROS_INFO("AdaptiveLocalPlanner hasn't initialized correctly! Quit.");
        return false;
    }
    this->global_plan.clear();
    this->global_plan = plan;
    this->reached = false;
    return true;
}
void AdaptiveLocalPlanner::initialize(std::string name, tf::TransformListener *tf, costmap_2d::Costmap2DROS *costmap_ros)
{
    /*some parameter*/
    if (!initialized_)
    {
        double dw, dh;
        ROS_INFO("Initialize adaptive local planner %s", name.c_str());
        private_nh = ros::NodeHandle(std::string("~") + "/" + name);
        private_nh.param<double>("robot_radius", this->robot_radius, 0.3f); 
        private_nh.param<double>("max_local_goal_dist", this->max_local_goal_dist, 0.5f);
        private_nh.param<double>("dmax_l_speed", this->dmax_l_speed, 0.2f);
        private_nh.param<double>("max_a_speed", this->max_a_speed, 1.2f);
        private_nh.param<std::string>("goal_frame_id", this->goal_frame_id, "map");
        private_nh.param<std::string>("cmd_frame_id", this->cmd_frame_id, "base_link");
        private_nh.param<std::string>("scan_topic", this->scan_topic, "/scan");
        private_nh.param<std::string>("vmap_topic", this->vmap_topic, "/scan_vmap");
        private_nh.param<bool>("verbose", this->verbose, true);
        private_nh.param<double>("perception_distance", this->perception_distance, 2.0f);

        local_goal_pub = private_nh.advertise<geometry_msgs::PoseStamped>("/local_goal", 1, true);
        vmap_publisher= private_nh.advertise<torob_msgs::VectorMap>( vmap_topic, 1, true);
        this->tf = tf;

        _vmap = new RosVmap();
        _vmap->setEpsilon(robot_radius);
        _vmap->a_min_scan_distance = 0.5 * robot_radius;
        _vmap->a_max_scan_distance = perception_distance;

        // subscribe to scan topic
        laser_sub = private_nh.subscribe<sensor_msgs::LaserScan>(this->scan_topic, 1,
                                                                 [this](const sensor_msgs::LaserScan::ConstPtr &msg) {
                                                                     this->scan = *msg;
                                                                 });
        initialized_ = true;
    }
    else
    {
        ROS_INFO("Adaptive local planner has ben initialized, do nothing");
    }
}
};