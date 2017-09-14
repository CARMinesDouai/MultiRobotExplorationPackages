/**
Copyright (c) 2017 Xuan Sang LE <xsang.le@gmail.com>

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

#include "pf_local_planner.h"
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(local_planner::PFLocalPlanner, nav_core::BaseLocalPlanner)

namespace local_planner
{

/*PFLocalPlanner::PFLocalPlanner()
    {
    }*/

map<int, geometry_msgs::Point> PFLocalPlanner::cc_min_dist_to_robot(tf::StampedTransform localToCmd, geometry_msgs::PoseStamped pose)
{
    
    map<int, geometry_msgs::Point> obs;
    SimpleCCL cclo;
    cclo.th = cosmap_th;
    cclo.setMap(this->local_map);
    //cclo.print();
    if (cclo.labels.size() == 0)
        return obs;
    set<int>::iterator it;

    geometry_msgs::Point dist;
    dist.z = 0;
    dist.x = -1.0;
    dist.y = -1.0;
    // init
    for (it = cclo.labels.begin(); it != cclo.labels.end(); it++)
    {
        if (cclo.labels_tree[*it].cnt < min_obstacle_size_px) continue;
        obs[*it] = dist;
    }
    geometry_msgs::Point offset;
    offset.x = this->local_map.info.origin.position.x;
    offset.y = this->local_map.info.origin.position.y;
    double resolution = this->local_map.info.resolution;
    tf::Vector3 tmp;
    int i, j, idx, cell;
    for (i = 0; i < cclo.dw; i++)
        for (j = 0; j < cclo.dh; j++)
        {
            idx = j * cclo.dw + i;
            cell = cclo.data[idx];
            if (cell != -1)
            {
                if (cclo.labels_tree[cell].cnt < min_obstacle_size_px) continue;

                tmp.setX(i * resolution + offset.x);
                tmp.setY(j * resolution + offset.y);
                tmp = localToCmd*tmp;
                dist.x = tmp.x();
                dist.y = tmp.y();
                dist.z = this->dist(pose.pose.position, dist);
                if (obs[cell].z == 0 || obs[cell].z > dist.z)
                {
                    obs[cell] = dist;
                }
            }
        }

    map<int, geometry_msgs::Point>::iterator mit;
    geometry_msgs::PoseArray poses;
    poses.header.stamp = ros::Time::now();
    poses.header.frame_id = this->cmd_frame_id;
    for (mit = obs.begin(); mit != obs.end(); mit++)
    {
        geometry_msgs::Pose p;
        p.position.x = mit->second.x;
        p.position.y = mit->second.y;
        p.orientation.w = 1.0;
        poses.poses.push_back(p);
    }
    obstacles_pub.publish(poses);

    return obs;
}
bool PFLocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
{
    geometry_msgs::PoseStamped pose;
    if (!this->my_pose(&pose))
    {
        ROS_ERROR("Cannot get pose on the goal frame: %s", this->goal_frame_id.c_str());
        return false;
    }

    geometry_msgs::PoseStamped goal;
    if (!this->select_goal(&goal))
    {
        ROS_ERROR("No local goal is selected");
        return false;
    }

    tf::StampedTransform localToCmd;
    try
    {
        this->tf->lookupTransform(this->cmd_frame_id, this->local_map.header.frame_id, ros::Time(0), localToCmd);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("TF: %s - %s : %s", this->cmd_frame_id.c_str(), this->local_map.header.frame_id.c_str() , ex.what());
        return false;
    }
    

    // now calculate the potential field toward the local goal
    double resolution = this->local_map.info.resolution;

    geometry_msgs::Point fatt;
    geometry_msgs::Point frep;
    fatt.x = 0.0;
    fatt.y = 0.0;
    frep.x = 0.0;
    frep.y = 0.0;

    // attractive potential
    double robot_to_goal = this->dist(pose.pose.position, goal.pose.position);

    if (robot_to_goal < 0.1) this->reached = true;

    if (robot_to_goal < safe_goal_dist)
    {
        fatt.x = attractive_gain * (goal.pose.position.x - pose.pose.position.x);
        fatt.y = attractive_gain * (goal.pose.position.y - pose.pose.position.y);
    }
    else
    {
        fatt.x = attractive_gain*(goal.pose.position.x - pose.pose.position.x) * safe_goal_dist / robot_to_goal;
        fatt.y = attractive_gain*(goal.pose.position.y - pose.pose.position.y) * safe_goal_dist / robot_to_goal;
    }

    // repulsive potential to the nearest obstacles

    int i, j, npix = 0;

    map<int, geometry_msgs::Point> obstacles = this->cc_min_dist_to_robot(localToCmd, pose);
    //return false;
    map<int, geometry_msgs::Point>::iterator mit;
    for (mit = obstacles.begin(); mit != obstacles.end(); mit++)
    {
        //double theta = atan2(mit->second.y, mit->second.x);
        if (mit->second.z <= this->safe_obs_dist)
        {
            double tmp;
            
            tmp = repulsive_gain * (1.0 / safe_obs_dist - 1.0 / mit->second.z) / (mit->second.z * mit->second.z);
            frep.x += tmp * mit->second.x;
            frep.y += tmp * mit->second.y;
        }
        else
        {
            frep.x += 0;
            frep.y += 0;
        }
    }

    if(verbose)
        ROS_INFO("attractive: %f, %f Respulsive: %f %f", fatt.x, fatt.y, frep.x, frep.y);
    // now calculate the velocity
    tf::Vector3 cmd;
    cmd.setX(fatt.x + frep.x);
    cmd.setY(fatt.y + frep.y);

    // now make a prediction of the future destination
    geometry_msgs::PoseStamped future_pose;

    int attemp = 10;
    bool is_collision = true;
    double yaw, future_d, theta;
    while(attemp != 0 && is_collision)
    {
        is_collision = false;

        future_pose.header.stamp = ros::Time::now();
        future_pose.header.frame_id = cmd_frame_id;
        yaw = atan2(cmd.y(), cmd.x()) - tf::getYaw(pose.pose.orientation);
        future_pose.pose.position.x = cmd.x() + (pose.pose.position.x*cos(yaw) - pose.pose.position.y*sin(yaw));
        future_pose.pose.position.y = cmd.y() + (pose.pose.position.x*sin(yaw) + pose.pose.position.y*cos(yaw));
    
        futur_pose_pub.publish(future_pose);

        geometry_msgs::Point normof_f, normof_obs;
        future_d = this->dist(future_pose.pose.position, pose.pose.position);
        normof_f.x = (future_pose.pose.position.x - pose.pose.position.x)/future_d;
        normof_f.y = (future_pose.pose.position.y - pose.pose.position.y)/future_d;

        for (mit = obstacles.begin(); mit != obstacles.end(); mit++)
        {
            normof_obs.x = (mit->second.x - pose.pose.position.x)/mit->second.z;
            normof_obs.y =  (mit->second.y - pose.pose.position.y)/mit->second.z;
            theta = acos(normof_f.x*normof_obs.x + normof_f.y*normof_obs.y);
            if(theta > M_PI/2.0) continue;
            double x = fabs(mit->second.z*sin(theta));
            if(x < robot_radius && mit->second.z < future_d)
            {
                double vf = (1.0 / robot_radius - 1.0 / mit->second.z) / (mit->second.z * mit->second.z);
                cmd.setX( cmd.x() + 0.5f*vf* mit->second.x);
                cmd.setY( cmd.y()+ 0.5f*vf* mit->second.y);
                is_collision = true;
                //break;
            }
        }
        attemp--;
    }


    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0; // ?
    cmd_vel.angular.z = yaw;
    if(is_collision)
    {
        ROS_ERROR("There will be a collision if i take this direction. I stop");
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
    }
    else 
    {
        
        cmd_vel.linear.x = cmd.x();
        cmd_vel.linear.y = cmd.y();
        // check if v is so big
        double v = sqrt(pow(cmd_vel.linear.x, 2)+ pow(cmd_vel.linear.y, 2));
        if(v > max_linear_v)
        {
            if(verbose) ROS_INFO("V is too big, scale it down");
            theta = atan2(cmd_vel.linear.y, cmd_vel.linear.x);
            cmd_vel.linear.x = max_linear_v*cos(theta);
            cmd_vel.linear.y = max_linear_v*sin(theta);
        }
    }
    
    if(verbose)
        ROS_INFO("CMD VEL: x:%f y:%f w:%f", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    return true;
}
/*
    get my pose on the base_link frame
*/
bool PFLocalPlanner::my_pose(geometry_msgs::PoseStamped *pose)
{
    tf::StampedTransform transform;
    try
    {
        this->tf->lookupTransform(cmd_frame_id, this->cmd_frame_id, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("Cannot get transform %s-%s: %s",cmd_frame_id.c_str(), this->cmd_frame_id.c_str(), ex.what());
        return false;
    }

    pose->header.stamp = ros::Time::now();
    pose->header.frame_id = cmd_frame_id;

    pose->pose.position.x = transform.getOrigin().getX();
    pose->pose.position.y = transform.getOrigin().getY();
    pose->pose.position.z = transform.getOrigin().getZ();

    pose->pose.orientation.x = transform.getRotation().getX();
    pose->pose.orientation.y = transform.getRotation().getY();
    pose->pose.orientation.z = transform.getRotation().getZ();
    pose->pose.orientation.w = transform.getRotation().getW();
    return true;
}
bool PFLocalPlanner::select_goal(geometry_msgs::PoseStamped *_goal)
{
    // just get last pose in global goal for now
    if (this->global_plan.size() == 0)
        return false;
   
    geometry_msgs::PoseStamped pose;
    if (!this->my_pose(&pose))
    {
        ROS_ERROR("Cannot get pose on the goal frame: %s", this->goal_frame_id.c_str());
        return false;
    }

    tf::StampedTransform goalToLocal;
    try
    {
        this->tf->lookupTransform(this->cmd_frame_id, this->local_map.header.frame_id, ros::Time(0), goalToLocal);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("Goal To Local: %s", ex.what());
        return false;
    }
    std::vector<geometry_msgs::PoseStamped>::iterator it;
    double d, dx, dy;
    bool has_goal = false;
    tf::Vector3 candidate;
    for (it = this->global_plan.begin(); it != this->global_plan.end(); it++)
    {
        candidate.setX(it->pose.position.x);
        candidate.setY(it->pose.position.y);
        candidate = goalToLocal * candidate;
        geometry_msgs::Point tmp;
        tmp.x = candidate.x();
        tmp.y = candidate.y();
        d = this->dist(pose.pose.position,tmp);
        if (d > max_local_goal_dist)
            break;
    }

    _goal->pose.position.x = candidate.x();
    _goal->pose.position.y = candidate.y();

    _goal->header.frame_id = this->cmd_frame_id;
    local_goal_pub.publish(*_goal);
    return true;
}

double PFLocalPlanner::dist(geometry_msgs::Point from, geometry_msgs::Point to)
{
    // Euclidiant dist between a point and the robot
    return sqrt(pow(to.x - from.x, 2) + pow(to.y - from.y, 2));
}
bool PFLocalPlanner::isGoalReached()
{
    return this->reached;
}

bool PFLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
{
    if (!initialized_)
    {
        ROS_INFO("PFLocalPlanner hasn't initialized correctly! Quit.");
        return false;
    }
    this->global_plan.clear();
    this->global_plan = plan;
    this->reached = false;
    return true;
}
void PFLocalPlanner::initialize(std::string name, tf::TransformListener *tf, costmap_2d::Costmap2DROS *costmap_ros)
{
    /*some parameter*/
    if (!initialized_)
    {
        double dw, dh;
        ROS_INFO("Initialize adaptive local planner %s", name.c_str());
        private_nh = ros::NodeHandle(std::string("~") + "/" + name);
        private_nh.param<double>("robot_radius", this->robot_radius, 0.3f);
        private_nh.param<std::string>("goal_frame_id", this->goal_frame_id, "map");
        private_nh.param<std::string>("cmd_frame_id", this->cmd_frame_id, "base_link");
        private_nh.param<std::string>("local_map_topic", this->local_map_topic, "/move_base/local_costmap/costmap");
        private_nh.param<double>("attractive_gain", this->attractive_gain, 1.0);
        private_nh.param<double>("repulsive_gain", this->repulsive_gain, 1.0);
        private_nh.param<double>("safe_goal_dist", this->safe_goal_dist, 1.0);
        private_nh.param<double>("safe_obs_dist", this->safe_obs_dist, 1.0);
        private_nh.param<double>("max_local_goal_dist", this->max_local_goal_dist, 0.5);
        private_nh.param<int>("min_obstacle_size_px", this->min_obstacle_size_px, 20);
        private_nh.param<bool>("verbose", this->verbose, true);
        private_nh.param<int>("cosmap_th", cosmap_th, 90);
        private_nh.param<double>("max_linear_v", max_linear_v, 0.3);
        ROS_INFO("Robot radius %f", this->robot_radius);
        ROS_INFO("Robot goal_frame_id %s", this->goal_frame_id.c_str());
        ROS_INFO("Robot command_frame_id %s", this->cmd_frame_id.c_str());
        ROS_INFO("localmap: %s", this->local_map_topic.c_str());
        ROS_INFO("Local map res: %f", this->map_resolution);

        local_goal_pub = private_nh.advertise<geometry_msgs::PoseStamped>("/local_goal", 1, true);
        futur_pose_pub = private_nh.advertise<geometry_msgs::PoseStamped>("/future_pose", 1, true);
        obstacles_pub = private_nh.advertise<geometry_msgs::PoseArray>("/obstacles", 1, true);
        this->tf = tf;
        // subscribe to scan topic
        cmap_sub = private_nh.subscribe<nav_msgs::OccupancyGrid>(this->local_map_topic, 10,
                                                                 [this](const nav_msgs::OccupancyGrid::ConstPtr &msg) {
                                                                     //ROS_INFO("Local map data found");
                                                                     this->local_map = *msg;
                                                                 });
        initialized_ = true;
    }
    else
    {
        ROS_INFO("Adaptive local planner has ben initialized, do nothing");
    }
}
};