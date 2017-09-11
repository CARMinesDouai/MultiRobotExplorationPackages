#include "adaptive_local_planner.h"
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(local_planner::AdaptiveLocalPlanner, nav_core::BaseLocalPlanner)

namespace local_planner
{

/*AdaptiveLocalPlanner::AdaptiveLocalPlanner()
    {
    }*/

map<int,geometry_msgs::Point> AdaptiveLocalPlanner::cc_min_dist_to_robot()
{
    SimpleCCL cclo;
    cclo.setMap(this->local_map);
    map<int,geometry_msgs::Point> obs;
    if(cclo.labels.size() == 0) return obs;
    set<int>::iterator it;

    geometry_msgs::Point dist;
    dist.z = 0;
    dist.x = -1.0;
    dist.y = -1.0;
    // init
    for(it = cclo.labels.begin(); it != cclo.labels.end(); it++)
    {
        if(cclo.labels_tree[*it].cnt < min_obstacle_size_px) continue;
        obs[*it] = dist;
    }
    geometry_msgs::Point offset;
    offset.x = fabs(this->local_map.info.origin.position.x);
    offset.y = fabs(this->local_map.info.origin.position.y);
    double resolution = this->local_map.info.resolution;
    int i,j,idx,cell;
    for(i = 0; i < cclo.dw; i++)
        for(j=0; j < cclo.dh;j++)
        {
            idx = j*cclo.dw + i;
            cell = cclo.data[idx];
            if(cell != -1)
            {
                if(cclo.labels_tree[cell].cnt < min_obstacle_size_px) continue;
                dist.x = i*resolution - offset.x;
                dist.y = j*resolution - offset.y;
                dist.z = this->dist(dist);
                if(obs[cell].z == 0 || obs[cell].z > dist.z)
                {
                    obs[cell] = dist;
                }
            }
        }

    map<int,geometry_msgs::Point>::iterator mit;
    geometry_msgs::PoseArray poses;
    poses.header.stamp = ros::Time::now();
    poses.header.frame_id = this->local_map.header.frame_id;
    for(mit = obs.begin(); mit != obs.end(); mit++)
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
bool AdaptiveLocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
{
    geometry_msgs::PoseStamped pose;
    if(!this->my_pose(&pose))
    {
        ROS_ERROR("Cannot get pose on the goal frame: %s", this->goal_frame_id.c_str());
        return false;
    }

    geometry_msgs::PoseStamped goal;
    if(!this->select_goal(&goal))
    {
        ROS_ERROR("No local goal is selected");
        return false;
    }

    tf::StampedTransform localToCmd;
    try
    {
        this->tf->lookupTransform(this->cmd_frame_id, this->local_map.header.frame_id, this->local_map.header.stamp, localToCmd);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return false;
    }
    
    /*SimpleCCL cclo;
    cclo.setMap(this->local_map);
    cclo.print();
    return false;*/

    // now calculate the potential field toward the local goal
    double resolution = this->local_map.info.resolution;
   

    geometry_msgs::Point fatt;
    geometry_msgs::Point frep;
    fatt.x = 0.0;
    fatt.y = 0.0;
    frep.x = 0.0;
    frep.y = 0.0;
    
    // attractive potential
    double robot_to_goal = this->dist(goal.pose.position);
    if(robot_to_goal < 0.1) this->reached = true;
    if(robot_to_goal < safe_goal_dist)
    {
        fatt.x = attractive_gain * goal.pose.position.x;
        fatt.y = attractive_gain * goal.pose.position.y;
    }
    else 
    {
        fatt.x = goal.pose.position.x*safe_goal_dist/robot_to_goal;
        fatt.y = goal.pose.position.y*safe_goal_dist/robot_to_goal; 
    }
    
    //cmd_vel.angular.z =  atan2(fatt.y,fatt.x) - tf::getYaw(pose.pose.orientation);
    /*if(robot_to_goal <= this->safe_goal_dist)
    {
        fatt.x += this->attractive_gain*goal.pose.position.x;
        fatt.y += this->attractive_gain*goal.pose.position.y;
    }
    else 
    {
        fatt.x = this->safe_goal_dist*this->attractive_gain*goal.pose.position.x / robot_to_goal;
        fatt.y = this->safe_goal_dist*this->attractive_gain*goal.pose.position.y / robot_to_goal;
    }*/

    // repulsive potential to the nearest obstacle

    int i,j, npix=0;

    map<int,geometry_msgs::Point> obstacles = this->cc_min_dist_to_robot();
    map<int,geometry_msgs::Point>::iterator mit;
    for(mit = obstacles.begin(); mit != obstacles.end(); mit++)
    {
        double theta = atan2(mit->second.y, mit->second.x);
        if(mit->second.z <= this->safe_obs_dist)
        {
            double tmp = repulsive_gain*(1.0/safe_obs_dist - 1.0/mit->second.z) / (mit->second.z*mit->second.z);
            frep.x += tmp*mit->second.x;
            frep.y += tmp*mit->second.y;
            //frep.x += (1.0/(mit->second.z ))*cos(theta);
            //frep.y += (1.0/(mit->second.z ))*sin(theta);
            //fatt.x  += 0.5*mit->second.x;
            //fatt.y += 0.5*mit->second.y;
        }
        else 
        {
            frep.x += 0;
            frep.y += 0;
        }
        //double oyaw = tf::getYaw(pose.pose.orientation);
       // cmd_vel.angular.z += atan2(frep.y, frep.z) -  tf::getYaw(pose.pose.orientation);
    }
    /*
        for(j = 0; j < this->local_map.info.height; j++)
        {
            double cell = this->local_map.data[i + j*this->local_map.info.width];
            if(cell == 100.0) // obstacle
            {
                obstacle.x = i*resolution - offset.x;
                obstacle.y = j*resolution - offset.y;
                //ROS_INFO("obstacle at %f %f", obstacle.x, obstacle.y);
                robot_to_obstacle = this->dist(obstacle);
                theta = atan2(obstacle.y, obstacle.x);
                if(robot_to_obstacle <= this->safe_obs_dist)
                {
                    npix++;
                    tmp = 0.03*(1.0/safe_obs_dist - 1.0/robot_to_obstacle) / (robot_to_obstacle*robot_to_obstacle);
                    frep.x += tmp*obstacle.x; //*cos(theta);
                    frep.y += tmp*obstacle.y;//*sin(theta);
                }
                else 
                {
                    frep.x += 0;
                    frep.y += 0;
                }
            }
        }
*/
    /*if(npix != 0)
    {
        frep.x /= npix;
        frep.y /= npix;
    }*/
    ROS_INFO("attractive: %f, %f Respulsive: %f %f", fatt.x, fatt.y, frep.x, frep.y);
    // now calculate the velocity
    tf::Vector3 cmd;
    cmd.setX(fatt.x + frep.x);
    cmd.setY(fatt.y + frep.y);
    cmd = localToCmd*cmd;
    double yaw =  atan2(cmd.y(),cmd.x()) - tf::getYaw(pose.pose.orientation);
    cmd_vel.linear.x = cmd.x();
    cmd_vel.linear.y = cmd.y();
    cmd_vel.linear.z = 0.0;
    
    //-fatt.x *(sin(mypose.)+yb_corner[i].getOrigin().y()*cos(tf_yb_origin_yaw)) + 
    
    //d_u_att_y * (yb_corner[i].getOrigin().x()*cos(tf_yb_origin_yaw)-yb_corner[i].getOrigin().y()*sin(tf_yb_origin_yaw));

    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0; // ?
    cmd_vel.angular.z = yaw;
    ROS_INFO("CMD VEL: x:%f y:%f w:%f", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
    
    return true;
}
/*
    get my pose on the goal frame
*/
bool AdaptiveLocalPlanner::my_pose(geometry_msgs::PoseStamped *pose)
{
    tf::StampedTransform transform;
    try
    {
        this->tf->lookupTransform(this->cmd_frame_id, this->cmd_frame_id, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("Cannot get transform %s-%s: %s", this->goal_frame_id.c_str(), this->cmd_frame_id.c_str(), ex.what());
        return false;
    }

    pose->header.stamp = ros::Time::now();
    pose->header.frame_id = this->cmd_frame_id; 

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
    // just get last pose in global goal for now
    if (this->global_plan.size() == 0)
        return false;
    geometry_msgs::PoseStamped  tmpgoal;//mypose
    //if(! this->my_pose(&mypose))
    //    return false;
    
    tf::StampedTransform goalToLocal;
    try
    {
        this->tf->lookupTransform(this->local_map.header.frame_id, this->goal_frame_id, this->local_map.header.stamp, goalToLocal);
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
    for(it = this->global_plan.begin(); it != this->global_plan.end(); it++)
    {
        candidate.setX(it->pose.position.x);
        candidate.setY(it->pose.position.y);
        candidate = goalToLocal*candidate;
        d = sqrt(pow(candidate.x(),2) + pow(candidate.y(),2));
        if(d > max_local_goal_dist) break;
        /*_goal->header.frame_id = this->local_map.header.frame_id;
        _goal->pose.position.x = _goal->pose.position.x - mypose.pose.position.x;
        _goal->pose.position.y = _goal->pose.position.y - mypose.pose.position.y;
        _goal->pose.position.z = _goal->pose.position.z - mypose.pose.position.z;
        //a = candidate.y() / candidate.x();
        dx = fabs(candidate.x()) / this->local_map.info.resolution;
        dy = fabs(candidate.y()) / this->local_map.info.resolution;
        if( dx < this->fw/2 && dy < this->fh/2) continue;
        //dist = sqrt( pow(theirpose.position.x, 2) + pow(theirpose.position.y,2) );
        break;
        */
    }

    _goal->pose.position.x = candidate.x();
    _goal->pose.position.y = candidate.y();

    tmpgoal = *_goal;
    tmpgoal.header.frame_id = this->local_map.header.frame_id;
    tmpgoal.pose.position.x =_goal->pose.position.x ;
    tmpgoal.pose.position.y = _goal->pose.position.y ;
    local_goal_pub.publish(tmpgoal);
    return true;
}

double AdaptiveLocalPlanner::dist(geometry_msgs::Point to)
{
    // Euclidiant dist between a point and the robot
    return sqrt(pow(to.x, 2) + pow(to.y, 2) );
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
        private_nh.param<std::string>("goal_frame_id", this->goal_frame_id, "map");
        private_nh.param<std::string>("cmd_frame_id", this->cmd_frame_id, "base_link");
        private_nh.param<std::string>("scan_topic", this->scan_topic, "/scan");
        private_nh.param<double>("local_map_resolution", this->map_resolution, 0.05);
        private_nh.param<double>("attractive_gain", this->attractive_gain, 1.0);
        private_nh.param<double>("repulsive_gain", this->repulsive_gain, 1.0);
        private_nh.param<double>("safe_goal_dist", this->safe_goal_dist, 1.0);
        private_nh.param<double>("safe_obs_dist", this->safe_obs_dist, 1.0);
        private_nh.param<double>("max_local_goal_dist", this->max_local_goal_dist, 0.5);
        private_nh.param<int>("min_obstacle_size_px", this->min_obstacle_size_px, 20);
        private_nh.param<double>("field_w", dw, 2.0);
        private_nh.param<double>("field_h", dh, 2.0);

        this->fw = round(dw / this->map_resolution);
        this->fh = round(dh / this->map_resolution);

        ROS_INFO("Robot radius %f", this->robot_radius);
        ROS_INFO("Robot goal_frame_id %s", this->goal_frame_id.c_str());
        ROS_INFO("Robot command_frame_id %s", this->cmd_frame_id.c_str());
        ROS_INFO("Scan: %s", this->scan_topic.c_str());
        ROS_INFO("Local map res: %f", this->map_resolution);
        ROS_INFO("Field width %d", this->fw);
        ROS_INFO("Field height %d", this->fh);
        map_builder = new local_map::MapBuilder(this->fw, this->fh, this->map_resolution);
        local_pub = private_nh.advertise<nav_msgs::OccupancyGrid>("/local_map", 1, true);
        local_goal_pub = private_nh.advertise<geometry_msgs::PoseStamped>("/local_goal",1,true);
        obstacles_pub = private_nh.advertise<geometry_msgs::PoseArray>("/obstacles",1, true);
        this->tf = tf;
        // subscribe to scan topic
        laser_sub = private_nh.subscribe<sensor_msgs::LaserScan>(this->scan_topic, 1,
                                                                 [this](const sensor_msgs::LaserScan::ConstPtr &msg) {
                                                                     this->map_builder->grow(*msg);
                                                                     this->local_map = map_builder->getMap();
                                                                     this->local_pub.publish(this->local_map);
                                                                 });
        initialized_ = true;
    }
    else
    {
        ROS_INFO("Adaptive local planner has ben initialized, do nothing");
    }
}
};