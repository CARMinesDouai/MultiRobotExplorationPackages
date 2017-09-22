#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <tf/transform_listener.h>
#include <stdlib.h>

double goal_tolerance, frontier_tolerance;
bool random_frontier;
std::string map_frame, base_frame, goal_topic;
sensor_msgs::PointCloud global_frontiers;
geometry_msgs::Point32 frontier, old_frontier;
tf::TransformListener* listener;
actionlib_msgs::GoalStatusArray global_status;
template <class T1, class T2>
double distance(T1 from, T2 to)
{
    return sqrt(pow(to.x - from.x, 2) + pow(to.y - from.y, 2));
}

bool  my_pose(geometry_msgs::Pose *pose)
{
    tf::StampedTransform transform;
    try {
        listener->lookupTransform(map_frame, base_frame, ros::Time(0), transform);
      } catch(tf::TransformException ex) {
        //ROS_ERROR("TF %s - %s: %s", map_frame.c_str(), base_frame.c_str(), ex.what());
        return false;
      }
    pose->position.x =  transform.getOrigin().getX();
    pose->position.y =  transform.getOrigin().getY();
    pose->position.z =  transform.getOrigin().getZ();
    pose->orientation.x = transform.getRotation().getX();
    pose->orientation.y = transform.getRotation().getY();
    pose->orientation.z = transform.getRotation().getZ();
    return true;
}

void frontier_callback(const sensor_msgs::PointCloud::ConstPtr &msg)
{
    global_frontiers = *msg;
}

void status_callback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg){
    global_status = *msg;
}

bool find_next_frontier()
{
   
    geometry_msgs::Pose pose;
    if(!my_pose(&pose))
    {
        //ROS_WARN("Cannot find my pose");
        return false;
    }

    double dist = distance<geometry_msgs::Point,  geometry_msgs::Point32>(pose.position, frontier);
    if(dist > goal_tolerance)
    {   
        for (int i = 0; i < global_status.status_list.size(); i++)
        {
            int stat = global_status.status_list[i].status;
            if (stat == 1) return false;
        }

    }

    
    if(global_frontiers.points.size() == 0)
    {
        ROS_WARN("No frontier to allocate at this time");
        return false;
    }

    if(random_frontier )
    {
        old_frontier = frontier;
        frontier = global_frontiers.points[ rand() % global_frontiers.points.size()];
        return true;
    }

    // nearest frontier allocation
    double mindist = 0, fr_dist;
    bool allocated = false;
    old_frontier = frontier;
    for (int i = 0; i < global_frontiers.points.size(); i++)
    {
        dist = distance<geometry_msgs::Point, geometry_msgs::Point32>(pose.position, global_frontiers.points[i]);
        fr_dist = distance<geometry_msgs::Point32, geometry_msgs::Point32>(old_frontier, global_frontiers.points[i]);
        if ( ( old_frontier.x == 0 && old_frontier.y == 0 ) ||( (mindist == 0 || dist < mindist) && dist > goal_tolerance && fr_dist > frontier_tolerance))
        {
            frontier = global_frontiers.points[i];
            mindist = dist;
            allocated = true;
        }
    }

    return allocated;
}


int main(int argc, char**argv)
{
    ros::init(argc, argv, "frontier_allocator");
    ros::NodeHandle private_nh("~");
    listener = new tf::TransformListener();
    private_nh.param<double>("goal_tolerance", goal_tolerance, 0.3);
    private_nh.param<double>("frontier_tolerance", frontier_tolerance, 0.3);
    private_nh.param<bool>("random_frontier", random_frontier, false);
    private_nh.param<std::string>("map_frame", map_frame, "map");
    private_nh.param<std::string>("base_frame", base_frame, "base_link");
    private_nh.param<std::string>("goal_topic", goal_topic, "/move_base_simple/goal");

    ros::Subscriber sub_ph = private_nh.subscribe<sensor_msgs::PointCloud>("/phrontier_global", 100, &frontier_callback);
    ros::Publisher  pub_goal = private_nh.advertise<geometry_msgs::PoseStamped>(goal_topic, 10);
    ros::Subscriber sub_status = private_nh.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 10, &status_callback);

    srand(time(NULL));

    ros::Rate loop_rate(1.0);
    while (ros::ok())
    {
        ros::spinOnce();
        if (find_next_frontier())
        {
            geometry_msgs::PoseStamped goal;
            goal.header.frame_id = map_frame;
            goal.pose.position.x = frontier.x;
            goal.pose.position.y = frontier.y;
        
            goal.pose.orientation.x = 0;
            goal.pose.orientation.y = 0;
            goal.pose.orientation.z = 0;
            goal.pose.orientation.w = 1;
        
            pub_goal.publish(goal);
        
        }
        loop_rate.sleep();
    }
}