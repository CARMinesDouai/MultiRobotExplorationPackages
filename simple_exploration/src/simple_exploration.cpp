#include <ros/ros.h>
#include <stdlib.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/MarkerArray.h>

#include <time.h> /* time */

using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
geometry_msgs::PoseStamped robot_cur_pose, simple_goal;
sensor_msgs::PointCloud phrontiers;
bool getingPose;
bool frontiersExist, random_frontier;
visualization_msgs::MarkerArray mArraySentGoals;
geometry_msgs::Point32 frontier, old_frontier;
ros::Publisher pub_goal, pub_goal_rviz;
actionlib_msgs::GoalStatusArray moveBaseStatus;
double goal_tolerance, frontier_tolerance;
int gols_inc;

void publishGoal();
void ProntiersTrack(geometry_msgs::PoseStamped p);

template <class T1, class T2>
double distance(T1 from, T2 to)
{
    // Euclidiant dist between a point and the robot
    return sqrt(pow(to.x - from.x, 2) + pow(to.y - from.y, 2));
}
/*bool frontier_check(geometry_msgs::Point32 fr)
{
    vector<geometry_msgs::Point>::iterator it;
    for(it = black_list.begin(); it != black_list.end(); it++)
    {
        double dist = distance<geometry_msgs::Point, geometry_msgs::Point32>(*it, fr);
        if(dist < 0.4) return false;
    }
    return true;
}*/
bool PhrontiersCallback(const sensor_msgs::PointCloud::ConstPtr &msg)
{
    frontiersExist = false;
    int pc_size = 0;
    if (msg->points.size())
    {
        //frontiersExist=true;
        pc_size = msg->points.size();
        ROS_INFO("Frontiers number: %d", pc_size);
    }
    else
    {
        ROS_INFO("!!![ No Frontiers, *** Exploration Done *** ]!!!");
        return false;
    }

    float mindist = 0, dist, fr_dist;
    if (!getingPose)
    {
        if(random_frontier)
        {
            frontiersExist=true;
            int r = rand() % pc_size;
            frontier = msg->points[r];
        }
        else 
        {
            old_frontier = frontier;
            for (int i = 0; i < pc_size; i++)
            {
                dist = distance<geometry_msgs::Point, geometry_msgs::Point32>(robot_cur_pose.pose.position, msg->points[i]);
                fr_dist = distance<geometry_msgs::Point32, geometry_msgs::Point32>(old_frontier, msg->points[i]);
                if ((mindist == 0 || dist < mindist) && dist > goal_tolerance && fr_dist > frontier_tolerance)
                {
                    frontier = msg->points[i];
                    mindist = dist;
                    frontiersExist = true;
                }
            }
        }
        

        /*if (!frontiersExist)
        {
            dist = distance<geometry_msgs::Point, geometry_msgs::Point32>(robot_cur_pose.pose.position, old_frontier);
            if(dist > frontier_tolerance)
            {
                frontier.x = robot_cur_pose.pose.position.x;
                frontier.y = robot_cur_pose.pose.position.y;
                publishGoal();
            }
            return false;
        }*/
    }
    return frontiersExist;
}
bool closeToGoal()
{
    double dist = distance<geometry_msgs::Point, geometry_msgs::Point>(robot_cur_pose.pose.position, simple_goal.pose.position);
    return dist < goal_tolerance;
}

void ProntiersTrack(geometry_msgs::PoseStamped p)
{
    //graph nodes
    visualization_msgs::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = ros::Time::now();
    m.id = gols_inc;
    m.ns = "simple_exploration";
    m.type = visualization_msgs::Marker::CYLINDER;
    m.pose.position.x = p.pose.position.x;
    m.pose.position.y = p.pose.position.y;
    m.pose.position.z = 0.0;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.5;
    m.color.a = 1.0;
    m.lifetime = ros::Duration(0);
    m.action = visualization_msgs::Marker::ADD;

    if (gols_inc >= 999)
        gols_inc = 0;
    //cout<<"gols_inc: "<<gols_inc<<endl;
    mArraySentGoals.markers[gols_inc] = (visualization_msgs::Marker(m));
    m.action = visualization_msgs::Marker::DELETE;

    //mArraySentGoals.markers.push_back(visualization_msgs::Marker(m));
    /*
    map_nd.pose.position.x = 2.0;//p.pose.position.x;
    map_nd.pose.position.y = 2.0;//p.pose.position.y;
    map_nd.action = visualization_msgs::Marker::ADD;
    mArraySentGoals.markers[1]=map_nd;//mArraySentGoals.markers.push_back( visualization_msgs::Marker(map_nd) );
    map_nd.pose.position.x = 0.0;//p.pose.position.x;
    map_nd.pose.position.y = 0.0;//p.pose.position.y;
    map_nd.action = visualization_msgs::Marker::ADD;
    mArraySentGoals.markers[1]=map_nd;
*/
    pub_goal_rviz.publish(mArraySentGoals);

    gols_inc++;
    //for(int j=0;j<mArraySentGoals.markers.size();j++)
    //   cout<<"goal X: "<<mArraySentGoals.markers[j].pose.position.x<<" goal Y: "<<mArraySentGoals.markers[j].pose.position.y<<endl;
}

void publishGoal()
{
    //goalWasPublished= true;
    simple_goal.header.frame_id = "/map";
    simple_goal.pose.position.x = frontier.x;
    simple_goal.pose.position.y = frontier.y;

    simple_goal.pose.orientation.x = 0;
    simple_goal.pose.orientation.y = 0;
    simple_goal.pose.orientation.z = 0;
    simple_goal.pose.orientation.w = 1;

    pub_goal.publish(simple_goal);

    //ProntiersTrack(simple_goal);
}

void RobotPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    robot_cur_pose.pose = msg->pose;
    getingPose = false;
}

bool StatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg)
{
    moveBaseStatus = *msg;
}

bool readyForNewGoal()
{
    if(frontier.x == 0 && frontier.y == 0) return true;
    if (!frontiersExist)
        return false;
    if (closeToGoal())
        return true;
    for (int i = 0; i < moveBaseStatus.status_list.size(); i++)
    {
        int stat = moveBaseStatus.status_list[i].status;
        if (stat == 1) return false;
        // if (stat == 4 || stat == 5) return true;
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_exploration");

    ros::NodeHandle n_("~");
    n_.param<double>("goal_tolerance", goal_tolerance, 0.3);
    n_.param<double>("frontier_tolerance", frontier_tolerance, 0.3);
    n_.param<bool>("random_frontier", random_frontier, false);

    ROS_INFO("Goal tolerance %f", goal_tolerance);
    ROS_INFO("frontier tolerance %f", frontier_tolerance);
    ROS_INFO("Random frontier:%s", random_frontier?"true":"false");
    //get the curent robot pose
    ros::Subscriber sub_ph = n_.subscribe<sensor_msgs::PointCloud>("/phrontier_global", 100, &PhrontiersCallback);
    ros::Subscriber sub_status = n_.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 100, &StatusCallback);
    ros::Subscriber sub_pr = n_.subscribe<geometry_msgs::PoseStamped>("/posegmapping", 10, &RobotPoseCallback);
    pub_goal = n_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    pub_goal_rviz = n_.advertise<visualization_msgs::MarkerArray>("/all_sent_goals", 10);
    frontier.x = 0.0;
    frontier.y = 0.0;
    getingPose = true;
    frontiersExist = false;
    //gols_inc = 0;
   //mArraySentGoals.markers.resize(1000);

    /* initialize random seed: */
    srand(time(NULL));

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        if (readyForNewGoal())
        {
            publishGoal();
        }
        loop_rate.sleep();
    }
    return 1;
}
