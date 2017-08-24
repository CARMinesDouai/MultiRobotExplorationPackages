#include <ros/ros.h>
#include <stdlib.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/MarkerArray.h>

#include <time.h>       /* time */

using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
geometry_msgs::PoseStamped robot_cur_pose, simple_goal, robot_prev_pose;
sensor_msgs::PointCloud phrontiers;
bool getingPose;
bool frontiersExist;
//vector<float> min_search_radius, max_search_radius;
bool cant_have_local_planner;
visualization_msgs::MarkerArray mArraySentGoals;
geometry_msgs::Point32 frontier;
ros::Publisher pub_goal, pub_goal_rviz;
actionlib_msgs::GoalStatusArray moveBaseStatus;
int gols_inc;

bool PhrontiersCallback(const sensor_msgs::PointCloud::ConstPtr& msg){
    //check the close ponits to visite

    int pc_size=0;
    if(msg->points.size()){
        frontiersExist=true;
        pc_size = msg->points.size();
        cout<<" -_- [ Frontiers number "<<pc_size<<" ] -_-  "<<endl;
    }else{
        frontiersExist=false;
        cout<<"!!![ No Frontiers, *** Exploration Done *** ]!!!"<<endl;
        return false;
    }

    float dist = 1000;
    float dx, dy;
    if (!getingPose)
    {
        int r = rand() % pc_size;
        cout<<"Choosing the next goal: "<<r<<endl;
        frontier = msg->points[r];
/*      for(int i=0;i<pc_size;i++)
        {
            dx=robot_cur_pose.pose.position.x-msg->points[i].x;
            dy=robot_cur_pose.pose.position.y-msg->points[i].y;
            if (sqrt(dx*dx+dy*dy)<dist)
            {
                frontier = msg->points[i];
                dist = sqrt(dx*dx+dy*dy);
            }
        }*/
    }
}
bool closeToGoal()
{
    float dx, dy;
    dx=robot_cur_pose.pose.position.x - simple_goal.pose.position.x;
    dy=robot_cur_pose.pose.position.y - simple_goal.pose.position.y;
    return sqrt(dx*dx+dy*dy) < 0.8;
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
    m.scale.x = 0.1;m.scale.y = 0.1;m.scale.z = 0.1;m.color.r = 1.0;
    m.color.g = 0.0;m.color.b = 0.5;m.color.a = 1.0;
    m.lifetime = ros::Duration(0);
    m.action = visualization_msgs::Marker::ADD;

    if(gols_inc>=999) gols_inc=0;
    cout<<"gols_inc: "<<gols_inc<<endl;
    mArraySentGoals.markers[gols_inc]=(visualization_msgs::Marker(m));
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

void publishGoal(){
    //goalWasPublished= true;
    simple_goal.header.frame_id="/map";
    simple_goal.pose.position.x=frontier.x;
    simple_goal.pose.position.y=frontier.y;

    simple_goal.pose.orientation.x=0;simple_goal.pose.orientation.y=0;simple_goal.pose.orientation.z=0;simple_goal.pose.orientation.w=1;
    cout<<"next frontier "<<simple_goal.pose.position.x<<" -- "<<simple_goal.pose.position.y<<endl;
    pub_goal.publish(simple_goal);

    ProntiersTrack(simple_goal);
}

void RobotPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    //check the close ponits to visite
    getingPose=true;
    robot_cur_pose.pose  = msg->pose;
    /*if(!prev_pose_was_taken)
        robot_prev_pose.pose = robot_cur_pose.pose;
    //we check if we are close to the prev. pose
    if(robot_prev_pose.pose.position.x==robot_cur_pose.pose.position.x && robot_prev_pose.pose.position.y==robot_cur_pose.pose.position.y)
    {
        //count 500 times, which is equivqlent to 5s
        inc_pose++;
        //cout<<inc_pose<<endl;
        if(inc_pose>500){
            goalWasPublished=false;
            searchAnotherFrontier=true;
            inc_pose=0;
            cout<<"!!!!!!!!!!!!! robot at the same position !!!!!!!!!!!!!"<<endl;
        }else{

        }
    }else{
        robot_prev_pose.pose = robot_cur_pose.pose;
    }*/

    getingPose=false;
    //cout<<"<<<<<<<<<<<<<<<<<<<<<<<from pose robot"<<endl;
}

bool StatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg){
    moveBaseStatus = *msg;
}

bool readyForNewGoal() {
    if(!frontiersExist) return false;
    for(int i = 0 ; i< moveBaseStatus.status_list.size(); i++)
    {
        int n=moveBaseStatus.status_list[i].status;
        //cout<<"move base status "<<closeToGoal()<<" "<<n<<endl;
        if (moveBaseStatus.status_list[i].status == 1) return (false || closeToGoal());
    }
    return true;
}

int main(int argc, char** argv)
{
   ros::init(argc, argv, "simple_exploration");

   ros::NodeHandle n_;

   //get the curent robot pose
   ros::Subscriber sub_ph     = n_.subscribe<sensor_msgs::PointCloud>("/phrontier_global", 100, &PhrontiersCallback);
   ros::Subscriber sub_status = n_.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 100, &StatusCallback);
   ros::Subscriber sub_pr     = n_.subscribe<geometry_msgs::PoseStamped>("/posegmapping", 10, &RobotPoseCallback);
   pub_goal                   = n_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",10);
   pub_goal_rviz              = n_.advertise<visualization_msgs::MarkerArray>("/all_sent_goals",10);

   getingPose=true;
   //goalWasPublished=false;
   frontiersExist=false;
   gols_inc=0;
   mArraySentGoals.markers.resize(1000);
   //values bellow can get from launch file
//   min_search_radius.resize(6);
//   min_search_radius[0]=2.0;min_search_radius[1]=1.8;min_search_radius[2]=1.6;min_search_radius[3]=1.4;min_search_radius[4]=1.2;min_search_radius[5]=1.0;
//   max_search_radius.resize(6);
//   max_search_radius[0]=5.0;max_search_radius[1]=6.8;max_search_radius[2]=7.6;max_search_radius[3]=8.4;max_search_radius[4]=9.2;max_search_radius[5]=30.0;

   /* initialize random seed: */
   srand (time(NULL));

   ros::Rate loop_rate(3);
   while(ros::ok())
   {
       if(readyForNewGoal())
        {
            publishGoal();
        }

        ros::spinOnce();
        loop_rate.sleep();
   }
   return 1;
}


