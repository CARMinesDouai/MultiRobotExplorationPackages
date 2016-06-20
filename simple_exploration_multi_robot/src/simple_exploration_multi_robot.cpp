/*
 * Copyright (c) 2015, Khelifa Baizid <Khelifa.Baizid@mines-douai.fr>
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of California, Berkeley nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE REGENTS AND CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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
string map_frame_topic_;

bool PhrontiersCallback(const sensor_msgs::PointCloud::ConstPtr& msg){
    //check the close ponits to visite

    int pc_size=0;
    if(msg->points.size()){
        frontiersExist=true;
        pc_size = msg->points.size();
        cout<<" -_- [ Frontiers number "<<pc_size<<" ] -_-  "<<endl;
    }else{
        frontiersExist=false;
        cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
        cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
        cout<<"  !!![ No Frontiers, *** Exploration Done *** ]!!!"<<endl;
        cout<<"  !!![ No Frontiers, *** Exploration Done *** ]!!!"<<endl;
        cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
        cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
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
//        {
//            dx=robot_cur_pose.pose.position.x-msg->points[i].x;
//            dy=robot_cur_pose.pose.position.y-msg->points[i].y;
//            if (sqrt(dx*dx+dy*dy)<dist)
//            {
//                frontier = msg->points[i];
//                dist = sqrt(dx*dx+dy*dy);
//            }
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
    m.header.frame_id = map_frame_topic_;
    m.header.stamp    = ros::Time::now();
    m.id              = gols_inc;
    m.ns              = "simple_exploration";
    m.type            = visualization_msgs::Marker::CYLINDER;
    m.pose.position.x = p.pose.position.x;
    m.pose.position.y = p.pose.position.y;
    m.pose.position.z = 0.0;
    m.scale.x         = 0.1;m.scale.y = 0.1;m.scale.z = 0.1;m.color.r = 1.0;
    m.color.g         = 0.0;m.color.b = 0.5;m.color.a = 1.0;
    m.lifetime        = ros::Duration(0);
    m.action          = visualization_msgs::Marker::ADD;

    if(gols_inc>=999) gols_inc=0;
    cout<<"gols_inc: "<<gols_inc<<endl;
    mArraySentGoals.markers[gols_inc]=(visualization_msgs::Marker(m));
    m.action = visualization_msgs::Marker::DELETE;

    pub_goal_rviz.publish(mArraySentGoals);

    gols_inc++;
    //for(int j=0;j<mArraySentGoals.markers.size();j++)
    //   cout<<"goal X: "<<mArraySentGoals.markers[j].pose.position.x<<" goal Y: "<<mArraySentGoals.markers[j].pose.position.y<<endl;
}

void publishGoal(){
    //goalWasPublished= true;
    simple_goal.header.frame_id    = map_frame_topic_;
    simple_goal.pose.position.x    = frontier.x;
    simple_goal.pose.position.y    = frontier.y;

    simple_goal.pose.orientation.x = 0;simple_goal.pose.orientation.y=0;simple_goal.pose.orientation.z=0;simple_goal.pose.orientation.w=1;
    cout<<"next frontier "<<simple_goal.pose.position.x<<" -- "<<simple_goal.pose.position.y<<endl;
    pub_goal.publish(simple_goal);
    ProntiersTrack(simple_goal);
}

void RobotPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    //check the close ponits to visite
    getingPose=true;
    robot_cur_pose.pose  = msg->pose;
    getingPose=false;
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
        if (moveBaseStatus.status_list[i].status == 1)
            return (false || closeToGoal());
    }
    return true;
}

int main(int argc, char** argv)
{
   ros::init(argc, argv, "simple_exploration_multi_robot");

   ros::NodeHandle n_;

   //get the curent robot pose
   ros::NodeHandle private_nh("~");

   string frontier_topic_       = "/tb2/phrontier_global";
   private_nh.getParam("frontier_topic", frontier_topic_);

   string move_base_status_     = "/tb2/move_base/status";
   private_nh.getParam("move_base_status", move_base_status_);

   string pose_topic_           = "/tb2/posegmapping";
   private_nh.getParam("pose_topic", pose_topic_);

   string goal_topic_           = "/tb2/current_goal";
   private_nh.getParam("goal_topic", goal_topic_);

   string all_sent_goals_topic_ = "/tb2/all_sent_goals";
   private_nh.getParam("all_sent_goals_topic", all_sent_goals_topic_);

   map_frame_topic_             = "/tb2/map";
   private_nh.getParam("map_frame_topic", map_frame_topic_);

   ros::Subscriber sub_ph     = n_.subscribe<sensor_msgs::PointCloud>(frontier_topic_, 100, &PhrontiersCallback);
   ros::Subscriber sub_status = n_.subscribe<actionlib_msgs::GoalStatusArray>(move_base_status_, 100, &StatusCallback);
   ros::Subscriber sub_pr     = n_.subscribe<geometry_msgs::PoseStamped>(pose_topic_, 10, &RobotPoseCallback);
   pub_goal                   = n_.advertise<geometry_msgs::PoseStamped>(goal_topic_,10);
   pub_goal_rviz              = n_.advertise<visualization_msgs::MarkerArray>(all_sent_goals_topic_,10);

   getingPose=true;
   //goalWasPublished=false;
   frontiersExist=false;
   gols_inc=0;
   mArraySentGoals.markers.resize(1000);

   /* initialize random seed: */
   srand (time(NULL));
   int inc_local;inc_local=0;
   ros::Rate loop_rate(3);
   while(ros::ok())
   {
       cout<<inc_local<<endl;
       inc_local++;
       if(readyForNewGoal())
       {
           sleep(5);
           publishGoal();
       }

        ros::spinOnce();
        loop_rate.sleep();
   }
   return 1;
}


