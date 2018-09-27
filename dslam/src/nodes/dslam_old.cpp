#include "localization/PFLocalization.h"
#include "localization/ICPLocalization.h"
#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <boost/thread/thread.hpp>
#include <queue>
#include "mapping/Mapping.h"

using namespace dslam;

typedef struct
{
    sensor_msgs::LaserScan scan;
    nav_msgs::Odometry odom;
} sensor_data_t;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> NoCloudSyncPolicy;
std::queue<sensor_data_t> data_queue;
BaseLocalization *matcher_;
PFLocalization pf_matcher;
ICPLocalization1 icp_matcher;
ros::Publisher marker_publisher_, local_map_pub;
ros::Publisher cloudpublisher_, trajectory_p_, particles_p_;
geometry_msgs::Pose pose;
Mapping map_builder_;
geometry_msgs::PoseArray estimated_poses_;

std::string map_frame, laser_frame, robot_base_frame;
nav_msgs::OccupancyGrid local_map_;
void run();

void sensorCallback(const sensor_msgs::LaserScanConstPtr &scan_msg, const nav_msgs::OdometryConstPtr &odom)
{
    //ROS_DEBUG("Data found");
    if(data_queue.empty())
    {
        sensor_data_t d;
        d.scan = *scan_msg;
        d.odom = *odom;
        data_queue.push(d);
    }
}

void populateMarkerMsg(const std::vector<Line> &lines,
                       visualization_msgs::Marker &marker_msg)
{
    marker_msg.ns = "line_extraction";
    marker_msg.id = 0;
    marker_msg.type = visualization_msgs::Marker::LINE_LIST;
    marker_msg.scale.x = 0.01;
    marker_msg.color.r = 1.0;
    marker_msg.color.g = 0.0;
    marker_msg.color.b = 0.0;
    marker_msg.color.a = 1.0;

    for (std::vector<Line>::const_iterator cit = lines.begin(); cit != lines.end(); ++cit)
    {
        geometry_msgs::Point p_start;
        p_start.x = cit->getStart()[0];
        p_start.y = cit->getStart()[1];
        p_start.z = 0;
        marker_msg.points.push_back(p_start);

        geometry_msgs::Point p_p;
        p_p.x = cit->getCenter()[0];
        p_p.y = cit->getCenter()[1];
        p_p.z = 0;

        geometry_msgs::Point p_end;
        p_end.x = cit->getEnd()[0];
        p_end.y = cit->getEnd()[1];
        p_end.z = 0;
        marker_msg.points.push_back(p_end);
    }
    marker_msg.header.frame_id = laser_frame;
    marker_msg.header.stamp = ros::Time::now();
}
const void callback(std::vector<Line> &lines, pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    visualization_msgs::Marker marker_msg;
    sensor_msgs::PointCloud2 pcloud;
    pcl::toROSMsg(cloud, pcloud);
    pcloud.header.frame_id = "odom";
    pcloud.header.stamp = ros::Time::now();
    populateMarkerMsg(lines, marker_msg);
    marker_publisher_.publish(marker_msg);
    cloudpublisher_.publish(pcloud);
}

void publish_tranform()
{ 
   while (ros::ok())
    { 
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        matcher_->getTransform(transform);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), map_frame, "odom"));
        //matcher_->visualize(particles_p_);
    }
    
}

void run()
{
    while (ros::ok())
    {
        // Extract the lines
        if (data_queue.empty())
            continue;
        sensor_data_t data = data_queue.front();
        data_queue.pop();
        matcher_->registerScan(data.scan, data.odom);
        bool newkf = matcher_->match(callback);

        if (newkf)
        {
            matcher_->getLastKnownPose(pose);
            estimated_poses_.poses.push_back(pose);
            estimated_poses_.header.frame_id = map_frame;
            estimated_poses_.header.stamp = ros::Time::now();
            trajectory_p_.publish(estimated_poses_);
        }
    }
}

void localmapping()
{
    while (ros::ok())
    {
        if (!matcher_->keyframes.empty())
        {
            map_builder_.buildFrom(matcher_->keyframes);
            nav_msgs::OccupancyGrid map = map_builder_.getMap();
            ros::Time now = ros::Time::now();
            map.info.map_load_time = now;
            map.header.stamp = now;
            local_map_pub.publish(map);
        }
    }
}
int main(int argc, char **argv)
{
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::init(argc, argv, "dslam");
    ros::NodeHandle nh_local("~");
    tf::TransformListener *listener = new tf::TransformListener();
    // Parameters used by this node

    std::string scan_topic, config_file, marker_topic, odom_topic, map_topic;
    nh_local.param<std::string>("conf_file", config_file, "config.lua");
    Configuration config(config_file);
    bool use_particle_filter = config.get<bool>("use_particle_filter", true);
    Configuration localisation = config.get<Configuration>("localisation", Configuration());
    Configuration mapping = config.get<Configuration>("mapping", Configuration());
    //Configuration local_map = mapping.get<Configuration>("submap", Configuration());
    map_builder_.configure(mapping);

    map_frame = localisation.get<std::string>("global_frame", "map");
    laser_frame = localisation.get<std::string>("laser_frame", "scan_frame");
    robot_base_frame = localisation.get<std::string>("robot_base", "base_link");

    scan_topic = config.get<std::string>("scan_topic", "/laser_scan");
    odom_topic = config.get<std::string>("odom_topic", "/odom");

    marker_topic = config.get<std::string>("line_seg_topic", "/seg_markers");
    map_topic = config.get<std::string>("map_topic", "/map");
    //scan_subscriber_ = nh_local.subscribe(scan_topic, 1, &laserScanCallback);
    message_filters::Subscriber<sensor_msgs::LaserScan> scan_subscriber_(nh_local, scan_topic, 1);
    message_filters::Subscriber<nav_msgs::Odometry> odom_subscriber_(nh_local, odom_topic, 1);
    message_filters::Synchronizer<NoCloudSyncPolicy> sync(NoCloudSyncPolicy(1), scan_subscriber_, odom_subscriber_);
    sync.registerCallback(boost::bind(&sensorCallback, _1, _2));

    marker_publisher_ = nh_local.advertise<visualization_msgs::Marker>(marker_topic, 1);
    cloudpublisher_ = nh_local.advertise<sensor_msgs::PointCloud2>("/keypoints", 1);
    trajectory_p_ = nh_local.advertise<geometry_msgs::PoseArray>("/trajectory", 1);

    particles_p_ = nh_local.advertise<sensor_msgs::PointCloud2>("/particles", 1);

    double frequency = config.get<double>("frequency", 25);
    ROS_DEBUG("Frequency set to %0.1f Hz", frequency);
    ros::Rate rate(frequency);
    local_map_pub = nh_local.advertise<nav_msgs::OccupancyGrid>(map_topic, 1, true);
    pose.position.x = 0.0; // - offset_.x();
    pose.position.y = 0.0; // - offset_.y();
    pose.position.z = 0.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    estimated_poses_.poses.push_back(pose);
    estimated_poses_.header.frame_id = map_frame;
    estimated_poses_.header.stamp = ros::Time::now();

    if (use_particle_filter)
        matcher_ = &pf_matcher;
    else
        matcher_ = &icp_matcher;
    matcher_->setTF(listener);

    matcher_->configure(localisation);

    boost::thread t1(&run);
    //boost::thread t2(&localmapping);
    boost::thread t3(&publish_tranform);
    //t1.join();

    while (ros::ok())
    {
        ros::spinOnce();
        //publish_tranform();
        //if(estimated_poses_.poses.size() > 0)
        rate.sleep();
    }
    return 0;
}
