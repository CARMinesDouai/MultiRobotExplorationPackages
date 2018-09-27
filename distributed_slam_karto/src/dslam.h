#ifndef DSLAM_H
#define DSLAM_H

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
//#include <visualization_msgs/Marker.h>
#include <distributed_slam_karto/Graph.h>
#include <distributed_slam_karto/SyncGraph.h>
#include "open_karto/Karto.h"
#include "open_karto/Mapper.h"

#include "open_karto/G2OSolver.h"

#include "utils/luaconf.h"

namespace dslam {
    typedef struct{
        std::string base_frame_id;
        sensor_msgs::LaserScan::ConstPtr scan;
    } AnnotatedScan;
    class DSlamKarto{
        public:
            DSlamKarto();
            ~DSlamKarto();
            void configure(Configuration &, ros::NodeHandle&);
            void publishTf();
            void publishGraph();
            void publishMap();
            void setTFListener(tf::TransformListener* tf) { tf_ = tf; };
            void laserCallback(const AnnotatedScan&);
            void laserCallback(const AnnotatedScan&, karto::Pose2&);
            karto::Vertex<karto::LocalizedRangeScan>* getVertex(karto::Name, int);
            karto::Vertex<karto::LocalizedRangeScan>* getVertex(int);
        private:
            bool getOdom(karto::Pose2&, const AnnotatedScan&);
            karto::LaserRangeFinder* getRFDevice(const AnnotatedScan&);
            bool registerScan(karto::LaserRangeFinder* ,
                 const AnnotatedScan& ,
                 karto::Pose2& );
            bool  updateMap();
            void getDevices(std::vector<distributed_slam_karto::LaserDevice> &);
            void transformPose(tf::StampedTransform&, geometry_msgs::Point&);
            bool syncGraphService(distributed_slam_karto::SyncGraph::Request&, distributed_slam_karto::SyncGraph::Response &);
            tf::TransformListener* tf_;
            tf::TransformBroadcaster tf_broadcaster_;
            tf::Transform fixed_to_odom_;

            bool map_init_, publish_graph_;

            std::string odom_frame_, fixed_frame_;//, base_frame_;

            boost::mutex mutex_;

            ros::Duration map_update_rate_;
            double resolution_;

            karto::Mapper* mapper_;
            karto::Dataset* database_;
            G2OSolver* optimizer_;
            
            unsigned int offset_id_;

            ros::Publisher map_pub_;
            ros::ServiceServer sync_srv_; 
            //ros::Publisher constraint_pub_;
            ros::Publisher graph_pub_;
            nav_msgs::OccupancyGrid map_;
             //int throttle_scans_;
             //std::map<std::string, int> num_scans_;
            std::map<std::string, karto::LaserRangeFinder*> devices_;
            ros::Time last_update_map_;
    };
}

#endif