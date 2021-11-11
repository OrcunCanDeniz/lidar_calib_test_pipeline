#pragma once

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>

#include <experimental/filesystem>
#include <sys/stat.h>


class data_handler
{

    public:
        data_handler();
    
    private:
        void setSubdirs(std::string parent_dir, bool is_dataset, int agent_idx=0)
        
        ros::NodeHandle nh_;
        
        std::string dataset_dir;

        std::vector<std::string> all_directories;
        int directory_index_;
        bool all_processed;

        ros::Time current_time_;

        sensor_msgs::PointCloud2 parent_msg;
        
        void service(bool next_trigger);
        
        void ReadScene(std::string in_file);

        void PublishPCD();

        std::vector<std::string> agent_dirs;
        std::vector<std::vector<std::string>> scenes_of_agent;
        int curr_agent_idx = 0;
        int curr_scene_idx = 0;

        // subscribers
        std::map<std::string, boost::shared_ptr<ros::Publisher>> pubs;

};