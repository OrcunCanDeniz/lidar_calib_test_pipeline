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
        ros::NodeHandle nh_;
        
        std::string pcd_image_input_dir_;
        std::string lidar_output_frame;

        std::vector<std::string> all_directories;
        int directory_index_;
        bool all_processed;

        ros::Time current_time_;

        sensor_msgs::PointCloud2 lidar_msg;
        
        void service(bool next_trigger);
        
        void ReadPCD(std::string in_file);

        void PublishPCD();

        // subscribers
        ros::Publisher points_pub_;

};