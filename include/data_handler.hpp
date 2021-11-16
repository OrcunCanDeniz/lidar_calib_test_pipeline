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
#include <std_srvs/SetBool.h>


namespace data_provider
{
    class data_handler
    {

        public:
            data_handler();
        
        private:
            void setSubdirs(std::string parent_dir, bool is_dataset, int agent_idx);
            void ReadScene(std::string in_file);
            bool IsPathExist(const std::string &s);
            bool cacheScene();
            bool serve(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);
            void createPcdPairs();
            void publishPcds(std::string parent_name, std::string child_name);
            std::string getFileName(std::string file_path);


            ros::NodeHandle nh_;
            
            std::string dataset_dir;

            std::vector<std::string> all_directories;
            int progress_in_scene;

            ros::Time current_time_;

            std::vector<std::string> agent_dirs;
            std::vector<std::vector<std::string>> scenes_of_agent;
            int curr_agent_idx = 0;
            int curr_scene_idx = 0;
            bool is_last_scene;

            sensor_msgs::PointCloud2 parent_msg, child_msg;

            // subscribers
            std::map<std::string, boost::shared_ptr<ros::Publisher>> pubs_map_;
            ros::ServiceServer service;
            
            std::map<std::string, std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> pointclouds_map_;

            std::vector<std::pair<std::string, std::string>> pcd_pairs; 
            std::vector<std::string> pcds_in_scene;
    };
} //namespace