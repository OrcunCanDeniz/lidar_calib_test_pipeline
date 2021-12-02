#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <ros/service.h>

#include "lidar_calib_test_comms/test_pointcloud.h"
#include "lidar_calib_test_comms/calib_result.h"
#include "std_srvs/SetBool.h"

#include "types.hpp" // custom transform type with custom rotation, translation 
#include "io_utils.hpp"



class evaluator
{
    public:
        evaluator();
        err_tf_pair compError(tf::StampedTransform tf_gt, tf::Transform tf_pred);
        void compStats(); 
        void dumpStats();
        bool serve(lidar_calib_test_comms::calib_result::Request &req, lidar_calib_test_comms::calib_result::Response &res);
        tf::StampedTransform getGTTF(std::string parent_frame, std::string child_frame); 
        tf::Transform fromMsg(geometry_msgs::TransformStamped received_result);
        bool isNormalized(tf::Quaternion q);
        float getCircularDiff(float b1, float b2);

    private:
        ros::NodeHandle nh_;
        ros::ServiceServer service; // this service will be called from calibrator
        ros::ServiceClient data_provider_client;

        ros::Subscriber parentPc_sub, childPc_sub; 
        
        bool end_of_dataset = false;

        const std::string module_name = "[EVALUATOR] ";

        std::map< std::string, std::map<std::string, std::vector<err_tf_pair> > > err_tf_map; // agent -> scene -> pair -> error
        std::map< std::string, std::map<std::string, statType > > statStore; // agent -> scene -> pair -> error
        std::map< std::string, statType> agent_stats;
        genericT overall_tf_stat, overall_err_stat;

        std::string agent_id, scene_id; 
        std::string parent_frame, child_frame; 
        
        tf::TransformListener listener;
};
/// Stats to be computed;
//          - std dev of transforms from the same agent 
//          - mean of transforms from the same agent
//          - std_dev of transform errors
//          - mean of transform errors