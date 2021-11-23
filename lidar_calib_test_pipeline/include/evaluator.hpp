#pragma once

#include <ros/ros.h>
#include <string>
#include <tf/transform_listener.h>
#include <ros/service.h>

#include "lidar_calib_test_comms/test_pointcloud.h"
#include "lidar_calib_test_comms/calib_result.h"

struct tfError {
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
};

class evaluator
{
    public:
        evaluator();
        tfError compError(tf::StampedTransform tf_gt, tf::Transform tf_pred); // TODO: take the result and GT TF. calculate the error between.
        void compStd();   // TODO: Compute standard deviation.
        bool serve(lidar_calib_test_comms::calib_result::Request &req, lidar_calib_test_comms::calib_result::Response &res);
        void callback(const lidar_calib_test_comms::test_pointcloud::ConstPtr& msg, const std::string frame_type);
        tf::StampedTransform getTFs(std::string parent_frame, std::string child_frame); 

    private:
        ros::NodeHandle nh_;
        ros::ServiceServer service; // this service will be called from calibrator
        ros::Subscriber parentPc_sub, childPc_sub; 
            
        const std::string module_name = "[EVALUATOR] ";

        std::map< std::string, std::map<std::string, std::vector<tfError> > > error_maps; // agent -> scene -> pair -> error

        std::string agent_id, scene_id; 
        std::map<std::string, std::string> frame_type_to_id_m; // frame_type to frame_id
        
        tf::TransformListener listener;
};