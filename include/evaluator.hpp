#pragma once

#include <ros/ros.h>
#include <string>
#include <tf/transform_listener.h>
#include <ros/service.h>

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
        tfError compError(tf::Transform tf_gt, tf::Transform tf_pred); // TODO: take the result and GT TF. calculate the error between.
        void compStd();   // TODO: Compute standard deviation.
        void serve();     
        tf::StampedTransform getTFs(std::string parent_frame, std::string child_frame); 

    private:
        ros::ServiceServer service; // this service will be called from calibrator
        const std::string module_name = "[EVALUATOR] ";

        // std::vector<std::vector<std::vector<tfError>>> errors;  // agent -> scene -> pair -> error
        std::map< std::string, std::map<std::string, std::vector<tfError> > > error_maps; // agent -> scene -> pair -> error

        int agent_idx, scene_idx; 
        
        tf::TransformListener listener;
};