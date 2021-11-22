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
                          // TODO: figure out a way to keep track of the processed agents and scenes.
        void compStd();   // TODO: Compute standard deviation.
        void serve();     // TODO: input result transform inside request

    private:

        ros::ServiceServer error_calculation_srv; // this service will be called from calibrator
        const std::string module_name = "[EVALUATOR] ";

        std::vector<std::vector<float>> translation_errors;  // agent -> scene -> error
        std::vector<std::vector<float>> rotation_errors;     // agent -> scene -> error

        int agent_idx, scene_idx; 
        
        float x_err, y_err, z_err, roll_err, pitch_err, yaw_err;
};