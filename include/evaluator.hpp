#pragma once

#include <ros/ros.h>
#include <string>
#include <tf/transform_listener.h>
#include <ros/service.h>

class evaluator
{
    public:
        evaluator();
        void compError(); // TODO: take the result and GT TF. calculate the error between.
                          // TODO: figure out a way to keep track of the processed agents and scenes.
        void compStd(); // TODO: Compute standard deviation.

    private:

        ros::ServiceServer error_calculation_srv; // this service will be called from calibrator
        const std::string module_name = "EVALUATOR ";

        std::vector<std::vector<float>> translation_errors;  // agent -> scene -> error
        std::vector<std::vector<float>> rotation_errors;     // agent -> scene -> error

        int agent_idx, scene_idx; 



};