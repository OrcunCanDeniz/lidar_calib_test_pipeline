#pragma once

#include "lidar_calib_test_comms/test_pointcloud.h"
#include "lidar_calib_test_comms/calib_result.h"

#include "std_srvs/SetBool.h"
#include "ros/ros.h"

#include "sensor_msgs/PointCloud2.h"

#include "message_filters/subscriber.h"
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include "Eigen/Core"
#include <Eigen/Geometry> 

#include <string>
#include <vector>

#include <functional>


namespace test_comm = lidar_calib_test_comms;

class calib_test_bridge
{
    public:
        template<typename F>
        calib_test_bridge(ros::NodeHandle *nh_, ros::NodeHandle *private_nh_, F callback);
        calib_test_bridge();
        void fromTestMsg(const test_comm::test_pointcloud::ConstPtr parent_test_pc, const test_comm::test_pointcloud::ConstPtr child_test_pc);
        lidar_calib_test_comms::calib_result toTestReq();
        void toEvalSrv(Eigen::Matrix4f guess);
        void toEvalSrv(geometry_msgs::TransformStamped tf);
        std::string getAgent();
        std::string getScene();


    private:
        std::string scene_id, agent_id;
        std::string parent_frame, child_frame;

        boost::function<void (const sensor_msgs::PointCloud2::ConstPtr, const sensor_msgs::PointCloud2::ConstPtr)> native_calibrator_func;

        ros::ServiceClient error_service_client;
        ros::NodeHandle *nh, *private_nh;
        typedef message_filters::sync_policies::ApproximateTime<lidar_calib_test_comms::test_pointcloud, lidar_calib_test_comms::test_pointcloud> SyncPolicyT;
        message_filters::Subscriber<lidar_calib_test_comms::test_pointcloud> *parent_sub;
        message_filters::Subscriber<lidar_calib_test_comms::test_pointcloud> *child_sub;
        message_filters::Synchronizer<SyncPolicyT> *pc_sync_;
};

template<typename F>
calib_test_bridge::calib_test_bridge(ros::NodeHandle *nh_, ros::NodeHandle *private_nh_, 
                    F callback): native_calibrator_func(callback)
{
    error_service_client = private_nh_->serviceClient<lidar_calib_test_comms::calib_result>("/evaluator/calculate_error");

    parent_sub = new message_filters::Subscriber<lidar_calib_test_comms::test_pointcloud>(*nh_, "parent/pointcloud", 1);
    child_sub = new message_filters::Subscriber<lidar_calib_test_comms::test_pointcloud>(*nh_, "child/pointcloud", 1);
    pc_sync_ = new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(10), *parent_sub, *child_sub);
    pc_sync_->registerCallback(boost::bind(&calib_test_bridge::fromTestMsg, this, _1, _2));
}
