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

class lidar_calib_test_bridge
{
    public:
        template<typename F>
        lidar_calib_test_bridge(ros::NodeHandle *nh_, ros::NodeHandle *private_nh_, F callback);
        lidar_calib_test_bridge();
        void fromTestMsg(const test_comm::test_pointcloud::ConstPtr parent_test_pc, const test_comm::test_pointcloud::ConstPtr child_test_pc);
        lidar_calib_test_comms::calib_result toTestReq();
        template<typename Type>
        void toEvalSrv(const Eigen::Matrix<Type, 4, 4>& guess);
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
lidar_calib_test_bridge::lidar_calib_test_bridge(ros::NodeHandle *nh_, ros::NodeHandle *private_nh_, 
                    F callback): native_calibrator_func(callback)
{
    error_service_client = private_nh_->serviceClient<lidar_calib_test_comms::calib_result>("/evaluator/calculate_error");

    parent_sub = new message_filters::Subscriber<lidar_calib_test_comms::test_pointcloud>(*nh_, "parent/pointcloud", 1);
    child_sub = new message_filters::Subscriber<lidar_calib_test_comms::test_pointcloud>(*nh_, "child/pointcloud", 1);
    pc_sync_ = new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(10), *parent_sub, *child_sub);
    pc_sync_->registerCallback(boost::bind(&lidar_calib_test_bridge::fromTestMsg, this, _1, _2));
}

template<typename Type>
void lidar_calib_test_bridge::toEvalSrv(const Eigen::Matrix<Type, 4, 4>& guess)
{
    Eigen::Matrix<Type, 3, 3> rot_mat = guess.block(0,0,3,3);
    Eigen::Quaternion<Type> q(rot_mat);
    Eigen::Matrix<Type, 3, 1> translation_vector = guess.block(0,3,3,1);

    geometry_msgs::TransformStamped tf;
    tf.child_frame_id = child_frame;
    tf.header.frame_id= parent_frame;

    tf.transform.translation.x = translation_vector(0);
    tf.transform.translation.y = translation_vector(1);
    tf.transform.translation.z = translation_vector(2);

    tf.transform.rotation.w = q.w();
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();

    test_comm::calib_result srv;
    srv.request.agent = agent_id;
    srv.request.scene = scene_id;

    tf.header.stamp = ros::Time::now();    
    srv.request.transform = tf;

    error_service_client.call(srv);
}
