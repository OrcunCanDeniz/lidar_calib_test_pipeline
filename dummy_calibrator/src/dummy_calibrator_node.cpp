#include <ros/ros.h>
#include <string>
#include <tf/transform_listener.h>
#include <ros/service.h>

#include "lidar_calib_test_comms/test_pointcloud.h"
#include "lidar_calib_test_comms/calib_result.h"

#include "geometry_msgs/TransformStamped.h"
#include <unistd.h>

#include "message_filters/subscriber.h"
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include "lidar_calib_test_bridge/interface.hpp"

class dummy_calibrator
{
    public:
        dummy_calibrator();
        void pcCallback(const sensor_msgs::PointCloud2::ConstPtr& parent, const sensor_msgs::PointCloud2::ConstPtr& child);
    
    private:
        int cnt = 0;
        ros::NodeHandle nh_;
        ros::ServiceClient error_service_client;

        typedef message_filters::sync_policies::ApproximateTime<lidar_calib_test_comms::test_pointcloud, lidar_calib_test_comms::test_pointcloud> SyncPolicyT;
        message_filters::Subscriber<lidar_calib_test_comms::test_pointcloud> *parent_sub;
        message_filters::Subscriber<lidar_calib_test_comms::test_pointcloud> *child_sub;

        message_filters::Synchronizer<SyncPolicyT> *pc_sync_;

        lidar_calib_test_comms::calib_result srv;

        calib_test_bridge* bridge; 

};

dummy_calibrator::dummy_calibrator()
{
    ros::NodeHandle private_nh("~");
    // error_service_client = private_nh.serviceClient<lidar_calib_test_comms::calib_result>("/evaluator/calculate_error");

    // parent_sub = new message_filters::Subscriber<lidar_calib_test_comms::test_pointcloud>(nh_, "parent/pointcloud", 1);
    // child_sub = new message_filters::Subscriber<lidar_calib_test_comms::test_pointcloud>(nh_, "child/pointcloud", 1);

    // pc_sync_ = new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(10), *child_sub, *parent_sub);
    // pc_sync_->registerCallback(boost::bind(&dummy_calibrator::pcCallback, this, _1, _2));

    bridge = new calib_test_bridge(&nh_, &private_nh, boost::bind(&dummy_calibrator::pcCallback, this, _1, _2));
}


void dummy_calibrator::pcCallback(const sensor_msgs::PointCloud2::ConstPtr& parent_msg, const sensor_msgs::PointCloud2::ConstPtr& child_msg)
{
    geometry_msgs::TransformStamped tf_empty;

    tf_empty.header.frame_id = parent_msg->header.frame_id;
    tf_empty.child_frame_id = child_msg->header.frame_id;
    if (cnt % 2 == 0)
    {
        tf_empty.transform.translation.x = 2;
        tf_empty.transform.translation.y = 2;
        tf_empty.transform.translation.z = 2;
        tf_empty.transform.rotation.w = 1;
    } else {
        tf_empty.transform.translation.x = 2.5;
        tf_empty.transform.translation.y = 2.5;
        tf_empty.transform.translation.z = 2.5;
        tf_empty.transform.rotation.x =0.707 ;
        tf_empty.transform.rotation.y =0.0 ;
        tf_empty.transform.rotation.z =0.707 ;
        tf_empty.transform.rotation.w =0.0 ;
    }
    cnt ++;

    bridge->toEvalSrv(tf_empty);


    sleep(2);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dummy_calibrator");
    dummy_calibrator app;

    ros::spin();

    return 0;
}