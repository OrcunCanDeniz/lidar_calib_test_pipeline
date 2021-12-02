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

class dummy_calibrator
{
    public:
        dummy_calibrator();
        void pcCallback(const lidar_calib_test_comms::test_pointcloud::ConstPtr& child_msg, const lidar_calib_test_comms::test_pointcloud::ConstPtr& parent_msg);
    
    private:
        ros::NodeHandle nh_;
        // ros::Subscriber parent_sub, child_sub;
        ros::ServiceClient error_service_client;
        // message_filters::Subscriber<lidar_calib_test_comms::test_pointcloud> parent_sub;
        // message_filters::Subscriber<lidar_calib_test_comms::test_pointcloud> child_sub;

        typedef message_filters::sync_policies::ApproximateTime<lidar_calib_test_comms::test_pointcloud, lidar_calib_test_comms::test_pointcloud> SyncPolicyT;
        message_filters::Subscriber<lidar_calib_test_comms::test_pointcloud> *parent_sub;
        message_filters::Subscriber<lidar_calib_test_comms::test_pointcloud> *child_sub;

        message_filters::Synchronizer<SyncPolicyT> *pc_sync_;

        lidar_calib_test_comms::calib_result srv;

};

dummy_calibrator::dummy_calibrator()
{
    ros::NodeHandle private_nh("~");
    error_service_client = private_nh.serviceClient<lidar_calib_test_comms::calib_result>("/evaluator/calculate_error");

    parent_sub = new message_filters::Subscriber<lidar_calib_test_comms::test_pointcloud>(nh_, "parent/pointcloud", 1);
    child_sub = new message_filters::Subscriber<lidar_calib_test_comms::test_pointcloud>(nh_, "child/pointcloud", 1);

    pc_sync_ = new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(10), *child_sub, *parent_sub);
    pc_sync_->registerCallback(boost::bind(&dummy_calibrator::pcCallback, this, _1, _2));

}

void dummy_calibrator::pcCallback(const lidar_calib_test_comms::test_pointcloud::ConstPtr& child_msg, const lidar_calib_test_comms::test_pointcloud::ConstPtr& parent_msg)
{
    geometry_msgs::TransformStamped tf_empty;

    tf_empty.header.frame_id = parent_msg->header.frame_id;
    tf_empty.child_frame_id = child_msg->header.frame_id;
    tf_empty.transform.rotation.w = 1;
    tf_empty.transform.translation.x = 2;
    tf_empty.transform.translation.y = 2;
    tf_empty.transform.translation.z = 2;
    tf_empty.header.stamp = ros::Time::now();


    srv.request.transform = tf_empty;
    srv.request.agent = parent_msg->agent;
    srv.request.scene = parent_msg->scene;
    std::cout<<"######################################"<<std::endl;
    ROS_INFO_STREAM("Agent: " << parent_msg->agent << " Scene: " << parent_msg->scene);

    sleep(2);

    if(error_service_client.call(srv)) 
    {
        ROS_INFO_STREAM("RESPONSE WAS: " << srv.response.ret);
    } else {
        ROS_INFO(":D");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dummy_calibrator");
    dummy_calibrator app;

    ros::spin();

    return 0;
}