#include "interface.hpp"


calib_test_bridge::calib_test_bridge(){};

calib_test_bridge::calib_test_bridge(ros::NodeHandle *nh_, ros::NodeHandle *private_nh_, 
                            boost::function<void(const sensor_msgs::PointCloud2::ConstPtr, const sensor_msgs::PointCloud2::ConstPtr)> calibrator_f)
{
    error_service_client = private_nh_->serviceClient<lidar_calib_test_comms::calib_result>("/evaluator/calculate_error");

    native_calibrator_func = calibrator_f;

    parent_sub = new message_filters::Subscriber<lidar_calib_test_comms::test_pointcloud>(*nh_, "parent/pointcloud", 1);
    child_sub = new message_filters::Subscriber<lidar_calib_test_comms::test_pointcloud>(*nh_, "child/pointcloud", 1);
    pc_sync_ = new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(10), *child_sub, *parent_sub);
    pc_sync_->registerCallback(boost::bind(&calib_test_bridge::fromTestMsg, this, _1, _2));
}


void calib_test_bridge::fromTestMsg(const test_comm::test_pointcloud::ConstPtr parent_test_pc, const test_comm::test_pointcloud::ConstPtr child_test_pc)
                                // boost::function<void (const sensor_msgs::PointCloud2::ConstPtr&, const sensor_msgs::PointCloud2::ConstPtr&)> native_calibrator_func)
{
    sensor_msgs::PointCloud2* parent_pc = new sensor_msgs::PointCloud2; 
    sensor_msgs::PointCloud2* child_pc = new sensor_msgs::PointCloud2;
    
    *parent_pc = parent_test_pc->pointcloud; // parent_test_pc.pointcloud is sensor_msgs/PointCloud2
    *child_pc = child_test_pc->pointcloud;
    parent_pc->header.frame_id = parent_test_pc->header.frame_id;
    child_pc->header.frame_id = child_test_pc->header.frame_id;

    if ( parent_test_pc->agent != child_test_pc->agent ) ROS_WARN("AGENT IDs DO NOT BETWEEN CHILD AND PARENT.");
    if ( parent_test_pc->scene != child_test_pc->scene ) ROS_WARN("SCENE IDs DO NOT BETWEEN CHILD AND PARENT.");

    agent_id = parent_test_pc->agent;
    scene_id = parent_test_pc->scene;

    const sensor_msgs::PointCloud2::ConstPtr parent_pc_ptr(parent_pc), child_pc_ptr(child_pc);

    native_calibrator_func(parent_pc_ptr, child_pc_ptr);
}


std::string calib_test_bridge::getAgent()
{
    return agent_id;
}

std::string calib_test_bridge::getScene()
{
    return scene_id;
}
