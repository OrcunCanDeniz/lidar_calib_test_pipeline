#include "interface.hpp"


calib_test_bridge::calib_test_bridge()
{
    std::cout<< "empty const" << std::endl;
}

void calib_test_bridge::fromTestMsg(const test_comm::test_pointcloud::ConstPtr parent_test_pc, const test_comm::test_pointcloud::ConstPtr child_test_pc)
{
    sensor_msgs::PointCloud2* parent_pc = new sensor_msgs::PointCloud2; 
    sensor_msgs::PointCloud2* child_pc = new sensor_msgs::PointCloud2;
    
    *parent_pc = parent_test_pc->pointcloud; // parent_test_pc.pointcloud is sensor_msgs/PointCloud2
    *child_pc = child_test_pc->pointcloud;
    parent_pc->header.frame_id = parent_test_pc->header.frame_id;
    child_pc->header.frame_id = child_test_pc->header.frame_id;

    if ( parent_test_pc->agent != child_test_pc->agent ) ROS_WARN("AGENT IDs DO NOT BETWEEN CHILD AND PARENT.");
    if ( parent_test_pc->scene != child_test_pc->scene ) ROS_WARN("SCENE IDs DO NOT BETWEEN CHILD AND PARENT.");

    agent_id = (std::string)parent_test_pc->agent;
    scene_id = (std::string)parent_test_pc->scene;

    parent_frame = parent_test_pc->header.frame_id; 
    child_frame =  child_test_pc->header.frame_id;

    const sensor_msgs::PointCloud2::ConstPtr parent_pc_ptr(parent_pc), child_pc_ptr(child_pc);
    native_calibrator_func(parent_pc_ptr, child_pc_ptr);
}

void calib_test_bridge::toEvalSrv(Eigen::Matrix4f guess)
{
    Eigen::Matrix3f rotation_matrix = guess.block(0,0,3,3);
    Eigen::Vector3f translation_vector = guess.block(0,3,3,1);

    test_comm::calib_result srv;
    srv.request.agent = agent_id;
    srv.request.agent = scene_id;

    geometry_msgs::TransformStamped tf;
    tf.child_frame_id = child_frame;
    tf.header.frame_id= parent_frame;

    tf.transform.translation.x = translation_vector(0);
    tf.transform.translation.y = translation_vector(1);
    tf.transform.translation.z = translation_vector(2);

 // TODO: from rotation matrix to quaternion

    
}


std::string calib_test_bridge::getAgent()
{
    return agent_id;
}

std::string calib_test_bridge::getScene()
{
    return scene_id;
}
