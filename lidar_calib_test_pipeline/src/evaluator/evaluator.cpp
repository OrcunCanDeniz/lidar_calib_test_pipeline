#include "evaluator.hpp"

bool evaluator::serve(lidar_calib_test_comms::calib_result::Request &req, lidar_calib_test_comms::calib_result::Response &res)   
// input parameter is tf_eval_srv: result_tf
{
    if (end_of_dataset) return false;

    parent_frame = req.transform.header.frame_id;
    child_frame = req.transform.child_frame_id;

    ROS_INFO_STREAM("Agent: " << req.agent << " Scene: " << req.scene);
    tf::StampedTransform gt_tf = getGTTF(parent_frame, child_frame);
    tf::Transform result_tf = fromMsg(req.transform);
    tfError inst_err = compError(gt_tf, result_tf); // calculate error between "GT_TF" and "result_tf" 
    ROS_INFO_STREAM( "Transform Error: " << 
                        inst_err.x << " " <<
                        inst_err.y << " " <<
                        inst_err.z << " " << 
                        inst_err.roll << " " <<
                        inst_err.pitch << " " <<
                        inst_err.yaw );

    error_maps[agent_id][scene_id].push_back(inst_err);

    std_srvs::SetBool srv;
    if(!data_provider_client.call(srv))
    {
        end_of_dataset = true; // returns false when dataset is finished 
        ROS_WARN_STREAM(module_name << "End of dataset!");
    }  

    res.ret = true;
    return true;  
}

tf::StampedTransform evaluator::getGTTF(std::string parent_frame, std::string child_frame)
{
    parent_frame = "gt_" + parent_frame; 
    child_frame = "gt_" + child_frame; 

    ROS_INFO_STREAM("Parent Frame: " << parent_frame << " Child Frame: " << child_frame);

    tf::StampedTransform transform;
    try
    {
        listener.waitForTransform(parent_frame, child_frame, ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform(parent_frame, child_frame, ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    return transform;
}

tf::Transform evaluator::fromMsg(geometry_msgs::TransformStamped received_result)
{
    tf::Quaternion q( received_result.transform.rotation.x,
                      received_result.transform.rotation.y,
                      received_result.transform.rotation.z,
                      received_result.transform.rotation.w ); 
    if (!isNormalized(q)) q.normalize(); 

    if (q.length() == 0) ROS_WARN_STREAM("INVALID QUATERNION. (ZERO)");
    

    tf::Vector3 t(  received_result.transform.translation.x,
                    received_result.transform.translation.y,
                    received_result.transform.translation.z );

    tf::Transform trans(q,t);

    return trans;
}

tfError evaluator::compError(tf::StampedTransform tf_gt, tf::Transform tf_pred)
{
    tfError err;
    tf::Vector3 gt_translation = tf_gt.getOrigin();
    tf::Vector3 pred_translation = tf_pred.getOrigin();
    tf::Matrix3x3 gt_m(tf_gt.getRotation());
    tf::Matrix3x3 pred_m(tf_pred.getRotation());
    double gt_roll, gt_yaw, gt_pitch;
    double pred_roll, pred_yaw, pred_pitch;

    gt_m.getRPY(gt_roll, gt_yaw, gt_pitch);
    pred_m.getRPY(pred_roll, pred_yaw, pred_pitch);

    err.x = gt_translation.getX() - pred_translation.getX();
    err.y = gt_translation.getY() - pred_translation.getY();
    err.z = gt_translation.getZ() - pred_translation.getZ();
    
    err.roll = (float) (gt_roll - pred_roll);
    err.pitch = (float) (gt_pitch - pred_pitch);
    err.yaw = (float) (gt_yaw - pred_yaw);

    return err;
}

bool evaluator::isNormalized(tf::Quaternion q)
{
    return q.length() == 1;
}

// void evaluator::callback(const lidar_calib_test_comms::test_pointcloud::ConstPtr& msg, const std::string frame_type)
// {
//     frame_type_to_id_m[frame_type] = msg->header.frame_id;
    
//     if (frame_type == "parent")
//     {
//         agent_id = msg->agent;
//         scene_id = msg->scene;
//     }
//     ROS_INFO("CB DONE");
// }


evaluator::evaluator()
{
    ros::NodeHandle private_nh("~");
    service = private_nh.advertiseService("calculate_error", &evaluator::serve, this);
    data_provider_client = private_nh.serviceClient<std_srvs::SetBool>("/data_provider_node/provide_pc_data");
}