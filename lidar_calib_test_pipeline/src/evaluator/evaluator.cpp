#include "evaluator.hpp"

bool evaluator::serve(lidar_calib_test_comms::calib_result::Request &req, lidar_calib_test_comms::calib_result::Response &res)   
// input parameter is tf_eval_srv: result_tf
//                                  
{
    if (end_of_dataset) return false;

    ROS_INFO_STREAM("Parent Frame: " << req.transform.header.frame_id << " Child Frame: " << req.transform.child_frame_id 
                        << " Agent: " << req.agent << " Scene: " << req.scene);
    tf::StampedTransform gt_tf = getTFs(frame_type_to_id_m["parent"], frame_type_to_id_m["child"]);
    // tfError inst_err = compError(gt_tf, result_tf); // calculate error between "GT_TF" and "result_tf" // TODO: handle geometry_msgs::transform usage
    // error_maps[agent_id][scene_id].push_back(inst_err);


    std_srvs::SetBool srv;
    if(!data_provider_client.call(srv))
    {
        end_of_dataset = true; // returns false when dataset is finished 
        ROS_WARN_STREAM(module_name << "End of dataset!");
    }  

    res.ret = true;
    return true;  
}

tf::StampedTransform evaluator::getTFs(std::string parent_frame, std::string child_frame)
{
    tf::StampedTransform transform;
    try
    {
    listener.lookupTransform(parent_frame, child_frame, // check "target" and "source" frames   
                            ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    return transform;
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

    // parentPc_sub = nh_.subscribe<lidar_calib_test_comms::test_pointcloud>("parent/pointcloud", 1, boost::bind(&evaluator::callback, this, _1, "parent"));
    // childPc_sub = nh_.subscribe<lidar_calib_test_comms::test_pointcloud>("child/pointcloud", 1, boost::bind(&evaluator::callback, this, _1, "child"));
    
    frame_type_to_id_m["parent"] = " ";
    frame_type_to_id_m["child"] = " " ;
}