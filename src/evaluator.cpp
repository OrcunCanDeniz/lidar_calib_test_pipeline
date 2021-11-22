#include "evaluator.hpp"


evaluator::serve() // TODO: HIGH PRIORITY: figure out the type of service  
// input parameter is tf_eval_srv: agent_name, scene_name, parent_frame_id, child_frame_id, result_tf
//                                  
{
    // tf::StampedTransform gt_tf = getTFs(parent_frame_id, child_frame_id);
    // tfError inst_err = compError(GT_TF, result_tf); // calculate error between "GT_TF" and "result_tf"
    // error_maps[agent_name][scene_name].push_back(inst_err);

    // TODO: if all data is processed, handle it
    //                      otherwise, call service to trigger data publisher            
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

evaluator::evaluator()
{
    service = private_nh.advertiseService("calculate_error", &evaluator::serve, this);
}