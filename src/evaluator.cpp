#include "evaluator.hpp"


evaluator::evaluator(){}

tfError evaluator::compError(tf::Transform tf_gt, tf::Transform tf_pred)
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