#include "evaluator.hpp"

bool evaluator::serve(lidar_calib_test_comms::calib_result::Request &req, lidar_calib_test_comms::calib_result::Response &res)   
// input parameter is tf_eval_srv: result_tf
{
    if (end_of_dataset) return false;

    parent_frame = req.transform.header.frame_id;
    child_frame = req.transform.child_frame_id;
    agent_id = req.agent;
    scene_id = req.scene;

    ROS_INFO_STREAM("Agent: " << req.agent << " Scene: " << req.scene);
    tf::StampedTransform gt_tf = getGTTF(parent_frame, child_frame);
    tf::Transform result_tf = fromMsg(req.transform);
    err_tf_pair inst_err_tf = compError(gt_tf, result_tf); // calculate error between "GT_TF" and "result_tf" 
    // ROS_INFO_STREAM( "Transform Error: " << 
    //                     inst_err.x << " " <<
    //                     inst_err.y << " " <<
    //                     inst_err.z << " " << 
    //                     inst_err.roll << " " <<
    //                     inst_err.pitch << " " <<
    //                     inst_err.yaw );

    err_tf_map[agent_id][scene_id].push_back(inst_err_tf);

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

float getCircularDiff(float b1, float b2) 
{
	float r = fmod(fabs(b2 - b1), 2*M_PI); 
	if (r < -M_PI)
		r += 2*M_PI;
	if (r >= M_PI)
		r -= 2*M_PI;
	return fabs(r);
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

err_tf_pair evaluator::compError(tf::StampedTransform tf_gt, tf::Transform tf_pred)
{
    genericTf err, result_tf;
    tf::Vector3 gt_translation = tf_gt.getOrigin();
    tf::Vector3 pred_translation = tf_pred.getOrigin();
    tf::Matrix3x3 gt_m(tf_gt.getRotation());
    tf::Matrix3x3 pred_m(tf_pred.getRotation());
    double gt_roll, gt_yaw, gt_pitch;
    double pred_roll, pred_yaw, pred_pitch;

    gt_m.getRPY(gt_roll, gt_yaw, gt_pitch);
    pred_m.getRPY(pred_roll, pred_yaw, pred_pitch);

    err.x = fabs(gt_translation.getX() - pred_translation.getX());
    err.y = fabs(gt_translation.getY() - pred_translation.getY());
    err.z = fabs(gt_translation.getZ() - pred_translation.getZ());
    
    err.roll = getCircularDiff(gt_roll, pred_roll);
    err.pitch = getCircularDiff(gt_pitch, pred_pitch);
    err.yaw = getCircularDiff(gt_yaw, pred_yaw);

    result_tf.x = pred_translation.getX() ; 
    result_tf.y = pred_translation.getY() ; 
    result_tf.z = pred_translation.getZ() ; 
    result_tf.roll = (float)pred_roll ;
    result_tf.pitch = (float)pred_pitch ;
    result_tf.yaw = (float)pred_yaw ;

    return std::make_pair(err, result_tf);
}

void evaluator::compStats()
{
    for (const auto &agent_scenePair : err_tf_map ) // agent_scenePair : std::pair< std::string, std::map<std::string, std::vector<err_tf_pair> > >
    {
        std::string cur_agent = agent_scenePair.first;

        for (const auto &pcd_combs_from_scene : agent_scenePair.second ) 
        {
            std::string cur_scene = pcd_combs_from_scene.first;
            errT cum_scene_err;
            tfT cum_scene_tf;
            float pair_dif, sum;
            for (const auto &err_tf_pair_of_pcd : pcd_combs_from_scene.second )
            {
                cum_scene_err += err_tf_pair_of_pcd.first; // to calculate mean errors for a scene
                cum_scene_tf += err_tf_pair_of_pcd.second; // to calculate mean tf for a scene
            }

            statStore[cur_agent][cur_scene].err.mean = (cum_scene_err / pcd_combs_from_scene.second.size())
            statStore[cur_agent][cur_scene].tf.mean = (cum_scene_err / pcd_combs_from_scene.second.size())
        }
    }
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