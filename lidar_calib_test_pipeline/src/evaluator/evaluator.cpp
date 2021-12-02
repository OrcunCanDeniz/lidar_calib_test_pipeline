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
    err_tf_pair inst_err = compError(gt_tf, result_tf); // calculate error between "GT_TF" and "result_tf" 
    ROS_INFO_STREAM( "Transform Error: " << 
                        inst_err.first.trans.x << " " <<
                        inst_err.first.trans.y << " " <<
                        inst_err.first.trans.z << " " << 
                        inst_err.first.rot.roll << " " <<
                        inst_err.first.rot.pitch << " " <<
                        inst_err.first.rot.yaw );

    err_tf_map[agent_id][scene_id].push_back(inst_err);

    std_srvs::SetBool srv;
    if(!data_provider_client.call(srv))
    {
        end_of_dataset = true; // returns false when dataset is finished 
        ROS_WARN_STREAM(module_name << "End of dataset! Calculating statistics ... ");
        compStats();
        ROS_INFO("Writing stats ...");
        dumpStats();
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
        listener.waitForTransform(parent_frame, child_frame, ros::Time(0), ros::Duration(10.0)); //target: parent, source:child 
        listener.lookupTransform(parent_frame, child_frame, ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    return transform;
}

float evaluator::getCircularDiff(float b1, float b2) 
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
    genericT err, result_tf;
    tf::Vector3 gt_translation = tf_gt.getOrigin();
    tf::Vector3 pred_translation = tf_pred.getOrigin();
    tf::Matrix3x3 gt_m(tf_gt.getRotation());
    tf::Matrix3x3 pred_m(tf_pred.getRotation());
    double gt_roll, gt_yaw, gt_pitch;
    double pred_roll, pred_yaw, pred_pitch;

    gt_m.getRPY(gt_roll, gt_yaw, gt_pitch);
    pred_m.getRPY(pred_roll, pred_yaw, pred_pitch);

    err.trans.x = fabs(gt_translation.getX() - pred_translation.getX());
    err.trans.y = fabs(gt_translation.getY() - pred_translation.getY());
    err.trans.z = fabs(gt_translation.getZ() - pred_translation.getZ());
    
    err.rot.roll = getCircularDiff(gt_roll, pred_roll);
    err.rot.pitch = getCircularDiff(gt_pitch, pred_pitch);
    err.rot.yaw = getCircularDiff(gt_yaw, pred_yaw);

    result_tf.trans.x = pred_translation.getX() ; 
    result_tf.trans.y = pred_translation.getY() ; 
    result_tf.trans.z = pred_translation.getZ() ; 
    result_tf.rot.roll = (float)pred_roll ;
    result_tf.rot.pitch = (float)pred_pitch ;
    result_tf.rot.yaw = (float)pred_yaw ;

    return std::make_pair(err, result_tf);
}

void evaluator::compStats()
{
    for (const auto &agent_scenePair : err_tf_map ) // agent_scenePair : std::pair< std::string, std::map<std::string, std::vector<err_tf_pair> > >
    {
        std::string cur_agent = agent_scenePair.first;
        genericT agent_err;
        for (const auto &pcd_combs_from_scene : agent_scenePair.second ) 
        {
            std::string cur_scene = pcd_combs_from_scene.first;
            genericT cum_scene_err;
            for (const auto &err_of_pcd_comb : pcd_combs_from_scene.second)
            {
                cum_scene_err += err_of_pcd_comb.first; // accumulate data in scene
                // cum_scene_err_tf.second += err_tf_pair_of_pcd.second; 
            }
            agent_err.accumulate(cum_scene_err) ; // accumulate scenes' data into agent based stat container

            // agent_err_tf.second += cum_scene_err.second; 
            statStore[cur_agent][cur_scene].mean = cum_scene_err.getMean(); // calculate scene's stats and store
            statStore[cur_agent][cur_scene].dev = cum_scene_err.getStDev();
            // statStore[cur_agent][cur_scene].tf.mean = cum_scene_err_tf.second.getMean();
            // statStore[cur_agent][cur_scene].tf.dev = cum_scene_err_tf.second.getStDev();
        }
        agent_stats[cur_agent].mean = agent_err.getMean();
        agent_stats[cur_agent].dev = agent_err.getStDev();
        // agent_stats[cur_agent].tf.mean = agent_err_tf.second.getMean(); //calculate agent stats from accumulated data
        // agent_stats[cur_agent].tf.dev = agent_err_tf.second.getStDev();
    }
}

bool evaluator::isNormalized(tf::Quaternion q)
{
    return q.length() == 1;
}

void evaluator::dumpStats()
{
    for (const auto &agent_scenePair : statStore ) // agent_scenePair : std::pair< std::string, std::map<std::string, std::vector<err_tf_pair> > >
    {
        std::string cur_agent = agent_scenePair.first;

        for (const auto &scene_stat : agent_scenePair.second ) 
        {
            std::string cur_scene = scene_stat.first;
            io_utils::addStat(cur_agent, cur_scene, scene_stat.second);
        }

        io_utils::prettyPrint(cur_agent, agent_stats[cur_agent]);  
     
    }

    io_utils::writeCsv();
}

evaluator::evaluator()
{
    ros::NodeHandle private_nh("~");
    service = private_nh.advertiseService("calculate_error", &evaluator::serve, this);
    data_provider_client = private_nh.serviceClient<std_srvs::SetBool>("/data_provider_node/provide_pc_data");
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "evaluator");
    evaluator app;

    ros::spin();

    return 0;
}