#include <extrinsics_parser.hpp>


namespace data_provider
{

    namespace fs = std::experimental::filesystem;

    extrinsics_manager::extrinsics_manager():paths_ready(0),agent_idx{0}, last_set_processed{false}
    {}

    void extrinsics_manager::setPaths(std::vector<std::string> agent_paths)
    // input: vector<string> : paths to agents 
    {
        all_agents = agent_paths;
        if (DEBUG) ROS_INFO_STREAM(module_name << "Agent paths: ");
        for(int i=0 ; i<all_agents.size() ; i++)
        {
            if (fs::exists(all_agents[i]))
            {
                if ( fs::exists(all_agents[i] + "/extrinsics/") ) 
                {
                    all_agents[i] += "/extrinsics/";
                    if (DEBUG) ROS_INFO_STREAM(module_name << "  - " << all_agents[i]);
                } else {
                    ROS_WARN_STREAM(module_name << all_agents[i] << " does not have extrinsics folder !");
                    all_agents.erase(all_agents.begin()+i);
                }
            } else {
                ROS_WARN_STREAM(module_name << all_agents[i] << " does not exist !");
                all_agents.erase(all_agents.begin()+i);
            }
        }   

        paths_ready = true;
    }

    void extrinsics_manager::parseYAML(std::string tf_yaml_path)
    // read yaml file and convert data inside to tf transform
    {
        YAML::Node raw_extrinsics;
        std::string parent_frame_id, child_frame_id;
        try
        {
            raw_extrinsics = YAML::LoadFile(tf_yaml_path);
        } catch (const YAML::ParserException& ex) {
            std::cout << ex.what() << std::endl;
            return;
        }

        // get parent and child frame ids
        parent_frame_id = raw_extrinsics["parent_frame_id"].as<std::string>();
        child_frame_id = raw_extrinsics["child_frame_id"].as<std::string>();
        
        // get translation
        tf::Vector3 t(  raw_extrinsics["translation"]["x"].as<float>(),
                        raw_extrinsics["translation"]["y"].as<float>(),
                        raw_extrinsics["translation"]["z"].as<float>() );
        // get rotation
        tf::Quaternion q(   raw_extrinsics["rotation"]["x"].as<float>(),
                            raw_extrinsics["rotation"]["y"].as<float>(),
                            raw_extrinsics["rotation"]["z"].as<float>(),
                            raw_extrinsics["rotation"]["w"].as<float>() );
        
        if (DEBUG)
        {
            ROS_INFO_STREAM(module_name << "Parent frame: "<< parent_frame_id);
            ROS_INFO_STREAM(module_name << "Child frame: "<< child_frame_id );
        }

        tf::Transform transform(q, t);
        initial_transforms_cache.push_back( tf::StampedTransform(addNoise(q,t), ros::Time::now(), parent_frame_id, child_frame_id));
        gt_transforms_cache.push_back(tf::StampedTransform(transform, ros::Time::now(), "gt_" + parent_frame_id, "gt_" + child_frame_id));
    }

    void extrinsics_manager::broadcastTFs(const ros::TimerEvent&)
    //used as a timer callback
    //publish cached transforms
    {
        static tf::TransformBroadcaster br;

        for (auto& transform: gt_transforms_cache)
        {
            transform.stamp_ = ros::Time::now(); // refresh transform timestamps
            br.sendTransform(transform);
        }

        for (auto& transform: initial_transforms_cache)
        {
            transform.stamp_ = ros::Time::now();
            br.sendTransform(transform);
        }
    }

    tf::Transform extrinsics_manager::addNoise(tf::Quaternion q, tf::Vector3 t)
    {
        double yaw, pitch, roll;
        tf::Matrix3x3 mat(q);
        mat.getEulerYPR(yaw, pitch, roll);

        yaw += std::round(rotation_noise_d(gen));
        pitch += std::round(rotation_noise_d(gen));
        roll += std::round(rotation_noise_d(gen));

        t.setX( t.x() + std::round(translation_noise_d(gen)) );
        t.setY( t.y() + std::round(translation_noise_d(gen)) );
        t.setZ( t.z() + std::round(translation_noise_d(gen)) );

        tf::Quaternion quat(yaw, pitch, roll);
        tf::Transform transform(quat, t);

        return transform;
    }

    bool extrinsics_manager::next()
    // read yaml files from disk and convert them to tf::transform
    // return true if there is still data left to process, otherwise false 
    {
        if (!paths_ready) ROS_ERROR("NO CALIBRATION FILES KNOWN TO EXTRINSICS MANAGER !");

        if (DEBUG) ROS_INFO_STREAM(module_name << "###### GETTING NEXT AGENT'S CALIBRATIONS ######");
        if (DEBUG) ROS_INFO_STREAM(module_name << "Caching -> " << all_agents[agent_idx]);
        gt_transforms_cache.clear(); // clear tf data from another agent
        initial_transforms_cache.clear();
        
        for(auto& subdir : fs::directory_iterator(all_agents[agent_idx]))
        {
            if (fs::path(subdir).extension() == ".yaml")
            {
                parseYAML(subdir.path().string());
            } else {
                ROS_WARN_STREAM(module_name << "    " << subdir << " does not exist !");
            }        
        }

        bool one_more_iter = updateIdx();
        if (DEBUG) ROS_INFO_STREAM(module_name << "##########################");

        return one_more_iter;
    }


    bool extrinsics_manager::updateIdx()
    // safely update idx pointing to the agent that will be processed
    // return false if idx is already reached the limit, true if it still can be incremented 
    {
        if (agent_idx<=(int)all_agents.size()-1 && !last_set_processed) // if idx is still less than max idx and unprocessed data still exists 
        {   
            if (agent_idx==(int)all_agents.size()-1) // if last set of data is processed
            {
                last_set_processed=true;
                return false;
            }

            agent_idx++;
            return true;

        } else {
            return false;
        }
    }
}