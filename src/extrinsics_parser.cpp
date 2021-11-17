#include <extrinsics_parser.hpp>


namespace data_provider
{
    namespace extrinsics
    {
        namespace fs = std::experimental::filesystem;

        extrinsics_manager::extrinsics_manager()
        {
            ROS_ERROR("No paths provided for extrinsic parameters !");
        }

        extrinsics_manager::extrinsics_manager(std::vector<std::string> agent_paths): all_agents{agent_paths}, agent_idx{0}, last_set_processed{false} 
        {
            // input: vector<string> : paths to agents 
            ROS_INFO_STREAM(module_name << "Agent paths: ");
            for(int i=0 ; i<all_agents.size() ; i++)
            {
                if (fs::exists(all_agents[i]))
                {
                    if ( fs::exists(all_agents[i] + "extrinsics/") ) 
                    {
                        all_agents[i] += "extrinsics/";
                        ROS_INFO_STREAM(module_name << "- " << all_agents[i]);
                    } else {
                        ROS_WARN_STREAM(module_name << all_agents[i] << " does not have extrinsics folder !");
                        all_agents.erase(all_agents.begin()+i);
                    }
                } else {
                    ROS_WARN_STREAM(module_name << all_agents[i] << " does not exist !");
                    all_agents.erase(all_agents.begin()+i);
                }
            }            
        }

        void extrinsics_manager::parseYAML(std::string tf_yaml_path)
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
            
            ROS_INFO_STREAM(module_name << "Parent frame: "<< parent_frame_id);
            ROS_INFO_STREAM(module_name << "Child frame: "<< child_frame_id );
           
            tf::Transform transform(q, t);
            broadcastTF(transform, parent_frame_id, child_frame_id );
        }

        void extrinsics_manager::broadcastTF(tf::Transform transform, std::string parent_frame, std::string child_frame)
        {
            static tf::TransformBroadcaster br;
            std::cout<< "kaljdşakjs"<< std::endl;

            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "fdsdsf"));
        }

        bool extrinsics_manager::next()
        {
            ROS_INFO_STREAM(module_name << all_agents[agent_idx]);
            // get next set of extrinsics  
            for(auto& subdir : fs::directory_iterator(all_agents[agent_idx]))
            {
                if (fs::path(subdir).extension() == ".yaml")
                {
                    // ROS_INFO_STREAM(module_name << "    Parsing " << subdir); 
                    parseYAML(subdir.path().string());
                } else {
                    ROS_WARN_STREAM(module_name << "    " << subdir << " does not exist !");
                }        
            }

            bool one_more_iter = updateIdx();

            return one_more_iter;
        }


        bool extrinsics_manager::updateIdx()
        {
            if (agent_idx<=(int)all_agents.size()-1 && !last_set_processed)
            {   
                if (agent_idx==(int)all_agents.size()-1) 
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
}


int main(int argc, char **argv)
{
    std::vector<std::string> agent_dirs{"/home/orcun/test_ws/src/data_handler_srv/dummy_dataset/agent_0/",
                                        "/home/orcun/test_ws/src/data_handler_srv/dummy_dataset/agent_1/",
                                        "/home/orcun/test_ws/src/data_handler_srv/dummy_dataset/agent_2/"};
    data_provider::extrinsics::extrinsics_manager Manager(agent_dirs);
    bool stat(true);

    ros::init(argc, argv, "ext_provider_node");


    while (stat)
    { 
      stat = Manager.next(); 
    }
    return 0;
}
