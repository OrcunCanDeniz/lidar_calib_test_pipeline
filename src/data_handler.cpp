#include <data_handler_srv/data_handler.hpp>

void service(data_handler_srv::data_handler::Request &req
            data_handler_srv::data_handler::Response &res)
{
    if (directory_index_ != all_directories.size() + 1 )
    {
        ReadPCD(all_directories.at(directory_index_) + std::string("/lidar.pcd"));
        res.new_data_response = true;
        directory_index_ ++;
    } else {
        res.new_data_response = false;
        return;
    }   
}

void data_handler::setSubdirs(std::string parent_dir, bool is_dataset, int agent_idx=0)
{
    for(auto& subdir : fs::directory_iterator(parent_dir))
    {
        if (std::experimental::filesystem::is_directory(subdir))
        {
            if (is_dataset) // subdirs are agents
            {
                agent_dirs.push_back(subdir.path().string());
            } else {
                if (scenes_of_agent.size() == agent_idx) 
                {
                    std::vector<std::string> tmp;
                    scenes_of_agent.push_back(tmp);
                }
                scenes_of_agent[agent_idx].push_back(subdir.path().string());
            }
        }
    }
}

bool data_handler::findPcd() //return true when all dataset is processed 
{
    // TODO !!!! !!!! read pcds from scenes_of_agent[curr_agent_idx][curr_scene_idx]; 
    // std::cout<<scenes_of_agent[curr_agent_idx][curr_scene_idx]<<std::endl;

    if (curr_scene_idx < scenes_of_agent.size())
    {
        curr_scene_idx++;
    } else {
        if( curr_agent_idx < scenes_of_agent.size()-1)
        {
            curr_agent_idx++;
            curr_scene_idx = 0;
        } else {
            std::cout<< "ALL DONE"<<std::endl;
            return true;
        }
    }
    return false;
}

// add extrinsic parser

void data_handler::ReadScene(std::string in_file)
{ /// get scene data
    //Create a PointCloud value
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

    //Open the PCD file
    if (pcl::io::loadPCDFile<pcl::PointXYZI> (in_file, *cloud) == -1) 
    {
        PCL_ERROR ("Couldn't read in_file\n");
    }
    pcl::toROSMsg(*cloud.get(), lidar_msg);
    std::cout << std::endl;
}

bool data_handler::IsPathExist(const std::string &s)
{
    struct stat buffer;
    return (stat (s.c_str(), &buffer) == 0);
}

std::vector<std::string> data_handler::get_directories(const std::string& s)
{
        std::vector<std::string> r;
        for(auto& p : std::experimental::filesystem::recursive_directory_iterator(s))
            if (std::experimental::filesystem::is_directory(p))
                r.push_back(p.path().string());
        return r;
}

data_handler::data_handler(): directory_index_(0)
{
    ros::NodeHandle private_nh("~");
    const std::string HOME = getenv("HOME");
    private_nh.param<std::string>("dataset_dir", dataset_dir, HOME + "/dataset/");
    
    if (!IsPathExist(dataset_dir))
    {
        ROS_ERROR_STREAM("Failed to find directory: " << dataset_dir);
    }

    //cache dirs of agents and scenes belonging to them
    setSubdirs(s, true); // get Agents
    int agent_idx = 0;
    for(auto& agent : agent_dirs)
    {
      setSubdirs(agent,false,agent_idx);
      agent_idx++;  
    } 

    ros::Rate loop_rate(10);

    pubs["parent"].reset(new ros::Publisher());
    pubs["child"].reset(new ros::Publisher());
    *pubs["parent"] = private_nh.advertise<sensor_msgs::PointCloud2>("/parent/pointcloud", 1); 
    *pubs["child"] = private_nh.advertise<sensor_msgs::PointCloud2>("/child/pointcloud", 1); 


    all_directories = get_directories(pcd_image_input_dir_);
    std::sort(all_directories.begin(), all_directories.end());
}