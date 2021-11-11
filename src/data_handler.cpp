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

void data_handler::GetAgent(std::string agent_data_path)
{
    // input: path to agent dir which contains scene subdirs with data
    // set extrinsics for agent
    // get dirs of scenes agent's been to
    

}

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
    private_nh.param<std::string>("pcd_image_input_dir", pcd_image_input_dir_, HOME + "/dataset/");
    
    if (!IsPathExist(pcd_image_input_dir_))
    {
        ROS_ERROR_STREAM("Failed to find directory: " << pcd_image_input_dir_);
    }

    ros::Rate loop_rate(10);

    pubs["parent"].reset(new ros::Publisher());
    pubs["child"].reset(new ros::Publisher());
    *pubs["parent"] = private_nh.advertise<sensor_msgs::PointCloud2>("/parent/pointcloud", 1); 
    *pubs["child"] = private_nh.advertise<sensor_msgs::PointCloud2>("/child/pointcloud", 1); 


    all_directories = get_directories(pcd_image_input_dir_);
    std::sort(all_directories.begin(), all_directories.end());
}