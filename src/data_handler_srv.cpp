#include <data_handler.hpp>
#include <std_srvs/SetBool.h>


namespace data_provider
{
    namespace fs = std::experimental::filesystem; 

    bool data_handler::serve(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res)
    // TODO: CHECK THIS FUNCTION 
    {
        if (progress_in_scene <= pcd_pairs.size()-1) // if all pcd pairs is not published for the current scene 
        {
            res.success = true;
            res.message = "Published PCD pair.";
        } else { // if all pcd pairs from the current scene are published
            if (!cacheScene()) // cache new scene 
            {
                res.success = false;
                res.message = "End of dataset !";
                return false;
<<<<<<< HEAD
            } else { // if unprocessed data still exists
                // TODO: create pcd pair
                createPcdPairs();
                progress_in_scene = 0;
=======
>>>>>>> parent of 5a9c478... fixed bug in processed data tracking
            }
            // TODO: create pcd pair
            progress_in_scene = 0;
        }
        //TODO:publish pcd pair
        ROS_INFO_STREAM("Agent idx: " << curr_agent_idx << " Scene idx: " << curr_scene_idx);
        ROS_INFO_STREAM("Progress in scene " << progress_in_scene );
        progress_in_scene ++;
        return true;
    }

    void data_handler::setSubdirs(std::string parent_dir, bool is_dataset, int agent_idx=0)
    {
        for(auto& subdir : fs::directory_iterator(parent_dir))
        {
            if (fs::is_directory(subdir))
            {
                if (is_dataset) // if subdirs are agents
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

    void data_handler::ReadScene(std::string scene_dir)
    {   /// input: path to scene dir
        std::vector<std::string> files_in_scene;
        for(auto& scene_file: fs::directory_iterator(scene_dir))
        {
            if(!fs::is_directory(scene_file) && IsPathExist(scene_file.path().string()))files_in_scene.push_back(scene_file.path().string());
        }

        //Create a PointCloud value
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
        
        for(auto& in_file: files_in_scene)
        {
            //Open the PCD file
            if (pcl::io::loadPCDFile<pcl::PointXYZI> (in_file, *cloud) == -1) 
            {
                PCL_ERROR("Couldn't read in_file\n");
            } else {
                pcds_in_scene.push_back(in_file);
                std::string pc_name = getFileName(in_file);
                pointclouds_map_[pc_name].reset(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::copyPointCloud(*cloud, *pointclouds_map_[pc_name]);
            }
        }
    }

    bool data_handler::cacheScene() 
    {
        //cache scene data from disk, set the next scene to be cached
        //return true when all dataset is processed 
        ReadScene(scenes_of_agent[curr_agent_idx][curr_scene_idx]); // cache scene data 

        if (curr_scene_idx < scenes_of_agent.size()) // update the directory to be cached in the next call
        {
            curr_scene_idx++;
        } else {
            if( curr_agent_idx < scenes_of_agent.size()-1)
            {
                curr_agent_idx++;
                curr_scene_idx = 0;
            } else {
                std::cout<< "ALL DONE"<<std::endl;
                return false;
            }
        }
        return true;
    }

    void data_handler::createPcdPairs() 
    {
        // create non repeating pairs from pcds in an individual scene
        for (int i=0; i<pcds_in_scene.size(); i++)
        {
            int tmp = i;
            for (j=i+1; j<pcds_in_scene.size(); j++)
            {
                pcd_pairs.push_back(std::make_pair(pcds_in_scene[tmp],pcds_in_scene[j]);
            }
        } 
    }
    
    // add extrinsic parser

    std::string data_handler::getFileName(std::string file_path)
    {
        std::string base_filename = file_path.substr(file_path.find_last_of("/\\") + 1);
        std::string::size_type const p(base_filename.find_last_of('.'));
        std::string file_without_extension = base_filename.substr(0, p);
        return file_without_extension;
    }

    bool data_handler::IsPathExist(const std::string &s)
    {
        struct stat buffer;
        return (stat (s.c_str(), &buffer) == 0);
    }

    // std::vector<std::string> data_handler::get_directories(const std::string& s)
    // {
    //     std::vector<std::string> r;
    //     for(auto& p : fs::recursive_directory_iterator(s))
    //         if (fs::is_directory(p))
    //             r.push_back(p.path().string());
    //     return r;
    // }

    data_handler::data_handler(): progress_in_scene(0)
    {
        ros::NodeHandle private_nh("~");
        // ROS_INFO("Running data handler.");
        const std::string HOME = getenv("HOME");
        private_nh.param<std::string>("dataset_dir", dataset_dir, HOME + "/test_ws/src/data_handler_srv/dummy_dataset/");
        
        if (!IsPathExist(dataset_dir))
        {
            ROS_ERROR_STREAM("Failed to find directory: " << dataset_dir);
        }

        //cache dirs of agents and scenes belonging to them
        setSubdirs(dataset_dir, true); // get Agents
        int agent_idx = 0;
        for(auto& agent : agent_dirs)
        {
        setSubdirs(agent,false,agent_idx);
        agent_idx++;  
        } 

        ros::Rate loop_rate(10);

        pcd_pairs.resize(3); // dummy size for test, change accordingly in the future

        pubs_map_["parent"].reset(new ros::Publisher());
        pubs_map_["child"].reset(new ros::Publisher());
        *pubs_map_["parent"] = private_nh.advertise<sensor_msgs::PointCloud2>("/parent/pointcloud", 1); 
        *pubs_map_["child"] = private_nh.advertise<sensor_msgs::PointCloud2>("/child/pointcloud", 1); 

        // all_directories = get_directories(pcd_image_input_dir_);
        // std::sort(all_directories.begin(), all_directories.end());

        service = private_nh.advertiseService("provide_pc_data", &data_handler::serve, this);

    }
} //namespace