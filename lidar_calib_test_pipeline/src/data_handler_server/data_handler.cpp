#include <data_handler.hpp>
#include <std_srvs/SetBool.h>


namespace data_provider
{
    namespace fs = std::experimental::filesystem;

    bool data_handler::serve(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res)
    {
        if (progress_in_scene+1 > pcd_pairs.size()) // if all pcd pairs is not published for the current scene 
        {   
            ROS_INFO("Trying to cache scene...");
            if (!cacheScene()) // cache new scene 
            {
                res.success = false;
                res.message = "End of dataset !";
                throw std::runtime_error("End of dataset !");
                return false;
            } else { // if unprocessed data still exists
                createPcdPairs();
                progress_in_scene = 0;
            }
        }        

        res.success = true;
        res.message = "Published PCD pair.";
        publishPcds(pcd_pairs[progress_in_scene].first, pcd_pairs[progress_in_scene].second);
        ROS_INFO_STREAM(progress_in_scene+1 << ". PCD pair" );
        progress_in_scene ++;
        return true;
    }

    bool data_handler::cacheScene()
    {
        //cache scene data from disk, set the next scene to be cached
        //return false when all dataset is processed 
        if (is_last_scene) return false;
        if (curr_scene_idx < (int)scenes_of_agent[curr_agent_idx].size()-1) // update the directory to be cached in the next call
        {
            curr_scene_idx++;
        } else {
            if( curr_agent_idx < (int)scenes_of_agent.size() -1)
            {
                curr_agent_idx++;
                extManager.next(); // set of extrinsics to manager for next agent
                ROS_INFO_STREAM(module_name << "CACHED NEW TF");
                curr_scene_idx = 0;
            } else { // dont let the idx to increase but raise flag that shows no more data left
                is_last_scene = true;
                ROS_WARN_STREAM(module_name << "ALL DATA HAS ALREADY BEEN PROCESSED.");
                return false;
            }
        }

        std::cout<< "Caching agent "<< curr_agent_idx << ", scene "<< curr_scene_idx << " is_last_scene: "<< is_last_scene << std::endl;
        ReadScene(scenes_of_agent[curr_agent_idx][curr_scene_idx]); // cache scene data
        return true;
    }

    void data_handler::ReadScene(std::string scene_dir)
    {   /// input: path to scene dir
        pcds_in_scene.clear();
        pointclouds_map_.clear();
        std::vector<std::string> files_in_scene;
        for(auto& scene_file: fs::directory_iterator(scene_dir))
        {
            if(!fs::is_directory(scene_file) && IsPathExist(scene_file.path().string())) files_in_scene.push_back(scene_file.path().string());
        }
        //Create a PointCloud value
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        
        for(auto& in_file: files_in_scene)
        {
            //Open the PCD file
            if (pcl::io::loadPCDFile<pcl::PointXYZ> (in_file, *cloud) == -1) 
            {
                PCL_ERROR("Couldn't read file \n");
            } else {
                std::string pc_name = getFileName(in_file);
                pcds_in_scene.push_back(pc_name);
                pointclouds_map_[pc_name].reset(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::copyPointCloud(*cloud, *pointclouds_map_[pc_name]);
                ROS_WARN_STREAM("Read, " << in_file);
            }
        }
    }

    void data_handler::publishPcds(std::string parent_name, std::string child_name)
    { 
        std::cout<<"[publishPcds] Publishing parent_frame "<< parent_name << ", child "<< child_name <<std::endl;
        pcl::toROSMsg(*pointclouds_map_[parent_name], parent_msg.pointcloud);
        pcl::toROSMsg(*pointclouds_map_[child_name], child_msg.pointcloud);
        parent_msg.header.frame_id = parent_name;
        child_msg.header.frame_id = child_name;  /// PCD NAMES MUST BE SAME WITH THEIR FRAME_ID
        std::string agent_id = fs::path(agent_dirs[curr_agent_idx]).filename();
        std::string scene_id = fs::path(scenes_of_agent[curr_agent_idx][curr_scene_idx]).filename();;
        
        parent_msg.agent = agent_id;
        parent_msg.scene = scene_id;
        child_msg.agent = agent_id;
        child_msg.scene = scene_id;
        pubs_map_["parent"]->publish(parent_msg);
        pubs_map_["child"]->publish(child_msg);
    }

    void data_handler::createPcdPairs() 
    {
        pcd_pairs.clear();
        // create non repeating pairs from pcds in an individual scene
        for (int i=0; i<pcds_in_scene.size(); i++)
        {
            for (int j=i+1; j<pcds_in_scene.size(); j++)
            {
                pcd_pairs.push_back(std::make_pair(pcds_in_scene[i], pcds_in_scene[j]));
            }
        } 
        std::cout<<"[createPcdPairs] Number of PCD combinations: "<<pcd_pairs.size()<<std::endl;
    }

    void data_handler::setSubdirs(std::string parent_dir, bool is_dataset, int agent_idx=0)
    {
        for(auto& subdir : fs::directory_iterator(parent_dir))
        {
            if (fs::is_directory(subdir) && subdir.path().filename() != "extrinsics")
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

    data_handler::data_handler(): progress_in_scene(2147483647), is_last_scene(false)
    {
        ros::NodeHandle private_nh("~");
        const std::string HOME = getenv("HOME");
        private_nh.param<std::string>("dataset_dir", dataset_dir, HOME + "/test_ws/src//lidar_calib_tester/dummy_dataset/"); //TODO: Parametrize directory selection

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
        extManager.setPaths(agent_dirs);

        extManager.next(); //load first set of extrinsics to manager

        timer = nh_.createTimer(ros::Duration(0.1), &extrinsics_manager::broadcastTFs, &extManager);
        pubs_map_["parent"].reset(new ros::Publisher());
        pubs_map_["child"].reset(new ros::Publisher());
        *pubs_map_["parent"] = private_nh.advertise<lidar_calib_test_comms::test_pointcloud>("/parent/pointcloud", 1); 
        *pubs_map_["child"] = private_nh.advertise<lidar_calib_test_comms::test_pointcloud>("/child/pointcloud", 1); 

        service = private_nh.advertiseService("provide_pc_data", &data_handler::serve, this);
        ROS_INFO_STREAM(module_name << "Ready for data requests");
    }
} //namespace