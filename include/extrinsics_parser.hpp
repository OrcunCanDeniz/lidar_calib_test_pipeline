#include <string>
#include <experimental/filesystem>
#include <sys/stat.h>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <random>
#include <cmath>


#define DEBUG 1
namespace data_provider
{
    class extrinsics_manager
    {
        public:
            extrinsics_manager();
            void parseYAML(std::string tf_name);
            void broadcastTFs(const ros::TimerEvent&);
            bool next();
            bool updateIdx();
            tf::Transform addNoise(tf::Quaternion q, tf::Vector3 t);
            void setPaths(std::vector<std::string> agent_paths);
        private:
            std::vector<std::string> all_agents;
            bool paths_ready;
            int agent_idx;
            bool last_set_processed;
            std::vector<tf::StampedTransform> gt_transforms_cache;
            std::vector<tf::StampedTransform> initial_transforms_cache;
            const std::string module_name = "[EXTRINSICS PARSER] ";

            // random noise generation
            std::random_device rd{};
            std::mt19937 gen{rd()};

            std::normal_distribution<> translation_noise_d{0,1}; // meters
            std::normal_distribution<> rotation_noise_d{0,0.4}; // radians
    };  
}