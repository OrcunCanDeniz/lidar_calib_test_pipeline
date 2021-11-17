#include <string>
#include <experimental/filesystem>
#include <sys/stat.h>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>


#define DEBUG 1
namespace data_provider
{
    namespace extrinsics
    {
        class extrinsics_manager
        {
            public:
                extrinsics_manager(std::vector<std::string> agent_paths);
                extrinsics_manager();
                void parseYAML(std::string tf_name);
                void broadcastTFs();
                bool next();
                bool updateIdx();
            
            private:
                std::vector<std::string> all_agents;
                int agent_idx;
                bool last_set_processed;
                std::vector<tf::StampedTransform> transforms_cache;
                const std::string module_name = "[EXTRINSICS PARSER] ";

        };
    }
}