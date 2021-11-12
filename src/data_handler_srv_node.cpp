#include <data_handler.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_provider_node");
    data_provider::data_handler app;

    ros::spin();
}