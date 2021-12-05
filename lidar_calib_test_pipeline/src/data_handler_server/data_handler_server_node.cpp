#include <data_handler.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_handling_server");
    data_provider::data_handler app;

    ros::spin();

    return 0;
}