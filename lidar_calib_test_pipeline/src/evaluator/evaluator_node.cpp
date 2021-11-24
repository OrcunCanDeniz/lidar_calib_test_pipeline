#include "evaluator.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "evaluator");
    evaluator app;

    ros::spin();

    return 0;
}