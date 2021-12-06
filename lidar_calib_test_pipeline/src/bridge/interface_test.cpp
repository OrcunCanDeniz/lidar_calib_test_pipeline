#include "interface.hpp"

class trial_cls
{
    public:
        trial_cls();
        calib_test_bridge bridge;
        void calibrator(const sensor_msgs::PointCloud2::ConstPtr, const sensor_msgs::PointCloud2::ConstPtr);
};

trial_cls::trial_cls()
{
    ros::NodeHandle nh_, pnh_; 
    // calib_test_bridge app;
    bridge = calib_test_bridge(&nh_, &pnh_, boost::bind(&trial_cls::calibrator, this, _1, _2));
}


void trial_cls::calibrator(const sensor_msgs::PointCloud2::ConstPtr, const sensor_msgs::PointCloud2::ConstPtr)
{
    std::cerr<<"not impl"<<std::endl;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "bridge_test");
    ros::NodeHandle nh_, pnh_; 
    trial_cls trl;
    calib_test_bridge app(&nh_, &pnh_, boost::bind(&trial_cls::calibrator, trl, _1, _2));
    ros::spin();

    return 0;
}