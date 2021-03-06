#include "io_utils.hpp"
#include <ros/package.h>
#include "ros/ros.h"


namespace io_utils
{
    void prettyPrint(std::string agent_name, statType stat)
    {
        std::stringstream line_st;
        std::cout << "\n########   "<< "Results for Agent: " << agent_name << "   ######## " << "\n"
        << "    Translation Error XYZ(mean): "<<stat.mean.trans.x<<", "<<stat.mean.trans.y<<", "<<stat.mean.trans.z<<", "<< std::endl
        << "    Rotation Error RPY(mean): "<<stat.mean.rot.roll<<", "<<stat.mean.rot.pitch<<", "<<stat.mean.rot.yaw<<", "<< std::endl
        << "    Stdev Translation Error XYZ: "<<stat.dev.trans.x<<", "<<stat.dev.trans.y<<", "<<stat.dev.trans.z<<", "<< std::endl
        << "    Stdev Rotation Error RPY: "<<stat.dev.rot.roll<<", "<<stat.dev.rot.pitch<<", "<<stat.dev.rot.yaw<< std::endl;
    }



    std::stringstream stat2Stream(statType stat)
    {
        std::stringstream line_st;
        line_st<<stat.mean.trans.x<<","<<stat.mean.trans.y<<","<<stat.mean.trans.z<<","
        <<stat.mean.rot.roll<<","<<stat.mean.rot.pitch<<","<<stat.mean.rot.yaw<< std::endl;

        return line_st;
    }

    std::stringstream stat2Stream(genericT stat)
    {
        std::stringstream line_st;
        line_st<<stat.trans.x<<","<<stat.trans.y<<","<<stat.trans.z<<","
        <<stat.rot.roll<<","<<stat.rot.pitch<<","<<stat.rot.yaw<< std::endl;

        return line_st;
    }


    void addStat(std::string agent, std::string scene, genericT stat)
    {
        std::stringstream stream_from_stat = stat2Stream(stat);
        std::stringstream line_st;

        line_st<<agent<<","<<scene<<","<<stream_from_stat.str();

        stats_csv << line_st.str();
    }

    void addStat(std::string agent, std::string scene, statType stat)
    {
        std::stringstream stream_from_stat = stat2Stream(stat);
        std::stringstream line_st;

        line_st<<agent<<","<<scene<<","<<stream_from_stat.str();

        stats_csv << line_st.str();
    }

    std::string stamp()
    {
        time_t t ;
        struct tm *tmp ;
        char MY_TIME[50];
        time( &t );
        
        tmp = localtime( &t );
        
        // using strftime to display time
        strftime(MY_TIME, sizeof(MY_TIME), "%Y_%m_%d__%H_%M", tmp);
        // std::string stamp = ("Formatted date & time : %s\n", MY_TIME );
        return MY_TIME;
    }

    void writeCsv()
    {
        std::stringstream columns;
        columns << "agent_id,scene_id,tx_err,ty_err,tz_err,R_err,P_err,Y_err\n";

        std::string filename = "TEST_" + stamp() + ".csv"; // set file name as date-time. place it to home dir
        std::string pkg_path = ros::package::getPath("lidar_calib_test_pipeline");
        std::string save_dir = pkg_path + "/test_runs/" + filename;
        std::ofstream csv_file( save_dir ); // create file on disk
        ROS_INFO_STREAM("\nDumping stats to: " << save_dir);
        columns << stats_csv.str();
        csv_file << columns.str(); // dump stats in file
        csv_file.close(); // Close the file
    }
} // namespace