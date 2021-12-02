#include "io_utils.hpp"


namespace io_utils
{
    void prettyPrint(std::string agent_name, stats stat)
    {
        std::stringstream line_st;
        std::cout << "########### "  
        << "Results for Agent: " << agent_name << "\n"
        << "    Translation XYZ(mean): "<<stat.tf.mean.trans.x<<", "<<stat.tf.mean.trans.y<<", "<<stat.tf.mean.trans.z<<"\n"
        << "    Rotation RPY(mean): "<<stat.tf.mean.rot.roll<<", "<<stat.tf.mean.rot.pitch<<", "<<stat.tf.mean.rot.yaw<<"\n"
        << "    Stdev Translation XYZ: "<<stat.tf.dev.trans.x<<", "<<stat.tf.dev.trans.y<<", "<<stat.tf.dev.trans.z<<"\n"
        << "    Stdev Rotation RPY: " <<stat.tf.dev.rot.roll<<", "<<stat.tf.dev.rot.pitch<<", "<<stat.tf.dev.rot.yaw<<"\n"
        << "   ---   ---   ---   ---   \n"
        << "    Translation Error XYZ(mean): "<<stat.err.mean.trans.x<<", "<<stat.err.mean.trans.y<<", "<<stat.err.mean.trans.z<<", "
        << "    Rotation Error RPY(mean): "<<stat.err.mean.rot.roll<<", "<<stat.err.mean.rot.pitch<<", "<<stat.err.mean.rot.yaw<<", "
        << "    Stdev Translation Error XYZ: "<<stat.err.dev.trans.x<<", "<<stat.err.dev.trans.y<<", "<<stat.err.dev.trans.z<<", "
        << "    Stdev Rotation Error RPY: "<<stat.err.dev.rot.roll<<", "<<stat.err.dev.rot.pitch<<", "<<stat.err.dev.rot.yaw<< std::endl;
    }



    std::stringstream stat2Stream(stats stat)
    {
        std::stringstream line_st;
        line_st<<stat.tf.mean.trans.x<<","<<stat.tf.mean.trans.y<<","<<stat.tf.mean.trans.z<<","
        <<stat.tf.mean.rot.roll<<","<<stat.tf.mean.rot.pitch<<","<<stat.tf.mean.rot.yaw<<","
        <<stat.tf.dev.trans.x<<","<<stat.tf.dev.trans.y<<","<<stat.tf.dev.trans.z<<","
        <<stat.tf.dev.rot.roll<<","<<stat.tf.dev.rot.pitch<<","<<stat.tf.dev.rot.yaw<<","

        <<stat.err.mean.trans.x<<","<<stat.err.mean.trans.y<<","<<stat.err.mean.trans.z<<","
        <<stat.err.mean.rot.roll<<","<<stat.err.mean.rot.pitch<<","<<stat.err.mean.rot.yaw<<","
        <<stat.err.dev.trans.x<<","<<stat.err.dev.trans.y<<","<<stat.err.dev.trans.z<<","
        <<stat.err.dev.rot.roll<<","<<stat.err.dev.rot.pitch<<","<<stat.err.dev.rot.yaw<<","<< std::endl;

        return line_st;
    }



    void addStat(std::string agent, std::string scene, stats stat)
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
        std::string filename = "./tests/TEST_" + stamp() + ".csv"; // set file name as date-time. place it to home dir
        std::ofstream csv_file(filename); // create file on disk
        csv_file << stats_csv.str(); // dump stats in file
        csv_file.close(); // Close the file
    }
} // namespace