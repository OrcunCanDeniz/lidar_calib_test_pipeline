#pragma once

#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include <fstream>
#include <chrono>
#include <ctime>    
#include "types.hpp"

namespace io_utils
{
    std::stringstream stats_csv;
    void addStat(std::string agent, std::string scene, statType stat);
    void addStat(std::string agent, std::string scene, genericT stat);
    std::stringstream stat2Stream(genericT stat);
    
    std::string stamp();
    void prettyPrint(std::string agent_name, statType stat);
    void writeCsv();
}