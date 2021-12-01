#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <chrono>
#include <ctime>    
#include <sstream>
#include "types.hpp"

namespace io_utils
{
    std::stringstream stats_csv;

    void addStat(std::string agent, std::string scene, stats stat);
    std::string stamp();
    void prettyPrint(std::string agent_name, stats stat);
    void writeCsv();
}